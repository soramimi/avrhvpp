
#include "../common/commands.h"
#include "AvrIsp.h"
#include "AvrUpdi.h"
#include "Connection.h"
#include "main.h"
#include "serial.h"
#include "target.h"
#include "target.h"
#include <assert.h>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#ifdef _WIN32
#else
#include <assert.h>
#include <unistd.h>
#endif


void msleep(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

enum class Device {
	HVPP,
	UPDI,
	AVRISP,
};

struct Options {
	Device device = Device::HVPP;
	Serial::Option serial_options;
	std::unique_ptr<Target> target;
	bool chip_erase_before_write_hex = true;
	bool help = false;
};

bool parse_device_type(std::string_view name, Device *device)
{
	if (name == "hvpp") {
		*device = Device::HVPP;
	} else if (name == "updi") {
		*device = Device::UPDI;
	} else if (name == "avrisp") {
		*device = Device::AVRISP;
	} else {
		fprintf(stderr, "unknown device type: %.*s (available: hvpp, avrisp, updi)\n", (int)name.size(), name.data());
		return false;
	}
	return true;
}

bool parse_hex_value(char const *text, uint32_t limit, uint32_t *value)
{
	errno = 0;
	char *end;
	unsigned long long v = strtoull(text, &end, 16);
	if (errno || end == text || *end || v > limit || std::string_view(text).find('-') != std::string_view::npos) {
		return false;
	}
	*value = static_cast<uint32_t>(v);
	return true;
}

bool parse_byte(char const *text, uint8_t *value)
{
	uint32_t parsed;
	if (!parse_hex_value(text, 0xff, &parsed)) return false;
	*value = static_cast<uint8_t>(parsed);
	return true;
}

bool parse_page(char const *text, Target const &target, uint16_t *page)
{
	errno = 0;
	char *end;
	int base = text[0] == '0' && (text[1] == 'x' || text[1] == 'X') ? 16 : 10;
	unsigned long value = strtoul(text, &end, base);
	if (errno || end == text || *end || value >= target.definition().flash_page_count) {
		return false;
	}
	*page = (uint16_t)value;
	return true;
}

bool make_flash_parameters(uint16_t page, Target const &target, FlashParameters *parameters)
{
	TargetDefinition const &definition = target.definition();
	uint32_t address = definition.flash_word_address(page, 0);
	*parameters = {(uint16_t)address, definition.flash_page_bytes};
	if (page >= definition.flash_page_count || address > 0xffffUL || !parameters->valid()) {
		fprintf(stderr, "invalid flash page or size for %.*s\n", (int)target.name().size(), target.name().data());
		return false;
	}
	return true;
}

bool read_page_file(char const *path, Target const &target, std::vector<uint8_t> *data)
{
	FILE *fp = fopen(path, "rb");
	if (!fp) {
		fprintf(stderr, "failed to open %s\n", path);
		return false;
	}
	data->resize(target.definition().flash_page_bytes + 1);
	size_t size = fread(data->data(), 1, data->size(), fp);
	bool ok = !ferror(fp);
	if (fclose(fp) != 0) {
		ok = false;
	}
	if (!ok) {
		fprintf(stderr, "failed to read %s\n", path);
		return false;
	}
	if (size != target.definition().flash_page_bytes) {
		fprintf(stderr, "%s must be exactly %u bytes for %.*s\n", path, (unsigned int)target.definition().flash_page_bytes, (int)target.name().size(), target.name().data());
		return false;
	}
	data->resize(target.definition().flash_page_bytes);
	return true;
}

int hex_value(char c)
{
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	return -1;
}

bool parse_hex_record(std::string const &line, std::vector<uint8_t> *record)
{
	if (line.size() < 11 || line[0] != ':' || ((line.size() - 1) & 1)) {
		return false;
	}
	record->clear();
	for (size_t i = 1; i < line.size(); i += 2) {
		int high = hex_value(line[i]);
		int low = hex_value(line[i + 1]);
		if (high < 0 || low < 0) {
			return false;
		}
		record->push_back((uint8_t)((high << 4) | low));
	}
	if (record->size() != (size_t)(record->at(0) + 5)) {
		return false;
	}
	uint8_t sum = 0;
	for (uint8_t value : *record) {
		sum += value;
	}
	return sum == 0;
}

bool read_hex_file(char const *path, Target const &target, std::vector<uint8_t> *image, std::vector<uint8_t> *page_mask)
{
	std::ifstream input(path, std::ios::binary);
	if (!input) {
		fprintf(stderr, "failed to open %s\n", path);
		return false;
	}

	image->assign(target.definition().flash_bytes(), 0xff);
	std::vector<bool> assigned(target.definition().flash_bytes(), false);
	page_mask->assign(target.definition().flash_page_count, 0);
	std::string line;
	std::vector<uint8_t> record;
	uint32_t base_address = 0;
	unsigned int line_number = 0;
	bool eof = false;
	while (std::getline(input, line)) {
		line_number++;
		if (!line.empty() && line.back() == '\r') {
			line.pop_back();
		}
		if (line.empty()) {
			continue;
		}
		if (eof) {
			fprintf(stderr, "%s:%u: data after EOF record\n", path, line_number);
			return false;
		}
		if (!parse_hex_record(line, &record)) {
			fprintf(stderr, "%s:%u: invalid Intel HEX record\n", path, line_number);
			return false;
		}

		uint8_t size = record[0];
		uint16_t address = ((uint16_t)record[1] << 8) | record[2];
		uint8_t type = record[3];
		if (type == 0x00) {
			uint64_t absolute = (uint64_t)base_address + address;
			if (absolute + size > target.definition().flash_bytes()) {
				fprintf(stderr, "%s:%u: address is outside %.*s flash\n", path, line_number, (int)target.name().size(), target.name().data());
				return false;
			}
			for (uint16_t i = 0; i < size; i++) {
				uint32_t offset = (uint32_t)(absolute + i);
				uint8_t value = record[4 + i];
				if (assigned[offset] && image->at(offset) != value) {
					fprintf(stderr, "%s:%u: conflicting data at address 0x%04lx\n", path, line_number, (unsigned long)offset);
					return false;
				}
				image->at(offset) = value;
				assigned[offset] = true;
				page_mask->at(offset / target.definition().flash_page_bytes) = 1;
			}
		} else if (type == 0x01) {
			if (size != 0 || address != 0) {
				fprintf(stderr, "%s:%u: invalid EOF record\n", path, line_number);
				return false;
			}
			eof = true;
		} else if (type == 0x02) {
			if (size != 2 || address != 0) {
				fprintf(stderr, "%s:%u: invalid extended segment address record\n", path, line_number);
				return false;
			}
			base_address = ((((uint32_t)record[4] << 8) | record[5]) << 4);
		} else if (type == 0x04) {
			if (size != 2 || address != 0) {
				fprintf(stderr, "%s:%u: invalid extended linear address record\n", path, line_number);
				return false;
			}
			base_address = ((((uint32_t)record[4] << 8) | record[5]) << 16);
		} else if (type == 0x03 || type == 0x05) {
			if (size != 4 || address != 0) {
				fprintf(stderr, "%s:%u: invalid start address record\n", path, line_number);
				return false;
			}
		} else {
			fprintf(stderr, "%s:%u: unsupported Intel HEX record type 0x%02x\n", path, line_number, type);
			return false;
		}
	}
	if (input.bad()) {
		fprintf(stderr, "failed to read %s\n", path);
		return false;
	}
	if (!eof) {
		fprintf(stderr, "%s: missing EOF record\n", path);
		return false;
	}
	return true;
}

bool write_hex_record(FILE *fp, uint16_t address, uint8_t type, uint8_t const *data, uint8_t size)
{
	uint8_t sum = size;
	sum += address >> 8;
	sum += address;
	sum += type;
	if (fprintf(fp, ":%02X%04X%02X", (unsigned int)size, (unsigned int)address, (unsigned int)type) < 0) {
		return false;
	}
	for (uint8_t i = 0; i < size; i++) {
		sum += data[i];
		if (fprintf(fp, "%02X", (unsigned int)data[i]) < 0) {
			return false;
		}
	}
	uint8_t checksum = 0 - sum;
	return fprintf(fp, "%02X\r\n", (unsigned int)checksum) >= 0;
}

bool write_hex_file(char const *path, std::vector<uint8_t> const &image, unsigned int *records)
{
	FILE *fp = fopen(path, "wb");
	if (!fp) {
		fprintf(stderr, "failed to open %s\n", path);
		return false;
	}

	bool ok = true;
	*records = 0;
	for (uint32_t address = 0; address < image.size(); address += 16) {
		bool blank = true;
		for (uint8_t i = 0; i < 16; i++) {
			if (image[address + i] != 0xff) {
				blank = false;
				break;
			}
		}
		if (blank) {
			continue;
		}
		if (!write_hex_record(fp, (uint16_t)address, 0x00, &image[address], 16)) {
			ok = false;
			break;
		}
		(*records)++;
	}
	if (ok && !write_hex_record(fp, 0, 0x01, nullptr, 0)) {
		ok = false;
	}
	if (fclose(fp) != 0) {
		ok = false;
	}
	if (!ok) {
		fprintf(stderr, "failed to write %s\n", path);
		return false;
	}
	return true;
}

void write_command(Connection *conn, uint8_t command)
{
	conn->write(&command, 1);
}

void write_encoded_byte(Connection *conn, uint8_t value, uint8_t command0, uint8_t command1)
{
	write_command(conn, CMD_LOAD_7BITS | (value & 0x7f));
	write_command(conn, (value & 0x80) ? command1 : command0);
}

bool decode_response(std::vector<char> const &line, size_t expected_bytes, std::vector<uint8_t> *data, ProtocolError *error)
{
	data->clear();
	*error = ProtocolError::None;
	if (line.size() == 3 && line[0] == 'E') {
		int high = hex_value(line[1]);
		int low = hex_value(line[2]);
		if (high < 0 || low < 0 || (high == 0 && low == 0)) return false;
		*error = (ProtocolError)((high << 4) | low);
		return true;
	}
	if (expected_bytes == 0) return line == "1";
	if (line.size() != expected_bytes * 2) return false;
	for (size_t i = 0; i < expected_bytes; i++) {
		int high = hex_value(line[i * 2]);
		int low = hex_value(line[i * 2 + 1]);
		if (high < 0 || low < 0) {
			data->clear();
			return false;
		}
		data->push_back((uint8_t)((high << 4) | low));
	}
	return true;
}

char const *protocol_error_message(ProtocolError error)
{
	switch (error) {
	case ProtocolError::InvalidArgument: return "invalid arguments or buffer length";
	case ProtocolError::ProgrammingMode: return "failed to enter programming mode";
	case ProtocolError::ReadyTimeout: return "RDY/BSY timeout";
	case ProtocolError::VerifyMismatch: return "read-back verification failed";
	case ProtocolError::NotBlank: return "flash is not blank and does not match the input";
	case ProtocolError::UnknownCommand: return "unknown command";
	default: return "unknown device error";
	}
}

bool read_response(Connection *conn, size_t expected_bytes, std::vector<uint8_t> *data, int timeout)
{
	std::vector<char> line;
	if (!conn->read_line(&line, timeout)) {
		fprintf(stderr, "no response\n");
		return false;
	}
	ProtocolError error;
	if (!decode_response(line, expected_bytes, data, &error)) {
		fprintf(stderr, "invalid response\n");
		return false;
	}
	if (error != ProtocolError::None) {
		fprintf(stderr, "device error E%02X: %s\n", (unsigned int)error, protocol_error_message(error));
		return false;
	}
	return true;
}

bool read_ack(Connection *conn, int timeout)
{
	std::vector<uint8_t> data;
	return read_response(conn, 0, &data, timeout);
}

bool write_flash_parameters(uint16_t address, uint16_t size, Connection *conn)
{
	FlashParameters parameters = {address, size};
	if (!parameters.valid()) {
		fprintf(stderr, "invalid flash parameters\n");
		return false;
	}
	write_command(conn, CMD_CLEAR);
	write_encoded_byte(conn, (uint8_t)address, CMD_DATA0, CMD_DATA1);
	write_encoded_byte(conn, (uint8_t)(address >> 8), CMD_DATA0, CMD_DATA1);
	write_encoded_byte(conn, (uint8_t)size, CMD_DATA0, CMD_DATA1);
	write_encoded_byte(conn, (uint8_t)(size >> 8), CMD_DATA0, CMD_DATA1);
	write_command(conn, CMD_LATCH_FLASH_PARAMS);
	return read_ack(conn, COMMAND_TIMEOUT_MS);
}

bool avr_fetch_status(Command command, uint8_t address, size_t bytes, std::vector<uint8_t> *data, Connection *conn)
{
	write_command(conn, CMD_CLEAR);
	write_encoded_byte(conn, address, CMD_DATA0, CMD_DATA1);
	write_command(conn, command);
	return read_response(conn, bytes, data, COMMAND_TIMEOUT_MS);
}

template <typename ReadStatus, typename Emit>
bool query_target(ReadStatus read_status, Emit emit, bool hvpp)
{
	std::vector<uint8_t> signature;
	if (!read_status(CMD_READ_SIGNATURE, 0, 3, &signature)) return false;
	char text[256];
	snprintf(text, sizeof(text), "signature: 0x%02x, 0x%02x, 0x%02x\n", signature[0], signature[1], signature[2]);
	emit(text);
	TargetDefinition const *target = find_target_by_signature(signature[0], signature[1], signature[2]);
	if (!target) {
		emit("chip: unknown\n");
		fprintf(stderr, "unsupported target signature\n");
		return false;
	}
	snprintf(text, sizeof(text), "chip: %s\npage size: %u bytes\npage count: %u\nflash size: %lu bytes\n",
		target->display_name, (unsigned int)target->flash_page_bytes,
		(unsigned int)target->flash_page_count, (unsigned long)target->flash_bytes());
	emit(text);
	if (target->supports(CapabilityUpdi)) {
		fprintf(stderr, "%s requires --device updi\n", target->display_name);
		return false;
	}
	if (hvpp && !target->supports(CapabilityHvpp)) {
		fprintf(stderr, "%s requires --device avrisp; HVPP is not supported\n", target->display_name);
		return false;
	}
	struct Field {
		TargetCapability capability;
		char const *label;
		Command command;
		uint8_t address;
	};
	static const Field fields[] = {
		{CapabilityFuseLow, "fuse lo", CMD_READ_FUSE, 0},
		{CapabilityFuseHigh, "fuse hi", CMD_READ_FUSE, 1},
		{CapabilityFuseExtended, "fuse ex", CMD_READ_FUSE, 2},
		{CapabilityLockBits, "lock bits", CMD_READ_LOCK, 0},
	};
	for (Field const &field : fields) {
		if (!target->supports(field.capability)) continue;
		std::vector<uint8_t> data;
		if (!read_status(field.command, field.address, 1, &data)) return false;
		snprintf(text, sizeof(text), "%s: 0x%02x\n", field.label, data[0]);
		emit(text);
	}
	return true;
}


bool hvpp_query(Connection *conn)
{
	return query_target([&](Command command, uint8_t address, size_t bytes, std::vector<uint8_t> *data) {
		return avr_fetch_status(command, address, bytes, data, conn);
	}, [](char const *text) { fputs(text, stdout); }, true);
}

bool hvpp_read_status(Operation const &op, Connection *conn)
{
	int bytes = 0;
	switch (op.command) {
	case CMD_READ_SIGNATURE:
		bytes = 3;
		break;
	case CMD_READ_FUSE:
		bytes = 1;
		break;
	case CMD_READ_LOCK:
		bytes = 1;
		break;
	default:
		break;
	}
	if (bytes == 0) {
		fprintf(stderr, "Invalid command\n");
		return false;
	}

	std::vector<uint8_t> data;
	if (!avr_fetch_status(op.command, (uint8_t)op.data, bytes, &data, conn)) {
		return false;
	}
	std::string text;
	int n = bytes;
	for (int i = 0; i < n; i++) {
		char tmp[10];
		if (i > 0) {
			text += ", ";
		}
		sprintf(tmp, "0x%02x", data[i]);
		text += tmp;
	}
	puts(text.c_str());
	return true;
}

bool hvpp_write_fuse(Operation const &op, Connection *conn)
{
	write_command(conn, CMD_CLEAR);
	write_encoded_byte(conn, (uint8_t)op.data, CMD_DATA0, CMD_DATA1);
	write_encoded_byte(conn, op.value, CMD_DATA0, CMD_DATA1);
	write_command(conn, CMD_WRITE_FUSE);

	if (read_ack(conn, COMMAND_TIMEOUT_MS)) {
		printf("fuse written: 0x%02x\n", op.value);
		return true;
	} else {
		fprintf(stderr, "failed to write fuse\n");
		return false;
	}
}

bool hvpp_write_lock(Operation const &op, Connection *conn)
{
	write_command(conn, CMD_CLEAR);
	write_encoded_byte(conn, op.value, CMD_DATA0, CMD_DATA1);
	write_command(conn, CMD_WRITE_LOCK);

	if (read_ack(conn, COMMAND_TIMEOUT_MS)) {
		printf("lock bits written: 0x%02x\n", op.value);
		return true;
	} else {
		fprintf(stderr, "failed to write lock bits\n");
		return false;
	}
}

bool avr_chip_erase(Connection *conn)
{
	write_command(conn, CMD_CHIP_ERASE);
	if (read_ack(conn, PAGE_WRITE_TIMEOUT_MS)) return true;
	fprintf(stderr, "failed to erase chip\n");
	return false;
}

bool hvpp_chip_erase(Connection *conn)
{
	if (!avr_chip_erase(conn)) return false;
	puts("chip erased");
	return true;
}


bool avr_fetch_page(uint16_t page, Target const &target, std::vector<uint8_t> *data, Connection *conn)
{
	data->clear();
	data->reserve(target.definition().flash_page_bytes);

	FlashParameters parameters;
	if (!make_flash_parameters(page, target, &parameters)
		|| !write_flash_parameters(parameters.word_address, parameters.byte_count, conn)) {
		return false;
	}
	write_command(conn, CMD_READ_PAGE);
	if (!read_ack(conn, COMMAND_TIMEOUT_MS)) {
		fprintf(stderr, "failed to read flash page %u\n", (unsigned int)page);
		return false;
	}

	write_encoded_byte(conn, 0, CMD_ADDR0, CMD_ADDR1);
	for (uint16_t offset = 0; offset < parameters.byte_count; offset += 16) {
		write_command(conn, CMD_READ_16BYTES);
		std::vector<uint8_t> bytes;
		if (!read_response(conn, 16, &bytes, COMMAND_TIMEOUT_MS)) {
			return false;
		}
		uint16_t remaining = parameters.byte_count - offset;
		data->insert(data->end(), bytes.begin(), bytes.begin() + (remaining < 16 ? remaining : 16));
	}
	return true;
}

bool write_page_file(char const *path, std::vector<uint8_t> const &data)
{
	FILE *fp = fopen(path, "wb");
	if (!fp) {
		fprintf(stderr, "failed to open %s\n", path);
		return false;
	}
	bool ok = fwrite(data.data(), 1, data.size(), fp) == data.size();
	if (fclose(fp) != 0) {
		ok = false;
	}
	if (!ok) {
		fprintf(stderr, "failed to write %s\n", path);
		return false;
	}
	return true;
}

class HvppFlash {
private:
	Target const &target_;
	Connection *conn_;
public:
	HvppFlash(Target const &target, Connection *conn) : target_(target), conn_(conn) {}

	bool read_page(uint16_t page, std::vector<uint8_t> *data)
	{
		return avr_fetch_page(page, target_, data, conn_);
	}

	bool write_page(uint16_t page, uint8_t const *data, size_t size)
	{
		FlashParameters parameters;
		if (size != target_.definition().flash_page_bytes || !make_flash_parameters(page, target_, &parameters) || !write_flash_parameters(parameters.word_address, parameters.byte_count, conn_)) return false;
		for (size_t i = 0; i < size; ++i) {
			write_encoded_byte(conn_, data[i], CMD_DATA0, CMD_DATA1);
		}
		write_command(conn_, CMD_WRITE_PAGE);
		if (read_ack(conn_, PAGE_WRITE_TIMEOUT_MS)) return true;
		fprintf(stderr, "failed to write flash page %u; the page must be blank or already match the input\n", (unsigned int)page);
		return false;
	}

	bool chip_erase() { return avr_chip_erase(conn_); }
};

template <typename Flash>
bool perform_flash_operation(Operation const &op, Flash &flash, std::string *text)
{
	if (!op.target) {
		fprintf(stderr, "flash operations require --target\n");
		return false;
	}
	auto const &target = op.target->definition();
	if (op.command == CMD_READ_PAGE || op.command == CMD_WRITE_PAGE) {
		if (op.data >= target.flash_page_count) return false;
		if (op.command == CMD_READ_PAGE) {
			std::vector<uint8_t> data;
			if (!flash.read_page(op.data, &data) || !write_page_file(op.path.c_str(), data)) return false;
			*text += "page " + std::to_string(op.data) + " read: " + op.path + "\n";
		} else {
			if (op.bytes.size() != target.flash_page_bytes) {
				fprintf(stderr, "invalid flash page data length\n");
				return false;
			}
			if (!flash.write_page(op.data, op.bytes.data(), op.bytes.size())) return false;
			*text += "page " + std::to_string(op.data) + " written: " + op.path + "\n";
		}
		return true;
	}
	if (op.command == _CMD_READ_HEX) {
		std::vector<uint8_t> image(target.flash_bytes());
		for (uint16_t page = 0; page < target.flash_page_count; ++page) {
			std::vector<uint8_t> data;
			if (!flash.read_page(page, &data)) return false;
			memcpy(image.data() + (size_t)page * target.flash_page_bytes, data.data(), target.flash_page_bytes);
		}
		unsigned int records;
		if (!write_hex_file(op.path.c_str(), image, &records)) return false;
		*text += "hex read: " + op.path + " (" + std::to_string(records) + " data records)\n";
		return true;
	}
	if (op.command != _CMD_WRITE_HEX) return false;
	if (op.bytes.size() != target.flash_bytes() || op.page_mask.size() != target.flash_page_count) {
		fprintf(stderr, "invalid flash image or page mask\n");
		return false;
	}
	if (op.chip_erase) {
		if (!flash.chip_erase()) return false;
		*text += "chip erased\n";
	}
	std::vector<uint8_t> pages_to_write(target.flash_page_count, 0);
	unsigned int pages_present = 0, pages_matching = 0, pages_written = 0;
	// Inspect every selected page before writing the first one.
	for (uint16_t page = 0; page < target.flash_page_count; ++page) {
		if (!op.page_mask[page]) continue;
		++pages_present;
		std::vector<uint8_t> current;
		if (!flash.read_page(page, &current)) return false;
		size_t offset = (size_t)page * target.flash_page_bytes;
		bool matches = !memcmp(current.data(), op.bytes.data() + offset, target.flash_page_bytes);
		bool blank = true;
		for (uint8_t byte : current) if (byte != 0xff) blank = false;
		if (matches) ++pages_matching;
		else if (blank) pages_to_write[page] = 1;
		else {
			fprintf(stderr, "flash page %u is not blank and does not match %s; no pages were written\n", (unsigned int)page, op.path.c_str());
			return false;
		}
	}
	for (uint16_t page = 0; page < target.flash_page_count; ++page) {
		if (!pages_to_write[page]) continue;
		if (!flash.write_page(page, op.bytes.data() + (size_t)page * target.flash_page_bytes, target.flash_page_bytes)) return false;
		++pages_written;
		*text += "page " + std::to_string(page) + " written: " + op.path + "\n";
	}
	*text += "hex written: " + op.path + " (" + std::to_string(pages_present) + " data pages, "
		+ std::to_string(pages_written) + " written, " + std::to_string(pages_matching) + " already matched"
		+ (op.chip_erase ? ", chip erased" : "") + ")\n";
	return true;
}

bool hvpp_flash_operation(Operation const &op, Connection *conn)
{
	if (!op.target) {
		fprintf(stderr, "flash operations require --target\n");
		return false;
	}
	HvppFlash flash(*op.target, conn);
	std::string text;
	bool ok = perform_flash_operation(op, flash, &text);
	fputs(text.c_str(), stdout);
	return ok;
}

bool hvpp_read_page(Operation const &op, Connection *conn) { return hvpp_flash_operation(op, conn); }
bool hvpp_read_hex(Operation const &op, Connection *conn) { return hvpp_flash_operation(op, conn); }
bool hvpp_write_page(Operation const &op, Connection *conn) { return hvpp_flash_operation(op, conn); }
bool hvpp_write_hex(Operation const &op, Connection *conn) { return hvpp_flash_operation(op, conn); }

bool hvpp_perform(Operation const &op, Connection *conn)
{
	if (!conn->enter_avrhvpp_mode()) {
		fprintf(stderr, "failed to enter avrhvpp mode\n");
		return false;
	}
	
	if (op.command == _CMD_DUMP_BUFFER) {
		write_command(conn, CMD_CLEAR);
		write_encoded_byte(conn, 0, CMD_ADDR0, CMD_ADDR1);
		for (int i = 0; i < 16; i++) {
			write_command(conn, CMD_READ_16BYTES);
			std::vector<uint8_t> data;
			if (!read_response(conn, 16, &data, COMMAND_TIMEOUT_MS)) {
				return false;
			}
			for (int j = 0; j < 16; j++) {
				if (j > 0) {
					putchar(' ');
				}
				printf("%02X", data[j]);
			}
			putchar('\n');
		}
		return true;
	} else if (op.command == _CMD_QUERY) {
		return hvpp_query(conn);
	} else if (op.command == _CMD_READ_HEX) {
		return hvpp_read_hex(op, conn);
	} else if (op.command == _CMD_WRITE_HEX) {
		return hvpp_write_hex(op, conn);
	} else if (op.command == CMD_WRITE_FUSE) {
		return hvpp_write_fuse(op, conn);
	} else if (op.command == CMD_WRITE_LOCK) {
		return hvpp_write_lock(op, conn);
	} else if (op.command == CMD_CHIP_ERASE) {
		return hvpp_chip_erase(conn);
	} else if (op.command == CMD_READ_PAGE) {
		return hvpp_read_page(op, conn);
	} else if (op.command == CMD_WRITE_PAGE) {
		return hvpp_write_page(op, conn);
	} else {
		return hvpp_read_status(op, conn);
	}
}

bool updi_supports(Command command)
{
	switch (command) {
	case CMD_DEBUG:
	case CMD_READ_SIGNATURE:
	case _CMD_QUERY:
	case _CMD_READ_NAMED_FUSE:
	case _CMD_WRITE_NAMED_FUSE:
	case CMD_READ_LOCK:
	case CMD_WRITE_LOCK:
	case CMD_READ_PAGE:
	case CMD_WRITE_PAGE:
	case _CMD_READ_HEX:
	case _CMD_WRITE_HEX:
	case CMD_CHIP_ERASE:
	case _CMD_ERASE_LOCKED:
		return true;
	default:
		return false;
	}
}

bool updi_operation(Operation const &op, AvrUpdi &updi, std::string *text)
{
	auto const &target = op.target->definition();
	char message[128];
	switch (op.command) {
	case CMD_DEBUG:
		return updi.debug(text);
	case CMD_READ_SIGNATURE:
	case _CMD_QUERY: {
		bool query = op.command == _CMD_QUERY;
		bool locked;
		if (query && !updi.read_lock(&locked)) return false;
		if (query && locked) {
			*text += "lock: locked\n";
			fprintf(stderr, "target is locked; signature and memory are unavailable\n");
			return false;
		}
		uint8_t signature[3];
		if (!updi.read_signature(signature)) return false;
		snprintf(message, sizeof(message), "0x%02x, 0x%02x, 0x%02x\n", signature[0], signature[1], signature[2]);
		if (!query) {
			*text += message;
			return true;
		}
		*text += "signature: ";
		*text += message;
		if (!target.matches_signature(signature[0], signature[1], signature[2])) {
			fprintf(stderr, "UPDI signature does not match %s\n", target.display_name);
			return false;
		}
		snprintf(message, sizeof(message), "chip: %s\npage size: %u\npages: %u\nflash size: %lu\nlock: unlocked\n",
			target.display_name, unsigned(target.flash_page_bytes), unsigned(target.flash_page_count),
			static_cast<unsigned long>(target.flash_bytes()));
		*text += message;
		for (uint8_t i = 0; i < target.updi.fuse_count; ++i) {
			auto const &fuse = target.updi.fuses[i];
			uint8_t value;
			if (!updi.read_fuse(fuse, &value)) return false;
			snprintf(message, sizeof(message), "%s: 0x%02x\n", fuse.name, value);
			*text += message;
		}
		return true;
	}
	case CMD_READ_LOCK: {
		bool locked;
		if (!updi.read_lock(&locked)) return false;
		*text += locked ? "lock: locked\n" : "lock: unlocked\n";
		return true;
	}
	case CMD_WRITE_LOCK:
		if (!updi.write_lock(op.updi_lock_value)) return false;
		*text += "lock: locked\n";
		return true;
	case _CMD_READ_NAMED_FUSE:
	case _CMD_WRITE_NAMED_FUSE: {
		auto const &fuse = target.updi.fuses[op.data];
		if (op.command == _CMD_WRITE_NAMED_FUSE) {
			if (!updi.write_fuse(fuse, op.value)) return false;
			snprintf(message, sizeof(message), "%s: fuse value verified; takes effect on reset\n", fuse.name);
		} else {
			uint8_t value;
			if (!updi.read_fuse(fuse, &value)) return false;
			snprintf(message, sizeof(message), "%s: 0x%02x\n", fuse.name, value);
		}
		*text += message;
		return true;
	}
	case CMD_CHIP_ERASE:
		if (!updi.chip_erase()) return false;
		*text += "chip erased\n";
		return true;
	case _CMD_ERASE_LOCKED:
		if (!updi.erase_locked()) return false;
		*text += "locked target erased and unlocked\n";
		return true;
	default:
		return perform_flash_operation(op, updi, text);
	}
}

bool updi_perform(std::vector<Operation> const &operations, Connection *conn)
{
	if (operations.empty() || !operations.front().target) return false;
	AvrUpdi programmer(conn, operations.front().target->definition());
	bool ok = programmer.begin();
	if (ok) {
		for (auto const &op : operations) {
			std::string text;
			ok = updi_operation(op, programmer, &text);
			fputs(text.c_str(), stdout);
			if (!ok) break;
		}
	}
	bool finished = programmer.finish();
	return ok && finished;
}

bool avrisp_flash_operation(Operation const &op, AvrIsp *programmer, std::string *text)
{
	if (!op.target) {
		fprintf(stderr, "flash operations require --target\n");
		return false;
	}
	AvrispFlash flash(programmer, *op.target);
	return perform_flash_operation(op, flash, text);
}

bool avrisp_supports(Command command)
{
	return command == CMD_READ_SIGNATURE || command == _CMD_QUERY || command == CMD_READ_FUSE
		|| command == CMD_READ_LOCK || command == CMD_READ_PAGE || command == _CMD_READ_HEX
		|| command == CMD_WRITE_FUSE || command == CMD_WRITE_LOCK || command == CMD_CHIP_ERASE || command == CMD_WRITE_PAGE || command == _CMD_WRITE_HEX;
}

bool avrisp_fetch_status(AvrIsp *programmer, Command command, uint8_t address, size_t bytes, std::vector<uint8_t> *data)
{
	data->resize(bytes);
	if (command == CMD_READ_SIGNATURE && bytes == 3) return programmer->read_signature(data->data());
	if (command == CMD_READ_FUSE && bytes == 1) return programmer->read_fuse(address, data->data());
	if (command == CMD_READ_LOCK && bytes == 1) return programmer->read_lock(data->data());
	return false;
}

bool avrisp_read_operation(Operation const &op, AvrIsp *programmer, TargetDefinition const **status_target, std::string *text)
{
	if (op.command == _CMD_QUERY) {
		return query_target([&](Command command, uint8_t address, size_t bytes, std::vector<uint8_t> *data) {
			return avrisp_fetch_status(programmer, command, address, bytes, data);
		}, [&](char const *part) { *text += part; }, false);
	}
	if (op.command == CMD_READ_PAGE || op.command == _CMD_READ_HEX) {
		return avrisp_flash_operation(op, programmer, text);
	}
	if (op.command == CMD_READ_FUSE || op.command == CMD_READ_LOCK) {
		if (!*status_target) {
			uint8_t signature[3];
			if (!programmer->read_signature(signature)) return false;
			*status_target = find_target_by_signature(signature[0], signature[1], signature[2]);
		}
		if (!*status_target || (*status_target)->supports(CapabilityUpdi)) {
			fprintf(stderr, "unsupported target signature\n");
			return false;
		}
		TargetCapability capability = CapabilityLockBits;
		if (op.command == CMD_READ_FUSE) {
			if (op.data > 2) return false;
			capability = op.data == 0 ? CapabilityFuseLow : op.data == 1 ? CapabilityFuseHigh : CapabilityFuseExtended;
		}
		if (!(*status_target)->supports(capability)) {
			fprintf(stderr, "requested fuse or lock bits are not supported by %s\n", (*status_target)->display_name);
			return false;
		}
	}
	std::vector<uint8_t> data;
	size_t bytes = op.command == CMD_READ_SIGNATURE ? 3 : 1;
	if (!avrisp_fetch_status(programmer, op.command, (uint8_t)op.data, bytes, &data)) return false;
	for (size_t i = 0; i < data.size(); ++i) {
		char byte[10];
		snprintf(byte, sizeof(byte), "0x%02x", data[i]);
		if (i) *text += ", ";
		*text += byte;
	}
	*text += "\n";
	return true;
}

bool avrisp_write_operation(Operation const &op, AvrIsp *programmer, TargetDefinition const &target, std::string *text)
{
	char message[64];
	if (op.command == CMD_CHIP_ERASE) {
		if (!programmer->chip_erase()) return false;
		*text = "chip erased\n";
	} else if (op.command == CMD_WRITE_FUSE) {
		if (op.data > 2 || !programmer->write_fuse((uint8_t)op.data, op.value)) return false;
		uint8_t value = op.value | (uint8_t)~target.isp.fuse_masks[op.data];
		snprintf(message, sizeof(message), "fuse written: 0x%02x\n", value);
		*text = message;
	} else if (op.command == CMD_WRITE_LOCK) {
		if (!programmer->write_lock(op.value)) return false;
		uint8_t value = op.value | (uint8_t)~target.isp.lock_mask;
		snprintf(message, sizeof(message), "lock bits written: 0x%02x\n", value);
		*text = message;
	} else {
		return avrisp_flash_operation(op, programmer, text);
	}
	return true;
}

bool avrisp_is_write(Command command)
{
	return command == CMD_CHIP_ERASE || command == CMD_WRITE_FUSE || command == CMD_WRITE_LOCK || command == CMD_WRITE_PAGE || command == _CMD_WRITE_HEX;
}

bool avrisp_perform(std::vector<Operation> const &operations, Connection *conn, TargetDefinition const *target)
{
	for (auto const &op : operations) {
		if (!avrisp_supports(op.command)) {
			fprintf(stderr, "operation is not supported with --device avrisp\n");
			return false;
		}
	}
	// Resolve ISP metadata before configuring and entering the write session.
	if (!target) {
		for (auto const &op : operations) {
			if (!avrisp_is_write(op.command)) continue;
			AvrIsp probe(conn);
			uint8_t signature[3];
			bool ok = probe.begin();
			if (ok) ok = probe.read_signature(signature);
			bool finished = probe.finish();
			if (!ok || !finished) return false;
			target = find_target_by_signature(signature[0], signature[1], signature[2]);
			if (!target || target->supports(CapabilityUpdi)) {
				fprintf(stderr, "unsupported target signature\n");
				return false;
			}
			break;
		}
	}
	for (auto const &op : operations) {
		if (target && op.command == CMD_WRITE_FUSE
			&& (op.data > 2 || !target->isp.fuse_masks[op.data])) {
			fprintf(stderr, "selected target does not support the requested fuse\n");
			return false;
		}
	}
	AvrIsp programmer(conn);
	bool ok = programmer.begin(target);
	std::string output, query_error, completed_writes;
	TargetDefinition const *status_target = target;
	if (ok) {
		for (auto const &op : operations) {
			std::string text;
			bool performed = avrisp_is_write(op.command)
				? avrisp_write_operation(op, &programmer, *target, &text)
				: avrisp_read_operation(op, &programmer, &status_target, &text);
			if (!performed) {
				if (avrisp_is_write(op.command)) completed_writes += text;
				if (op.command == _CMD_QUERY) query_error = text;
				ok = false;
				break;
			}
			output += text;
			if (avrisp_is_write(op.command)) completed_writes += text;
		}
	}
	bool finished = programmer.finish();
	if (!ok || !finished) {
		fputs(completed_writes.c_str(), stdout);
		fputs(query_error.c_str(), stdout);
		return false;
	}
	fputs(output.c_str(), stdout);
	return true;
}

bool avrisp_perform(Operation const &op, Connection *conn)
{
	return avrisp_perform({op}, conn, op.target ? &op.target->definition() : nullptr);
}

bool perform(Device device, Operation const &op, Connection *conn)
{
	switch (device) {
	case Device::HVPP: return hvpp_perform(op, conn);
	case Device::UPDI: return updi_perform({op}, conn);
	case Device::AVRISP: return avrisp_perform(op, conn);
	}
	fprintf(stderr, "unsupported device type\n");
	return false;
}

void test(Options const &, Connection *conn)
{
	if (!conn->enter_bitbang_mode()) {
		fprintf(stderr, "failed to enter bitbang mode\n");
		return;
	}

	conn->write_pin(Connection::PIN_RST, true);
	conn->write_pin(Connection::PIN_SCK, false);
	conn->write_pin(Connection::PIN_MOSI, true);
	conn->write_pin(Connection::PIN_SCK, true);
	conn->write_pin(Connection::PIN_SCK, false);
	conn->write_pin(Connection::PIN_MOSI, false);
}

struct OperationOption {
	std::string_view name;
	Command command;
	uint16_t data;
	int argument_count;
};

struct OperationArguments {
	OperationOption const *option;
	std::string_view arguments[2];
};

bool parse_options(int argc, char **argv, Options *opts, std::vector<OperationArguments> *pending)
{
	static constexpr OperationOption operation_options[] = {
		{"--query", _CMD_QUERY, 0, 0},
		{"-q", _CMD_QUERY, 0, 0},
		{"--debug", CMD_DEBUG, 0, 0},
		{"-rs", CMD_READ_SIGNATURE, 0, 0},
		{"--read-signature", CMD_READ_SIGNATURE, 0, 0},
		{"-rfl", CMD_READ_FUSE, 0, 0},
		{"--read-fuse-low", CMD_READ_FUSE, 0, 0},
		{"-rfh", CMD_READ_FUSE, 1, 0},
		{"--read-fuse-high", CMD_READ_FUSE, 1, 0},
		{"-rfe", CMD_READ_FUSE, 2, 0},
		{"--read-fuse-extended", CMD_READ_FUSE, 2, 0},
		{"-rl", CMD_READ_LOCK, 0, 0},
		{"--read-fuse", _CMD_READ_NAMED_FUSE, 0, 1},
		{"--write-fuse", _CMD_WRITE_NAMED_FUSE, 0, 2},
		{"--erase-locked", _CMD_ERASE_LOCKED, 0, 0},
		{"--read-lock", CMD_READ_LOCK, 0, 0},
		{"--read-page", CMD_READ_PAGE, 0, 2},
		{"--write-page", CMD_WRITE_PAGE, 0, 2},
		{"--read-hex", _CMD_READ_HEX, 0, 1},
		{"--write-hex", _CMD_WRITE_HEX, 0, 1},
		{"-wfl", CMD_WRITE_FUSE, 0, 1},
		{"--write-fuse-low", CMD_WRITE_FUSE, 0, 1},
		{"-wfh", CMD_WRITE_FUSE, 1, 1},
		{"--write-fuse-high", CMD_WRITE_FUSE, 1, 1},
		{"-wfe", CMD_WRITE_FUSE, 2, 1},
		{"--write-fuse-extended", CMD_WRITE_FUSE, 2, 1},
		{"-wl", CMD_WRITE_LOCK, 0, 1},
		{"--write-lock", CMD_WRITE_LOCK, 0, 1},
		{"--chip-erase", CMD_CHIP_ERASE, 0, 0},
		{"--dump-buffer", _CMD_DUMP_BUFFER, 0, 0},
	};
	opts->serial_options.speed = 115200;
	std::string_view target_name;
	bool target_specified = false;
	bool device_specified = false;

	int argi = 1;
	while (argi < argc) {
		std::string_view arg = argv[argi++];
		if (arg == "-p" || arg == "--port") {
			if (argi < argc) {
				opts->serial_options.port = argv[argi++];
			} else {
				fprintf(stderr, "-p requires a port name\n");
				return false;
			}
		} else if (arg == "-d" || arg == "--device") {
			if (device_specified) {
				fprintf(stderr, "device specified more than once\n");
				return false;
			}
			if (argi >= argc || argv[argi][0] == '-') {
				fprintf(stderr, "%.*s requires a device name (hvpp, avrisp, updi)\n", (int)arg.size(), arg.data());
				return false;
			}
			if (!parse_device_type(argv[argi++], &opts->device)) return false;
			device_specified = true;
		} else if (arg == "--no-chip-erase") {
			opts->chip_erase_before_write_hex = false;
		} else if (arg == "--help" || arg == "-h") {
			opts->help = true;
		} else if (arg == "-t" || arg == "--target") {
			if (target_specified) {
				fprintf(stderr, "target specified more than once\n");
				return false;
			}
			if (argi >= argc || argv[argi][0] == '-') {
				fprintf(stderr, "%.*s requires a target name (atmega8, atmega48, atmega88, atmega168, atmega328p, attiny2313, attiny13a, attiny202, avr64dd28, avr64dd32, avr64du32)\n", (int)arg.size(), arg.data());
				return false;
			}
			target_name = argv[argi++];
			target_specified = true;
		} else if (arg == "-s" || arg == "--speed") {
			if (argi < argc) {
				opts->serial_options.speed = atoi(argv[argi++]);
			} else {
				fprintf(stderr, "-s requires a speed value\n");
				return false;
			}
		} else {
			OperationOption const *option = nullptr;
			for (OperationOption const &candidate : operation_options) {
				if (candidate.name == arg) {
					option = &candidate;
					break;
				}
			}
			if (!option) {
				fprintf(stderr, "unknown option: %.*s\n", (int)arg.size(), arg.data());
				return false;
			}
			if (argc - argi < option->argument_count) {
				fprintf(stderr, "%.*s requires %d argument(s)\n", (int)arg.size(), arg.data(), option->argument_count);
				return false;
			}
			OperationArguments operation = {option, {}};
			for (int i = 0; i < option->argument_count; i++) {
				operation.arguments[i] = argv[argi++];
			}
			pending->push_back(operation);
		}
	}

	if (target_specified) {
		opts->target = create_target(target_name);
		if (!opts->target) {
			fprintf(stderr, "unknown target: %.*s (available: atmega8, atmega48, atmega88, atmega168, atmega328p, attiny2313, attiny13a, attiny202, avr64dd28, avr64dd32, avr64du32)\n", (int)target_name.size(), target_name.data());
			return false;
		}
	}
	return true;
}

bool build_operations(Options const &opts, std::vector<OperationArguments> const &pending, std::vector<Operation> *operations)
{
	bool updi = opts.device == Device::UPDI;
	bool updi_target = opts.target && opts.target->definition().supports(CapabilityUpdi);
	if (updi && !updi_target) {
		fprintf(stderr, "--device updi requires a supported UPDI --target (attiny202, avr64dd28, avr64dd32, avr64du32)\n");
		return false;
	}
	if (!updi && updi_target) {
		fprintf(stderr, "%s requires --device updi\n", opts.target->definition().display_name);
		return false;
	}
	if (opts.device == Device::HVPP && opts.target && !opts.target->definition().supports(CapabilityHvpp)) {
		fprintf(stderr, "%s requires --device avrisp; HVPP is not supported\n", opts.target->definition().display_name);
		return false;
	}
	for (OperationArguments const &arguments : pending) {
		OperationOption const &option = *arguments.option;
		if (opts.device == Device::AVRISP && !avrisp_supports(option.command)) {
			fprintf(stderr, "%.*s is not supported with --device avrisp\n", (int)option.name.size(), option.name.data());
			return false;
		}
		if (opts.device == Device::AVRISP && (option.command == CMD_READ_FUSE || option.command == CMD_WRITE_FUSE) && option.data == 2
			&& opts.target && !opts.target->definition().supports(CapabilityFuseExtended)) {
			fprintf(stderr, "extended fuse is not supported by the selected target\n");
			return false;
		}
		bool named_fuse = option.command == _CMD_READ_NAMED_FUSE || option.command == _CMD_WRITE_NAMED_FUSE;
		if ((updi && !updi_supports(option.command)) || (!updi && (named_fuse || option.command == _CMD_ERASE_LOCKED))) {
			fprintf(stderr, "%.*s is not supported with the selected device\n", int(option.name.size()), option.name.data());
			return false;
		}
		Operation operation;
		operation.command = option.command;
		operation.data = option.data;
		operation.target = opts.target.get();
		bool flash_operation = option.command == CMD_READ_PAGE || option.command == CMD_WRITE_PAGE || option.command == _CMD_READ_HEX || option.command == _CMD_WRITE_HEX;
		if (flash_operation && !opts.target) {
			fprintf(stderr, "%.*s requires --target\n", (int)option.name.size(), option.name.data());
			return false;
		}
		if (flash_operation) {
			auto const &target = opts.target->definition();
			if (!target.flash_page_count) return false;
			if (opts.device == Device::HVPP) {
				FlashParameters parameters;
				if (!make_flash_parameters(target.flash_page_count - 1, *opts.target, &parameters)) return false;
			}
		}
		if (option.command == CMD_READ_PAGE || option.command == CMD_WRITE_PAGE) {
			if (!parse_page(arguments.arguments[0].data(), *opts.target, &operation.data)) {
				fprintf(stderr, "%.*s requires a page number from 0 to %u\n", (int)option.name.size(), option.name.data(), (unsigned int)(opts.target->definition().flash_page_count - 1));
				return false;
			}
			operation.path = arguments.arguments[1];
			if (option.command == CMD_WRITE_PAGE && !read_page_file(operation.path.c_str(), *opts.target, &operation.bytes)) {
				return false;
			}
		} else if (option.command == _CMD_READ_HEX || option.command == _CMD_WRITE_HEX) {
			operation.path = arguments.arguments[0];
			if (option.command == _CMD_WRITE_HEX) {
				if (!read_hex_file(operation.path.c_str(), *opts.target, &operation.bytes, &operation.page_mask)) {
					return false;
				}
				operation.chip_erase = opts.chip_erase_before_write_hex;
			}
		} else if (named_fuse) {
			auto const &target = opts.target->definition().updi;
			while (operation.data < target.fuse_count && arguments.arguments[0] != target.fuses[operation.data].name) {
				++operation.data;
			}
			if (operation.data == target.fuse_count) {
				fprintf(stderr, "unknown UPDI fuse: %.*s\n", int(arguments.arguments[0].size()), arguments.arguments[0].data());
				return false;
			}
			if (option.command == _CMD_WRITE_NAMED_FUSE && !parse_byte(arguments.arguments[1].data(), &operation.value)) {
				fprintf(stderr, "--write-fuse requires a byte value\n");
				return false;
			}
			auto const &fuse = target.fuses[operation.data];
			if (option.command == _CMD_WRITE_NAMED_FUSE) {
				if (!fuse.write_mask) {
					fprintf(stderr, "UPDI fuse %s is read-only\n", fuse.name);
					return false;
				}
				if (fuse.valid_value && !fuse.valid_value(operation.value)) {
					fprintf(stderr, "invalid value for UPDI fuse %s\n", fuse.name);
					return false;
				}
			}
		} else if (updi && option.command == CMD_WRITE_LOCK) {
			auto const &target = opts.target->definition().updi;
			uint32_t limit = target.lock_bytes == 1 ? 0xff : UINT32_MAX;
			if (!parse_hex_value(arguments.arguments[0].data(), limit, &operation.updi_lock_value)) {
				fprintf(stderr, "UPDI --write-lock requires a %u-bit hex value\n", unsigned(target.lock_bytes) * 8);
				return false;
			}
			if (&arguments != &pending.back()) {
				fprintf(stderr, "UPDI --write-lock must be the last operation\n");
				return false;
			}
			if (operation.updi_lock_value == target.unlocked_value) {
				fprintf(stderr, "UPDI lock cannot be cleared using --write-lock; use --erase-locked to erase and unlock\n");
				return false;
			}
		} else if (option.command == CMD_WRITE_FUSE || option.command == CMD_WRITE_LOCK) {
			if (!parse_byte(arguments.arguments[0].data(), &operation.value)) {
				fprintf(stderr, "%.*s requires a byte value\n", (int)option.name.size(), option.name.data());
				return false;
			}
		}
		operations->push_back(std::move(operation));
	}
	if (operations->empty()) {
		fprintf(stderr, "no command specified\n");
		return false;
	}
	return true;
}

void print_usage()
{
	puts("Usage: avrhvpp --port PORT [--device hvpp|avrisp|updi] [--target CHIP] OPERATIONS\n"
		"Operations: --query, --read-signature, --read-lock, --read-page PAGE FILE,\n"
		"  --write-page PAGE FILE, --read-hex FILE, --write-hex FILE, --chip-erase\n"
		"UPDI targets: attiny202, avr64dd28, avr64dd32, avr64du32.\n"
		"UPDI also supports --debug,\n"
		"  --read-fuse NAME, --write-fuse NAME BYTE, --write-lock HEX and --erase-locked.\n"
		"  --write-lock 0x00 locks the target and must be the last operation.\n"
		"  --write-hex erases by default. Use --no-chip-erase to skip erase.\n"
		"  Normal UPDI chip erase also clears EEPROM unless EESAVE is set.\n"
		"  --erase-locked clears Flash and EEPROM without prior signature verification.\n"
		"  On AVR64DU32, chip erase can also clear the Boot Row; see README.md.\n"
		"See README.md for device-specific operations.");
}

int main(int argc, char **argv)
{
	Options opts;
	std::vector<OperationArguments> pending;
	std::vector<Operation> operations;
	if (!parse_options(argc, argv, &opts, &pending)) return 1;
	if (opts.help) {
		print_usage();
		return 0;
	}
	if (!build_operations(opts, pending, &operations)) return 1;

#ifdef _WIN32
	if (strncmp(opts.serial_options.port.c_str(), "COM", 3) == 0) {
		opts.serial_options.port = "\\\\.\\" + opts.serial_options.port;
	}
#else
#endif

	Connection conn;
	if (!conn.open(&opts.serial_options)) {
		fprintf(stderr, "failed to open %s\n", opts.serial_options.port.c_str());
		return 1;
	}

	if (opts.device == Device::UPDI) {
		bool ok = updi_perform(operations, &conn);
		conn.close();
		return ok ? 0 : 1;
	}
	if (opts.device == Device::AVRISP) {
		bool ok = avrisp_perform(operations, &conn, opts.target ? &opts.target->definition() : nullptr);
		conn.close();
		return ok ? 0 : 1;
	}

	for (Operation const &operation : operations) {
		if (!perform(opts.device, operation, &conn)) {
			conn.close();
			return 1;
		}
	}

	conn.close();
	return 0;
}
