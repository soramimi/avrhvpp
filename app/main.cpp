
#include "serial.h"
#include "target.h"
#include <chrono>
#include <cstdio>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>
#include <cstring>
#include <assert.h>
#include <cerrno>
#include <cctype>
#include <cstdlib>
#include "../common/commands.h"

#ifdef _WIN32
#else
#include <assert.h>
#include <unistd.h>
#endif

constexpr int BITBANG_MODE_TIMEOUT_MS = 10;
constexpr int PIN_IO_TIMEOUT_MS = 10;
constexpr int MODE_ENTER_TIMEOUT_MS = 100;
constexpr int COMMAND_TIMEOUT_MS = 1000;
constexpr int PAGE_WRITE_TIMEOUT_MS = 2000;

void msleep(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

static inline bool operator == (std::vector<char> const &a, std::string_view b)
{
	if (a.size() != b.size()) return false;
	return memcmp(a.data(), b.data(), a.size()) == 0;
}


class Connection {
public:
	static constexpr int PIN_RST  = 0;
	static constexpr int PIN_SCK  = 1;
	static constexpr int PIN_MOSI = 2;
	static constexpr int PIN_MISO = 3;

	static constexpr int CMD_READ        = 0x80;
	static constexpr int CMD_READ_PULLUP = 0x90;
	static constexpr int CMD_WRITE_LOW   = 0xa0;
	static constexpr int CMD_WRITE_HIGH  = 0xb0;

private:
	Serial serial_;
public:
	int write(void const *ptr, int len)
	{
		return serial_.write(ptr, len);
	}

	int read(void *ptr, int len, int timeout)
	{
		return serial_.read(ptr, len, timeout);
	}

	// read until '\n' or timeout. trailing CR/LF is removed.
	bool read_line(std::vector<char> *out, int timeout)
	{
		out->clear();
		auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout);
		while (1) {
			auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count();
			if (remain <= 0) return false;
			char c;
			int n = serial_.read(&c, 1, (int)remain);
			if (n < 0) return false;
			if (n == 0) continue;
			if (c == '\n') {
				while (!out->empty() && out->back() == '\r') {
					out->pop_back();
				}
				return true;
			}
			out->push_back(c);
		}
	}
	bool sync()
	{
		int n;
		char tmp[100];
		for (int i = 0; i < 50; i++) {
			tmp[0] = '0';
			serial_.write(tmp, 1);
			n = serial_.read(tmp, sizeof(tmp), BITBANG_MODE_TIMEOUT_MS);
			if (n == 0) {
				tmp[0] = ' ';
				serial_.write(tmp, 1);
				n = serial_.read(tmp, sizeof(tmp), BITBANG_MODE_TIMEOUT_MS);
				if (n == 2 && tmp[0] == 0x14 && tmp[1] == 0x10) return true; // OK
			}
			msleep(10);
		}
		return false;
	}
public:
	Connection()
	{
	}

	~Connection()
	{
		close();
	}

	bool open(Serial::Option *option)
	{
		return serial_.open(option);
	}

	void close()
	{
		serial_.close();
	}

	enum Mode {
		STK500,
		BITBANG,
		AVRHVPP,
		AVRISP,
	};
	Mode mode_ = STK500;
	Mode mode() const
	{
		return mode_;
	}

	bool enter_bitbang_mode()
	{
		if (mode() == BITBANG) return true;

		static constexpr std::string_view command = ".BITBANG.";
		static constexpr std::string_view expect = "BITBANG";

		if (sync()) {
			serial_.write(command.data(), (int)command.size());

			std::vector<char> line;
			if (read_line(&line, MODE_ENTER_TIMEOUT_MS) && line == expect) {
				mode_ = BITBANG;
				return true;
			}
		}

		return false;
	}

	bool enter_avrhvpp_mode()
	{
		if (mode() == AVRHVPP) return true;

		static constexpr std::string_view command = ".AVRHVPP.";
		static constexpr std::string_view expect = "AVRHVPP";

		if (sync()) {
			serial_.write(command.data(), (int)command.size());

			std::vector<char> line;
			if (read_line(&line, MODE_ENTER_TIMEOUT_MS) && line == expect) {
				mode_ = AVRHVPP;
				return true;
			}
		}

		return false;
	}

	int read_pin(int pin, bool pullup)
	{
		char c = char(pullup ? CMD_READ_PULLUP : CMD_READ) | (pin & 0x0f);
		char d = 0;
		serial_.write(&c, 1);
		int n = serial_.read(&d, 1, PIN_IO_TIMEOUT_MS);
		return n == 1 ? (unsigned char)d : -1;
	}

	bool write_pin(int pin, bool v)
	{
		char c = char(v ? CMD_WRITE_HIGH : CMD_WRITE_LOW) | (pin & 0x0f);
		char d;
		serial_.write(&c, 1);
		int n = serial_.read(&d, 1, PIN_IO_TIMEOUT_MS);
		return n == 1;
	}
};

struct Options {
	Connection::Mode device = Connection::Mode::AVRHVPP;
	Serial::Option serial_options;
	std::unique_ptr<Target> target;
	bool chip_erase_before_write_hex = true;
};

struct Operation {
	Command command;
	uint16_t data = 0;
	uint8_t value = 0;
	std::string path;
	std::vector<uint8_t> bytes;
	std::vector<uint8_t> page_mask;
	bool chip_erase = false;
	Target const *target = nullptr;
};

Connection::Mode parse_device_type(std::string_view name)
{
	if (name == "hvpp") {
		return Connection::Mode::AVRHVPP;
	} else if (name == "avrisp") {
		return Connection::Mode::AVRISP;
	} else {
		fprintf(stderr, "unknown device type: %.*s\n", (int)name.size(), name.data());
		return Connection::Mode::AVRHVPP;
	}
}

bool parse_byte(char const *text, uint8_t *value)
{
	errno = 0;
	char *end;
	unsigned long v = strtoul(text, &end, 16);
	if (errno || end == text || *end || v > 0xff) {
		return false;
	}
	*value = (uint8_t)v;
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

bool hvpp_query(Connection *conn)
{
	std::vector<uint8_t> signature;
	if (!avr_fetch_status(CMD_READ_SIGNATURE, 0, 3, &signature, conn)) return false;
	printf("signature: 0x%02x, 0x%02x, 0x%02x\n", signature[0], signature[1], signature[2]);
	TargetDefinition const *target = find_target_by_signature(signature[0], signature[1], signature[2]);
	if (!target) {
		puts("chip: unknown");
		fprintf(stderr, "unsupported target signature\n");
		return false;
	}
	printf("chip: %s\n", target->display_name);
	printf("page size: %u bytes\n", (unsigned int)target->flash_page_bytes);
	printf("page count: %u\n", (unsigned int)target->flash_page_count);
	printf("flash size: %lu bytes\n", (unsigned long)target->flash_bytes());
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
		if (!avr_fetch_status(field.command, field.address, 1, &data, conn)) return false;
		printf("%s: 0x%02x\n", field.label, data[0]);
	}
	return true;
}

bool hvpp_read_status(Operation const &op, Connection *conn)
{
	int bytes = 0;
	switch (op.command) {
	case CMD_DEBUG:
		bytes = 7;
		break;
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

bool hvpp_chip_erase(Connection *conn)
{
	write_command(conn, CMD_CHIP_ERASE);

	if (read_ack(conn, PAGE_WRITE_TIMEOUT_MS)) {
		puts("chip erased");
		return true;
	}
	fprintf(stderr, "failed to erase chip\n");
	return false;
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

bool hvpp_read_page(Operation const &op, Connection *conn)
{
	if (!op.target) {
		fprintf(stderr, "flash operations require --target\n");
		return false;
	}
	Target const &target = *op.target;
	std::vector<uint8_t> data;
	if (!avr_fetch_page(op.data, target, &data, conn)) {
		return false;
	}

	FILE *fp = fopen(op.path.c_str(), "wb");
	if (!fp) {
		fprintf(stderr, "failed to open %s\n", op.path.c_str());
		return false;
	}
	bool ok = fwrite(data.data(), 1, data.size(), fp) == data.size();
	if (fclose(fp) != 0) {
		ok = false;
	}
	if (!ok) {
		fprintf(stderr, "failed to write %s\n", op.path.c_str());
		return false;
	}
	printf("page %u read: %s\n", (unsigned int)op.data, op.path.c_str());
	return true;
}

bool hvpp_read_hex(Operation const &op, Connection *conn)
{
	if (!op.target) {
		fprintf(stderr, "flash operations require --target\n");
		return false;
	}
	Target const &target = *op.target;
	std::vector<uint8_t> image(target.definition().flash_bytes());
	for (uint16_t page = 0; page < target.definition().flash_page_count; page++) {
		std::vector<uint8_t> data;
		if (!avr_fetch_page(page, target, &data, conn)) {
			return false;
		}
		memcpy(&image[(size_t)page * target.definition().flash_page_bytes], data.data(), target.definition().flash_page_bytes);
	}

	unsigned int records;
	if (!write_hex_file(op.path.c_str(), image, &records)) {
		return false;
	}
	printf("hex read: %s (%u data records)\n", op.path.c_str(), records);
	return true;
}

bool hvpp_write_page(Operation const &op, Connection *conn)
{
	if (!op.target) {
		fprintf(stderr, "flash operations require --target\n");
		return false;
	}
	Target const &target = *op.target;
	if (op.bytes.size() != target.definition().flash_page_bytes) {
		fprintf(stderr, "invalid flash page data length\n");
		return false;
	}
	FlashParameters parameters;
	if (!make_flash_parameters(op.data, target, &parameters)
		|| !write_flash_parameters(parameters.word_address, parameters.byte_count, conn)) {
		return false;
	}
	for (uint8_t data : op.bytes) {
		write_encoded_byte(conn, data, CMD_DATA0, CMD_DATA1);
	}
	write_command(conn, CMD_WRITE_PAGE);

	if (read_ack(conn, PAGE_WRITE_TIMEOUT_MS)) {
		printf("page %u written: %s\n", (unsigned int)op.data, op.path.c_str());
		return true;
	}
	fprintf(stderr, "failed to write flash page %u; the page must be blank or already match the input\n", (unsigned int)op.data);
	return false;
}

bool hvpp_write_hex(Operation const &op, Connection *conn)
{
	if (!op.target) {
		fprintf(stderr, "flash operations require --target\n");
		return false;
	}
	Target const &target = *op.target;
	if (op.bytes.size() != target.definition().flash_bytes()
		|| op.page_mask.size() != target.definition().flash_page_count) {
		fprintf(stderr, "invalid flash image or page mask\n");
		return false;
	}
	if (op.chip_erase && !hvpp_chip_erase(conn)) {
		return false;
	}

	std::vector<uint8_t> pages_to_write(target.definition().flash_page_count, 0);
	unsigned int pages_present = 0;
	unsigned int pages_matching = 0;
	for (uint16_t page = 0; page < target.definition().flash_page_count; page++) {
		size_t offset = (size_t)page * target.definition().flash_page_bytes;
		if (!op.page_mask[page]) {
			continue;
		}
		pages_present++;

		std::vector<uint8_t> current;
		if (!avr_fetch_page(page, target, &current, conn)) {
			return false;
		}
		bool matches = true;
		bool blank = true;
		for (uint16_t i = 0; i < target.definition().flash_page_bytes; i++) {
			if (current[i] != op.bytes[offset + i]) {
				matches = false;
			}
			if (current[i] != 0xff) {
				blank = false;
			}
		}
		if (matches) {
			pages_matching++;
		} else if (blank) {
			pages_to_write[page] = 1;
		} else {
			fprintf(stderr, "flash page %u is not blank and does not match %s; no pages were written\n", (unsigned int)page, op.path.c_str());
			return false;
		}
	}

	unsigned int pages_written = 0;
	for (uint16_t page = 0; page < target.definition().flash_page_count; page++) {
		if (!pages_to_write[page]) {
			continue;
		}
		size_t offset = (size_t)page * target.definition().flash_page_bytes;
		Operation page_operation;
		page_operation.command = CMD_WRITE_PAGE;
		page_operation.target = op.target;
		page_operation.data = page;
		page_operation.path = op.path;
		page_operation.bytes.assign(op.bytes.begin() + offset, op.bytes.begin() + offset + target.definition().flash_page_bytes);
		if (!hvpp_write_page(page_operation, conn)) {
			return false;
		}
		pages_written++;
	}

	printf("hex written: %s (%u data pages, %u written, %u already matched%s)\n",
		op.path.c_str(), pages_present, pages_written, pages_matching,
		op.chip_erase ? ", chip erased" : "");
	return true;
}

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

bool avrisp_read_signature(Connection *conn)
{
	char tmp[10];
	int n;
	conn->write("P\x20", 2);
	n = conn->read(tmp, sizeof(tmp), 100);
	if (n == 2 && tmp[0] == 0x14 && tmp[1] == 0x10) { // STK_INSYNC && STK_OK
		// pass
	} else {
		fprintf(stderr, "failed to enter programming mode\n");
		return false;
	}
	conn->write("\x75\x20", 2);
	n = conn->read(tmp, sizeof(tmp), 100);
	if (n == 5) {
		if (tmp[0] == 0x14) { // STK_INSYNC
			if (tmp[4] == 0x10) { // STK_OK
				std::string text;
				char const *ptr = tmp + 1;
				size_t len = 3;
				for (size_t i = 0; i < len; i++) {
					if (i > 0) {
						text += ", ";
					}
					char tmp2[10];
					sprintf(tmp2, "0x%02x", (unsigned char)ptr[i]);
					text += tmp2;
				}
				puts(text.c_str());
			}
		}
	}
	
	conn->write("Q\x20", 2);
	n = conn->read(tmp, sizeof(tmp), 100);
	if (n == 2 && tmp[0] == 0x14 && tmp[1] == 0x10) { // STK_INSYNC && STK_OK
		// pass
	} else {
		fprintf(stderr, "failed to enter programming mode\n");
		return false;
	}
	
	return true;
}

bool avrisp_perform(Operation const &op, Connection *conn)
{
	if (!conn->sync()) {
		fprintf(stderr, "failed to sync with AVRISP\n");
		return false;
	}
	if (op.command == CMD_READ_SIGNATURE) {
		return avrisp_read_signature(conn);
	}
	
	// not implemented yet
	
	
	
	
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
			if (argi < argc) {
				std::string_view name = argv[argi++];
				opts->device = parse_device_type(name);
			} else {
				fprintf(stderr, "-d requires a device name\n");
				return false;
			}
		} else if (arg == "--no-chip-erase") {
			opts->chip_erase_before_write_hex = false;
		} else if (arg == "-t" || arg == "--target") {
			if (target_specified) {
				fprintf(stderr, "target specified more than once\n");
				return false;
			}
			if (argi >= argc || argv[argi][0] == '-') {
				fprintf(stderr, "%.*s requires a target name (atmega8, atmega48, atmega88, atmega168, atmega328p)\n", (int)arg.size(), arg.data());
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
			fprintf(stderr, "unknown target: %.*s (available: atmega8, atmega48, atmega88, atmega168, atmega328p)\n", (int)target_name.size(), target_name.data());
			return false;
		}
	}
	return true;
}

bool build_operations(Options const &opts, std::vector<OperationArguments> const &pending, std::vector<Operation> *operations)
{
	for (OperationArguments const &arguments : pending) {
		OperationOption const &option = *arguments.option;
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
			FlashParameters parameters;
			if (!opts.target->definition().flash_page_count
				|| !make_flash_parameters(opts.target->definition().flash_page_count - 1, *opts.target, &parameters)) {
				return false;
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

int main(int argc, char **argv)
{
	Options opts;
	std::vector<OperationArguments> pending;
	std::vector<Operation> operations;
	if (!parse_options(argc, argv, &opts, &pending) || !build_operations(opts, pending, &operations)) {
		return 1;
	}

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

	for (Operation const &operation : operations) {
		if (opts.device == Connection::Mode::AVRHVPP) {
			if (!hvpp_perform(operation, &conn)) {
				conn.close();
				return 1;
			}
		} else if (opts.device == Connection::Mode::AVRISP) {
			if (!avrisp_perform(operation, &conn)) {
				conn.close();
				return 1;
			}
		} else {
			fprintf(stderr, "unsupported device type\n");
			conn.close();
			return 1;
		}
	}

	conn.close();
	return 0;
}
