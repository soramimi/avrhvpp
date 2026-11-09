// Exercise real host encoding/decoding and firmware commands without serial hardware.
#define main avrhvpp_main
#include "../app/main.cpp"
#undef main
#include "../firmware-atmega32u2/avrhvpp.h"
#include <algorithm>
#include <array>

uint8_t DDRB, DDRC, DDRD, PORTB, PORTC, PORTD, PINC;
uint8_t flush_buffer[FlashBufferBytes];
static std::array<uint8_t, 131072> flash;
static std::deque<char> response;
static std::vector<uint8_t> sent;
static std::vector<std::pair<uint8_t, uint16_t>> loaded_words;
static AVRHVPP *device;
static uint8_t command, low_address, high_address, low_data, high_data;
static uint8_t manufacturer = 0x1e, lock_value;
static uint8_t signature1, signature2;
static std::array<uint8_t, 4> fuse_values;
static std::vector<uint8_t> fuse_reads;
static bool fail_fuse_read, invalid_signature_response;
static bool ready = true, timeout_on_write, corrupt_write, reject_latch;
static unsigned int write_pulses;

uint8_t fake_pinb() { return ready ? 1 << 3 : 0; }

uint8_t fake_pind()
{
	assert(PORTC & (1 << 2));
	if (command == 0x08) return low_address == 0 ? manufacturer : low_address == 1 ? signature1 : signature2;
	if (command == 0x04) {
		bool bs1 = PORTC & (1 << 6), bs2 = PORTC & (1 << 7);
		uint8_t index = bs2 ? (bs1 ? 1 : 2) : (bs1 ? 3 : 0);
		fuse_reads.push_back(index);
		return fuse_values[index];
	}
	if (command == 0x02) {
		uint32_t word = ((uint16_t)high_address << 8) | low_address;
		return flash[word * 2 + ((PORTC & (1 << 6)) ? 1 : 0)];
	}
	return 0xff;
}

void waitloop(unsigned int)
{
	static uint8_t previous_b, previous_c;
	if (PORTC & (1 << 2)) {
		if ((PORTB & (1 << 7)) && !(previous_b & (1 << 7))) {
			if (PORTB & (1 << 5)) {
				command = PORTD;
				if (command == 0x10) loaded_words.clear();
			} else if (PORTB & (1 << 6)) {
				if (PORTC & (1 << 6)) high_data = PORTD;
				else low_data = PORTD;
			} else {
				if (PORTC & (1 << 6)) high_address = PORTD;
				else low_address = PORTD;
			}
		}
		if ((PORTB & (1 << 4)) && !(previous_b & (1 << 4))) {
			loaded_words.emplace_back(low_address, (uint16_t)low_data | ((uint16_t)high_data << 8));
		}
		if (!(PORTC & (1 << 5)) && (previous_c & (1 << 5))) {
			++write_pulses;
			if (command == 0x10) {
				assert(!(PORTC & (1 << 6))); // BS1 must be low before Flash WR.
				for (auto word : loaded_words) {
					uint32_t offset = (((uint16_t)high_address << 8) | word.first) * 2UL;
					flash[offset] = word.second;
					flash[offset + 1] = word.second >> 8;
				}
				if (corrupt_write && !loaded_words.empty()) flash[((uint32_t)high_address << 9) | (loaded_words[0].first * 2)] ^= 1;
			} else if (command == 0x20) {
				lock_value = low_data;
			} else if (command == 0x80) {
				flash.fill(0xff);
			}
			if (timeout_on_write) ready = false;
		}
	}
	previous_b = PORTB;
	previous_c = PORTC;
}

void msleep(unsigned int) { waitloop(0); }
void AVRHVPP::write_byte(char c)
{
	assert(!(PORTC & (1 << 2)));
	assert(!(DDRB & 0xf8) && !(DDRC & 0xf0) && DDRD == 0);
	response.push_back(c);
}
void AVRHVPP::write_string(char const *text) { while (*text) write_byte(*text++); }

namespace isp_test {
enum class Fault { None, Header, Status, Truncated, Read };
static bool enabled, programming, read_error, zero_write;
static int read_chunk, write_chunk, write_calls, fail_write_call, faults_left, fault_skip, open_calls;
static uint8_t fault_command;
static std::array<uint8_t, 3> signature;
static std::array<uint8_t, 4> status;
static Fault fault;
static std::vector<uint8_t> packet, commands;
static std::deque<uint8_t> replies;

static std::vector<std::vector<uint8_t>> packets;
static std::array<uint8_t, 32768> memory;
static std::array<uint8_t, 20> parameters;
static uint16_t address;
static unsigned int close_calls;

enum class Pending { None, Fuse, Lock, Erase, Flash };
static Pending pending;
static bool write_model, forever_busy, ignore_write, corrupt_flash_write, corrupt_erase, retain_lock, reserved_zero;
static int busy_polls;
static bool universal_started;
static uint8_t universal_result;
static uint8_t pending_fuse, pending_value;
static size_t pending_offset;
static std::vector<uint8_t> pending_data;
static std::chrono::steady_clock::time_point ready_at;

static bool is_atmega8() { return signature[1] == 0x93 && signature[2] == 0x07; }

static void apply_pending()
{
	if (pending == Pending::Fuse) {
		if (!ignore_write && (status[3] & 1)) status[pending_fuse] = pending_value;
	} else if (pending == Pending::Lock) {
		if (!ignore_write) status[3] &= pending_value;
	} else if (pending == Pending::Erase) {
		if (!ignore_write) {
			memory.fill(0xff);
			if (!retain_lock) status[3] = 0xff;
			if (corrupt_erase) memory[32767] = 0;
		}
	} else if (pending == Pending::Flash) {
		if (!ignore_write) {
			for (size_t i = 0; i < pending_data.size(); ++i) memory[pending_offset + i] &= pending_data[i];
			if (corrupt_flash_write) memory[pending_offset] ^= 1;
		}
	}
	pending = Pending::None;
}

static void update_pending()
{
	if (pending == Pending::None || forever_busy) return;
	if (is_atmega8() ? std::chrono::steady_clock::now() >= ready_at : busy_polls == 0) apply_pending();
}

static void start_pending(Pending operation)
{
	assert(pending == Pending::None);
	pending = operation;
	busy_polls = 2;
	ready_at = std::chrono::steady_clock::now() + std::chrono::milliseconds(operation == Pending::Erase ? 9 : 5);
}

static uint8_t read_status(uint8_t index)
{
	update_pending();
	if (write_model) assert(pending == Pending::None);
	uint8_t mask = index == 2 ? (signature[1] == 0x92 ? 1 : 7) : index == 3 ? (signature[1] == 0x92 ? 3 : 0x3f) : 0xff;
	return reserved_zero ? status[index] & mask : status[index];
}

static uint8_t universal(std::vector<uint8_t> const &packet)
{
	uint8_t first = packet[1], second = packet[2];
	if (first == 0xf0 && write_model) {
		assert(!is_atmega8());
		update_pending();
		if (pending == Pending::None) return 0xa0; // Only bit zero is busy.
		if (!forever_busy && busy_polls) --busy_polls;
		return 0xa1;
	}
	if (first == 0x30) return signature.at(packet[3]);
	if (first == 0x50) return read_status(second == 8 ? 2 : 0);
	if (first == 0x58) return read_status(second == 8 ? 1 : 3);
	if (first == 0xac && write_model) {
		assert(packet[3] == 0);
		if (second == 0x80) start_pending(Pending::Erase);
		else if (second == 0xe0) {
			uint8_t mask = signature[1] == 0x92 ? 3 : 0x3f;
			assert((packet[4] | mask) == 0xff);
			pending_value = packet[4];
			start_pending(Pending::Lock);
		} else {
			assert(second == 0xa0 || second == 0xa8 || second == 0xa4);
			pending_fuse = second == 0xa0 ? 0 : second == 0xa8 ? 1 : 2;
			assert(pending_fuse != 2 || !is_atmega8());
			uint8_t mask = pending_fuse == 2 ? (signature[1] == 0x92 ? 1 : 7) : 0xff;
			assert((packet[4] | mask) == 0xff);
			assert(pending_fuse != 1 || !((packet[4] ^ status[1]) & 0x20));
			pending_value = packet[4];
			start_pending(Pending::Fuse);
		}
		return 0;
	}
	return 0x20; // Deliberately binary transport test.
}



static void reset()
{
	enabled = true;
	programming = read_error = zero_write = false;
	read_chunk = 1;
	write_chunk = 1;
	write_calls = fail_write_call = faults_left = fault_skip = open_calls = 0;
	signature = {{0x1e, 0x95, 0x0f}};
	status = {{0x62, 0xd9, 0xfd, 0xfc}};
	fault_command = 0;
	fault = Fault::None;
	packet.clear(); commands.clear(); replies.clear();
	packets.clear(); memory.fill(0xff); parameters.fill(0); address = 0; close_calls = 0;
	pending = Pending::None;
	write_model = forever_busy = ignore_write = corrupt_flash_write = corrupt_erase = retain_lock = reserved_zero = false;
	busy_polls = 0; pending_data.clear(); universal_started = false;
}


static size_t packet_size()
{
	switch (packet[0]) {
	case 0x42: return 22;
	case 0x45: return 7;
	case 0x55: return 4;
	case 0x56: return 6;
	case 0x74: return 5;
	case 0x64: return packet.size() < 4 ? 0 : 5 + ((size_t)packet[1] << 8) + packet[2];
	default: return 2;
	}
}

static int write(uint8_t const *bytes, int size)
{
	if (++write_calls == fail_write_call) return zero_write ? 0 : -1;
	int count = std::min(size, write_chunk);
	for (int i = 0; i < count; i++) {
		packet.push_back(bytes[i]);
		// The real firmware transfers Universal's four SPI bytes before reading EOP.
		if (write_model && packet[0] == 0x56 && packet.size() == 5) {
			universal_result = universal(packet);
			universal_started = true;
		}
		size_t expected = packet_size();
		if (!expected || packet.size() < expected) continue;
		assert(packet.size() == expected && packet.back() == 0x20);
		uint8_t command = packet[0];
		commands.push_back(command);
		packets.push_back(packet);
		if (command == 0x50) programming = true;
		if (command == 0x51) programming = false;
		std::vector<uint8_t> reply = {0x14};
		if (command == 0x42) {
			assert(!programming);
			std::copy(packet.begin() + 1, packet.begin() + 21, parameters.begin());
		} else if (command == 0x45) {
			assert(!programming && packet[1] == 5);
		} else if (command == 0x75) {
			assert(programming);
			reply.insert(reply.end(), signature.begin(), signature.end());
		} else if (command == 0x56) {
			assert(programming);
			reply.push_back(universal_started ? universal_result : universal(packet));
		} else if (command == 0x55) {
			assert(programming);
			address = packet[1] | ((uint16_t)packet[2] << 8);
		} else if (command == 0x74 || command == 0x64) {
			assert(programming && packet[3] == 'F');
			unsigned int length = ((unsigned int)packet[1] << 8) | packet[2];
			assert(length && length <= 256 && length % 2 == 0);
			assert(address * 2U + length <= memory.size());
			if (command == 0x74) {
				update_pending();
				if (write_model) assert(pending == Pending::None);
				reply.insert(reply.end(), memory.begin() + address * 2U, memory.begin() + address * 2U + length);
			} else {
				assert(length == ((unsigned int)parameters[12] << 8 | parameters[13]));
				if (write_model) {
					pending_offset = address * 2U;
					pending_data.assign(packet.begin() + 4, packet.end() - 1);
					start_pending(Pending::Flash);
				} else std::copy(packet.begin() + 4, packet.end() - 1, memory.begin() + address * 2U);
			}
			address += length / 2;
		}
		packet.clear();
		universal_started = false;
		reply.push_back(0x10);
		if (command == fault_command && faults_left > 0) {
			if (fault_skip > 0) {
				--fault_skip;
			} else {
				--faults_left;
				switch (fault) {
				case Fault::Header: reply[0] = 0x15; break;
				case Fault::Status: reply.back() = 0x11; break;
				case Fault::Truncated: reply.resize(reply.size() / 2); break;
				case Fault::Read: read_error = true; break;
				case Fault::None: break;
				}
			}
		}
		replies.insert(replies.end(), reply.begin(), reply.end());
	}
	return count;
}

static int read(uint8_t *bytes, int size)
{
	if (read_error) {
		read_error = false;
		return -1;
	}
	int count = std::min({size, read_chunk, (int)replies.size()});
	for (int i = 0; i < count; i++) {
		bytes[i] = replies.front();
		replies.pop_front();
	}
	return count;
}
}

Serial::Serial() {}
bool Serial::open(Option *) { ++isp_test::open_calls; assert(isp_test::enabled); return true; }
void Serial::cancel() {}
void Serial::close() { if (isp_test::enabled) ++isp_test::close_calls; }
int Serial::write(void const *pointer, int length)
{
	auto bytes = static_cast<uint8_t const *>(pointer);
	if (isp_test::enabled) return isp_test::write(bytes, length);
	for (int i = 0; i < length; ++i) {
		assert(bytes[i] >= 0x40);
		sent.push_back(bytes[i]);
		if (fail_fuse_read && bytes[i] == CMD_READ_FUSE) ready = false;
		device->process(bytes[i]);
		if (invalid_signature_response && bytes[i] == CMD_READ_SIGNATURE) {
			response.clear();
			for (char c : std::string("1E95GG\r\n")) response.push_back(c);
		}
		if (reject_latch && bytes[i] == CMD_LATCH_FLASH_PARAMS) {
			response.clear();
			for (char c : std::string("E01\r\n")) response.push_back(c);
		}
	}
	return length;
}
int Serial::read(void *pointer, int length, int)
{
	if (isp_test::enabled) return isp_test::read(static_cast<uint8_t *>(pointer), length);
	if (!length || response.empty()) return 0;
	*static_cast<char *>(pointer) = response.front();
	response.pop_front();
	return 1; // Responses arrive one byte at a time.
}

static void reset()
{
	ready = true;
	manufacturer = 0x1e;
	signature1 = signature2 = 0;
	fuse_values.fill(0xff);
	fuse_reads.clear();
	fail_fuse_read = invalid_signature_response = false;
	timeout_on_write = corrupt_write = reject_latch = false;
	write_pulses = 0;
	device->reset_protocol();
	response.clear();
	sent.clear();
	flash.fill(0xff);
}

static void expect(std::string const &text)
{
	assert(std::string(response.begin(), response.end()) == text);
	assert(!(PORTC & (1 << 2)));
	response.clear();
}

static void load(Connection *connection, std::vector<uint8_t> const &bytes)
{
	for (auto byte : bytes) write_encoded_byte(connection, byte, CMD_DATA0, CMD_DATA1);
}

static bool parse_arguments(std::vector<std::string> arguments, Options *options, std::vector<Operation> *operations)
{
	std::vector<char *> argv;
	for (auto &argument : arguments) argv.push_back(argument.data());
	std::vector<OperationArguments> pending;
	return parse_options((int)argv.size(), argv.data(), options, &pending)
		&& build_operations(*options, pending, operations);
}

template <typename Callable>
static std::string capture_output(Callable call, bool *ok)
{
	assert(fflush(stdout) == 0);
	int saved_stdout = dup(STDOUT_FILENO);
	FILE *output = tmpfile();
	assert(saved_stdout >= 0 && output);
	assert(dup2(fileno(output), STDOUT_FILENO) >= 0);
	*ok = call();
	assert(fflush(stdout) == 0);
	assert(dup2(saved_stdout, STDOUT_FILENO) >= 0);
	close(saved_stdout);
	long length = ftell(output);
	assert(length >= 0 && fseek(output, 0, SEEK_SET) == 0);
	std::string text((size_t)length, '\0');
	assert(fread(text.data(), 1, text.size(), output) == text.size());
	fclose(output);
	return text;
}

static std::string capture_operation(Operation const &operation, Connection *connection, bool *ok, Device device = Device::HVPP)
{
	return capture_output([&]() { return perform(device, operation, connection); }, ok);
}

static bool read_signature_session(Avrisp *programmer, uint8_t *signature)
{
	bool ok = programmer->begin();
	if (ok) ok = programmer->read_signature(signature);
	bool finished = programmer->finish();
	return ok && finished;
}

static void test_avrisp(Connection *connection)
{
	for (int read_chunk : {1, 2, 8}) {
		for (int write_chunk : {1, 2}) {
			isp_test::reset();
			isp_test::read_chunk = read_chunk;
			isp_test::write_chunk = write_chunk;
			Operation operation;
			operation.command = CMD_READ_SIGNATURE;
			bool ok;
			assert(capture_operation(operation, connection, &ok, Device::AVRISP) == "0x1e, 0x95, 0x0f\n" && ok);
			assert(isp_test::commands == (std::vector<uint8_t>{0x30, 0x50, 0x75, 0x51}));
			assert(!isp_test::programming);
			assert(connection->mode() == Connection::STK500);
		}
	}
	for (uint8_t command : {0x30, 0x50, 0x75, 0x51}) {
		for (auto fault : {isp_test::Fault::Header, isp_test::Fault::Status, isp_test::Fault::Truncated, isp_test::Fault::Read}) {
			isp_test::reset();
			isp_test::fault_command = command;
			isp_test::fault = fault;
			isp_test::faults_left = command == 0x30 ? 3 : 1;
			Avrisp programmer(connection, 3);
			uint8_t signature[3];
			assert(!read_signature_session(&programmer, signature));
			assert(!isp_test::programming);
			if (command == 0x30) {
				assert(isp_test::commands == (std::vector<uint8_t>{0x30, 0x30, 0x30}));
			} else {
				assert(isp_test::commands.back() == 0x51);
			}
			if (command == 0x50 || command == 0x75) assert(isp_test::replies.empty());
		}
	}
	isp_test::reset();
	isp_test::fault_command = 0x30;
	isp_test::fault = isp_test::Fault::Header;
	isp_test::faults_left = 1;
	Avrisp retrying(connection, 3);
	uint8_t signature[3];
	assert(read_signature_session(&retrying, signature));
	assert(isp_test::commands == (std::vector<uint8_t>{0x30, 0x30, 0x50, 0x75, 0x51}));
	for (int failed_call : {1, 2}) {
		isp_test::reset();
		isp_test::fail_write_call = failed_call;
		Avrisp programmer(connection, 3);
		assert(read_signature_session(&programmer, signature));
		assert(!isp_test::programming);
	}
	for (int failed_call : {4, 6, 8}) {
		for (bool zero : {false, true}) {
			isp_test::reset();
			isp_test::fail_write_call = failed_call;
			isp_test::zero_write = zero;
			Avrisp programmer(connection, 3);
			assert(!read_signature_session(&programmer, signature));
			assert(!isp_test::programming); // A partial Q is completed, then retried.
		}
	}
	for (auto const &arguments : std::vector<std::vector<std::string>>{
		{"avrhvpp", "-d", "typo", "--chip-erase"},
		{"avrhvpp", "--device"}, {"avrhvpp", "-d", "--read-signature"},
		{"avrhvpp", "-d", "avrisp", "--device", "hvpp", "-rs"},
		{"avrhvpp", "--device", "avrisp", "--debug"},
		{"avrhvpp", "-rs", "--dump-buffer", "-d", "avrisp"},
		{"avrhvpp", "--write-hex", "/missing.hex", "-d", "avrisp"}}) {
		isp_test::reset();
		Options options;
		std::vector<Operation> operations;
		assert(!parse_arguments(arguments, &options, &operations));
		auto strings = arguments;
		std::vector<char *> argv;
		for (auto &argument : strings) argv.push_back(argument.data());
		assert(avrhvpp_main((int)argv.size(), argv.data()) != 0);
		assert(isp_test::open_calls == 0 && isp_test::commands.empty());
	}
	for (bool trailing : {false, true}) {
		Options options;
		std::vector<Operation> operations;
		assert(parse_arguments(trailing ? std::vector<std::string>{"avrhvpp", "--port", "unused", "-rs", "-d", "avrisp"}
			: std::vector<std::string>{"avrhvpp", "--device", "avrisp", "-p", "unused", "--read-signature"}, &options, &operations));
		assert(options.device == Device::AVRISP && options.serial_options.port == "unused");
		assert(operations.size() == 1 && operations[0].command == CMD_READ_SIGNATURE);
	}
	isp_test::reset();
	isp_test::fault_command = 0x75;
	isp_test::fault = isp_test::Fault::Header;
	isp_test::faults_left = 1;
	std::vector<std::string> arguments = {"avrhvpp", "-d", "avrisp", "-rs", "-rs"};
	std::vector<char *> argv;
	for (auto &argument : arguments) argv.push_back(argument.data());
	bool ok;
	assert(capture_output([&]() { return avrhvpp_main((int)argv.size(), argv.data()) == 0; }, &ok).empty() && !ok);
	assert(isp_test::commands == (std::vector<uint8_t>{0x30, 0x50, 0x75, 0x51}));
	assert(!isp_test::programming);
	isp_test::enabled = false;
}

static void test_avrisp_sessions(Connection *connection)
{
	struct TargetCase {
		TargetDefinition const *target;
		uint8_t code, page_bytes, eeprom_hi, eeprom_lo, flash_hi, flash_lo, fuse_bytes;
		uint16_t last_word;
	};
	TargetCase const cases[] = {
		{&TargetATmega8, 0x70, 64, 2, 0, 0x20, 0, 2, 0x0fe0},
		{&TargetATmega48, 0x59, 64, 1, 0, 0x10, 0, 3, 0x07e0},
		{&TargetATmega88, 0x73, 64, 2, 0, 0x20, 0, 3, 0x0fe0},
		{&TargetATmega168, 0x86, 128, 2, 0, 0x40, 0, 3, 0x1fc0},
		{&TargetATmega328P, 0x86, 128, 4, 0, 0x80, 0, 3, 0x3fc0},
	};
	for (auto const &test : cases) {
		for (int read_chunk : {1, 3, 256}) {
			for (int write_chunk : {1, 7, 256}) {
				isp_test::reset();
				isp_test::read_chunk = read_chunk;
				isp_test::write_chunk = write_chunk;
				Avrisp programmer(connection, 20);
				assert(programmer.begin(test.target));
				std::vector<uint8_t> expected = {0x42, test.code, 0, 0, 1, 1, 1, 1, test.fuse_bytes,
					0xff, 0xff, 0xff, 0xff, 0, test.page_bytes, test.eeprom_hi, test.eeprom_lo,
					0, 0, test.flash_hi, test.flash_lo, 0x20};
				assert(isp_test::packets[1] == expected);
				assert(isp_test::packets[2] == (std::vector<uint8_t>{0x45, 5, 4, 0xd7, 0xc2, 1, 0x20}));
				assert(isp_test::commands == (std::vector<uint8_t>{0x30, 0x42, 0x45, 0x50}));
				uint8_t signature[3], result;
				assert(programmer.read_signature(signature));
				uint8_t instruction[] = {0x30, 0, 2, 0};
				assert(programmer.universal(instruction, &result) && result == signature[2]);
				uint8_t binary[] = {0x00, 0x20, 0xff, 0x20};
				assert(programmer.universal(binary, &result) && result == 0x20);
				std::vector<uint8_t> payload(test.page_bytes), output(test.page_bytes);
				uint8_t const values[] = {0x00, 0x20, 0xff, 0x14, 0x10, 0x51};
				for (size_t i = 0; i < payload.size(); ++i) payload[i] = values[i % sizeof(values)];
				uint16_t last = test.target->flash_page_count - 1;
				assert(programmer.program_page(last, payload.data(), payload.size()));
				assert(isp_test::packets[7] == (std::vector<uint8_t>{0x55, (uint8_t)test.last_word, (uint8_t)(test.last_word >> 8), 0x20}));
				expected = {0x64, 0, test.page_bytes, 'F'};
				expected.insert(expected.end(), payload.begin(), payload.end());
				expected.push_back(0x20);
				assert(isp_test::packets[8] == expected);
				assert(programmer.read_page(last, output.data()) && output == payload);
				assert(isp_test::packets[10] == (std::vector<uint8_t>{0x74, 0, test.page_bytes, 'F', 0x20}));
				assert(programmer.finish() && programmer.finish());
				assert(!isp_test::programming && isp_test::packet.empty() && isp_test::replies.empty());
				assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x50) == 1);
				assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x51) == 1);
			}
		}
	}
	// The CLI shares one session and prints only after successful cleanup.
	isp_test::reset();
	std::vector<std::string> arguments = {"avrhvpp", "-d", "avrisp", "-t", "atmega8", "-rs", "-rs"};
	std::vector<char *> argv;
	for (auto &argument : arguments) argv.push_back(argument.data());
	bool ok;
	assert(capture_output([&]() { return avrhvpp_main((int)argv.size(), argv.data()) == 0; }, &ok)
		== "0x1e, 0x95, 0x0f\n0x1e, 0x95, 0x0f\n" && ok);
	assert(isp_test::commands == (std::vector<uint8_t>{0x30, 0x42, 0x45, 0x50, 0x75, 0x75, 0x51}));
	isp_test::reset();
	{
		Avrisp programmer(connection, 20);
		assert(programmer.begin());
	}
	assert(!isp_test::programming && isp_test::commands.back() == 0x51);
	// Exercise both length bytes at the shared buffer's 256-byte limit.
	isp_test::reset();
	{
		TargetDefinition target = TargetATmega328P;
		target.flash_page_bytes = 256;
		target.flash_page_count = 1;
		Avrisp programmer(connection, 20);
		assert(programmer.begin(&target));
		std::array<uint8_t, 256> payload, output;
		for (size_t i = 0; i < payload.size(); ++i) payload[i] = i;
		assert(programmer.program_page(0, payload.data(), payload.size()));
		assert(isp_test::packets.back().size() == 261 && isp_test::packets.back()[1] == 1 && isp_test::packets.back()[2] == 0);
		assert(programmer.read_page(0, output.data()) && output == payload);
		assert(programmer.finish());
	}
	// Exhausted cleanup retries remain failures, even if Q reached the device.
	for (auto fault : {isp_test::Fault::Header, isp_test::Fault::Status, isp_test::Fault::Truncated, isp_test::Fault::Read}) {
		isp_test::reset();
		Avrisp programmer(connection, 10);
		assert(programmer.begin());
		isp_test::fault_command = 0x51;
		isp_test::fault = fault;
		isp_test::faults_left = 2;
		assert(!programmer.finish() && !programmer.finish());
		assert(isp_test::close_calls == 1);
		assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x51) == 2);
	}
	for (bool wrong_size : {false, true}) {
		isp_test::reset();
		Avrisp programmer(connection, 20);
		assert(programmer.begin(&TargetATmega328P));
		auto count = isp_test::packets.size();
		uint8_t buffer[128] = {};
		assert(!(wrong_size ? programmer.program_page(0, buffer, 127) : programmer.read_page(256, buffer)));
		assert(!programmer.read_signature(buffer));
		assert(isp_test::packets.size() == count);
		assert(!programmer.finish() && !isp_test::programming);
	}
	isp_test::enabled = false;
}

static void test_avrisp_variable_failures(Connection *connection)
{
	// Never finish or retry an incomplete variable-length request, including a
	// request missing only its terminator. Q would otherwise become argument data.
	for (uint8_t command : {0x42, 0x45, 0x55, 0x56, 0x64, 0x74}) {
		unsigned int length = command == 0x42 ? 22 : command == 0x45 ? 7
			: command == 0x55 ? 4 : command == 0x56 ? 6 : command == 0x64 ? 133 : 5;
		for (unsigned int offset = 0; offset < length; ++offset) {
			for (bool zero : {false, true}) {
				isp_test::reset();
				Avrisp programmer(connection, 20);
				isp_test::zero_write = zero;
				uint8_t buffer[128] = {};
				bool ok;
				if (command == 0x42 || command == 0x45) {
					isp_test::fail_write_call = 2 + (command == 0x45 ? 22 : 0) + offset + 1;
					ok = programmer.begin(&TargetATmega328P);
				} else {
					assert(programmer.begin(&TargetATmega328P));
					isp_test::fail_write_call = isp_test::write_calls + offset + 1
						+ (command == 0x64 || command == 0x74 ? 4 : 0);
					if (command == 0x55) ok = programmer.load_address(0x2000);
					else if (command == 0x56) ok = programmer.universal(buffer, buffer);
					else if (command == 0x64) ok = programmer.program_page(0, buffer, sizeof(buffer));
					else ok = programmer.read_page(0, buffer);
				}
				assert(!ok);
				auto packet = isp_test::packet;
				assert(packet.size() == offset);
				assert(!programmer.read_signature(buffer));
				assert(!programmer.finish());
				if (offset) {
					assert(isp_test::packet == packet && isp_test::close_calls == 1);
					assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x51) == 0);
					assert(!programmer.finish() && isp_test::close_calls == 1);
				} else {
					assert(!isp_test::programming && isp_test::packet.empty());
				}
				assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x64) == 0);
			}
		}
	}
	// A complete request with a failed response is not replayed either.
	for (uint8_t command : {0x42, 0x45, 0x55, 0x56, 0x64, 0x74}) {
		for (auto fault : {isp_test::Fault::Header, isp_test::Fault::Status, isp_test::Fault::Truncated, isp_test::Fault::Read}) {
			isp_test::reset();
			Avrisp programmer(connection, 10);
			uint8_t buffer[128] = {0x20, 0xff, 0x00, 0x14, 0x10};
			if (command != 0x42 && command != 0x45) assert(programmer.begin(&TargetATmega328P));
			isp_test::fault_command = command;
			isp_test::fault = fault;
			isp_test::faults_left = 1;
			bool ok;
			if (command == 0x42 || command == 0x45) ok = programmer.begin(&TargetATmega328P);
			else if (command == 0x55) ok = programmer.load_address(0);
			else if (command == 0x56) ok = programmer.universal(buffer, buffer);
			else if (command == 0x64) ok = programmer.program_page(0, buffer, sizeof(buffer));
			else ok = programmer.read_page(0, buffer);
			assert(!ok && !programmer.finish());
			assert(!isp_test::programming && isp_test::packet.empty() && isp_test::replies.empty());
			assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), command) == 1);
		}
	}
	isp_test::enabled = false;
}

static void assert_avrisp_read_only()
{
	std::vector<uint8_t> const allowed = {0x30, 0x42, 0x45, 0x50, 0x51, 0x55, 0x56, 0x74, 0x75};
	for (auto const &packet : isp_test::packets) {
		assert(std::find(allowed.begin(), allowed.end(), packet[0]) != allowed.end());
		if (packet[0] == 0x56) {
			assert(packet[1] == 0x50 || packet[1] == 0x58);
			assert(packet[2] == 0 || packet[2] == 8);
			assert(packet[3] == 0 && packet[4] == 0);
		}
	}
	assert(!isp_test::programming && isp_test::packet.empty());
}

static void test_avrisp_reads(Connection *connection)
{
	char directory[] = "/tmp/avrhvpp-isp-reads-XXXXXX";
	assert(mkdtemp(directory));
	std::string page_path = std::string(directory) + "/page.bin";
	std::string hex_path = std::string(directory) + "/image.hex";
	bool ok;
	for (char const *name : {"atmega8", "atmega48", "atmega88", "atmega168", "atmega328p"}) {
		auto target = create_target(name);
		auto const &definition = target->definition();
		// Query uses the detected chip even with a conflicting explicit target.
		for (bool explicit_target : {false, true}) {
			isp_test::reset();
			std::copy(definition.signature, definition.signature + 3, isp_test::signature.begin());
			Options options;
			std::vector<Operation> operations;
			std::vector<std::string> arguments = {"avrhvpp", "-d", "avrisp", "-q"};
			if (explicit_target) arguments.insert(arguments.end(), {"-t", "atmega328p"});
			assert(parse_arguments(arguments, &options, &operations));
			std::string text = capture_operation(operations[0], connection, &ok, Device::AVRISP);
			char heading[256];
			snprintf(heading, sizeof(heading), "signature: 0x%02x, 0x%02x, 0x%02x\nchip: %s\npage size: %u bytes\npage count: %u\nflash size: %lu bytes\n",
				definition.signature[0], definition.signature[1], definition.signature[2], definition.display_name,
				(unsigned int)definition.flash_page_bytes, (unsigned int)definition.flash_page_count, (unsigned long)definition.flash_bytes());
			std::string expected = std::string(heading) + "fuse lo: 0x62\nfuse hi: 0xd9\n";
			if (definition.supports(CapabilityFuseExtended)) expected += "fuse ex: 0xfd\n";
			expected += "lock bits: 0xfc\n";
			assert(ok && text == expected);
			std::vector<std::vector<uint8_t>> universal;
			for (auto const &packet : isp_test::packets) if (packet[0] == 0x56) universal.push_back(packet);
			std::vector<std::vector<uint8_t>> expected_universal = {{0x56, 0x50, 0, 0, 0, 0x20}, {0x56, 0x58, 8, 0, 0, 0x20}};
			if (definition.supports(CapabilityFuseExtended)) expected_universal.push_back({0x56, 0x50, 8, 0, 0, 0x20});
			expected_universal.push_back({0x56, 0x58, 0, 0, 0, 0x20});
			assert(universal == expected_universal);
			assert(!explicit_target || options.target->name() == "atmega328p");
			assert_avrisp_read_only();
		}
		// Explicit and detected targets support the same individual read options.
		for (bool explicit_target : {false, true}) {
			isp_test::reset();
			std::copy(definition.signature, definition.signature + 3, isp_test::signature.begin());
			Options options;
			std::vector<Operation> operations;
			std::vector<std::string> arguments = {"avrhvpp", "-d", "avrisp", "-rfl", "-rfh", "-rl"};
			if (definition.supports(CapabilityFuseExtended)) arguments.push_back("-rfe");
			if (explicit_target) arguments.insert(arguments.end(), {"-t", name});
			assert(parse_arguments(arguments, &options, &operations));
			std::string text = capture_output([&]() { return avrisp_perform(operations, connection, explicit_target ? &definition : nullptr); }, &ok);
			assert(ok && text == (std::string("0x62\n0xd9\n0xfc\n") + (definition.supports(CapabilityFuseExtended) ? "0xfd\n" : "")));
			assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x75) == (explicit_target ? 0 : 1));
			assert_avrisp_read_only();
		}
		for (int read_chunk : {1, 7}) {
			isp_test::reset();
			isp_test::read_chunk = read_chunk;
			isp_test::write_chunk = 7;
			uint32_t size = definition.flash_bytes();
			for (unsigned int i = 0; i < 16; ++i) {
				isp_test::memory[i] = i; // Includes 00 and control bytes in the last record.
				isp_test::memory[size - 16 + i] = i + 0x10;
			}
			isp_test::memory[size - 1] = 0x20;
			Options options;
			std::vector<Operation> operations;
			assert(parse_arguments({"avrhvpp", "-d", "avrisp", "--read-page", std::to_string(definition.flash_page_count - 1), page_path,
				"--read-hex", hex_path, "--target", name}, &options, &operations));
			std::string output = capture_output([&]() { return avrisp_perform(operations, connection, &definition); }, &ok);
			assert(ok && output == "page " + std::to_string(definition.flash_page_count - 1) + " read: " + page_path
				+ "\nhex read: " + hex_path + " (2 data records)\n");
			std::vector<uint8_t> data, image, mask;
			assert(read_page_file(page_path.c_str(), *target, &data));
			assert(std::equal(data.begin(), data.end(), isp_test::memory.begin() + size - definition.flash_page_bytes));
			assert(read_hex_file(hex_path.c_str(), *target, &image, &mask));
			assert(image.size() == size && std::equal(image.begin(), image.end(), isp_test::memory.begin()));
			assert(std::count(mask.begin(), mask.end(), 1) == 2);
			assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x74) == definition.flash_page_count + 1);
			assert_avrisp_read_only();
		}
		for (auto const &arguments : std::vector<std::vector<std::string>>{
			{"avrhvpp", "-d", "avrisp", "--read-page", "0", page_path},
			{"avrhvpp", "--read-hex", hex_path, "-d", "avrisp"},
			{"avrhvpp", "-t", name, "--read-page", std::to_string(definition.flash_page_count), page_path, "-d", "avrisp"}}) {
			isp_test::reset();
			std::vector<char *> argv;
			auto strings = arguments;
			for (auto &argument : strings) argv.push_back(argument.data());
			assert(avrhvpp_main((int)argv.size(), argv.data()) != 0);
			assert(isp_test::open_calls == 0 && isp_test::commands.empty());
		}
	}
	// Unknown signatures never lead to fuse/lock reads or subsequent operations.
	for (bool query : {false, true}) {
		isp_test::reset();
		isp_test::signature[2] = 0x07;
		Options options;
		std::vector<Operation> operations;
		assert(parse_arguments({"avrhvpp", "-d", "avrisp", query ? "--query" : "-rl", "-rs"}, &options, &operations));
		auto output = capture_output([&]() { return avrisp_perform(operations, connection, nullptr); }, &ok);
		assert(!ok && output == (query ? "signature: 0x1e, 0x95, 0x07\nchip: unknown\n" : ""));
		assert(isp_test::commands == (std::vector<uint8_t>{0x30, 0x50, 0x75, 0x51}));
		assert_avrisp_read_only();
	}
	// An explicit unsupported fuse fails before opening; autodetection rejects it before Universal.
	isp_test::reset();
	Options options;
	std::vector<Operation> operations;
	assert(!parse_arguments({"avrhvpp", "-d", "avrisp", "-t", "atmega8", "-rfe"}, &options, &operations));
	assert(isp_test::commands.empty() && isp_test::open_calls == 0);
	isp_test::signature = {{0x1e, 0x93, 0x07}};
	Operation extended;
	extended.command = CMD_READ_FUSE; extended.data = 2;
	assert(capture_operation(extended, connection, &ok, Device::AVRISP).empty() && !ok);
	assert_avrisp_read_only();
	assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x56) == 0);
	// A query must not replace the selected Flash target.
	isp_test::reset();
	isp_test::signature = {{0x1e, 0x93, 0x07}};
	options = Options(); operations.clear();
	assert(parse_arguments({"avrhvpp", "-d", "avrisp", "-t", "atmega328p", "-q", "--read-page", "255", page_path}, &options, &operations));
	capture_output([&]() { return avrisp_perform(operations, connection, &options.target->definition()); }, &ok);
	assert(ok && isp_test::address == 0x4000 && options.target->name() == "atmega328p");
	assert_avrisp_read_only();
	// An entirely erased image contains only EOF.
	isp_test::reset();
	operations.erase(operations.begin());
	operations[0].command = _CMD_READ_HEX; operations[0].path = hex_path;
	capture_output([&]() { return avrisp_perform(operations, connection, &options.target->definition()); }, &ok);
	assert(ok);
	std::ifstream input(hex_path, std::ios::binary);
	assert(std::string((std::istreambuf_iterator<char>(input)), {}) == ":00000001FF\r\n");
	assert_avrisp_read_only();
	assert(unlink(page_path.c_str()) == 0 && unlink(hex_path.c_str()) == 0 && rmdir(directory) == 0);
	isp_test::enabled = false;
}

static void test_avrisp_read_failures(Connection *connection)
{
	char directory[] = "/tmp/avrhvpp-isp-failures-XXXXXX";
	assert(mkdtemp(directory));
	std::string path = std::string(directory) + "/output";
	auto target = create_target("atmega328p");
	for (bool hex : {false, true}) {
		for (bool exists : {false, true}) {
			for (bool last : {false, true}) {
				isp_test::reset();
				if (exists) { std::ofstream file(path); file << "keep"; }
				Operation read, next;
				read.command = hex ? _CMD_READ_HEX : CMD_READ_PAGE;
				read.target = target.get(); read.path = path;
				next.command = CMD_READ_SIGNATURE;
				isp_test::fault_command = 0x74;
				isp_test::fault = isp_test::Fault::Status;
				isp_test::faults_left = 1;
				isp_test::fault_skip = hex && last ? 255 : 0;
				bool ok;
				assert(capture_output([&]() { return avrisp_perform({read, next}, connection, &target->definition()); }, &ok).empty() && !ok);
				assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x74) == (hex && last ? 256 : 1));
				assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x75) == 0);
				assert_avrisp_read_only();
				if (exists) {
					std::ifstream input(path);
					assert(std::string((std::istreambuf_iterator<char>(input)), {}) == "keep");
					assert(unlink(path.c_str()) == 0);
				} else assert(access(path.c_str(), F_OK) != 0);
			}
		}
	}
	// Fuse failure stops the query and subsequent commands, and still leaves the mode.
	for (uint8_t command : {0x75, 0x56}) {
		isp_test::reset();
		isp_test::fault_command = command;
		isp_test::fault = isp_test::Fault::Header;
		isp_test::faults_left = 1;
		Operation query;
		query.command = _CMD_QUERY;
		bool ok;
		auto text = capture_operation(query, connection, &ok, Device::AVRISP);
		assert(!ok && (command == 0x75 ? text.empty() : text.find("chip: ATmega328P\n") != std::string::npos));
		assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x56) == (command == 0x75 ? 0 : 1));
		assert_avrisp_read_only();
	}
	// A file error must stop before the following operation.
	isp_test::reset();
	Operation read, next;
	read.command = CMD_READ_PAGE; read.path = directory; read.target = target.get();
	next.command = CMD_READ_SIGNATURE;
	bool ok;
	assert(capture_output([&]() { return avrisp_perform({read, next}, connection, &target->definition()); }, &ok).empty() && !ok);
	assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x75) == 0);
	assert_avrisp_read_only();
	// Successful data remains available when the final mode-release ACK fails.
	isp_test::reset();
	read.path = path;
	isp_test::fault_command = 0x51;
	isp_test::fault = isp_test::Fault::Status;
	isp_test::faults_left = 1;
	assert(capture_operation(read, connection, &ok, Device::AVRISP).empty() && !ok);
	std::vector<uint8_t> data;
	assert(read_page_file(path.c_str(), *target, &data));
	assert(data == std::vector<uint8_t>(128, 0xff));
	assert_avrisp_read_only();
	assert(unlink(path.c_str()) == 0);
	assert(rmdir(directory) == 0);
	isp_test::enabled = false;
}

static void reset_isp_write_model(TargetDefinition const &target)
{
	isp_test::reset();
	isp_test::write_model = true;
	std::copy(target.signature, target.signature + 3, isp_test::signature.begin());
	isp_test::status[3] = 0xff;
}

static unsigned int count_isp_instruction(uint8_t first, uint8_t second)
{
	unsigned int count = 0;
	for (auto const &packet : isp_test::packets) {
		if (packet[0] == 0x56 && packet[1] == first && packet[2] == second) ++count;
	}
	return count;
}

static void assert_isp_write_finished()
{
	assert(!isp_test::programming && isp_test::packet.empty());
	assert(isp_test::commands.back() == 0x51);
}

static void test_avrisp_writes(Connection *connection)
{
	char directory[] = "/tmp/avrhvpp-isp-writes-XXXXXX";
	assert(mkdtemp(directory));
	std::string path = std::string(directory) + "/page.bin";
	for (char const *name : {"atmega8", "atmega48", "atmega88", "atmega168", "atmega328p"}) {
		auto target = create_target(name);
		auto const &definition = target->definition();
		bool ok;
		Operation erase;
		erase.command = CMD_CHIP_ERASE; erase.target = target.get();
		reset_isp_write_model(definition);
		isp_test::memory.fill(0);
		isp_test::status[3] = 0xfc;
		auto fuses = isp_test::status;
		assert(capture_operation(erase, connection, &ok, Device::AVRISP) == "chip erased\n" && ok);
		assert(std::all_of(isp_test::memory.begin(), isp_test::memory.begin() + definition.flash_bytes(), [](uint8_t byte) { return byte == 0xff; }));
		assert(std::equal(fuses.begin(), fuses.begin() + 3, isp_test::status.begin()));
		assert(isp_test::status[3] == 0xff && count_isp_instruction(0xac, 0x80) == 1);
		assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x74) == definition.flash_page_count);
		assert((count_isp_instruction(0xf0, 0) == 0) == (std::string(name) == "atmega8"));
		assert_isp_write_finished();

		reset_isp_write_model(definition);
		std::vector<uint8_t> bytes(definition.flash_page_bytes);
		for (size_t i = 0; i < bytes.size(); ++i) bytes[i] = i; // Includes 00, 20, 51 and 64.
		bytes.back() = 0xff;
		{ std::ofstream file(path, std::ios::binary); file.write((char const *)bytes.data(), bytes.size()); }
		Options options;
		std::vector<Operation> operations;
		assert(parse_arguments({"avrhvpp", "--write-page", std::to_string(definition.flash_page_count - 1), path,
			"-t", name, "-d", "avrisp"}, &options, &operations));
		std::string expected = "page " + std::to_string(definition.flash_page_count - 1) + " written: " + path + "\n";
		assert(capture_operation(operations[0], connection, &ok, Device::AVRISP) == expected && ok);
		size_t offset = definition.flash_bytes() - definition.flash_page_bytes;
		assert(std::equal(bytes.begin(), bytes.end(), isp_test::memory.begin() + offset));
		assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x64) == 1);
		assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x74) == 2);
		assert(count_isp_instruction(0xac, 0x80) == 0);
		assert_isp_write_finished();
		// A matching page needs no write or wait. A conflicting page must remain intact.
		isp_test::packets.clear(); isp_test::commands.clear();
		assert(capture_operation(operations[0], connection, &ok, Device::AVRISP) == expected && ok);
		assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x64) == 0);
		assert(count_isp_instruction(0xf0, 0) == 0);
		operations[0].bytes[0] ^= 1;
		Operation next; next.command = CMD_READ_SIGNATURE;
		isp_test::packets.clear(); isp_test::commands.clear();
		assert(capture_output([&]() { return avrisp_perform({operations[0], next}, connection, &definition); }, &ok).empty() && !ok);
		assert(std::equal(bytes.begin(), bytes.end(), isp_test::memory.begin() + offset));
		assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x64) == 0);
		assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x75) == 0);
		assert_isp_write_finished();

		for (bool explicit_target : {false, true}) {
			reset_isp_write_model(definition);
			isp_test::reserved_zero = true;
			options = Options(); operations.clear();
			std::vector<std::string> arguments = {"avrhvpp", "-d", "avrisp", "-wfl", "63", "-wfh", "c9"};
			bool extended = definition.supports(CapabilityFuseExtended);
			if (extended) arguments.insert(arguments.end(), {"-wfe", "00"});
			arguments.insert(arguments.end(), {"-wl", "3c"});
			if (explicit_target) arguments.insert(arguments.end(), {"-t", name});
			assert(parse_arguments(arguments, &options, &operations));
			uint8_t extended_value = std::string(name) == "atmega48" ? 0xfe : 0xf8;
			std::string text = "fuse written: 0x63\nfuse written: 0xc9\n";
			if (extended) text += std::string("fuse written: ") + (extended_value == 0xfe ? "0xfe\n" : "0xf8\n");
			text += "lock bits written: 0xfc\n";
			assert(capture_output([&]() { return avrisp_perform(operations, connection, explicit_target ? &definition : nullptr); }, &ok) == text && ok);
			assert(isp_test::status[0] == 0x63 && isp_test::status[1] == 0xc9 && isp_test::status[3] == 0xfc);
			assert(!extended || isp_test::status[2] == extended_value);
			assert(count_isp_instruction(0xac, 0xa0) == 1 && count_isp_instruction(0xac, 0xa8) == 1);
			assert(count_isp_instruction(0xac, 0xa4) == (extended ? 1 : 0));
			assert(count_isp_instruction(0xac, 0xe0) == 1 && count_isp_instruction(0xac, 0x80) == 0);
			assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x75) == (explicit_target ? 0 : 1));
			assert_isp_write_finished();
			// Unused bits read as zero must not cause unnecessary writes or mismatches.
			isp_test::packets.clear(); isp_test::commands.clear();
			assert(capture_output([&]() { return avrisp_perform(operations, connection, &definition); }, &ok) == text && ok);
			assert(count_isp_instruction(0xac, 0xa0) == 0 && count_isp_instruction(0xac, 0xa8) == 0
				&& count_isp_instruction(0xac, 0xa4) == 0 && count_isp_instruction(0xac, 0xe0) == 0);
		}
		// SPIEN changes and 0->1 lock changes must not send any write or erase.
		for (bool lock : {false, true}) {
			reset_isp_write_model(definition);
			isp_test::status[3] = 0xfc;
			Operation op;
			op.command = lock ? CMD_WRITE_LOCK : CMD_WRITE_FUSE;
			op.data = 1; op.value = lock ? 0xff : 0xf9; op.target = target.get();
			assert(capture_operation(op, connection, &ok, Device::AVRISP).empty() && !ok);
			for (auto const &packet : isp_test::packets) assert(packet[0] != 0x56 || packet[1] != 0xac);
			assert_isp_write_finished();
		}
	}
	assert(unlink(path.c_str()) == 0 && rmdir(directory) == 0);
}

static void test_avrisp_write_failures(Connection *connection)
{
	auto target = create_target("atmega328p");
	auto const &definition = target->definition();
	Operation next; next.command = CMD_READ_SIGNATURE;
	for (Command command : {CMD_WRITE_FUSE, CMD_WRITE_LOCK, CMD_WRITE_PAGE, CMD_CHIP_ERASE}) {
		for (bool timeout : {false, true}) {
			reset_isp_write_model(definition);
			isp_test::forever_busy = timeout;
			isp_test::ignore_write = !timeout;
			Operation op;
			op.command = command; op.target = target.get();
			op.value = command == CMD_WRITE_LOCK ? 0xfc : 0x63;
			op.bytes.assign(128, 0x5a);
			if (command == CMD_CHIP_ERASE) isp_test::memory.fill(0);
			bool ok;
			assert(capture_output([&]() { return avrisp_perform({op, next}, connection, &definition); }, &ok).empty() && !ok);
			assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x75) == 0);
			assert_isp_write_finished();
			if (command == CMD_WRITE_PAGE) assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x64) == 1);
			else assert(count_isp_instruction(0xac, command == CMD_CHIP_ERASE ? 0x80 : command == CMD_WRITE_LOCK ? 0xe0 : 0xa0) == 1);
		}
	}
	for (bool lock : {false, true}) {
		reset_isp_write_model(definition);
		isp_test::retain_lock = lock;
		isp_test::corrupt_erase = !lock;
		isp_test::status[3] = 0xfc;
		Operation erase; erase.command = CMD_CHIP_ERASE; erase.target = target.get();
		bool ok;
		assert(capture_operation(erase, connection, &ok, Device::AVRISP).empty() && !ok);
		assert(count_isp_instruction(0xac, 0x80) == 1);
		assert_isp_write_finished();
	}
	reset_isp_write_model(definition);
	isp_test::corrupt_flash_write = true;
	Operation page; page.command = CMD_WRITE_PAGE; page.target = target.get(); page.bytes.assign(128, 0x5a);
	bool ok;
	assert(capture_operation(page, connection, &ok, Device::AVRISP).empty() && !ok);
	assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x64) == 1);
	assert_isp_write_finished();
	// Failure of a write ACK, busy-poll response or readback must never replay the write.
	for (int skip : {1, 2, 5}) {
		reset_isp_write_model(definition);
		isp_test::fault_command = 0x56; isp_test::fault = isp_test::Fault::Status;
		isp_test::fault_skip = skip; isp_test::faults_left = 1;
		Operation fuse; fuse.command = CMD_WRITE_FUSE; fuse.value = 0x63; fuse.target = target.get();
		assert(capture_output([&]() { return avrisp_perform({fuse, next}, connection, &definition); }, &ok).empty() && !ok);
		assert(count_isp_instruction(0xac, 0xa0) == 1);
		assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x75) == 0);
		assert_isp_write_finished();
	}
	// Missing EOP is also unsafe: all four SPI bytes may already have executed.
	for (unsigned int offset = 1; offset < 6; ++offset) {
		reset_isp_write_model(definition);
		Avrisp programmer(connection, 20);
		assert(programmer.begin(&definition));
		isp_test::fail_write_call = isp_test::write_calls + 6 + offset + 1;
		assert(!programmer.write_fuse(0, 0x63) && !programmer.finish());
		assert(isp_test::packet.size() == offset && isp_test::packet[0] == 0x56);
		assert(isp_test::close_calls == 1 && isp_test::programming);
		assert((isp_test::pending != isp_test::Pending::None) == (offset == 5));
		assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x51) == 0);
	}
	// Preserve the record of verified writes if a later operation or cleanup fails.
	for (bool cleanup : {false, true}) {
		reset_isp_write_model(definition);
		Operation low, high;
		low.command = high.command = CMD_WRITE_FUSE;
		low.value = 0x63; high.data = 1; high.value = 0xf9;
		if (cleanup) {
			isp_test::fault_command = 0x51; isp_test::fault = isp_test::Fault::Status; isp_test::faults_left = 1;
		}
		assert(capture_output([&]() { return avrisp_perform(cleanup ? std::vector<Operation>{low} : std::vector<Operation>{low, high, next}, connection, &definition); }, &ok)
			== "fuse written: 0x63\n" && !ok);
		assert(isp_test::status[0] == 0x63 && isp_test::status[1] == 0xd9);
		assert(count_isp_instruction(0xac, 0xa0) == 1 && count_isp_instruction(0xac, 0xa8) == 0);
		assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x75) == 0);
		assert_isp_write_finished();
	}
	// Known signatures provide the geometry needed for erase verification.
	reset_isp_write_model(definition);
	isp_test::memory.fill(0); isp_test::status[3] = 0xfc;
	Operation detected_erase; detected_erase.command = CMD_CHIP_ERASE;
	assert(capture_operation(detected_erase, connection, &ok, Device::AVRISP) == "chip erased\n" && ok);
	assert(count_isp_instruction(0xac, 0x80) == 1);
	assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x75) == 1);
	assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x74) == 256);
	assert_isp_write_finished();
	// Reject unsupported inferred fuses before executing any requested write.
	reset_isp_write_model(TargetATmega8);
	Operation low, extended;
	low.command = extended.command = CMD_WRITE_FUSE;
	low.value = 0x63; extended.data = 2;
	assert(capture_output([&]() { return avrisp_perform({low, extended}, connection, nullptr); }, &ok).empty() && !ok);
	assert(count_isp_instruction(0xac, 0xa0) == 0 && count_isp_instruction(0xac, 0xa4) == 0);
	assert_isp_write_finished();
	for (auto const &arguments : std::vector<std::vector<std::string>>{
		{"avrhvpp", "-d", "avrisp", "-t", "atmega8", "-wfe", "00"},
		{"avrhvpp", "-d", "avrisp", "--write-page", "0", "/missing-page.bin"},
		{"avrhvpp", "-d", "avrisp", "-t", "atmega328p", "--write-page", "256", "/missing-page.bin"},
		{"avrhvpp", "-d", "avrisp", "-t", "atmega328p", "--write-page", "0", "/missing-page.bin"},
		{"avrhvpp", "-d", "avrisp", "-wfl", "invalid"},
		{"avrhvpp", "-d", "avrisp", "--write-hex", "/missing.hex"}}) {
		isp_test::reset();
		auto strings = arguments;
		std::vector<char *> argv;
		for (auto &argument : strings) argv.push_back(argument.data());
		assert(avrhvpp_main((int)argv.size(), argv.data()) != 0);
		assert(isp_test::open_calls == 0 && isp_test::commands.empty());
	}
	// No target definition: only the signature probe is allowed to run.
	reset_isp_write_model(definition); isp_test::signature[2] = 0x07;
	Operation erase; erase.command = CMD_CHIP_ERASE;
	assert(capture_operation(erase, connection, &ok, Device::AVRISP).empty() && !ok);
	assert(count_isp_instruction(0xac, 0x80) == 0);
	assert_isp_write_finished();
	isp_test::enabled = false;
}

static void test_avrisp_hex(Connection *connection)
{
	char directory[] = "/tmp/avrhvpp-isp-hex-XXXXXX";
	assert(mkdtemp(directory));
	std::string source = std::string(directory) + "/input.hex";
	std::string output = std::string(directory) + "/output.hex";
	std::string invalid = std::string(directory) + "/invalid.hex";
	for (char const *name : {"atmega8", "atmega48", "atmega88", "atmega168", "atmega328p"}) {
		auto target = create_target(name);
		auto const &definition = target->definition();
		uint32_t size = definition.flash_bytes();
		uint8_t const bytes[] = {0, 0x20, 0xff, 0x14, 0x10, 0x51, 0x64, 0xac};
		std::vector<uint8_t> expected(size, 0xff);
		std::copy(std::begin(bytes), std::end(bytes), expected.begin());
		std::copy(std::begin(bytes), std::end(bytes), expected.begin() + 512);
		expected.back() = 0xa5;
		FILE *file = fopen(source.c_str(), "wb");
		assert(file && write_hex_record(file, 0, 0, bytes, sizeof(bytes)));
		assert(write_hex_record(file, 512, 0, bytes, sizeof(bytes)));
		assert(write_hex_record(file, size - 1, 0, &expected.back(), 1));
		assert(write_hex_record(file, 0, 1, nullptr, 0) && fclose(file) == 0);
		for (bool flag_first : {false, true}) {
			reset_isp_write_model(definition);
			// One selected page already matches; an unselected page must survive.
			std::copy(expected.begin() + 512, expected.begin() + 512 + definition.flash_page_bytes, isp_test::memory.begin() + 512);
			size_t preserved = definition.flash_page_bytes * 2 + 3;
			isp_test::memory[preserved] = 0x37;
			std::vector<std::string> arguments = {"avrhvpp"};
			if (flag_first) arguments.push_back("--no-chip-erase");
			arguments.insert(arguments.end(), {"--write-hex", source, "--read-hex", output, "--device", "avrisp", "--target", name});
			if (!flag_first) arguments.push_back("--no-chip-erase");
			std::vector<char *> argv;
			for (auto &argument : arguments) argv.push_back(argument.data());
			bool ok;
			std::string text = capture_output([&]() { return avrhvpp_main((int)argv.size(), argv.data()) == 0; }, &ok);
			assert(ok && text.find("(3 data pages, 2 written, 1 already matched)\n") != std::string::npos);
			assert(text.find("hex written:") < text.find("hex read:"));
			auto preserved_image = expected; preserved_image[preserved] = 0x37;
			assert(std::equal(preserved_image.begin(), preserved_image.end(), isp_test::memory.begin()));
			std::vector<uint8_t> image, mask;
			assert(read_hex_file(output.c_str(), *target, &image, &mask) && image == preserved_image);
			assert(count_isp_instruction(0xac, 0x80) == 0);
			assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x64) == 2);
			assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x50) == 1);
			// The first write may start only after all three selected pages were inspected.
			unsigned int reads = 0;
			for (uint8_t command : isp_test::commands) {
				if (command == 0x64) break;
				if (command == 0x74) ++reads;
			}
			assert(reads >= 3);
			assert_isp_write_finished();
			Options options;
			std::vector<Operation> operations;
			assert(parse_arguments(arguments, &options, &operations));
			isp_test::packets.clear(); isp_test::commands.clear();
			text = capture_operation(operations[0], connection, &ok, Device::AVRISP);
			assert(ok && text.find("(3 data pages, 0 written, 3 already matched)\n") != std::string::npos);
			assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x64) == 0);
			assert(count_isp_instruction(0xac, 0x80) == 0 && isp_test::memory[preserved] == 0x37);
			// A conflict on the final selected page prevents every write.
			reset_isp_write_model(definition);
			isp_test::memory[size - 1] = 0;
			Operation next; next.command = CMD_READ_SIGNATURE;
			assert(capture_output([&]() { return avrisp_perform({operations[0], next}, connection, &definition); }, &ok).empty() && !ok);
			assert(isp_test::memory[0] == 0xff && isp_test::memory[size - 1] == 0);
			assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x64) == 0);
			assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x75) == 0);
			assert(count_isp_instruction(0xac, 0x80) == 0);
			assert_isp_write_finished();
		}
		reset_isp_write_model(definition);
		isp_test::memory.fill(0); isp_test::status[3] = 0xfc;
		auto fuses = isp_test::status;
		Options options;
		std::vector<Operation> operations;
		assert(parse_arguments({"avrhvpp", "--write-hex", source, "-t", name, "-d", "avrisp"}, &options, &operations));
		bool ok;
		auto text = capture_operation(operations[0], connection, &ok, Device::AVRISP);
		assert(ok && text.find("chip erased\n") == 0 && text.find("(3 data pages, 3 written, 0 already matched, chip erased)\n") != std::string::npos);
		assert(std::equal(expected.begin(), expected.end(), isp_test::memory.begin()));
		assert(std::equal(fuses.begin(), fuses.begin() + 3, isp_test::status.begin()));
		assert(count_isp_instruction(0xac, 0x80) == 1);
		assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x64) == 3);
		assert_isp_write_finished();
		// All-FF records and EOF-only files preserve the requested default erase policy.
		for (bool blank_record : {false, true}) {
			file = fopen(source.c_str(), "wb");
			uint8_t blank = 0xff;
			assert(file && (!blank_record || write_hex_record(file, size - 1, 0, &blank, 1)));
			assert(write_hex_record(file, 0, 1, nullptr, 0) && fclose(file) == 0);
			for (bool erase : {false, true}) {
				reset_isp_write_model(definition); isp_test::memory[0] = 0x37;
				options = Options(); operations.clear();
				assert(parse_arguments({"avrhvpp", "-d", "avrisp", "-t", name, "--write-hex", source}, &options, &operations));
				operations[0].chip_erase = erase;
				capture_operation(operations[0], connection, &ok, Device::AVRISP);
				assert(ok && isp_test::memory[0] == (erase ? 0xff : 0x37));
				assert(count_isp_instruction(0xac, 0x80) == (erase ? 1 : 0));
				assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x64) == 0);
				assert_isp_write_finished();
			}
		}
		// All inputs, including later operations, are validated before connecting.
		for (std::string const &bad : {std::string("not hex\n"), std::string(":00000001FE\n"), std::string(":0100000000FF\n"),
			std::string(":020000040001F9\n:0100000000FF\n:00000001FF\n")}) {
			{ std::ofstream file(invalid); file << bad; }
			isp_test::reset();
			std::vector<std::string> arguments = {"avrhvpp", "--write-hex", source, "--write-hex", invalid, "-t", name, "-d", "avrisp"};
			std::vector<char *> argv;
			for (auto &argument : arguments) argv.push_back(argument.data());
			assert(avrhvpp_main((int)argv.size(), argv.data()) != 0);
			assert(isp_test::open_calls == 0 && isp_test::commands.empty());
		}
	}
	assert(unlink(source.c_str()) == 0 && unlink(output.c_str()) == 0 && unlink(invalid.c_str()) == 0 && rmdir(directory) == 0);
	isp_test::enabled = false;
}

static void test_avrisp_hex_failures(Connection *connection)
{
	auto target = create_target("atmega328p");
	auto const &definition = target->definition();
	Operation hex, next;
	hex.command = _CMD_WRITE_HEX; hex.target = target.get(); hex.path = "test.hex";
	hex.bytes.assign(32768, 0xff); hex.page_mask.assign(256, 0);
	for (uint16_t page : {uint16_t(0), uint16_t(4), uint16_t(255)}) {
		hex.page_mask[page] = 1; hex.bytes[page * 128] = 0x5a;
	}
	next.command = CMD_READ_SIGNATURE;
	for (int failure = 0; failure < 7; ++failure) {
		reset_isp_write_model(definition);
		hex.chip_erase = failure == 1 || failure == 2 || failure == 6;
		if (failure == 0) {
			isp_test::fault_command = 0x74; isp_test::fault_skip = 2;
		} else if (failure == 1) {
			isp_test::fault_command = 0x56;
		} else if (failure == 2) {
			isp_test::status[3] = 0xfc; isp_test::retain_lock = true;
		} else if (failure == 3) {
			isp_test::fault_command = 0x64; isp_test::fault_skip = 1;
		} else if (failure == 4 || failure == 6) isp_test::corrupt_flash_write = true;
		else isp_test::forever_busy = true;
		if (failure == 0 || failure == 1 || failure == 3) {
			isp_test::fault = isp_test::Fault::Status; isp_test::faults_left = 1;
		}
		bool ok;
		auto text = capture_output([&]() { return avrisp_perform({hex, next}, connection, &definition); }, &ok);
		assert(!ok && text.find("hex written:") == std::string::npos);
		assert((text.find("chip erased\n") != std::string::npos) == (failure == 6));
		assert((text.find("page 0 written: test.hex\n") != std::string::npos) == (failure == 3));
		assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x64) == (failure < 3 ? 0 : failure == 3 ? 2 : 1));
		assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x75) == 0);
		assert(count_isp_instruction(0xac, 0x80) == (hex.chip_erase ? 1 : 0));
		assert(isp_test::memory[255 * 128] == 0xff);
		assert_isp_write_finished();
	}
	// Malformed in-memory images are rejected before erasing or writing pages.
	reset_isp_write_model(definition);
	hex.chip_erase = true; hex.bytes.pop_back();
	bool ok;
	assert(capture_operation(hex, connection, &ok, Device::AVRISP).empty() && !ok);
	assert(count_isp_instruction(0xac, 0x80) == 0);
	assert(std::count(isp_test::commands.begin(), isp_test::commands.end(), 0x64) == 0);
	assert_isp_write_finished();
	isp_test::enabled = false;
}

static void test_query(Connection *connection)
{
	struct QueryCase {
		char const *name;
		uint8_t signature1, signature2;
		char const *expected;
	};
	for (auto const &test : {
		QueryCase{"atmega48", 0x92, 0x05, "signature: 0x1e, 0x92, 0x05\nchip: ATmega48\npage size: 64 bytes\npage count: 64\nflash size: 4096 bytes\n"},
		QueryCase{"atmega88", 0x93, 0x0a, "signature: 0x1e, 0x93, 0x0a\nchip: ATmega88\npage size: 64 bytes\npage count: 128\nflash size: 8192 bytes\n"},
		QueryCase{"atmega168", 0x94, 0x06, "signature: 0x1e, 0x94, 0x06\nchip: ATmega168\npage size: 128 bytes\npage count: 128\nflash size: 16384 bytes\n"}}) {
		reset();
		signature1 = test.signature1;
		signature2 = test.signature2;
		Options options;
		std::vector<Operation> operations;
		assert(parse_arguments({"avrhvpp", "--query", "--target", test.name}, &options, &operations));
		bool ok;
		std::string expected = test.expected;
		expected += "fuse lo: 0xff\nfuse hi: 0xff\nfuse ex: 0xff\nlock bits: 0xff\n";
		assert(capture_operation(operations[0], connection, &ok) == expected && ok);
		assert(fuse_reads == (std::vector<uint8_t>{0, 1, 2, 3}));
		assert(write_pulses == 0);
	}
	for (bool small : {false, true}) {
		reset();
		signature1 = small ? 0x93 : 0x95;
		signature2 = small ? 0x07 : 0x0f;
		fuse_values = {{0x62, 0xd9, 0xff, 0xfc}};
		Options options;
		std::vector<Operation> operations;
		assert(parse_arguments({"avrhvpp", small ? "-q" : "--query", "--read-signature"}, &options, &operations));
		assert(!options.target && operations.size() == 2 && operations[0].command == _CMD_QUERY);
		assert(operations[1].command == CMD_READ_SIGNATURE);
		bool ok;
		std::string text = capture_operation(operations[0], connection, &ok);
		std::string expected = small ? "signature: 0x1e, 0x93, 0x07\nchip: ATmega8\n" : "signature: 0x1e, 0x95, 0x0f\nchip: ATmega328P\n";
		expected += small ? "page size: 64 bytes\npage count: 128\nflash size: 8192 bytes\n"
			: "page size: 128 bytes\npage count: 256\nflash size: 32768 bytes\n";
		expected += "fuse lo: 0x62\nfuse hi: 0xd9\n";
		if (!small) expected += "fuse ex: 0xff\n";
		expected += "lock bits: 0xfc\n";
		assert(ok && text == expected);
		assert(fuse_reads == (small ? std::vector<uint8_t>{0, 1, 3} : std::vector<uint8_t>{0, 1, 2, 3}));
		assert(write_pulses == 0 && response.empty());
	}
	reset();
	signature1 = 0x93; signature2 = 0x07;
	Options options;
	std::vector<Operation> operations;
	assert(parse_arguments({"avrhvpp", "--target", "atmega328p", "--query"}, &options, &operations));
	bool ok;
	std::string text = capture_operation(operations[0], connection, &ok);
	assert(ok && text.find("chip: ATmega8\n") != std::string::npos);
	assert(text.find("fuse ex") == std::string::npos);
	assert(options.target->name() == "atmega328p"); // Query does not change subsequent Flash target.
	reset();
	signature1 = 0x95; signature2 = 0x07; // Same first two bytes as 328P, different final byte.
	text = capture_operation(operations[0], connection, &ok);
	assert(!ok && text == "signature: 0x1e, 0x95, 0x07\nchip: unknown\n");
	assert(fuse_reads.empty() && write_pulses == 0);
	reset();
	invalid_signature_response = true;
	assert(capture_operation(operations[0], connection, &ok).empty() && !ok);
	assert(fuse_reads.empty() && write_pulses == 0);
	reset();
	signature1 = 0x95; signature2 = 0x0f;
	fail_fuse_read = true;
	text = capture_operation(operations[0], connection, &ok);
	assert(!ok && text == "signature: 0x1e, 0x95, 0x0f\nchip: ATmega328P\n"
		"page size: 128 bytes\npage count: 256\nflash size: 32768 bytes\n");
	assert(std::count(sent.begin(), sent.end(), CMD_READ_FUSE) == 1);
	assert(std::find(sent.begin(), sent.end(), CMD_READ_LOCK) == sent.end());
	assert(write_pulses == 0 && response.empty());
}

static void test_targets(Connection *connection)
{
	char directory[] = "/tmp/avrhvpp-targets-XXXXXX";
	assert(mkdtemp(directory));
	std::string page_path = std::string(directory) + "/page.bin";
	std::string hex_path = std::string(directory) + "/image.hex";
	std::string output_path = std::string(directory) + "/output.hex";
	for (char const *name : {"atmega8", "atmega48", "atmega88", "atmega168", "atmega328p"}) {
		auto target = create_target(name);
		auto const &definition = target->definition();
		uint16_t page_bytes = definition.flash_page_bytes;
		uint16_t page_count = definition.flash_page_count;
		uint16_t last_page = page_count - 1;
		uint32_t capacity = definition.flash_bytes();
		std::vector<uint8_t> payload(page_bytes, 0x5a);
		{
			std::ofstream page(page_path, std::ios::binary);
			page.write((char const *)payload.data(), payload.size());
			FILE *hex = fopen(hex_path.c_str(), "wb");
			assert(hex);
			uint8_t byte = 0x5a;
			assert(write_hex_record(hex, capacity - 1, 0, &byte, 1));
			assert(write_hex_record(hex, 0, 1, nullptr, 0));
			assert(fclose(hex) == 0);
		}
		std::vector<Operation> reference;
		Options reference_options;
		assert(parse_arguments({"avrhvpp", "-t", name, "--read-page", std::to_string(last_page), "output.bin",
			"--write-page", std::to_string(last_page), page_path, "--write-hex", hex_path, "--no-chip-erase"}, &reference_options, &reference));
		assert(reference.size() == 3 && reference[1].bytes == payload);
		assert(reference[2].bytes.size() == capacity && reference[2].page_mask.size() == page_count);
		assert(reference[2].page_mask[last_page] == 1 && reference[2].bytes.back() == 0x5a && !reference[2].chip_erase);
		Options trailing_options;
		std::vector<Operation> trailing;
		assert(parse_arguments({"avrhvpp", "--no-chip-erase", "--read-page", std::to_string(last_page), "output.bin",
			"--write-page", std::to_string(last_page), page_path, "--write-hex", hex_path, "--target", name}, &trailing_options, &trailing));
		for (size_t i = 0; i < reference.size(); ++i) {
			assert(reference[i].command == trailing[i].command && reference[i].data == trailing[i].data);
			assert(reference[i].bytes == trailing[i].bytes && reference[i].page_mask == trailing[i].page_mask);
			assert(reference[i].chip_erase == trailing[i].chip_erase);
			assert(trailing[i].target == trailing_options.target.get());
		}
		for (auto const &arguments : std::vector<std::vector<std::string>>{
			{"avrhvpp", "--read-page", "0", "output.bin"},
			{"avrhvpp", "--write-hex", hex_path},
			{"avrhvpp", "-t", name, "--read-page", std::to_string(page_count), "output.bin"},
			{"avrhvpp", "--read-page", "-1", "output.bin", "-t", name}}) {
			Options options;
			std::vector<Operation> operations;
			assert(!parse_arguments(arguments, &options, &operations));
		}
		reset();
		assert(hvpp_write_page(reference[1], connection));
		assert(std::equal(payload.begin(), payload.end(), flash.begin() + last_page * page_bytes));
		for (uint16_t page : {uint16_t(0), uint16_t(256 / definition.flash_page_words() - 1),
			uint16_t(256 / definition.flash_page_words()), last_page}) {
			uint32_t offset = (uint32_t)page * page_bytes;
			for (uint16_t i = 0; i < page_bytes; ++i) flash[offset + i] = (uint8_t)(page + i);
			std::vector<uint8_t> actual;
			assert(avr_fetch_page(page, *target, &actual, connection));
			assert(actual.size() == page_bytes && std::equal(actual.begin(), actual.end(), flash.begin() + offset));
		}
		reset();
		assert(hvpp_write_hex(reference[2], connection));
		assert(write_pulses == 1 && flash[capacity - 1] == 0x5a);
		assert(hvpp_write_hex(reference[2], connection));
		assert(write_pulses == 1); // Matching page skips writing.
		reset();
		Operation conflict = reference[2];
		conflict.page_mask.assign(page_count, 0);
		conflict.page_mask[0] = conflict.page_mask[1] = 1;
		conflict.bytes.assign(capacity, 0xff);
		conflict.bytes[0] = 0x11;
		conflict.bytes[page_bytes] = 0x22;
		flash[page_bytes] = 0x33;
		assert(!hvpp_write_hex(conflict, connection));
		assert(write_pulses == 0 && flash[0] == 0xff); // No partial writes before conflict.
		assert(std::find(sent.begin(), sent.end(), CMD_WRITE_PAGE) == sent.end());
		reset();
		flash.fill(0x33);
		Operation erased = reference[2];
		erased.chip_erase = true;
		assert(hvpp_write_hex(erased, connection));
		assert(write_pulses == 2 && flash[0] == 0xff && flash[capacity - 1] == 0x5a);
		reset();
		flash[0] = 0x12; flash[capacity - 1] = 0x34;
		Operation read_hex;
		read_hex.command = _CMD_READ_HEX;
		read_hex.target = target.get();
		read_hex.path = output_path;
		assert(hvpp_read_hex(read_hex, connection));
		std::vector<uint8_t> image, mask;
		assert(read_hex_file(output_path.c_str(), *target, &image, &mask));
		assert(image.size() == capacity && std::equal(image.begin(), image.end(), flash.begin()));
		for (uint32_t address : {capacity, capacity - 1}) {
			FILE *hex = fopen(hex_path.c_str(), "wb");
			assert(hex);
			uint8_t bytes[2] = {0, 1};
			assert(write_hex_record(hex, address, 0, bytes, 2));
			assert(write_hex_record(hex, 0, 1, nullptr, 0));
			assert(fclose(hex) == 0);
			assert(!read_hex_file(hex_path.c_str(), *target, &image, &mask));
		}
		{
			std::ofstream wrong_page(page_path, std::ios::binary);
			std::string wrong(page_bytes == 64 ? 128 : 64, 'x');
			wrong_page.write(wrong.data(), wrong.size());
		}
		assert(!read_page_file(page_path.c_str(), *target, &image));
		FlashParameters parameters;
		assert(make_flash_parameters(last_page, *target, &parameters));
		assert(parameters.byte_count == page_bytes && parameters.word_address == (capacity - page_bytes) / 2);
		assert(!make_flash_parameters(page_count, *target, &parameters));
	}
	std::remove(page_path.c_str());
	std::remove(hex_path.c_str());
	std::remove(output_path.c_str());
	assert(rmdir(directory) == 0);
}

int main()
{
	AVRHVPP firmware;
	device = &firmware;
	Connection connection;
	connection.mode_ = Connection::AVRHVPP;
	auto target = create_target("atmega328p");
	for (uint16_t size : {0, 1, 3, 257, 258, 512}) assert((!FlashParameters{0, size}.valid()));
	assert((FlashParameters{0xffff, 2}.valid()));
	assert((!FlashParameters{0xffff, 4}.valid()));
	assert((FlashParameters{0xff80, 256}.valid()));
	assert((!FlashParameters{0xff81, 256}.valid()));

	for (auto const &test : std::vector<std::pair<std::string, size_t>>{
		{"", 0}, {"0", 0}, {"E00", 0}, {"EG1", 0}, {"E1", 0}, {"11", 0}, {"1", 1}, {"GG", 1}, {"00", 16}}) {
		std::vector<uint8_t> data;
		ProtocolError error;
		assert(!decode_response({test.first.begin(), test.first.end()}, test.second, &data, &error));
	}
	for (unsigned int code = 1; code <= 6; ++code) {
		char text[4];
		sprintf(text, "E%02X", code);
		std::vector<uint8_t> data;
		ProtocolError error;
		assert(decode_response({text, text + 3}, 16, &data, &error));
		assert((unsigned int)error == code && data.empty());
	}
	{
		std::vector<uint8_t> data;
		ProtocolError error;
		assert(decode_response({'E', '0'}, 1, &data, &error));
		assert(error == ProtocolError::None && data[0] == 0xe0);
	}

	for (auto const &bytes : std::vector<std::vector<uint8_t>>{
		{}, {0}, {0, 0}, {0, 0, 2}, {0, 0, 2, 0, 0},
		{0, 0, 0, 0}, {0, 0, 3, 0}, {0, 0, 2, 1}, {255, 255, 4, 0}}) {
		reset();
		load(&connection, bytes);
		write_command(&connection, CMD_LATCH_FLASH_PARAMS);
		expect("E01\r\n");
		write_command(&connection, CMD_READ_PAGE);
		expect("E01\r\n");
		assert(write_pulses == 0);
	}
	reset();
	write_command(&connection, 0x7e); // Retired page-number command.
	expect("E06\r\n");
	write_command(&connection, CMD_READ_16BYTES);
	assert(read_ack(&connection, 10) == false); // Data is not an ACK.

	for (uint16_t size : {64, 128, 256}) {
		reset();
		assert(write_flash_parameters(0x100, size, &connection));
		std::vector<uint8_t> payload(size);
		for (uint16_t i = 0; i < size; ++i) payload[i] = (uint8_t)i;
		load(&connection, payload);
		write_command(&connection, CMD_WRITE_PAGE);
		assert(read_ack(&connection, 10));
		assert(write_pulses == 1);
		assert(std::equal(payload.begin(), payload.end(), flash.begin() + 0x200));
		assert(!(PORTC & (1 << 2)));
		assert(write_flash_parameters(0x100, size, &connection));
		load(&connection, payload);
		write_command(&connection, CMD_WRITE_PAGE);
		assert(read_ack(&connection, 10) && write_pulses == 1); // Matching data skips WR.
		assert(write_flash_parameters(0x100, size, &connection));
		write_command(&connection, CMD_READ_PAGE);
		assert(read_ack(&connection, 10));
		for (uint16_t i = 0; i < size; i += 16) {
			write_command(&connection, CMD_READ_16BYTES);
			std::vector<uint8_t> actual;
			assert(read_response(&connection, 16, &actual, 10));
			assert(std::equal(actual.begin(), actual.end(), payload.begin() + i));
		}
		if (size == 256) {
			write_command(&connection, CMD_READ_16BYTES);
			expect("E01\r\n"); // No wrap to buffer address zero.
		}
		write_command(&connection, CMD_READ_PAGE);
		expect("E01\r\n"); // Parameters are consumed.
	}

	reset();
	flash[0x1fe] = 1; flash[0x1ff] = 2; flash[0x200] = 3; flash[0x201] = 4;
	assert(write_flash_parameters(0xff, 4, &connection));
	write_command(&connection, CMD_READ_PAGE);
	assert(read_ack(&connection, 10));
	assert(flush_buffer[0] == 1 && flush_buffer[1] == 2 && flush_buffer[2] == 3 && flush_buffer[3] == 4);
	assert(write_flash_parameters(0xffff, 2, &connection));
	write_command(&connection, CMD_READ_PAGE);
	assert(read_ack(&connection, 10));
	for (uint16_t page : {3, 4, 255}) {
		reset();
		uint32_t offset = (uint32_t)page * 128;
		for (unsigned int i = 0; i < 128; ++i) flash[offset + i] = (uint8_t)(page + i);
		std::vector<uint8_t> actual;
		assert(avr_fetch_page(page, *target, &actual, &connection));
		assert(actual.size() == 128);
		assert(std::equal(actual.begin(), actual.end(), flash.begin() + offset));
	}
	reset();
	Operation last_page;
	last_page.command = CMD_WRITE_PAGE;
	last_page.target = target.get();
	last_page.data = 255;
	last_page.bytes.assign(128, 0xa5);
	assert(hvpp_write_page(last_page, &connection));
	assert(std::equal(last_page.bytes.begin(), last_page.bytes.end(), flash.begin() + 0x7f80));

	for (int failure = 0; failure < 4; ++failure) {
		reset();
		assert(write_flash_parameters(0, 64, &connection));
		load(&connection, std::vector<uint8_t>(failure == 0 ? 63 : 64, 0x5a));
		if (failure == 1) flash[0] = 0;
		if (failure == 2) timeout_on_write = true;
		if (failure == 3) corrupt_write = true;
		write_command(&connection, CMD_WRITE_PAGE);
		expect(failure == 0 ? "E01\r\n" : failure == 1 ? "E05\r\n" : failure == 2 ? "E03\r\n" : "E04\r\n");
		assert(write_pulses == (failure >= 2 ? 1U : 0U));
	}

	reset();
	assert(write_flash_parameters(0, 256, &connection));
	load(&connection, std::vector<uint8_t>(257, 0x5a));
	write_command(&connection, CMD_WRITE_PAGE);
	expect("E01\r\n");
	assert(write_pulses == 0);
	reset();
	assert(write_flash_parameters(0, 64, &connection));
	firmware.reset_protocol();
	write_command(&connection, CMD_READ_PAGE);
	expect("E01\r\n");
	reset();
	write_encoded_byte(&connection, 1, CMD_ADDR0, CMD_ADDR1);
	load(&connection, {0, 0, 64, 0});
	write_command(&connection, CMD_LATCH_FLASH_PARAMS);
	expect("E01\r\n");

	for (bool busy : {false, true}) {
		reset();
		assert(write_flash_parameters(0, 64, &connection));
		manufacturer = busy ? 0x1e : 0;
		ready = !busy;
		write_command(&connection, CMD_READ_PAGE);
		expect(busy ? "E03\r\n" : "E02\r\n");
	}
	reset();
	Operation lock;
	lock.command = CMD_WRITE_LOCK;
	lock.value = 0xfc;
	assert(hvpp_write_lock(lock, &connection));
	assert(lock_value == 0xfc && write_pulses == 1);
	reset();
	reject_latch = true;
	Operation page;
	page.command = CMD_WRITE_PAGE;
	page.target = target.get();
	page.bytes.assign(128, 0);
	assert(!hvpp_write_page(page, &connection));
	assert(std::find(sent.begin(), sent.end(), CMD_WRITE_PAGE) == sent.end());
	assert(write_pulses == 0);
	reset();
	write_command(&connection, CMD_READ_FUSE);
	expect("E01\r\n");
	load(&connection, {4, 0xff});
	write_command(&connection, CMD_WRITE_FUSE);
	expect("E01\r\n");
	assert(write_pulses == 0);
	reset();
	load(&connection, {2});
	write_command(&connection, CMD_READ_FUSE); // No chip-specific capability check.
	expect("FF\r\n");
	reset();
	assert(write_flash_parameters(0, 64, &connection));
	write_command(&connection, CMD_LATCH_FLASH_PARAMS); // Missing replacement invalidates prior parameters.
	expect("E01\r\n");
	write_command(&connection, CMD_READ_PAGE);
	expect("E01\r\n");
	for (uint8_t command : {uint8_t(CMD_READ_PAGE), uint8_t(0x7e), uint8_t(CMD_WRITE_FUSE)}) {
		reset();
		PORTC |= 1 << 2;
		DDRD = 0xff;
		write_command(&connection, command);
		expect(command == 0x7e ? "E06\r\n" : "E01\r\n");
		assert(write_pulses == 0);
	}
	for (uint8_t command : {uint8_t(CMD_READ_SIGNATURE), uint8_t(CMD_READ_FUSE), uint8_t(CMD_READ_LOCK),
		uint8_t(CMD_WRITE_FUSE), uint8_t(CMD_WRITE_LOCK), uint8_t(CMD_CHIP_ERASE),
		uint8_t(CMD_READ_PAGE), uint8_t(CMD_WRITE_PAGE)}) {
		for (bool busy : {false, true}) {
			reset();
			if (command == CMD_READ_PAGE || command == CMD_WRITE_PAGE) {
				assert(write_flash_parameters(0, 64, &connection));
				if (command == CMD_WRITE_PAGE) load(&connection, std::vector<uint8_t>(64, 0x5a));
			} else if (command == CMD_READ_FUSE || command == CMD_WRITE_LOCK) {
				load(&connection, {0});
			} else if (command == CMD_WRITE_FUSE) {
				load(&connection, {0, 0x62});
			}
			manufacturer = busy ? 0x1e : 0;
			ready = !busy;
			write_command(&connection, command);
			expect(busy ? "E03\r\n" : "E02\r\n");
			assert(write_pulses == 0);
		}
	}
	for (uint8_t command : {uint8_t(CMD_WRITE_FUSE), uint8_t(CMD_WRITE_LOCK), uint8_t(CMD_CHIP_ERASE)}) {
		reset();
		if (command == CMD_WRITE_FUSE) load(&connection, {0, 0x62});
		if (command == CMD_WRITE_LOCK) load(&connection, {0xfc});
		timeout_on_write = true;
		write_command(&connection, command);
		expect("E03\r\n");
		assert(write_pulses == 1);
	}
	test_targets(&connection);
	test_query(&connection);
	test_avrisp(&connection);
	test_avrisp_sessions(&connection);
	test_avrisp_variable_failures(&connection);
	test_avrisp_reads(&connection);
	test_avrisp_read_failures(&connection);
	test_avrisp_writes(&connection);
	test_avrisp_write_failures(&connection);
	test_avrisp_hex(&connection);
	test_avrisp_hex_failures(&connection);
	puts("Protocol integration and boundary tests passed without hardware.");
}
