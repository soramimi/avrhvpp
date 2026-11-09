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
static int read_chunk, write_chunk, write_calls, fail_write_call, faults_left, open_calls;
static uint8_t fault_command;
static Fault fault;
static std::vector<uint8_t> packet, commands;
static std::deque<uint8_t> replies;

static void reset()
{
	enabled = true;
	programming = read_error = zero_write = false;
	read_chunk = 1;
	write_chunk = 1;
	write_calls = fail_write_call = faults_left = open_calls = 0;
	fault_command = 0;
	fault = Fault::None;
	packet.clear(); commands.clear(); replies.clear();
}

static int write(uint8_t const *bytes, int size)
{
	if (++write_calls == fail_write_call) return zero_write ? 0 : -1;
	int count = std::min(size, write_chunk);
	for (int i = 0; i < count; i++) {
		packet.push_back(bytes[i]);
		if (bytes[i] != 0x20) continue;
		if (packet.size() != 2) {
			replies.push_back(0x15);
			packet.clear();
			continue;
		}
		uint8_t command = packet[0];
		commands.push_back(command);
		packet.clear();
		if (command == 0x50) programming = true;
		if (command == 0x51) programming = false;
		std::vector<uint8_t> reply = {0x14};
		if (command == 0x75) {
			assert(programming);
			reply.insert(reply.end(), {0x1e, 0x95, 0x0f});
		}
		reply.push_back(0x10);
		if (command == fault_command && faults_left > 0) {
			--faults_left;
			switch (fault) {
			case Fault::Header: reply[0] = 0x15; break;
			case Fault::Status: reply.back() = 0x11; break;
			case Fault::Truncated: reply.resize(command == 0x75 ? 3 : 1); break;
			case Fault::Read: read_error = true; break;
			case Fault::None: break;
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
void Serial::close() {}
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
			assert(!programmer.read_signature(signature));
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
	assert(retrying.read_signature(signature));
	assert(isp_test::commands == (std::vector<uint8_t>{0x30, 0x30, 0x50, 0x75, 0x51}));
	for (int failed_call : {1, 2}) {
		isp_test::reset();
		isp_test::fail_write_call = failed_call;
		Avrisp programmer(connection, 3);
		assert(programmer.read_signature(signature));
		assert(!isp_test::programming);
	}
	for (int failed_call : {4, 6, 8}) {
		for (bool zero : {false, true}) {
			isp_test::reset();
			isp_test::fail_write_call = failed_call;
			isp_test::zero_write = zero;
			Avrisp programmer(connection, 3);
			assert(!programmer.read_signature(signature));
			assert(!isp_test::programming); // A partial Q is completed, then retried.
		}
	}
	for (auto const &arguments : std::vector<std::vector<std::string>>{
		{"avrhvpp", "-d", "typo", "--chip-erase"},
		{"avrhvpp", "--device"}, {"avrhvpp", "-d", "--read-signature"},
		{"avrhvpp", "-d", "avrisp", "--device", "hvpp", "-rs"},
		{"avrhvpp", "--device", "avrisp", "--query"},
		{"avrhvpp", "-rs", "--chip-erase", "-d", "avrisp"},
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
	puts("Protocol integration and boundary tests passed without hardware.");
}
