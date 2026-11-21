// Drive the CLI through the real host UPDI encoder and a simulated serial bridge.
#define main avrhvpp_main
#include "../app/main.cpp"
#undef main
#include <algorithm>
#include <array>
#include <cassert>
#include <deque>
#include <unistd.h>

namespace {
constexpr uint16_t FlashBase = 0x8000;
constexpr uint16_t NvmBase = 0x1000;
constexpr uint16_t FuseBase = 0x1280;

struct Bridge {
	enum class Key { None, Program, Erase };
	std::vector<uint8_t> flash;
	uint32_t flash_base;
	uint16_t fuse_base;
	uint16_t sigrow;
	uint16_t nvm_status;
	uint8_t nvm_version;
	uint8_t nvm_mode = 0;
	uint32_t pointer = 0;
	unsigned repeat = 0;
	std::array<uint8_t, 64> page_buffer;
	std::array<uint8_t, 3> signature = {0x1e, 0x91, 0x23};
	std::array<uint8_t, 12> fuses = {0, 0, 2, 0, 0, 0xf6, 7, 0, 0};
	std::deque<uint8_t> replies;
	std::vector<uint8_t> frame;
	std::vector<uint8_t> commands;
	Key key = Key::None;
	uint8_t low = 0;
	uint8_t fuse_data = 0;
	uint8_t fuse_setup_bytes = 0;
	uint32_t unlocked_value;
	uint32_t lock_fuse;
	uint16_t fuse_address = 0;
	uint32_t page_address = 0;
	uint32_t store_address = 0;
	uint8_t store_size = 0;
	uint32_t fail_read_address = 0;
	uint32_t fail_store_address = 0;
	bool locked = false;
	bool programming = false;
	bool reset = false;
	bool bridge = false;
	bool fail_disable = false;
	bool corrupt_program = false;
	bool nvm_stuck_busy = false;
	bool ignore_lock_on_reset = false;
	uint8_t nvm_error = 0;
	int opens = 0;
	int breaks = 0;
	int disables = 0;
	int unlocks = 0;
	int busy_polls = 0;

	explicit Bridge(uint8_t version = 0)
		: flash(version ? 0x10000 : 0x800, 0xff), flash_base(version ? 0x800000 : FlashBase),
		  fuse_base(version ? 0x1050 : FuseBase), sigrow(version == 4 ? 0x1080 : 0x1100),
		  nvm_status(NvmBase + (version == 4 ? 6 : 2)), nvm_version(version),
		  unlocked_value(version ? 0x5cc5c55c : 0xc5), lock_fuse(unlocked_value)
	{
		page_buffer.fill(0xff);
		if (version) {
			fuses[2] = 0;
			fuses[5] = 0xd0;
			fuses[6] = 8;
			fuses[10] = 3;
		}
	}

	void erase_flash()
	{
		std::fill(flash.begin(), flash.end(), 0xff);
	}

	void nvm_command(uint8_t command)
	{
		commands.push_back(command);
		busy_polls = 1;
		if (nvm_version) {
			assert(nvm_mode == 0 || command == 0);
			assert(command == 0 || command == 0x02 || command == 0x13 || command == 0x20);
			nvm_mode = command;
			if (command == 0x20) erase_flash();
			return;
		}
		switch (command) {
		case 0x01:
			assert(page_address >= FlashBase);
			assert((page_address - FlashBase) % page_buffer.size() == 0);
			std::copy(page_buffer.begin(), page_buffer.end(), flash.begin() + page_address - FlashBase);
			if (corrupt_program) flash[page_address - FlashBase] ^= 1;
			break;
		case 0x04: page_buffer.fill(0xff); break;
		case 0x05: erase_flash(); break;
		case 0x07:
			assert(fuse_setup_bytes == 3);
			if (fuse_address == FuseBase + 0x0a) lock_fuse = fuse_data;
			else fuses.at(fuse_address - FuseBase) = fuse_data;
			fuse_setup_bytes = 0;
			break;
		default: assert(false);
		}
	}

	void write_memory()
	{
		assert(programming && !locked);
		uint8_t value = frame[0];
		if (store_address == NvmBase) {
			assert(store_size == 1);
			nvm_command(value);
		} else if (store_address >= flash_base) {
			assert(store_size == 2);
			size_t offset = store_address - flash_base;
			if (nvm_version) {
				assert(nvm_mode == 0x02);
				flash.at(offset) &= value;
				flash.at(offset + 1) &= frame[1];
			} else {
				page_address = flash_base + (offset / page_buffer.size()) * page_buffer.size();
				page_buffer[offset % page_buffer.size()] = value;
				page_buffer[(offset + 1) % page_buffer.size()] = frame[1];
			}
		} else if (nvm_version) {
			assert(store_size == 1 && nvm_mode == 0x13);
			if (store_address >= 0x1040 && store_address <= 0x1043) {
				unsigned shift = (store_address - 0x1040) * 8;
				lock_fuse = (lock_fuse & ~(uint32_t(0xff) << shift)) | (uint32_t(value) << shift);
			} else {
				fuses.at(store_address - fuse_base) = value;
			}
		} else if (store_address == NvmBase + 6) {
			assert(store_size == 1 && fuse_setup_bytes == 2);
			fuse_data = value;
			++fuse_setup_bytes;
		} else if (store_address == NvmBase + 8) {
			assert(store_size == 1 && fuse_setup_bytes == 0);
			fuse_address = value;
			++fuse_setup_bytes;
		} else if (store_address == NvmBase + 9) {
			assert(store_size == 1 && fuse_setup_bytes == 1);
			fuse_address |= uint16_t(value) << 8;
			++fuse_setup_bytes;
		} else {
			assert(false);
		}
	}

	void read_memory(uint32_t address)
	{
		assert(programming && !locked);
		if (address == fail_read_address) return;
		if (address >= flash_base) replies.push_back(flash.at(address - flash_base));
		else if (address >= sigrow && address < sigrow + signature.size()) replies.push_back(signature.at(address - sigrow));
		else if (address >= fuse_base) replies.push_back(fuses.at(address - fuse_base));
		else if (address == nvm_status) {
			replies.push_back(nvm_stuck_busy || busy_polls > 0 ? 1 : nvm_error);
			if (busy_polls > 0) --busy_polls;
		} else assert(false);
	}

	void control(uint8_t reg, uint8_t value)
	{
		if (reg == 3 && value == 0x0c) {
			++disables;
			if (fail_disable) replies.clear();
			key = Key::None;
			return;
		}
		if (reg != 8) return;
		if (value == 0x59) {
			reset = true;
			programming = false;
		} else {
			assert(value == 0);
			if (reset && lock_fuse != unlocked_value && !ignore_lock_on_reset) locked = true;
			if (reset && key == Key::Program && !locked) programming = true;
			if (reset && key == Key::Erase) {
				lock_fuse = unlocked_value;
				locked = false;
				erase_flash();
				++unlocks;
			}
			if (reset) nvm_mode = 0;
			reset = false;
			key = Key::None;
		}
	}

	void updi(uint8_t byte)
	{
		replies.push_back(byte);
		frame.push_back(byte);
		if (store_size) {
			if (frame.size() < store_size) return;
			write_memory();
			store_size = 0;
			replies.push_back(0x40);
			frame.clear();
			return;
		}
		assert(frame[0] == 0x55);
		if (frame.size() == 1) return;
		uint8_t command = frame[1];
		if ((command & 0xf0) == 0x80) {
			uint8_t value = 0;
			switch (command & 0x0f) {
			case 0: value = 0x20; break;
			case 7: value = key == Key::Program ? 0x10 : key == Key::Erase ? 0x08 : 0; break;
			case 0x0b: value = (locked ? 1 : 0) | (programming ? 8 : 0) | (reset ? 0x20 : 0); break;
			default: assert(false);
			}
			replies.push_back(value);
		} else if ((command & 0xf0) == 0xc0) {
			if (frame.size() < 3) return;
			control(command & 0x0f, frame[2]);
		} else if (command == 0xe0) {
			if (frame.size() < 10) return;
			std::string received(frame.rbegin(), frame.rbegin() + 8);
			if (received == "NVMProg ") key = Key::Program;
			else if (received == "NVMErase") key = Key::Erase;
			else assert(false);
		} else if (command == 0xe6) {
			std::string sib = nvm_version ? "AVR     P:2D:1-3M2 " : "tinyAVR P:0D:0-3M2 (02.59B14.0)";
			sib[10] = '0' + nvm_version;
			sib.resize(32, ' ');
			replies.insert(replies.end(), sib.begin(), sib.end());
		} else if (command == 0x6a) {
			assert(nvm_version);
			if (frame.size() < 5) return;
			pointer = frame[2] | (uint32_t(frame[3]) << 8) | (uint32_t(frame[4]) << 16);
			replies.push_back(0x40);
		} else if (command == 0xa0) {
			assert(nvm_version && !repeat);
			if (frame.size() < 3) return;
			repeat = unsigned(frame[2]) + 1;
		} else if (command == 0x24) {
			assert(nvm_version && repeat && repeat <= 64);
			while (repeat > 0) {
				read_memory(pointer++);
				--repeat;
			}
		} else if (command == 0x04 || command == 0x44 || command == 0x45
			|| command == 0x08 || command == 0x48 || command == 0x49) {
			bool wide = command & 0x08;
			assert(wide == bool(nvm_version));
			if (frame.size() < (wide ? 5 : 4)) return;
			uint32_t address = frame[2] | (uint32_t(frame[3]) << 8);
			if (wide) address |= uint32_t(frame[4]) << 16;
			if (!(command & 0x40)) read_memory(address);
			else {
				store_address = address;
				store_size = (command & 1) ? 2 : 1;
				if (address != fail_store_address) replies.push_back(0x40);
			}
		} else assert(false);
		frame.clear();
	}

	void write(uint8_t const *bytes, int size)
	{
		if (size == 9 && memcmp(bytes, ".AVRUPDI.", 9) == 0) {
			bridge = true;
			std::string reply = "AVRUPDI\r\n";
			replies.insert(replies.end(), reply.begin(), reply.end());
			return;
		}
		for (int i = 0; i < size; ++i) {
			uint8_t byte = bytes[i];
			if (byte == '0') {
				bridge = false;
				frame.clear();
				store_size = 0;
			} else if (byte == ' ') {
				replies.push_back(0x14);
				replies.push_back(0x10);
			} else {
				assert(bridge);
				if (byte == CMD_UPDI_BREAK) ++breaks;
				else if (byte >= CMD_LOAD_7BITS) low = byte & 0x7f;
				else {
					assert(byte == CMD_UPDI_SEND0 || byte == CMD_UPDI_SEND1);
					updi(low | (byte == CMD_UPDI_SEND1 ? 0x80 : 0));
				}
			}
		}
	}
};

Bridge bridge;

int run(std::initializer_list<std::string> args, char const *target = "attiny202")
{
	std::vector<std::string> options = {"avrhvpp", "-p", "mock", "-d", "updi", "-t", target};
	options.insert(options.end(), args.begin(), args.end());
	std::vector<char *> argv;
	for (auto &option : options) argv.push_back(option.data());
	return avrhvpp_main(argv.size(), argv.data());
}

std::string read_file(std::string const &path)
{
	std::ifstream input(path, std::ios::binary);
	return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}
}

Serial::Serial() = default;
bool Serial::open(Option *) { ++bridge.opens; return true; }
void Serial::close() {}
int Serial::write(void const *input, int size)
{
	auto const *bytes = static_cast<uint8_t const *>(input);
	// Fragment only the encoded UPDI stream. Mode negotiation uses the existing Connection code.
	if (size >= 2 && bytes[0] >= CMD_LOAD_7BITS) size = 1;
	bridge.write(bytes, size);
	return size;
}
int Serial::read(void *output, int size, int)
{
	if (bridge.bridge) size = std::min(size, 1);
	int received = std::min(size, static_cast<int>(bridge.replies.size()));
	auto *bytes = static_cast<uint8_t *>(output);
	for (int i = 0; i < received; ++i) {
		bytes[i] = bridge.replies.front();
		bridge.replies.pop_front();
	}
	return received;
}

int main()
{
	char directory[] = "/tmp/avrhvpp-updi-tests.XXXXXX";
	assert(mkdtemp(directory));
	std::string root(directory);
	std::string page = root + "/page.bin", hex = root + "/image.hex", dump = root + "/dump.bin";

	bridge.flash.back() = 0x42;
	assert(run({"--debug", "--query", "--read-lock", "--read-page", "31", dump}) == 0);
	assert(bridge.breaks == 1 && bridge.disables == 1 && !bridge.bridge && !bridge.programming);
	assert(read_file(dump) == std::string(63, char(0xff)) + char(0x42));
	bridge.fail_read_address = FlashBase + 63;
	assert(run({"--read-page", "0", dump, "--chip-erase"}) == 1);
	assert(read_file(dump) == std::string(63, char(0xff)) + char(0x42));
	assert(bridge.commands.empty() && !bridge.programming);

	bridge = Bridge{};
	std::ofstream(page, std::ios::binary) << std::string(64, 'A');
	assert(run({"--write-page", "0", page, "--write-page", "0", page, "--read-hex", hex}) == 0);
	assert((bridge.commands == std::vector<uint8_t>{0x04, 0x01}));
	assert(read_file(hex).find(":10000000") == 0);
	bridge.flash[0] = 'B';
	assert(run({"--no-chip-erase", "--write-hex", hex}) == 1);
	assert(bridge.commands.size() == 2);
	assert(run({"--write-hex", hex}) == 0);
	assert((bridge.commands == std::vector<uint8_t>{0x04, 0x01, 0x05, 0x04, 0x01}));
	assert(run({"--write-page", "1", page, "--read-hex", hex}) == 0);
	std::fill_n(bridge.flash.begin(), 64, 0xff);
	bridge.flash[64] = 'B';
	size_t command_count = bridge.commands.size();
	assert(run({"--no-chip-erase", "--write-hex", hex}) == 1);
	assert(bridge.flash[0] == 0xff && bridge.commands.size() == command_count);

	bridge = Bridge{};
	bridge.signature[2] = 0x24;
	assert(run({"--chip-erase"}) == 1);
	assert(bridge.commands.empty() && !bridge.programming);
	bridge = Bridge{};
	bridge.corrupt_program = true;
	assert(run({"--write-page", "0", page, "--chip-erase"}) == 1);
	assert((bridge.commands == std::vector<uint8_t>{0x04, 0x01}));
	bridge = Bridge{};
	bridge.fail_store_address = NvmBase;
	assert(run({"--chip-erase", "--write-page", "0", page}) == 1);
	assert(bridge.commands.empty() && !bridge.bridge);
	bridge = Bridge{};
	bridge.nvm_error = 4;
	assert(run({"--chip-erase"}) == 1);
	assert(bridge.commands.empty());
	bridge = Bridge{};
	bridge.nvm_stuck_busy = true;
	assert(run({"--chip-erase"}) == 1);
	assert(bridge.commands.empty() && !bridge.programming);

	bridge = Bridge{};
	assert(run({"--write-fuse", "SYSCFG0", "0xf7", "--read-fuse", "SYSCFG0"}) == 0);
	assert(bridge.fuses[5] == 0xf7 && (bridge.commands == std::vector<uint8_t>{0x07}));
	assert(run({"--write-fuse", "SYSCFG0", "0xf3"}) == 1);
	assert(bridge.fuses[5] == 0xf7);
	int opens = bridge.opens;
	assert(run({"--read-fuse", "RESERVED"}) == 1 && bridge.opens == opens);
	assert(run({"--write-fuse", "OSCCFG", "0x03"}) == 1 && bridge.opens == opens);
	assert(run({"--read-page", "32", dump}) == 1 && bridge.opens == opens);
	assert(run({"--write-lock", "0xc5"}) == 1 && bridge.opens == opens);
	assert(run({"--write-lock", "0x00", "--read-lock"}) == 1 && bridge.opens == opens);
	assert(run({"--write-lock", "0x00", "--write-lock", "0x00"}) == 1 && bridge.opens == opens);

	bridge = Bridge{};
	bridge.flash[0] = 0;
	assert(run({"--erase-locked"}) == 1);
	assert(bridge.unlocks == 0 && bridge.flash[0] == 0);
	assert(run({"--read-signature", "--write-lock", "0x00"}) == 0);
	assert(bridge.locked && bridge.lock_fuse == 0 && !bridge.programming && !bridge.bridge);
	assert((bridge.commands == std::vector<uint8_t>{0x07}));
	assert(run({"--read-lock"}) == 0);
	assert(run({"--write-lock", "0x01"}) == 1);
	assert(run({"--write-hex", hex}) == 1);
	assert(bridge.locked && bridge.flash[0] == 0 && bridge.commands.size() == 1);
	assert(run({"--erase-locked", "--read-signature"}) == 0);
	assert(!bridge.locked && bridge.lock_fuse == 0xc5 && bridge.unlocks == 1 && bridge.flash[0] == 0xff);

	bridge = Bridge{};
	bridge.ignore_lock_on_reset = true;
	assert(run({"--write-lock", "0x00"}) == 1);
	assert((bridge.commands == std::vector<uint8_t>{0x07}));
	assert(!bridge.bridge && bridge.disables == 1);

	bridge = Bridge{};
	bridge.fail_disable = true;
	assert(run({"--read-signature"}) == 1);
	assert(!bridge.bridge && bridge.disables == 1);

	struct Part {
		char const *name;
		uint8_t version;
		uint8_t signature;
	};
	for (auto const &part : {Part{"avr64dd28", 2, 0x1b}, Part{"avr64dd32", 2, 0x1a}, Part{"avr64du32", 4, 0x21}}) {
		bridge = Bridge{part.version};
		bridge.signature = {0x1e, 0x96, part.signature};
		std::string payload = std::string(510, char(0xff)) + char(0x34) + char(0x12);
		std::ofstream(page, std::ios::binary | std::ios::trunc) << payload;
		assert(run({"--query", "--write-page", "127", page, "--read-page", "127", dump}, part.name) == 0);
		assert(read_file(dump) == payload);
		assert(bridge.flash.back() == 0x12 && bridge.flash[0x7fff] == 0xff);
		assert((bridge.commands == std::vector<uint8_t>{0x02, 0}));
		assert(run({"--read-hex", hex}, part.name) == 0);
		assert(read_file(hex).find(":10FFF000") == 0);
		assert(run({"--no-chip-erase", "--write-hex", hex}, part.name) == 0);
		assert(bridge.commands.size() == 2);
		assert(run({"--write-hex", hex}, part.name) == 0);
		assert((bridge.commands == std::vector<uint8_t>{0x02, 0, 0x20, 0, 0x02, 0}));
		assert(run({"--write-fuse", "SYSCFG0", "0xd1"}, part.name) == 0 && bridge.fuses[5] == 0xd1);
		assert(run({"--write-fuse", "SYSCFG0", "0xc1"}, part.name) == 1 && bridge.fuses[5] == 0xd1);
		int connection_count = bridge.opens;
		if (part.version == 4) {
			assert(run({"--write-fuse", "PDICFGL", "0"}, part.name) == 1 && bridge.opens == connection_count);
			bridge.fuses[10] = 0x52;
			bridge.fuses[11] = 0xb4;
			size_t prior_commands = bridge.commands.size();
			assert(run({"--write-lock", "0"}, part.name) == 1);
			assert(bridge.commands.size() == prior_commands && !bridge.locked);
			bridge.fuses[10] = 3;
			bridge.fuses[11] = 0;
			connection_count = bridge.opens;
		}
		assert(run({"--write-lock", "0x5cc5c55c"}, part.name) == 1 && bridge.opens == connection_count);
		assert(run({"--write-lock", "0x100000000"}, part.name) == 1 && bridge.opens == connection_count);
		assert(run({"--write-lock", "0x12345678"}, part.name) == 0);
		assert(bridge.locked && bridge.lock_fuse == 0x12345678);
		assert(run({"--erase-locked"}, part.version == 2 ? "avr64du32" : "avr64dd28") == 1);
		assert(bridge.locked && bridge.unlocks == 0);
		assert(run({"--erase-locked", "--read-lock"}, part.name) == 0);
		assert(!bridge.locked && bridge.unlocks == 1 && bridge.flash.back() == 0xff);
		bridge.nvm_error = 0x40;
		assert(run({"--chip-erase"}, part.name) == 1);
		bridge.signature[2] ^= 1;
		size_t prior_commands = bridge.commands.size();
		assert(run({"--chip-erase"}, part.name) == 1);
		assert(bridge.commands.size() == prior_commands);
	}

	assert(unlink(page.c_str()) == 0 && unlink(hex.c_str()) == 0 && unlink(dump.c_str()) == 0);
	assert(rmdir(directory) == 0);
	puts("UPDI CLI and NVM tests passed without hardware.");
}
