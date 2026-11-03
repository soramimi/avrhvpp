
#include "serial.h"
#include <chrono>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>
#include <cstring>
#include <assert.h>
#include <cerrno>
#include <cctype>
#include <cstdlib>
#include "../common/commands.h"
#include "../common/target.h"

#ifdef _WIN32
#else
#include <assert.h>
#include <unistd.h>
#endif

constexpr int BITBANG_MODE_TIMEOUT_MS = 10;
constexpr int PIN_IO_TIMEOUT_MS = 10;
constexpr int MODE_ENTER_TIMEOUT_MS = 100;
constexpr int COMMAND_TIMEOUT_MS = 1000;

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
	Serial::Option serial_options;
};

struct Operation {
	Command command;
	uint16_t data = 0;
	uint8_t value = 0;
	std::string path;
};

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

bool parse_page(char const *text, uint16_t *page)
{
	errno = 0;
	char *end;
	int base = text[0] == '0' && (text[1] == 'x' || text[1] == 'X') ? 16 : 10;
	unsigned long value = strtoul(text, &end, base);
	if (errno || end == text || *end || value >= Target::FlashPageCount) {
		return false;
	}
	*page = (uint16_t)value;
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

bool avr_read_status(Operation const &op, Connection *conn)
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

	std::vector<char> line;
	write_command(conn, CMD_CLEAR);
	write_encoded_byte(conn, (uint8_t)op.data, CMD_DATA0, CMD_DATA1);
	write_command(conn, op.command);
	if (!conn->read_line(&line, COMMAND_TIMEOUT_MS)) {
		fprintf(stderr, "no response\n");
		return false;
	}
	if (line.size() != (size_t)bytes * 2) {
		fprintf(stderr, "failed to enter programming mode\n");
		return false;
	}
	std::string text;
	int n = bytes;
	for (int i = 0; i < n; i++) {
		char tmp[10];
		tmp[0] = line[i * 2 + 0];
		tmp[1] = line[i * 2 + 1];
		tmp[2] = 0;
		int v = (int)strtol(tmp, nullptr, 16);
		if (i > 0) {
			text += ", ";
		}
		sprintf(tmp, "0x%02x", v);
		text += tmp;
	}
	puts(text.c_str());
	return true;
}

bool avr_write_fuse(Operation const &op, Connection *conn)
{
	write_command(conn, CMD_CLEAR);
	write_encoded_byte(conn, (uint8_t)op.data, CMD_DATA0, CMD_DATA1);
	write_encoded_byte(conn, op.value, CMD_DATA0, CMD_DATA1);
	write_command(conn, CMD_WRITE_FUSE);

	std::vector<char> line;
	if (!conn->read_line(&line, COMMAND_TIMEOUT_MS)) {
		fprintf(stderr, "no response\n");
		return false;
	}
	if (line == "1") {
		printf("fuse written: 0x%02x\n", op.value);
		return true;
	} else {
		fprintf(stderr, "failed to write fuse\n");
		return false;
	}
}

bool avr_write_lock(Operation const &op, Connection *conn)
{
	write_command(conn, CMD_CLEAR);
	write_encoded_byte(conn, op.value, CMD_DATA0, CMD_DATA1);
	write_command(conn, CMD_WRITE_LOCK);

	std::vector<char> line;
	if (!conn->read_line(&line, COMMAND_TIMEOUT_MS)) {
		fprintf(stderr, "no response\n");
		return false;
	}
	if (line == "1") {
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

	std::vector<char> line;
	if (!conn->read_line(&line, COMMAND_TIMEOUT_MS)) {
		fprintf(stderr, "no response\n");
		return false;
	}
	if (line == "1") {
		puts("chip erased");
		return true;
	}
	fprintf(stderr, "failed to erase chip\n");
	return false;
}

int hex_value(char c)
{
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	return -1;
}

bool avr_read_page(Operation const &op, Connection *conn)
{
	std::vector<char> line;
	std::vector<uint8_t> data;
	data.reserve(Target::FlashPageBytes);

	write_command(conn, CMD_CLEAR);
	write_encoded_byte(conn, (uint8_t)op.data, CMD_PAGE0, CMD_PAGE1);
	write_command(conn, CMD_READ_PAGE);
	if (!conn->read_line(&line, COMMAND_TIMEOUT_MS)) {
		fprintf(stderr, "no response\n");
		return false;
	}
	if (!(line == "1")) {
		fprintf(stderr, "failed to read flash page %u\n", (unsigned int)op.data);
		return false;
	}

	write_encoded_byte(conn, 0, CMD_ADDR0, CMD_ADDR1);
	for (int block = 0; block < Target::FlashPageBytes / 16; block++) {
		write_command(conn, CMD_READ_16BYTES);
		if (!conn->read_line(&line, COMMAND_TIMEOUT_MS)) {
			fprintf(stderr, "no response\n");
			return false;
		}
		if (line.size() != 32) {
			fprintf(stderr, "invalid page data\n");
			return false;
		}
		for (int i = 0; i < 16; i++) {
			int high = hex_value(line[i * 2]);
			int low = hex_value(line[i * 2 + 1]);
			if (high < 0 || low < 0) {
				fprintf(stderr, "invalid page data\n");
				return false;
			}
			data.push_back((uint8_t)((high << 4) | low));
		}
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

bool perform(Operation const &op, Connection *conn)
{
	if (!conn->enter_avrhvpp_mode()) {
		fprintf(stderr, "failed to enter avrhvpp mode\n");
		return false;
	}
	
	if (op.command == _CMD_DUMP_BUFFER) {
		std::vector<char> line;
		write_command(conn, CMD_CLEAR);
		write_encoded_byte(conn, 0, CMD_ADDR0, CMD_ADDR1);
		for (int i = 0; i < 16; i++) {
			write_command(conn, CMD_READ_16BYTES);
			if (!conn->read_line(&line, COMMAND_TIMEOUT_MS)) {
				fprintf(stderr, "no response\n");
				return false;
			}
			for (int j = 0; j < 16; j++) {
				char tmp[10];
				tmp[0] = line[j * 2 + 0];
				tmp[1] = line[j * 2 + 1];
				tmp[2] = 0;
				if (isxdigit((unsigned char)tmp[0]) && isxdigit((unsigned char)tmp[1])) {
					tmp[0] = toupper((unsigned char)tmp[0]);
					tmp[1] = toupper((unsigned char)tmp[1]);
				} else {
					tmp[0] = '-';
					tmp[1] = '-';
				}
				if (j > 0) {
					putchar(' ');
				}
				fwrite(tmp, 1, 2, stdout);
			}
			putchar('\n');
		}
		return true;
	} else if (op.command == CMD_WRITE_FUSE) {
		return avr_write_fuse(op, conn);
	} else if (op.command == CMD_WRITE_LOCK) {
		return avr_write_lock(op, conn);
	} else if (op.command == CMD_CHIP_ERASE) {
		return avr_chip_erase(conn);
	} else if (op.command == CMD_READ_PAGE) {
		return avr_read_page(op, conn);
	} else {
		return avr_read_status(op, conn);
	}
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

int main(int argc, char **argv)
{
	Options opts;
	opts.serial_options.speed = 115200;
	std::vector<Operation> operations;

	int argi = 1;
	while (argi < argc) {
		std::string_view arg = argv[argi++];
		if (arg == "-p") {
			if (argi < argc) {
				opts.serial_options.port = argv[argi++];
			} else {
				fprintf(stderr, "-p requires a port name\n");
				return 1;
			}
		} else if (arg == "--debug") {
			operations.push_back({CMD_DEBUG});
		} else if (arg == "-rs" || arg == "--read-signature") {
			operations.push_back({CMD_READ_SIGNATURE});
		} else if (arg == "-rfl" || arg == "--read-fuse-low") {
			operations.push_back({CMD_READ_FUSE, 0});
		} else if (arg == "-rfh" || arg == "--read-fuse-high") {
			operations.push_back({CMD_READ_FUSE, 1});
		} else if (arg == "-rfe" || arg == "--read-fuse-extended") {
			operations.push_back({CMD_READ_FUSE, 2});
		} else if (arg == "-rl" || arg == "--read-lock") {
			operations.push_back({CMD_READ_LOCK});
		} else if (arg == "--read-page") {
			uint16_t page;
			if (argi >= argc || !parse_page(argv[argi++], &page)) {
				fprintf(stderr, "--read-page requires a page number from 0 to %u\n", (unsigned int)(Target::FlashPageCount - 1));
				return 1;
			}
			if (argi >= argc) {
				fprintf(stderr, "--read-page requires an output file\n");
				return 1;
			}
			std::string path = argv[argi++];
			operations.push_back({CMD_READ_PAGE, page, 0, path});
		} else if (arg == "-wfl" || arg == "--write-fuse-low") {
			uint8_t value;
			if (argi >= argc || !parse_byte(argv[argi++], &value)) {
				fprintf(stderr, "%.*s requires a byte value\n", (int)arg.size(), arg.data());
				return 1;
			}
			operations.push_back({CMD_WRITE_FUSE, 0, value});
		} else if (arg == "-wfh" || arg == "--write-fuse-high") {
			uint8_t value;
			if (argi >= argc || !parse_byte(argv[argi++], &value)) {
				fprintf(stderr, "%.*s requires a byte value\n", (int)arg.size(), arg.data());
				return 1;
			}
			operations.push_back({CMD_WRITE_FUSE, 1, value});
		} else if (arg == "-wfe" || arg == "--write-fuse-extended") {
			uint8_t value;
			if (argi >= argc || !parse_byte(argv[argi++], &value)) {
				fprintf(stderr, "%.*s requires a byte value\n", (int)arg.size(), arg.data());
				return 1;
			}
			operations.push_back({CMD_WRITE_FUSE, 2, value});
		} else if (arg == "-wl" || arg == "--write-lock") {
			uint8_t value;
			if (argi >= argc || !parse_byte(argv[argi++], &value)) {
				fprintf(stderr, "%.*s requires a byte value\n", (int)arg.size(), arg.data());
				return 1;
			}
			operations.push_back({CMD_WRITE_LOCK, 0, value});
		} else if (arg == "--chip-erase") {
			operations.push_back({CMD_CHIP_ERASE});
		} else if (arg == "-s" || arg == "--speed") {
			if (argi < argc) {
				opts.serial_options.speed = atoi(argv[argi++]);
			} else {
				fprintf(stderr, "-s requires a speed value\n");
				return 1;
			}
		} else if (arg == "--dump-buffer") {
			operations.push_back({_CMD_DUMP_BUFFER});
		} else {
			fprintf(stderr, "unknown option: %.*s\n", (int)arg.size(), arg.data());
			return 1;
		}
	}

#ifdef _WIN32
	if (strncmp(opts.serial_options.port.c_str(), "COM", 3) == 0) {
		opts.serial_options.port = "\\\\.\\" + opts.serial_options.port;
	}
#else
#endif

	if (operations.empty()) {
		fprintf(stderr, "no command specified\n");
		return 1;
	}

	Connection conn;
	if (!conn.open(&opts.serial_options)) {
		fprintf(stderr, "failed to open %s\n", opts.serial_options.port.c_str());
		return 1;
	}

	for (Operation const &operation : operations) {
		if (!perform(operation, &conn)) {
			conn.close();
			return 1;
		}
	}

	conn.close();
	return 0;
}
