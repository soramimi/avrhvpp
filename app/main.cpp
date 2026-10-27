
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
	bool read_line(std::string *out, int timeout)
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

	bool enter_bitbang_mode()
	{
		static constexpr std::string_view command = ".BITBANG.";
		static constexpr std::string_view expect = "BITBANG";
		
		if (sync()) {
			serial_.write(command.data(), (int)command.size());
			
			std::string line;
			if (read_line(&line, MODE_ENTER_TIMEOUT_MS) && line == expect) {
				return true;
			}
		}
		
		return false;
	}
	
	bool enter_avrpara_mode()
	{
		static constexpr std::string_view command = ".AVRPARA.";
		static constexpr std::string_view expect = "AVRPARA";
		
		if (sync()) {
			serial_.write(command.data(), (int)command.size());

			std::string line;
			if (read_line(&line, MODE_ENTER_TIMEOUT_MS) && line == expect) {
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
	bool restore_fuse = false;
};


void main2(Options const &opts, Connection *conn)
{
	if (!conn->enter_avrpara_mode()) {
		fprintf(stderr, "failed to enter avrpara mode\n");
		return;
	}
	
	char c;
	std::string line;
	
	if (opts.restore_fuse) {
		c = (char)0xe0;
		conn->write(&c, 1);
		if (conn->read_line(&line, COMMAND_TIMEOUT_MS) && line == "1") {
			fprintf(stderr, "fuses restored\n");
		} else {
			fprintf(stderr, "failed to restore fuses\n");
			return;
		}
	}
	
	c = '@';
	conn->write(&c, 1);
	if (!conn->read_line(&line, COMMAND_TIMEOUT_MS)) {
		fprintf(stderr, "no response\n");
		return;
	}
	if (line.size() != 14) { // signature x3, fuse low/high/ext, lock
		fprintf(stderr, "failed to enter programming mode\n");
		return;
	}
	int n = 7;
	for (int i = 0; i < n; i++) {
		printf("%c%c", line[i * 2 + 0], line[i * 2 + 1]);
		putchar(i + 1 < n ? ' ' : '\n');
	}
}

void test(Options const &opts, Connection *conn)
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
		} else if (arg == "--restore-fuse") {
			opts.restore_fuse = true;
		} else if (arg == "-s") {
			if (argi < argc) {
				opts.serial_options.speed = atoi(argv[argi++]);
			} else {
				fprintf(stderr, "-s requires a speed value\n");
				return 1;
			}
		}
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
	
	if (1) {
		main2(opts, &conn);
	} else {
		test(opts, &conn);
	}
	
	conn.close();
	return 0;
}
