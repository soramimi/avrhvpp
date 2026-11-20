#include "Connection.h"
#include <chrono>
#include "main.h"

int Connection::write(const void *ptr, int len)
{
	return serial_.write(ptr, len);
}

int Connection::read(void *ptr, int len, int timeout)
{
	return serial_.read(ptr, len, timeout);
}

bool Connection::read_line(std::vector<char> *out, int timeout)
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

bool Connection::sync()
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

Connection::Connection()
{
}

Connection::~Connection()
{
	close();
}

bool Connection::open(Serial::Option *option)
{
	return serial_.open(option);
}

void Connection::close()
{
	serial_.close();
}

Connection::Mode Connection::mode() const
{
	return mode_;
}

bool Connection::enter_bitbang_mode()
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

bool Connection::enter_avrhvpp_mode()
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

bool Connection::enter_avrupdi_mode()
{
	if (mode() == AVRUPDI) return true;
	
	static constexpr std::string_view command = ".AVRUPDI.";
	static constexpr std::string_view expect = "AVRUPDI";
	
	if (sync()) {
		serial_.write(command.data(), (int)command.size());
		
		std::vector<char> line;
		if (read_line(&line, MODE_ENTER_TIMEOUT_MS) && line == expect) {
			mode_ = AVRUPDI;
			return true;
		}
	}
	
	return false;
}

int Connection::read_pin(int pin, bool pullup)
{
	char c = char(pullup ? CMD_READ_PULLUP : CMD_READ) | (pin & 0x0f);
	char d = 0;
	serial_.write(&c, 1);
	int n = serial_.read(&d, 1, PIN_IO_TIMEOUT_MS);
	return n == 1 ? (unsigned char)d : -1;
}

bool Connection::write_pin(int pin, bool v)
{
	char c = char(v ? CMD_WRITE_HIGH : CMD_WRITE_LOW) | (pin & 0x0f);
	char d;
	serial_.write(&c, 1);
	int n = serial_.read(&d, 1, PIN_IO_TIMEOUT_MS);
	return n == 1;
}
