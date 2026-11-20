#ifndef CONNECTION_H
#define CONNECTION_H
#include "serial.h"
#include <vector>
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
	int write(void const *ptr, int len);
	
	int read(void *ptr, int len, int timeout);
	
	// read until '\n' or timeout. trailing CR/LF is removed.
	bool read_line(std::vector<char> *out, int timeout);
	bool sync();
public:
	Connection();
	
	~Connection();
	
	bool open(Serial::Option *option);
	
	void close();
	
	enum Mode {
		STK500,
		BITBANG,
		AVRHVPP,
		AVRUPDI,
	};
	Mode mode_ = STK500;
	Mode mode() const;
	
	bool enter_bitbang_mode();
	
	bool enter_avrhvpp_mode();
	
	bool enter_avrupdi_mode();
	
	int read_pin(int pin, bool pullup);
	
	bool write_pin(int pin, bool v);
};

#endif // CONNECTION_H
