#ifndef AVRUPDI_H
#define AVRUPDI_H

#include <cstdlib>
#include <cstdint>

class Connection;

class AvrUpdi {
private:
	Connection *conn_ = nullptr;
	size_t offset_ = 0;
	
	void PrintByte(uint8_t c);
	
	int receive(void *ptr, int len, int timeout_ms);
	
	void receive_all(int timeout_ms);
	
	void send_break();
	
	void send_byte(uint8_t c);
	
	void send_bytes(void *p, int n);
	
	int default_timeout() const;
	
	int updi_LDS(uint16_t addr);
	
	int updi_LDCS(uint8_t addr, int timeout_ms);
	
	bool updi_STCS(uint8_t addr, uint8_t value);
	
	int read_control_status(uint8_t addr);
	
	bool write_control_status(uint8_t addr, uint8_t value);
	
	bool configure_and_check_updi_link();
	
	bool wait_for_reset(int timeout_ms);
public:
	void setup(Connection *conn);
	bool cmd_debug();
};

#endif // AVRUPDI_H
