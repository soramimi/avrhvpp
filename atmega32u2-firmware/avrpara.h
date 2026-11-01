#ifndef AVRPARA_H
#define AVRPARA_H

#include "avrgpio.h"


enum PinMode {
	Input,
	InputPullUp,
	InputPullDown,
	Output
};

class AVRPARA {
private:
	avr::GPIO<avr::B, 3> pin_rdybsy_;
	avr::GPIO<avr::B, 4> pin_pagel_;
	avr::GPIO<avr::B, 5> pin_xa1_;
	avr::GPIO<avr::B, 6> pin_xa0_;
	avr::GPIO<avr::B, 7> pin_xtal1_;
	avr::GPIO<avr::C, 2> pin_reset12v_;
	avr::GPIO<avr::C, 4> pin_oe_;
	avr::GPIO<avr::C, 5> pin_wr_;
	avr::GPIO<avr::C, 6> pin_bs1_;
	avr::GPIO<avr::C, 7> pin_bs2_;
	bool set_programming_mode(bool f);
	void write_data_pins(bool iscommand, uint8_t value);
	bool read_rdybsy();
	void write_oe(bool b);
	void write_wr(bool b);
	void write_bs1(bool b);
	void write_bs2(bool b);
	void write_xa0(bool b);
	void write_xa1(bool b);
	void write_pagel(bool b);
	void write_xtal1(bool b);
	void write_reset12v(bool b);
	void set_data_pin_mode(PinMode mode);
	uint8_t read_data_pins();
	void write_data_pins(uint8_t c);
	void pulse_xtal1();
	void pulse_wr();
	uint8_t read_data_();
private:
	void write_byte(char c);
	void write_string(const char *p);
public:
	AVRPARA();
	~AVRPARA();
	void process(uint8_t c);
	void reset_command_state();
	void set_command(uint8_t c);
	void set_low_address(uint8_t c);
	bool wait_ready();
	uint8_t read_data();
	uint8_t read_signature(int addr);
	void load_data_low(uint8_t c);

	enum class Fuse {
		Low,
		High,
		Extended,
		Lock,
	};

	uint8_t read_fuse(Fuse fuse);
	bool write_lock(uint8_t value);
	bool write_fuse(Fuse fuse, uint8_t value);

	bool chip_erase();
};

#endif // AVRPARA_H
