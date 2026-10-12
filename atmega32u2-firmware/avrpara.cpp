#include "avrpara.h"
#include "waitloop.h"

AVRPARA::AVRPARA()
{
	pin_reset12v_.write(0);
	pin_rdybsy_.read();
	pin_oe_.write(0);
	pin_wr_.write(0);
	pin_bs1_.write(0);
	pin_bs2_.write(0);
	pin_xa0_.write(0);
	pin_xa1_.write(0);
	pin_pagel_.write(0);
	pin_xtal1_.write(0);
	set_data_pin_mode(PinMode::Input);
	msleep(5);
}

AVRPARA::~AVRPARA()
{
	set_programming_mode(false);
}

void AVRPARA::set_programming_mode(bool f)
{
	if (f) {
		write_oe(1);
		write_wr(1);
		write_xa0(0);
		write_xa1(0);
		write_bs1(0);
		write_bs2(0);
		write_xtal1(0);
		write_pagel(0);
		set_data_pin_mode(PinMode::Output);
		
		for (int i = 0; i < 6; i++) {
			write_xtal1(1);
			usleep(1);
			write_xtal1(0);
			usleep(1);
		}
		
		write_reset12v(1);
		usleep(100);
		wait_ready();
		
	} else {
		write_reset12v(0);
		pin_rdybsy_.read();
		pin_oe_.read();
		pin_wr_.read();
		pin_bs1_.read();
		pin_bs2_.read();
		pin_xa0_.read();
		pin_xa1_.read();
		pin_pagel_.read();
		pin_xtal1_.read();
		set_data_pin_mode(PinMode::Input);
		msleep(1);
		pin_reset12v_.write(0);
	}
}

void AVRPARA::write_data_pins(bool iscommand, uint8_t value)
{
	write_data_pins(value);
	set_data_pin_mode(PinMode::Output);
	write_xa1(iscommand);
	pulse_xtal1();
	write_xa1(0);
	set_data_pin_mode(PinMode::Input);
}

bool AVRPARA::read_rdybsy()
{
	return pin_rdybsy_.read();
}

bool AVRPARA::wait_ready()
{
	for (int i = 0; i < 1000; i++) {
		if (pin_rdybsy_.read()) {
			return true;
		}
		msleep(1);
	}
	return false;
}

void AVRPARA::write_oe(bool b)
{
	pin_oe_.write(b);
}

void AVRPARA::write_wr(bool b)
{
	pin_wr_.write(b);
}

void AVRPARA::write_bs1(bool b)
{
	pin_bs1_.write(b);
}

void AVRPARA::write_bs2(bool b)
{
	pin_bs2_.write(b);
}

void AVRPARA::write_xa0(bool b)
{
	pin_xa0_.write(b);
}

void AVRPARA::write_xa1(bool b)
{
	pin_xa1_.write(b);
}

void AVRPARA::write_pagel(bool b)
{
	pin_pagel_.write(b);
}

void AVRPARA::write_xtal1(bool b)
{
	pin_xtal1_.write(b);
}

void AVRPARA::write_reset12v(bool b)
{
	pin_reset12v_.write(b);
}

void AVRPARA::set_data_pin_mode(PinMode mode)
{
	DDRD = (mode == PinMode::Output) ? 0xff : 0x00;
}

uint8_t AVRPARA::read_data_pins()
{
	return PIND;
}

void AVRPARA::write_data_pins(uint8_t c)
{
	PORTD = c;
}

void AVRPARA::pulse_xtal1()
{
	usleep(1);
	write_xtal1(1);
	usleep(1);
	write_xtal1(0);
	usleep(1);
}

void AVRPARA::pulse_wr()
{
	usleep(10);
	write_wr(0);
	usleep(10);
	write_wr(1);
	usleep(10);
}

void AVRPARA::set_command(uint8_t c)
{
	write_data_pins(true, c);
}

void AVRPARA::set_low_address(uint8_t c)
{
	write_data_pins(false, c);
}

uint8_t AVRPARA::read_data()
{
	set_data_pin_mode(PinMode::Input);
	write_oe(0);
	usleep(1);
	int c = read_data_pins();
	write_oe(1);
	usleep(1);
	return c;
}

uint8_t AVRPARA::read_signature(int addr)
{
	write_bs1(0);
	write_bs2(0);
	set_command(0x08);
	set_low_address(addr);
	usleep(1);
	return read_data();
}

uint8_t AVRPARA::read_fuse(bool bs1, bool bs2)
{
	set_command(0x04);
	set_data_pin_mode(PinMode::Input);
	write_bs1(bs1);
	write_bs2(bs2);
	write_oe(false);
	usleep(1);
	return read_data();
}

uint8_t AVRPARA::write_fuse(bool bs1, bool bs2, uint8_t value)
{
	set_command(0x40);
	write_data_pins(value);
	write_bs1(bs1);
	write_bs2(bs2);
	pulse_wr();
	write_bs1(0);
	write_bs2(0);
	return read_data();
}

void AVRPARA::chip_erase()
{
	write_xa0(0);
	write_xa1(1);
	write_bs1(0);
	write_bs2(0);
	set_command(0x80);
	pulse_wr();
	wait_ready();
}

void AVRPARA::process(uint8_t c)
{
	if (c == '@') {
		char tmp[20];
		auto WriteHex = [](char *p, uint8_t c){
			static char hex[] = "0123456789ABCDEF";
			p[0] = hex[c >> 4];
			p[1] = hex[c & 15];
		};
		set_programming_mode(true);
		chip_erase();
		msleep(100);
		WriteHex(tmp + 0, read_signature(0));
		WriteHex(tmp + 2, read_signature(1));
		WriteHex(tmp + 4, read_signature(2));
		write_fuse(0, 0, 0x62);
		write_fuse(1, 1, 0xd9);
		write_fuse(0, 1, 0xff);
		WriteHex(tmp + 6, read_fuse(0, 0));
		WriteHex(tmp + 8, read_fuse(1, 1));
		WriteHex(tmp + 10, read_fuse(0, 1));
		WriteHex(tmp + 12, read_fuse(1, 0));
		tmp[14] = '\r';
		tmp[15] = '\n';
		tmp[16] = 0;
		set_programming_mode(false);
		write_string(tmp);
	} else if (c == 0xe0) {
		char tmp[3];
		set_programming_mode(true);
		tmp[1] = 0xdf; // high
		tmp[0] = 0xef; // low
		write_fuse(1, 0, tmp[1]);
		write_fuse(0, 0, tmp[0]);
		tmp[14] = '\r';
		tmp[15] = '\n';
		tmp[16] = 0;
		set_programming_mode(false);
	} else if (c == 0xee) {
		set_programming_mode(false);
		chip_erase();
		write_string("1\r\n");
	}
}
