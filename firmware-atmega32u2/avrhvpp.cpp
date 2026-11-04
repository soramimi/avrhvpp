#include "avrhvpp.h"
#include "waitloop.h"
#include <string.h>

#include <avr/pgmspace.h>

#include "../common/commands.h"
#include "../common/target.h"

extern uint8_t flush_buffer[256];

static_assert(Target::FlashPageBytes <= sizeof(flush_buffer), "flash page does not fit in buffer");

AVRHVPP::AVRHVPP()
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

AVRHVPP::~AVRHVPP()
{
	set_programming_mode(false);
}

bool AVRHVPP::set_programming_mode(bool f)
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
		msleep(1); // datasheet: wait at least 300us before giving commands
		if (!wait_ready()) return false;

		// RDY/BSY may float high while not in programming mode,
		// so confirm by the manufacturer code (0x1E = Atmel/Microchip)
		return read_signature(0) == 0x1e;
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
		return true;
	}
}

void AVRHVPP::write_data_pins(bool iscommand, uint8_t value)
{
	write_data_pins(value);
	set_data_pin_mode(PinMode::Output);
	write_xa1(iscommand);
	pulse_xtal1();
	write_xa1(0);
	set_data_pin_mode(PinMode::Input);
}

bool AVRHVPP::read_rdybsy()
{
	return pin_rdybsy_.read();
}

bool AVRHVPP::wait_ready()
{
	for (int i = 0; i < 1000; i++) {
		if (pin_rdybsy_.read()) {
			return true;
		}
		msleep(1);
	}
	return false;
}

void AVRHVPP::write_oe(bool b)
{
	pin_oe_.write(b);
}

void AVRHVPP::write_wr(bool b)
{
	pin_wr_.write(b);
}

void AVRHVPP::write_bs1(bool b)
{
	pin_bs1_.write(b);
}

void AVRHVPP::write_bs2(bool b)
{
	pin_bs2_.write(b);
}

void AVRHVPP::write_xa0(bool b)
{
	pin_xa0_.write(b);
}

void AVRHVPP::write_xa1(bool b)
{
	pin_xa1_.write(b);
}

void AVRHVPP::write_pagel(bool b)
{
	pin_pagel_.write(b);
}

void AVRHVPP::write_xtal1(bool b)
{
	pin_xtal1_.write(b);
}

void AVRHVPP::write_reset12v(bool b)
{
	pin_reset12v_.write(b);
}

void AVRHVPP::set_data_pin_mode(PinMode mode)
{
	DDRD = (mode == PinMode::Output) ? 0xff : 0x00;
}

uint8_t AVRHVPP::read_data_pins()
{
	return PIND;
}

void AVRHVPP::write_data_pins(uint8_t c)
{
	PORTD = c;
}

void AVRHVPP::pulse_xtal1()
{
	usleep(1);
	write_xtal1(1);
	usleep(1);
	write_xtal1(0);
	usleep(1);
}

void AVRHVPP::pulse_wr()
{
	usleep(10);
	write_wr(0);
	usleep(10);
	write_wr(1);
	usleep(10);
}

void AVRHVPP::pulse_pagel()
{
	usleep(1);
	write_pagel(1);
	usleep(1);
	write_pagel(0);
	usleep(1);
}

void AVRHVPP::load_flash_command(uint8_t command)
{
	write_oe(1);
	write_xa1(1);
	write_xa0(0);
	write_bs1(0);
	write_bs2(0);
	write_data_pins(command);
	set_data_pin_mode(PinMode::Output);
	pulse_xtal1();
	set_data_pin_mode(PinMode::Input);
	write_xa1(0);
}

void AVRHVPP::load_flash_address(uint8_t address, bool high)
{
	write_oe(1);
	write_xa1(0);
	write_xa0(0);
	write_bs1(high);
	write_bs2(0);
	write_data_pins(address);
	set_data_pin_mode(PinMode::Output);
	pulse_xtal1();
	set_data_pin_mode(PinMode::Input);
}

void AVRHVPP::load_flash_data(uint8_t data, bool high)
{
	write_oe(1);
	write_xa1(0);
	write_xa0(1);
	write_bs1(high);
	write_bs2(0);
	write_data_pins(data);
	set_data_pin_mode(PinMode::Output);
	pulse_xtal1();
	set_data_pin_mode(PinMode::Input);
	write_xa0(0);
}

void AVRHVPP::read_flash_word(uint16_t address, uint8_t *low, uint8_t *high)
{
	load_flash_address((uint8_t)address, false);
	set_data_pin_mode(PinMode::Input);
	write_bs1(0);
	write_oe(0);
	usleep(1);
	*low = read_data_pins();
	write_bs1(1);
	usleep(1);
	*high = read_data_pins();
	write_oe(1);
	write_bs1(0);
	usleep(1);
}

void AVRHVPP::set_command(uint8_t c)
{
	write_data_pins(true, c);
}

void AVRHVPP::set_low_address(uint8_t c)
{
	write_data_pins(false, c);
}

uint8_t AVRHVPP::read_data()
{
	set_data_pin_mode(PinMode::Input);
	write_oe(0);
	usleep(1);
	int c = read_data_pins();
	write_oe(1);
	usleep(1);
	return c;
}

uint8_t AVRHVPP::read_signature(int addr)
{
	write_bs1(0);
	write_bs2(0);
	set_command(0x08);
	set_low_address(addr);
	usleep(1);
	return read_data();
}

bool AVRHVPP::matches_target_signature()
{
	return read_signature(0) == Target::Signature0
		&& read_signature(1) == Target::Signature1
		&& read_signature(2) == Target::Signature2;
}

bool AVRHVPP::read_flash_page(uint16_t page, uint8_t *data, uint16_t size)
{
	if (page >= Target::FlashPageCount || size != Target::FlashPageBytes) {
		return false;
	}

	uint16_t address = Target::flash_word_address(page, 0);
	load_flash_command(0x02);
	load_flash_address((uint8_t)(address >> 8), true);
	for (uint8_t word = 0; word < Target::FlashPageWords; word++) {
		read_flash_word(address + word, data + word * 2, data + word * 2 + 1);
	}
	return true;
}

bool AVRHVPP::compare_flash_page(uint16_t page, uint8_t const *data, uint16_t size, bool *matches, bool *blank)
{
	if (page >= Target::FlashPageCount || size != Target::FlashPageBytes) {
		return false;
	}

	*matches = true;
	*blank = true;
	uint16_t address = Target::flash_word_address(page, 0);
	load_flash_command(0x02);
	load_flash_address((uint8_t)(address >> 8), true);
	for (uint8_t word = 0; word < Target::FlashPageWords; word++) {
		uint8_t low;
		uint8_t high;
		read_flash_word(address + word, &low, &high);
		if (low != data[word * 2] || high != data[word * 2 + 1]) {
			*matches = false;
		}
		if (low != 0xff || high != 0xff) {
			*blank = false;
		}
	}
	return true;
}

bool AVRHVPP::write_flash_page(uint16_t page, uint8_t const *data, uint16_t size)
{
	if (page >= Target::FlashPageCount || size != Target::FlashPageBytes) {
		return false;
	}

	uint16_t address = Target::flash_word_address(page, 0);
	load_flash_command(0x10);
	for (uint8_t word = 0; word < Target::FlashPageWords; word++) {
		load_flash_address((uint8_t)(address + word), false);
		load_flash_data(data[word * 2], false);
		load_flash_data(data[word * 2 + 1], true);
		write_bs1(1);
		pulse_pagel();
	}
	load_flash_address((uint8_t)(address >> 8), true);
	pulse_wr();
	if (!wait_ready()) {
		return false;
	}
	load_flash_command(0x00);
	return true;
}

bool AVRHVPP::verify_flash_page(uint16_t page, uint8_t const *data, uint16_t size)
{
	bool matches;
	bool blank;
	return compare_flash_page(page, data, size, &matches, &blank) && matches;
}

void AVRHVPP::load_data_low(uint8_t c)
{
	write_xa1(0);
	write_xa0(1);
	write_bs1(0);
	write_data_pins(c);
	set_data_pin_mode(PinMode::Output);
	pulse_xtal1();
	set_data_pin_mode(PinMode::Input);
	write_xa0(0);
}

uint8_t AVRHVPP::read_fuse(Fuse fuse)
{
	bool bs1 = 0;
	bool bs2 = 0;
	if (fuse == Fuse::Low) {
		bs1 = 0;
		bs2 = 0;
	} else if (fuse == Fuse::High) {
		bs1 = 1;
		bs2 = 1;
	} else if (fuse == Fuse::Extended) {
		bs1 = 0;
		bs2 = 1;
	} else if (fuse == Fuse::Lock) {
		bs1 = 1;
		bs2 = 0;
	} else {
		return 0;
	}
	set_command(0x04);
	set_data_pin_mode(PinMode::Input);
	write_bs1(bs1);
	write_bs2(bs2);
	write_oe(false);
	usleep(1);
	return read_data();
}

bool AVRHVPP::write_lock(uint8_t value)
{
	write_bs1(0);
	write_bs2(0);
	set_command(0x20);
	load_data_low(value);
	pulse_wr();
	bool ok = wait_ready();
	write_bs1(0);
	write_bs2(0);
	return ok;
}

// low: (0,0), high: (1,0), extended: (0,1)
// note: BS1/BS2 assignment differs from read_fuse()
bool AVRHVPP::write_fuse(Fuse fuse, uint8_t value)
{
	bool bs1 = 0;
	bool bs2 = 0;
	if (fuse == Fuse::Low) {
		bs1 = 0;
		bs2 = 0;
	} else if (fuse == Fuse::High) {
		bs1 = 1;
		bs2 = 0;
	} else if (fuse == Fuse::Extended) {
		bs1 = 0;
		bs2 = 1;
	} else if (fuse == Fuse::Lock) {
		return write_lock(value);
	} else {
		return false;
	}
	write_bs1(0);
	write_bs2(0);
	set_command(0x40);
	load_data_low(value);
	write_bs1(bs1);
	write_bs2(bs2);
	pulse_wr();
	bool ok = wait_ready();
	write_bs1(0);
	write_bs2(0);
	return ok;
}

bool AVRHVPP::chip_erase()
{
	write_xa0(0);
	write_xa1(1);
	write_bs1(0);
	write_bs2(0);
	set_command(0x80);
	pulse_wr();
	return wait_ready();
}

uint8_t data_7bits_ = 0;
uint8_t addr_ = 0;

uint16_t page_num_ = 0;
bool page_num_valid_ = false;

uint16_t buffer_data_count_ = 0;
bool buffer_data_valid_ = false;

static inline uint8_t address()
{
	return flush_buffer[0];
}

static inline uint8_t writedata()
{
	return flush_buffer[1];
}

void write_hex(char *p, uint8_t c)
{
	static const char PROGMEM hex[]= "0123456789ABCDEF";
	p[0] = pgm_read_byte(hex + (c >> 4));
	p[1] = pgm_read_byte(hex + (c & 15));
}

void rewind() // don't clear page num
{
	addr_ = 0;
	buffer_data_count_ = 0;
	buffer_data_valid_ = true;
	memset(flush_buffer, 0, sizeof(flush_buffer));
}

void cmd_clear()
{
	data_7bits_ = 0;
	page_num_ = 0;
	page_num_valid_ = false;
	rewind();
}

void AVRHVPP::process(uint8_t c)
{
	auto SendACK = [this]() {
		write_string("1\r\n");
	};
	auto SendNACK = [this]() {
		write_string("0\r\n");
	};

	auto EnterProgrammingMode = [&]() {
		if (!set_programming_mode(true)) {
			set_programming_mode(false);
			SendNACK();
			return false;
		}
		return true;
	};

	auto AppendFlashBuffer = [&](uint8_t c) {
		if (addr_ < sizeof(flush_buffer)) {
			if (!buffer_data_valid_ || addr_ != buffer_data_count_) {
				buffer_data_valid_ = false;
			} else {
				buffer_data_count_++;
			}
			flush_buffer[addr_] = c;
			addr_++;
		}
	};

	if (c == CMD_DEBUG) {
#if 0
		if (EnterProgrammingMode()) {
			char tmp[20];
			write_hex(tmp + 0, read_signature(0));
			write_hex(tmp + 2, read_signature(1));
			write_hex(tmp + 4, read_signature(2));
			write_hex(tmp + 6, read_fuse(Fuse::Low));
			write_hex(tmp + 8, read_fuse(Fuse::High));
			write_hex(tmp + 10, read_fuse(Fuse::Extended));
			write_hex(tmp + 12, read_fuse(Fuse::Lock));
			tmp[14] = '\r';
			tmp[15] = '\n';
			tmp[16] = 0;
			set_programming_mode(false);
			write_string(tmp);
		}
#endif
	} else if (c == CMD_CLEAR) {
		cmd_clear();
	} else if (c == CMD_ADDR0) {
		addr_ = data_7bits_ & 0x7f;
		buffer_data_valid_ = false;
	} else if (c == CMD_ADDR1) {
		addr_ = (data_7bits_ & 0x7f) | 0x80;
		buffer_data_valid_ = false;
	} else if (c == CMD_DATA0) {
		AppendFlashBuffer(data_7bits_ & 0x7f);
	} else if (c == CMD_DATA1) {
		AppendFlashBuffer((data_7bits_ & 0x7f) | 0x80);
	} else if (c == CMD_LATCH_PAGE_NUM) {
		page_num_valid_ = false;
		if (buffer_data_valid_ && buffer_data_count_ == 2) {
			page_num_ = (uint16_t)flush_buffer[0]
				| ((uint16_t)flush_buffer[1] << 8);
			page_num_valid_ = true;
		}
		rewind();
	} else if (c == CMD_READ_SIGNATURE) {
		if (EnterProgrammingMode()) {
			char tmp[20];
			write_hex(tmp + 0, read_signature(0));
			write_hex(tmp + 2, read_signature(1));
			write_hex(tmp + 4, read_signature(2));
			tmp[6] = '\r';
			tmp[7] = '\n';
			tmp[8] = 0;
			set_programming_mode(false);
			write_string(tmp);
		}
	} else if (c == CMD_READ_FUSE) {
		uint8_t addr = address();
		Fuse fuse = (Fuse)addr;
		if (EnterProgrammingMode()) {
			char tmp[20];
			write_hex(tmp + 0, read_fuse(fuse));
			tmp[2] = '\r';
			tmp[3] = '\n';
			tmp[4] = 0;
			set_programming_mode(false);
			write_string(tmp);
		}
	} else if (c == CMD_WRITE_FUSE) {
		uint8_t addr = address();
		uint8_t value = writedata();
		if (EnterProgrammingMode()) {
			bool ok = write_fuse((Fuse)addr, value);
			set_programming_mode(false);
			if (ok) {
				SendACK();
			} else {
				SendNACK();
			}
		}
	} else if (c == CMD_READ_LOCK) {
		if (EnterProgrammingMode()) {
			char tmp[20];
			write_hex(tmp + 0, read_fuse(Fuse::Lock));
			tmp[2] = '\r';
			tmp[3] = '\n';
			tmp[4] = 0;
			set_programming_mode(false);
			write_string(tmp);
		}
	} else if (c == CMD_WRITE_LOCK) {
		uint8_t value = writedata();
		if (EnterProgrammingMode()) {
			bool ok = write_fuse(Fuse::Lock, value);
			set_programming_mode(false);
			if (ok) {
				SendACK();
			} else {
				SendNACK();
			}
		}
	} else if (c == CMD_CHIP_ERASE) {
		if (EnterProgrammingMode()) {
			bool ok = chip_erase();
			set_programming_mode(false);
			if (ok) {
				SendACK();
			} else {
				SendNACK();
			}
		}
	} else if (c == CMD_READ_PAGE) {
		uint16_t page = page_num_;
		bool parameters_valid = page_num_valid_ && page < Target::FlashPageCount;
		page_num_valid_ = false;
		if (!parameters_valid) {
			SendNACK();
			return;
		}
		if (EnterProgrammingMode()) {
			bool ok = matches_target_signature()
				&& read_flash_page(page, flush_buffer, Target::FlashPageBytes);
			set_programming_mode(false);
			if (ok) {
				addr_ = 0;
				SendACK();
			} else {
				SendNACK();
			}
		}
	} else if (c == CMD_WRITE_PAGE) {
		uint16_t page = page_num_;
		bool parameters_valid = page_num_valid_
			&& page < Target::FlashPageCount
			&& buffer_data_valid_
			&& buffer_data_count_ == Target::FlashPageBytes;
		page_num_valid_ = false;
		buffer_data_valid_ = false;
		buffer_data_count_ = 0;
		if (!parameters_valid) {
			SendNACK();
			return;
		}
		if (EnterProgrammingMode()) {
			bool matches = false;
			bool blank = false;
			bool ok = matches_target_signature()
				&& compare_flash_page(page, flush_buffer, Target::FlashPageBytes, &matches, &blank);
			if (ok && !matches) {
				ok = blank
					&& write_flash_page(page, flush_buffer, Target::FlashPageBytes)
					&& verify_flash_page(page, flush_buffer, Target::FlashPageBytes);
			}
			set_programming_mode(false);
			if (ok) {
				SendACK();
			} else {
				SendNACK();
			}
		}
	} else if (c == CMD_READ_16BYTES) {
		char tmp[35];
		for (uint8_t i = 0; i < 16; i++) {
			if (addr_ < sizeof(flush_buffer)) {
				write_hex(tmp + i * 2, flush_buffer[addr_]);
				addr_++;
			} else {
				tmp[i * 2 + 0] = '-';
				tmp[i * 2 + 1] = '-';
			}
		}
		tmp[32] = '\r';
		tmp[33] = '\n';
		tmp[34] = 0;
		write_string(tmp);
	} else if (c >= CMD_LOAD_7BITS) {
		data_7bits_ = c & 0x7f;
	}
}
