// USB-CDC ATMEGA32U2
// Copyright (C) 2022 S.Fuchita (@soramimi_jp)

#include "usb.h"
#include <avr/interrupt.h>
#include <string.h>
#include "avrgpio.h"
#include "avrhvpp.h"

extern "C" uint8_t usb_read_available_();
extern "C" uint8_t usb_read_byte_();

uint8_t data_tx_buffer[TX_EP_SIZE];
uint8_t data_tx_buffer_i;
uint8_t data_tx_buffer_n;

uint8_t data_rx_buffer[256];
int data_rx_buffer_i;
int data_rx_buffer_n;

extern "C" void clear_buffers()
{
	data_tx_buffer_i = 0;
	data_tx_buffer_n = 0;
	data_rx_buffer_i = 0;
	data_rx_buffer_n = 0;
}

void usb_poll_tx()
{
	uint8_t tmp[TX_EP_SIZE];
	while (1) {
		int n = data_tx_buffer_n;
		n = n < sizeof(tmp) ? n : sizeof(tmp);
		if (n == 0) break;
		for (uint8_t i = 0; i < data_tx_buffer_n; i++) {
			tmp[i] = data_tx_buffer[data_tx_buffer_i];
			data_tx_buffer_i = (data_tx_buffer_i + 1) % sizeof(data_tx_buffer);
		}
		usb_data_tx(tmp, n);
		data_tx_buffer_n -= n;
	}
}


static inline void usb_poll_rx()
{
	int space = sizeof(data_rx_buffer) - 1 - data_rx_buffer_n;
	while (space > 0) {
		uint8_t tmp[16];
		int n = sizeof(tmp);
		n = usb_data_rx(tmp, n < space ? n : space);
		if (n == 0) break;
		for (uint8_t i = 0; i < n; i++) {
			int j = (data_rx_buffer_i + data_rx_buffer_n) % sizeof(data_rx_buffer);
			data_rx_buffer[j] = tmp[i];
			data_rx_buffer_n++;
		}
		space -= n;
	}
}

void usb_poll()
{
	usb_poll_tx();
	usb_poll_rx();
}

static inline int usb_read_available()
{
	return data_rx_buffer_n + usb_read_available_();
}

static inline uint8_t usb_read_byte()
{
	for (int i = 0; i < 2; i++) {
		if (data_rx_buffer_n > 0) {
			uint8_t c = data_rx_buffer[data_rx_buffer_i];
			data_rx_buffer_i = (data_rx_buffer_i + 1) % sizeof(data_rx_buffer);
			data_rx_buffer_n--;
			return c;
		}
		usb_poll_rx();
	}
	return 0;
}

void usb_write_byte(char c)
{
	while (1) {
		if (data_tx_buffer_n < sizeof(data_tx_buffer)) {
			int8_t i = (data_tx_buffer_i + data_tx_buffer_n) % sizeof(data_tx_buffer);
			data_tx_buffer[i] = c;
			data_tx_buffer_n++;
			if (data_tx_buffer_n >= TX_EP_SIZE - 1) {
				usb_poll_tx();
			}
			return;
		}
		usb_poll();
	}
}

void usb_write_string(char const *p)
{
	while (*p) {
		usb_write_byte(*p);
		p++;
	}
}

void msleep(int ms)
{
	while (ms > 0) {
		usb_poll();
		_delay_ms(1);
		ms--;
	}
}

void usleep(int us)
{
	while (us >= 10) {
		_delay_us(10);
		us -= 10;
	}
	while (us > 0) {
		_delay_us(1);
		us--;
	}
}

#define CLOCK 16000000UL
#define SCALE 125
static unsigned short _scale = 0;
static volatile unsigned long _system_tick_count;
static volatile unsigned long _tick_count;
static volatile unsigned long _time_s;
static unsigned short _time_ms;
uint8_t interval_1ms_flag;
ISR(TIMER0_OVF_vect, ISR_NOBLOCK)
{
	_system_tick_count++;
	_scale += 16;
	if (_scale >= SCALE) {
		_scale -= SCALE;
		_tick_count++;
		_time_ms++;
		if (_time_ms >= 1000) {
			_time_ms = 0;
			_time_s++;
		}
		interval_1ms_flag = 1;
	}
}

static inline int clamp(int v, int min, int max)
{
	if (v > max) v = max;
	if (v < min) v = min;
	return v;
}

////

AVRHVPP avrhvpp;
void AVRHVPP::write_byte(char c)
{
	usb_write_byte(c);
}
void AVRHVPP::write_string(char const *p)
{
	usb_write_string(p);
}

////

avr::GPIO<avr::B, 0> icsp_rst;
avr::GPIO<avr::B, 1> icsp_sck;
avr::GPIO<avr::B, 2> icsp_mosi;
avr::GPIO<avr::B, 3> icsp_miso;

static inline void led_pmode(bool f)
{
}

static inline void led_error(bool f)
{
}

#define HWVER 2
#define SWMAJ 1
#define SWMIN 18

// STK Definitions
#define STK_OK 0x10
#define STK_FAILED 0x11
#define STK_UNKNOWN 0x12
#define STK_INSYNC 0x14
#define STK_NOSYNC 0x15
#define CRC_EOP 0x20 // ok it is a space...

#define USE_HARDWARE_SPI

#ifdef USE_HARDWARE_SPI
class SPI {
public:
	void begin()
	{
		icsp_sck.write(0);
		icsp_mosi.write(0);
		icsp_miso.read();

		// SPI: Mode 0
		SPCR = 0x53; // F_CLK/128 (125kHz)
//		SPCR = 0x52; // F_CLK/64  (250kHz)
		SPSR = 0x00;
	}

	void end()
	{
		icsp_sck.read();
		icsp_mosi.read();
		icsp_miso.read();
		SPCR = 0x00;
		SPSR = 0x00;
	}

	uint8_t transfer(uint8_t b)
	{
		SPDR = b;
		while (!(SPSR & 0x80));
		return SPDR;
	}
};
#else
class SPI {
public:
	void begin()
	{
		icsp_sck.write(0);
		icsp_mosi.write(0);
		icsp_miso.read();
	}

	void end()
	{
	}

	uint8_t transfer(uint8_t b)
	{
		for (uint8_t i = 0; i < 8; ++i) {
			icsp_mosi.write(b & 0x80);
			icsp_sck.write(1);
			_delay_us(1);
			b = (b << 1) | icsp_miso.read();
			icsp_sck.write(0);
			_delay_us(1);
		}
		return b;
	}
};
#endif

static SPI spi;

enum class ISPMODE {
	AVRISP,
	AVRHVPP,
	BITBANG,
};

ISPMODE ispmode = ISPMODE::AVRISP;

int error = 0;
bool pmode = 0;
// address for reading and writing, set by 'U' command
unsigned int here;
uint8_t buff[256]; // global block storage

#define beget16(addr) (*addr * 256 + *(addr + 1))
struct parameter_t {
	uint8_t devicecode;
	uint8_t revision;
	uint8_t progtype;
	uint8_t parmode;
	uint8_t polling;
	uint8_t selftimed;
	uint8_t lockbytes;
	uint8_t fusebytes;
	uint8_t flashpoll;
	uint16_t eeprompoll;
	uint16_t pagesize;
	uint16_t eepromsize;
	uint32_t flashsize;
};

parameter_t param;

static bool rst_active_high;

void reset_target(bool reset)
{
	icsp_rst.write((reset && rst_active_high) || (!reset && !rst_active_high));
}

uint8_t getch()
{
	while (usb_read_available() == 0);
	return usb_read_byte();
}

void fill(int n)
{
	for (int x = 0; x < n; x++) {
		buff[x] = getch();
	}
}

uint8_t spi_transaction(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
	spi.transfer(a);
	spi.transfer(b);
	spi.transfer(c);
	return spi.transfer(d);
}

void empty_reply()
{
	if (CRC_EOP == getch()) {
		usb_write_byte((char)STK_INSYNC);
		usb_write_byte((char)STK_OK);
	} else {
		error++;
		usb_write_byte((char)STK_NOSYNC);
	}
}

void breply(uint8_t b)
{
	if (CRC_EOP == getch()) {
		usb_write_byte((char)STK_INSYNC);
		usb_write_byte((char)b);
		usb_write_byte((char)STK_OK);
	} else {
		error++;
		usb_write_byte((char)STK_NOSYNC);
	}
}

void get_version(uint8_t c)
{
	switch (c) {
	case 0x80:
		breply(HWVER);
		break;
	case 0x81:
		breply(SWMAJ);
		break;
	case 0x82:
		breply(SWMIN);
		break;
	case 0x93:
		breply('S'); // serial programmer
		break;
	default:
		breply(0);
	}
}

void set_parameters()
{
	// call this after reading parameter packet into buff[]
	param.devicecode = buff[0];
	param.revision = buff[1];
	param.progtype = buff[2];
	param.parmode = buff[3];
	param.polling = buff[4];
	param.selftimed = buff[5];
	param.lockbytes = buff[6];
	param.fusebytes = buff[7];
	param.flashpoll = buff[8];
	// ignore buff[9] (= buff[8])
	// following are 16 bits (big endian)
	param.eeprompoll = beget16(&buff[10]);
	param.pagesize = beget16(&buff[12]);
	param.eepromsize = beget16(&buff[14]);

	// 32 bits flashsize (big endian)
	param.flashsize = buff[16] * 0x01000000
		+ buff[17] * 0x00010000
		+ buff[18] * 0x00000100
		+ buff[19];

	// AVR devices have active low reset, AT89Sx are active high
	rst_active_high = (param.devicecode >= 0xe0);
}

void start_pmode()
{
	// Reset target before driving PIN_SCK or PIN_MOSI

	// spi.begin() will configure SS as output, so SPI master mode is selected.
	// We have defined RESET as pin 10, which for many Arduinos is not the SS pin.
	// So we have to configure RESET as output here,
	// (reset_target() first sets the correct level)
	reset_target(true);
	spi.begin();

	// See AVR datasheets, chapter "SERIAL_PRG Programming Algorithm":

	// Pulse RESET after PIN_SCK is low:
	icsp_sck.write(0);
	msleep(20); // discharge PIN_SCK, value arbitrarily chosen
	reset_target(false);
	// Pulse must be minimum 2 target CPU clock cycles so 100 usec is ok for CPU
	// speeds above 20 KHz
	usleep(100);
	reset_target(true);

	// Send the enable programming command:
	msleep(50); // datasheet: must be > 20 msec
	spi_transaction(0xAC, 0x53, 0x00, 0x00);
	pmode = 1;
}

void end_pmode()
{
	spi.end();
	icsp_mosi.read();
	icsp_sck.read();
	reset_target(false);
	pmode = 0;
}

void universal()
{
	uint8_t ch;

	fill(4);
	ch = spi_transaction(buff[0], buff[1], buff[2], buff[3]);
	breply(ch);
}

void flash(uint8_t hilo, unsigned int addr, uint8_t data)
{
	spi_transaction(0x40 + 8 * hilo, addr >> 8 & 0xFF, addr & 0xFF, data);
}

void commit(unsigned int addr)
{
	spi_transaction(0x4C, (addr >> 8) & 0xFF, addr & 0xFF, 0);
	msleep(10);
}

unsigned int current_page()
{
	if (param.pagesize == 32) {
		return here & 0xFFFFFFF0;
	}
	if (param.pagesize == 64) {
		return here & 0xFFFFFFE0;
	}
	if (param.pagesize == 128) {
		return here & 0xFFFFFFC0;
	}
	if (param.pagesize == 256) {
		return here & 0xFFFFFF80;
	}
	return here;
}

uint8_t write_flash_pages(int length)
{
	int x = 0;
	unsigned int page = current_page();
	while (x < length) {
		if (page != current_page()) {
			commit(page);
			page = current_page();
		}
		flash(0, here, buff[x++]);
		flash(1, here, buff[x++]);
		here++;
	}

	commit(page);

	return STK_OK;
}

void write_flash(int length)
{
	fill(length);
	if (CRC_EOP == getch()) {
		usb_write_byte((char)STK_INSYNC);
		usb_write_byte((char)write_flash_pages(length));
	} else {
		error++;
		usb_write_byte((char)STK_NOSYNC);
	}
}

// write (length) bytes, (start) is a byte address
uint8_t write_eeprom_chunk(unsigned int start, unsigned int length)
{
	// this writes byte-by-byte, page writing may be faster (4 bytes at a time)
	fill(length);
	for (unsigned int x = 0; x < length; x++) {
		unsigned int addr = start + x;
		spi_transaction(0xC0, (addr >> 8) & 0xFF, addr & 0xFF, buff[x]);
		msleep(45);
	}
	return STK_OK;
}

#define EECHUNK (32)
uint8_t write_eeprom(unsigned int length)
{
	// here is a word address, get the byte address
	unsigned int start = here * 2;
	unsigned int remaining = length;
	if (length > param.eepromsize) {
		error++;
		return STK_FAILED;
	}
	while (remaining > EECHUNK) {
		write_eeprom_chunk(start, EECHUNK);
		start += EECHUNK;
		remaining -= EECHUNK;
	}
	write_eeprom_chunk(start, remaining);
	return STK_OK;
}

void program_page()
{
	char result = (char)STK_FAILED;
	unsigned int length = 256 * getch();
	length += getch();
	char memtype = getch();
	// flash memory @here, (length) bytes
	if (memtype == 'F') {
		write_flash(length);
		return;
	} else if (memtype == 'E') {
		result = (char)write_eeprom(length);
		if (CRC_EOP == getch()) {
			usb_write_byte((char)STK_INSYNC);
			usb_write_byte(result);
		} else {
			error++;
			usb_write_byte((char)STK_NOSYNC);
		}
		return;
	}
	usb_write_byte((char)STK_FAILED);
	return;
}

uint8_t flash_read(uint8_t hilo, unsigned int addr)
{
	return spi_transaction(0x20 + hilo * 8, (addr >> 8) & 0xFF, addr & 0xFF, 0);
}

char flash_read_page(int length)
{
	for (int x = 0; x < length; x += 2) {
		uint8_t low = flash_read(0, here);
		usb_write_byte((char)low);
		uint8_t high = flash_read(1, here);
		usb_write_byte((char)high);
		here++;
	}
	return STK_OK;
}

char eeprom_read_page(int length)
{
	// here again we have a word address
	int start = here * 2;
	for (int x = 0; x < length; x++) {
		int addr = start + x;
		uint8_t ee = spi_transaction(0xA0, (addr >> 8) & 0xFF, addr & 0xFF, 0xFF);
		usb_write_byte((char)ee);
	}
	return STK_OK;
}

void read_page()
{
	char result = (char)STK_FAILED;
	int length = 256 * getch();
	length += getch();
	char memtype = getch();
	if (CRC_EOP != getch()) {
		error++;
		usb_write_byte((char)STK_NOSYNC);
		return;
	}
	usb_write_byte((char)STK_INSYNC);
	if (memtype == 'F') {
		result = flash_read_page(length);
	} else if (memtype == 'E') {
		result = eeprom_read_page(length);
	}
	usb_write_byte(result);
}

void read_signature()
{
	if (CRC_EOP != getch()) {
		error++;
		usb_write_byte((char)STK_NOSYNC);
		return;
	}
	usb_write_byte((char)STK_INSYNC);
	uint8_t high = spi_transaction(0x30, 0x00, 0x00, 0x00);
	usb_write_byte((char)high);
	uint8_t middle = spi_transaction(0x30, 0x00, 0x01, 0x00);
	usb_write_byte((char)middle);
	uint8_t low = spi_transaction(0x30, 0x00, 0x02, 0x00);
	usb_write_byte((char)low);
	usb_write_byte((char)STK_OK);
}

void avrisp(uint8_t ch)
{
	switch (ch) {
	case '0': // signon
		error = 0;
		empty_reply();
		break;
	case '1':
		if (getch() == CRC_EOP) {
			usb_write_byte((char)STK_INSYNC);
			usb_write_string("AVR ISP");
			usb_write_byte((char)STK_OK);
		} else {
			error++;
			usb_write_byte((char)STK_NOSYNC);
		}
		break;
	case 'A':
		get_version(getch());
		break;
	case 'B':
		fill(20);
		set_parameters();
		empty_reply();
		break;
	case 'E': // extended parameters - ignore for now
		fill(5);
		empty_reply();
		break;
	case 'P':
		if (!pmode) {
			start_pmode();
		}
		empty_reply();
		break;
	case 'U': // set address (word)
		here = getch();
		here += 256 * getch();
		empty_reply();
		break;

	case 0x60: // STK_PROG_FLASH
		getch(); // low addr
		getch(); // high addr
		empty_reply();
		break;
	case 0x61: // STK_PROG_DATA
		getch(); // data
		empty_reply();
		break;

	case 0x64: // STK_PROG_PAGE
		program_page();
		break;

	case 0x74: // STK_READ_PAGE 't'
		read_page();
		break;

	case 'V': // 0x56
		universal();
		break;
	case 'Q': // 0x51
		error = 0;
		end_pmode();
		empty_reply();
		break;

	case 0x75: // STK_READ_SIGN 'u'
		read_signature();
		break;

	// expecting a command, not CRC_EOP
	// this is how we can get back in sync
	case CRC_EOP:
		error++;
		usb_write_byte((char)STK_NOSYNC);
		break;

	// anything else we will return STK_UNKNOWN
	default:
		error++;
		if (CRC_EOP == getch()) {
			usb_write_byte((char)STK_UNKNOWN);
		} else {
			usb_write_byte((char)STK_NOSYNC);
		}
	}
}




////

enum {
	INPUT,
	OUTPUT,
};
enum {
	LOW,
	HIGH,
};
enum {
	PIN_RESET,
	PIN_SCK,
	PIN_MOSI,
	PIN_MISO,
};

void pinMode(uint8_t pin, bool output)
{
	// nop
}

bool digitalRead(uint8_t pin)
{
	switch (pin) {
	case PIN_RESET: return icsp_rst.read();
	case PIN_SCK:   return icsp_sck.read();
	case PIN_MOSI:  return icsp_mosi.read();
	case PIN_MISO:  return icsp_miso.read();
	}
	return true;
}

void digitalWrite(uint8_t pin, bool val)
{
	switch (pin) {
	case PIN_RESET: icsp_rst.write(val); break;
	case PIN_SCK:   icsp_sck.write(val); break;
	case PIN_MOSI:  icsp_mosi.write(val); break;
	case PIN_MISO:  icsp_miso.write(val); break;
	}
}

void bitbang(uint8_t c)
{
	uint8_t out = '\n';

	uint8_t pin = 0xff;
	switch (c & 0x0f) {
	case 0: pin = PIN_RESET; break;
	case 1: pin = PIN_SCK;   break;
	case 2: pin = PIN_MOSI;  break;
	case 3: pin = PIN_MISO;  break;
	}

	if ((c & 0xf0) == 0x80) { // set input
		pinMode(pin, INPUT);
		digitalWrite(pin, LOW);
		bool b = digitalRead(pin);
		out = b ? '1' : '0';
	} else if ((c & 0xf0) == 0x90) { // set input with pullup
		pinMode(pin, INPUT);
		digitalWrite(pin, HIGH);
		bool b = digitalRead(pin);
		out = b ? '1' : '0';
	} else if ((c & 0xf0) == 0xa0) { // write output low
		pinMode(pin, OUTPUT);
		digitalWrite(pin, LOW);
	} else if ((c & 0xf0) == 0xb0) { // write output high
		pinMode(pin, OUTPUT);
		digitalWrite(pin, HIGH);
	} else if ((c & 0xf0) == 0xc0) { // read all pins
		out = 0;
		if (digitalRead(PIN_RESET)) out |= 0x01;
		if (digitalRead(PIN_SCK))   out |= 0x02;
		if (digitalRead(PIN_MOSI))  out |= 0x04;
		if (digitalRead(PIN_MISO))  out |= 0x08;
		out += out < 10 ? '0' : ('A' - 10);
	} else if ((c & 0xf0) == 0xd0) { // write all pins
		digitalWrite(PIN_RESET, c & 0x01);
		digitalWrite(PIN_SCK,   c & 0x02);
		digitalWrite(PIN_MOSI,  c & 0x04);
		digitalWrite(PIN_MISO,  c & 0x08);
		out = c & 0x0f;
		out += out < 10 ? '0' : ('A' - 10);
	} else {
		out = '?';
	}

	usb_write_byte(out);
}

/////

void reset_ports()
{
	spi.end();
	icsp_rst.read();
	icsp_sck.read();
	icsp_mosi.read();
	icsp_miso.read();
}

void isp_poll()
{
	static char const *state_avrhvpp = nullptr;
	static char const *state_bitbang = nullptr;
	if (usb_read_available() > 0) {
		uint8_t c = getch();
		if (c < 0x40) {
			if (ispmode != ISPMODE::AVRISP) {
				if (ispmode == ISPMODE::AVRHVPP) {
					avrhvpp.reset_command_state();
				}
				ispmode = ISPMODE::AVRISP;
				state_avrhvpp = nullptr;
				state_bitbang = nullptr;
				reset_ports();
			}
		}
		if (ispmode == ISPMODE::AVRISP) {
			if (state_avrhvpp) {
				if (*state_avrhvpp == c) {
					state_avrhvpp++;
					if (!*state_avrhvpp) {
						state_avrhvpp = nullptr;
						usb_write_string("AVRHVPP\r\n");
						avrhvpp.reset_command_state();
						ispmode = ISPMODE::AVRHVPP;
						reset_ports();
					}
					return;
				} else {
					state_avrhvpp = nullptr;
				}
			}
			if (state_bitbang) {
				if (*state_bitbang == c) {
					state_bitbang++;
					if (!*state_bitbang) {
						state_bitbang = nullptr;
						usb_write_string("BITBANG\r\n");
						ispmode = ISPMODE::BITBANG;
						reset_ports();
					}
					return;
				} else {
					state_bitbang = nullptr;
				}
			}
			if (c == '.') {
				state_avrhvpp = "AVRHVPP.";
				state_bitbang = "BITBANG.";
				return;
			}
			avrisp(c);
		} else if (ispmode == ISPMODE::AVRHVPP) {
			avrhvpp.process(c);
		} else if (ispmode == ISPMODE::BITBANG) {
			bitbang(c);
		}
	}
}

////




////

void setup()
{
	// 16 MHz clock
	CLKPR = 0x80;
	CLKPR = 0;
	// Disable JTAG
	MCUCR |= 0x80;
	MCUCR |= 0x80;

	PORTB = 0x00;
	PORTC = 0x00;
	PORTD = 0x00;
	DDRB = 0x00;
	DDRC = 0x00;
	DDRD = 0x00;

	TCCR0B = 0x02; // 1/8 prescaling
	TIMSK0 |= 1 << TOIE0;

	clear_buffers();

	usb_init();
	while (!is_usb_configured()) {
		_delay_ms(100);
	}

	reset_ports();
}

void loop()
{
	usb_poll();
#if 0
	if (usb_read_available()) {
		uint8_t c = usb_read_byte();
		usb_write_byte(c);
	}
#else
	isp_poll();
#endif
}

int main()
{
	setup();
	sei();
	while (1) {
		loop();
	}
}

