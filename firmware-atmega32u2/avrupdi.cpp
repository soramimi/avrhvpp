#include "avrupdi.h"
#include "uart.h"
#include <util/delay.h>
#include "../common/commands.h"

void msleep(int ms);
void usb_write_byte(char c);
void usb_poll();

namespace {
uint8_t data_7bits_ = 0;
}

AVRUPDI::AVRUPDI()
{
}

void _msleep(int ms)
{
	while (ms > 0) {
		usb_poll();
		updi_poll();
		_delay_ms(1);
		ms--;
	}
}

void AVRUPDI::process(uint8_t c)
{
	if (c == CMD_UPDI_BREAK) {
		uart_enable(true);
		uart_send_break(true);
		_msleep(50);
		uart_send_break(false);
	} else if (c == CMD_DEBUG) {
		// uart_enable(true);
		// uart_send_break(true);
		// _msleep(50);
		// uart_send_break(false);
		for (int i = 0; i < 2; i++) {
#if 0
			uart_send_byte(0x55);
			uart_send_byte(0xc3);
			uart_send_byte(0x08);
			uart_send_byte(0x55);
			uart_send_byte(0xc2);
			uart_send_byte(0x80);
			uart_send_byte(0x55);
			uart_send_byte(0x80);
			_msleep(3);
			uart_send_byte(0x55);
			uart_send_byte(0x8b);
			_msleep(3);
			uart_send_byte(0x55);
			uart_send_byte(0xe6);
			_msleep(10);
#endif
#if 0			
			uart_send_byte(0x55);
			uart_send_byte(0xc3);
			uart_send_byte(0x08);
			uart_send_byte(0x55);
			uart_send_byte(0xc2);
			uart_send_byte(0x80);
			uart_send_byte(0x55);
			uart_send_byte(0x80);
			_msleep(3);
			uart_send_byte(0x55);
			uart_send_byte(0x8b);
			_msleep(3);
			
			uart_send_byte(0x55);
			uart_send_byte(0xc8);
			uart_send_byte(0x59);
			uart_send_byte(0x55);
			uart_send_byte(0xe0);
			_msleep(1);
			uart_send_byte(0x20);
			uart_send_byte(0x67);
			uart_send_byte(0x6f);
			uart_send_byte(0x72);
			uart_send_byte(0x50);
			uart_send_byte(0x4d);
			uart_send_byte(0x56);
			uart_send_byte(0x4e);
			_msleep(1);
#endif
#if 0			
			uart_send_byte(0x55);
			uart_send_byte(0x87);
			_msleep(3);
			uart_send_byte(0x55);
			uart_send_byte(0xc8);
			uart_send_byte(0x59);
			uart_send_byte(0x55);
			uart_send_byte(0xc8);
			uart_send_byte(0x00);
			uart_send_byte(0x55);
			uart_send_byte(0x8b);
			_msleep(3);
			uart_send_byte(0x55);
			uart_send_byte(0x8b);
			_msleep(3);
			uart_send_byte(0x55);
			uart_send_byte(0x8b);
			_msleep(3);
			uart_send_byte(0x55);
			uart_send_byte(0x8b);
			_msleep(3);
			uart_send_byte(0x55);
			uart_send_byte(0x8b);
			_msleep(3);
#endif
#if 0
			uart_send_byte(0x55);
			uart_send_byte(0x04);
			uart_send_byte(0x00);
			uart_send_byte(0x11);
			_msleep(10);
			uart_send_byte(0x55);
			uart_send_byte(0x04);
			uart_send_byte(0x01);
			uart_send_byte(0x11);
			_msleep(10);
			uart_send_byte(0x55);
			uart_send_byte(0x04);
			uart_send_byte(0x02);
			uart_send_byte(0x11);
			_msleep(10);
#endif
		}
		// _msleep(500);
	} else if (c == CMD_DATA0) {
		uint8_t d = data_7bits_ & 0x7f;
		uart_send_byte(d);
	} else if (c == CMD_DATA1) {
		uint8_t d = (data_7bits_ & 0x7f) | 0x80;
		uart_send_byte(d);
	} else if (c >= CMD_LOAD_7BITS) {
		data_7bits_ = c & 0x7f;
	}
}

void updi_poll()
{
	for (int i = 0; i < 16; i++) {
		int16_t c = uart_recv_byte();
		if (c < 0) break;
		usb_write_byte(c);
	}
}
