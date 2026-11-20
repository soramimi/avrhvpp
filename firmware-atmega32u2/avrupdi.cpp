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
	if (c == CMD_DEBUG) {
	} else if (c == CMD_UPDI_BREAK) {
		uart_enable(true);
		uart_send_break(true);
		_msleep(50);
		uart_send_break(false);
	} else if (c == CMD_UPDI_SEND0 || c == CMD_UPDI_SEND1) {
		uint8_t d = data_7bits_ & 0x7f;
		if (c == CMD_UPDI_SEND1) {
			d |= 0x80;
		}
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
