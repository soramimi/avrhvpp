#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>

#include "avrgpio.h"

avr::GPIO<avr::D, 2> pin_rxd1_;
avr::GPIO<avr::D, 3> pin_txd1_;

// F_CPU must match the actual CPU clock (including the clock prescaler).
// At 16 MHz, double speed with UBRR1 = 16 gives 117647 baud (+2.12%).
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#define BAUD 115200UL
#define BAUD_TOL 3
#include <util/setbaud.h>

#define UART_TX_SIZE 128
#define UART_RX_SIZE 128

extern uint8_t flush_buffer[256];

#define uart_tx_buffer (flush_buffer)
uint8_t uart_tx_buffer_i;
uint8_t uart_tx_buffer_n;

#define uart_rx_buffer (flush_buffer + UART_TX_SIZE)
uint8_t uart_rx_buffer_i;
uint8_t uart_rx_buffer_n;


ISR(USART1_RX_vect)
{
	for (uint8_t i = 0; i < UART_RX_SIZE; i++) {
		volatile uint8_t c = UDR1;
		uint8_t space = UART_RX_SIZE - uart_rx_buffer_n;
		if (space > 0) {
			int j = (uart_rx_buffer_i + uart_rx_buffer_n) % UART_RX_SIZE;
			uart_rx_buffer[j] = c;
			uart_rx_buffer_n++;
		}
		if (!(UCSR1A & _BV(RXC1))) break;
	}
}

ISR(USART1_UDRE_vect)
{
	if (uart_tx_buffer_n > 0) {
		UDR1 = uart_tx_buffer[uart_tx_buffer_i];
		uart_tx_buffer_i = (uart_tx_buffer_i + 1) % UART_TX_SIZE;
		uart_tx_buffer_n--;
	} else {
		UCSR1B &= ~_BV(UDRIE1);
	}
}

static void uart_init(void)
{
	uart_tx_buffer_i = 0;
	uart_tx_buffer_n = 0;
	uart_rx_buffer_i = 0;
	uart_rx_buffer_n = 0;
	
	UCSR1B = 0;
	UBRR1H = UBRRH_VALUE;
	UBRR1L = UBRRL_VALUE;
#if USE_2X
	UCSR1A = _BV(U2X1);
#else
	UCSR1A = 0;
#endif
	// Asynchronous USART, 8 data bits, even parity, 2 stop bits (8E2).
	// USART1: RXD1 = PD2, TXD1 = PD3; no hardware flow control.
	UCSR1C = _BV(UPM11) | _BV(USBS1) | _BV(UCSZ11) | _BV(UCSZ10);
	UCSR1D = 0;
	UCSR1B = _BV(RXEN1) | _BV(TXEN1);
	
	// enable USART1 receive complete interrupt
	UCSR1B |= _BV(RXCIE1);
}

void uart_enable(bool enable)
{
	if (enable) {
		uart_init();
	} else {
		UCSR1B = 0;
	}
}

void uart_clear_error(void)
{
	// Clear USART1 error flags by reading UCSR1A and UDR1.
	volatile uint8_t tmp;
	tmp = UCSR1A;
	tmp = UDR1;
	(void)tmp;
}

int16_t uart_recv_byte()
{
	int16_t c = -1;
	cli();
	if (uart_rx_buffer_n > 0) {
		c = uart_rx_buffer[uart_rx_buffer_i];
		uart_rx_buffer_i = (uart_rx_buffer_i + 1) % UART_RX_SIZE;
		uart_rx_buffer_n--;
	}
	sei();
	return c;
}

bool uart_send_byte(uint8_t c)
{
	bool ret = false;
	cli();
	if (uart_tx_buffer_n < UART_TX_SIZE) {
		int i = (uart_tx_buffer_i + uart_tx_buffer_n) % UART_TX_SIZE;
		uart_tx_buffer[i] = c;
		uart_tx_buffer_n++;
		UCSR1B |= _BV(UDRIE1);
		ret = true;
	}
	sei();
	return ret;
}

void uart_send_break(bool brk)
{
	if (brk) {
		pin_txd1_.write(false);
		UCSR1B &= ~_BV(TXEN1);
	} else {
		UCSR1B |= _BV(TXEN1);
	}
}



