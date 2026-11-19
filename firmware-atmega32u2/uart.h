
#ifndef UART_H
#define UART_H

#include <stdint.h>

void uart_enable(bool enable);
void uart_clear_error(void);
int16_t uart_recv_byte();
bool uart_send_byte(uint8_t c);
void uart_send_break(bool brk);

#endif

