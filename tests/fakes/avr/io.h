#ifndef TEST_AVR_IO_H
#define TEST_AVR_IO_H

#include <stdint.h>

extern uint8_t DDRB, DDRC, DDRD, PORTB, PORTC, PORTD, PINC;
uint8_t fake_pinb();
uint8_t fake_pind();
#define PINB fake_pinb()
#define PIND fake_pind()

#endif
