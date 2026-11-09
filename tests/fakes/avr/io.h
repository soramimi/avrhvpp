#ifndef TEST_AVR_IO_H
#define TEST_AVR_IO_H

#include <stdint.h>

extern uint8_t DDRB, DDRC, DDRD, PORTB, PORTC, PORTD, PINC;
uint8_t fake_pinb();
uint8_t fake_pind();
#define PINB fake_pinb()
#define PIND fake_pind()

#ifdef TEST_STK500
extern uint8_t SPCR, SPSR, CLKPR, MCUCR, TCCR0B, TIMSK0;
constexpr uint8_t TOIE0 = 0;
struct FakeSpiRegister {
	void operator=(uint8_t value);
	operator uint8_t() const;
};
extern FakeSpiRegister SPDR;
#endif

#endif
