#ifndef AVRGPIO_H
#define AVRGPIO_H

#include <avr/io.h>

namespace avr {
enum {
	B,
	C,
	D,
};

template <int PORT, int PIN> class GPIO {
public:
private:
	static constexpr uint8_t bit = 1 << PIN;
	bool pullup_ = false;
	void setdir(bool output)
	{
		switch (PORT) {
		case avr::B: if (output) { DDRB |= bit; } else { DDRB &= ~bit; } return;
		case avr::C: if (output) { DDRC |= bit; } else { DDRC &= ~bit; } return;
		case avr::D: if (output) { DDRD |= bit; } else { DDRD &= ~bit; } return;
		}
	}
	void _write(bool b)
	{
		switch (PORT) {
		case avr::B: if (b) { PORTB |= bit; } else { PORTB &= ~bit; } return;
		case avr::C: if (b) { PORTC |= bit; } else { PORTC &= ~bit; } return;
		case avr::D: if (b) { PORTD |= bit; } else { PORTD &= ~bit; } return;
		}
	}
public:
	GPIO()
	{
		setdir(false);
		pullup(false);
	}
	void pullup(bool pullup)
	{
		setdir(false);
		_write(pullup);
	}
	bool read()
	{
		setdir(false);
		switch (PORT) {
		case avr::B: return (PINB & bit);
		case avr::C: return (PINC & bit);
		case avr::D: return (PIND & bit);
		}
		return false;
	}
	void write(bool b)
	{
		setdir(true);
		_write(b);
	}
};

} // namespace avr

#endif // AVRGPIO_H
