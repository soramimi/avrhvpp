#ifndef TEST_UTIL_DELAY_H
#define TEST_UTIL_DELAY_H

void fake_delay_us(double us);
inline void _delay_us(double us) { fake_delay_us(us); }
inline void _delay_ms(double ms) { fake_delay_us(ms * 1000); }

#endif
