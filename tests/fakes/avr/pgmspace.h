#ifndef TEST_AVR_PGMSPACE_H
#define TEST_AVR_PGMSPACE_H

#include <stdint.h>
#define PROGMEM
#define pgm_read_byte(pointer) (*(uint8_t const *)(pointer))

#endif
