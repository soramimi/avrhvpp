#ifndef TEST_AVR_INTERRUPT_H
#define TEST_AVR_INTERRUPT_H

#define ISR(vector, ...) void vector()
#define ISR_NOBLOCK
#define sei() ((void)0)

#endif
