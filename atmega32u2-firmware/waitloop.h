
#ifndef WAITLOOP_H
#define WAITLOOP_H


extern void waitloop(unsigned int n);
#define wait_40us()		waitloop(40)
#define wait_1ms()		waitloop(1000)
#define usleep(us)		waitloop(us)

void msleep(unsigned int ms);
void wait_100ms();
void wait_1s();

#endif
