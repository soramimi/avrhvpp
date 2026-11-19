#ifndef AVRUPDI_H
#define AVRUPDI_H

#include <stdint.h>

class AVRUPDI {
public:
	AVRUPDI();
	void process(uint8_t c);
};

void updi_poll();

#endif // AVRUPDI_H
