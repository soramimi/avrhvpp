MCU := atmega32u2
F_CPU := 16000000

NAME := avrhvpp
PROJDIR := .

SRCS := main.cpp avrhvpp.cpp waitloop.cpp usb.c

LIBS := 

CFLAGS := -Os -mmcu=$(MCU) -DF_CPU=$(F_CPU)
CXXFLAGS := -Os -mmcu=$(MCU) -DF_CPU=$(F_CPU)
CC := avr-gcc
CXX := avr-g++ -std=c++11
LD := $(CXX)
INCLUDEPATH :=
#CFLAGS := -O3 $(INCLUDEPATH)
#CXXFLAGS := -O3 $(INCLUDEPATH)

OBJS := $(SRCS:%.c=%.o)
OBJS := $(OBJS:%.cpp=%.o)
OBJS := $(OBJS:%.cc=%.o)
DEPS := $(SRCS:%.c=%.d)
DEPS := $(DEPS:%.cpp=%.d)
DEPS := $(DEPS:%.cc=%.d)

all: $(NAME).hex
	avr-size -C --mcu $(MCU) $(NAME).elf

$(NAME).hex: $(NAME).elf
	avr-objcopy -O ihex $< $@

$(NAME).elf: $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $(NAME).elf

.c.o:
	$(CC) $(CFLAGS) -MMD -MP -MF $(<:%.c=%.d) -c $< -o $(<:%.c=%.o)

.cc.o:
	$(CXX) $(CXXFLAGS) -MMD -MP -MF $(<:%.cc=%.d) -c $< -o $(<:%.cc=%.o)

.cpp.o:
	$(CXX) $(CXXFLAGS) -MMD -MP -MF $(<:%.cpp=%.d) -c $< -o $(<:%.cpp=%.o)

.PHONY: clean
clean:
	-rm $(NAME)
	find $(PROJDIR) -name "*.o" -exec rm {} \;
	find $(PROJDIR) -name "*.d" -exec rm {} \;

-include $(DEPS)

#

write: $(NAME).hex
	avrdude -c avrisp -P /dev/ttyACM0 -b 19200 -p $(MCU) -U hfuse:w:0xd9:m -U lfuse:w:0x5e:m -U flash:w:$(NAME).hex

fetch:
	-avrdude -c avrisp -P /dev/ttyACM0 -b 19200 -p $(MCU)

readfuse:
	avrdude -c avrisp -P /dev/ttyACM0 -b 19200 -p atmega32u2 -U hfuse:r:-:h -U lfuse:r:-:h


#

AVRHVPP := ../app/_bin/avrhvpp -p /dev/ttyACM0
TARGET := atmega88

q:
	$(AVRHVPP) -q

test:
	$(AVRHVPP) -rs -rfl -rfh -rfe

readpage:
	$(AVRHVPP) --read-page 0 /tmp/page0.bin


writepage:
	$(AVRHVPP) --write-page 0 /tmp/page0.bin

readhex:
	$(AVRHVPP) --target $(TARGET) --read-hex /tmp/a.hex

writehex:
	$(AVRHVPP) --target $(TARGET) --write-hex /tmp/a.hex -wfl e2

