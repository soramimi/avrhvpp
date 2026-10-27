MCU = atmega32u2
F_CPU = 16000000

OBJ = \
	usb.o \
	main.o \
	avrpara.o \
	waitloop.o

CFLAGS = -Os -mmcu=$(MCU) -DF_CPU=$(F_CPU)
CC = avr-gcc $(CFLAGS)
CXX = avr-g++ $(CFLAGS) -std=c++11

all: main.hex
	avr-size -C --mcu $(MCU) main.elf

main.hex: main.elf
	avr-objcopy -O ihex $< $@

main.elf: $(OBJ)
	$(CC) -c usb.c
	$(CXX) $(OBJ) -o main.elf

main.o: main.cpp
	$(CXX) -c $^ -o $@

queue16.o: queue16.cpp
	$(CXX) -c $^ -o $@

waitloop.o: waitloop.cpp
	$(CXX) -c $^ -o $@

usb.o: usb.c
	$(CC) -c $^ -o $@

clean:
	rm *.o
	rm *.elf
	rm *.hex

write: main.hex
	avrdude -c avrisp -P /dev/ttyACM0 -b 19200 -p $(MCU) -U hfuse:w:0xd9:m -U lfuse:w:0x5e:m -U flash:w:main.hex

fetch:
	-avrdude -c avrisp -P /dev/ttyACM0 -b 19200 -p $(MCU)

readfuse:
	avrdude -c avrisp -P /dev/ttyACM0 -b 19200 -p atmega32u2 -U hfuse:r:-:h -U lfuse:r:-:h

