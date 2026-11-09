
PORT := /dev/ttyACM0

main:
	./app/_bin/avrhvpp -p $(PORT) -rs -rfl -rfh -rfe


debug:
	avrdude -c avrisp -P /dev/ttyACM0 -p attiny13a -U lfuse:r:-:h -U hfuse:r:-:h
