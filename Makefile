
PORT := /dev/ttyACM0

main:
	./app/_bin/avrhvpp -p $(PORT) -rs -rfl -rfh -rfe
