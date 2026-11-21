debug:
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --debug

query:
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --query

write:
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --write-hex attiny202blink.hex

read:
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --read-hex /tmp/attiny202blink.hex

write2:
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --write-hex /tmp/attiny202blink.hex

chiperase:
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --chip-erase

readlock:
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --read-lock
	
fusetest:
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --read-fuse SYSCFG0
	# EESAVE をセットし、別セッションで 0xf7 を確認
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --write-fuse SYSCFG0 0xf7
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --read-fuse SYSCFG0
	# 元の値に戻し、0xf6 を確認
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --write-fuse SYSCFG0 0xf6
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --read-fuse SYSCFG0

rwpage:
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --read-page 0 /tmp/page0.bin
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --read-page 1 /tmp/page1.bin
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --chip-erase
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --write-page 0 /tmp/page0.bin
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --write-page 1 /tmp/page1.bin

locktest:
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --write-lock 0x00
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --read-lock
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --erase-locked
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t attiny202 --read-lock
