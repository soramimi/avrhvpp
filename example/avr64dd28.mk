
	TARGET := avr64dd28

debug:
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --debug

query:
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --query

write:
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --write-hex $(TARGET)blink.hex

read:
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --read-hex /tmp/$(TARGET)blink.hex

write2:
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --write-hex /tmp/$(TARGET)blink.hex

chiperase:
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --chip-erase

readlock:
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --read-lock
	
fusetest:
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --read-fuse SYSCFG0
	# EESAVE をセットし、別セッションで 0xf7 を確認
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --write-fuse SYSCFG0 0xf7
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --read-fuse SYSCFG0
	# 元の値に戻し、0xf6 を確認
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --write-fuse SYSCFG0 0xf6
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --read-fuse SYSCFG0

rwpage:
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --read-page 0 /tmp/page0.bin
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --read-page 1 /tmp/page1.bin
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --chip-erase
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --write-page 0 /tmp/page0.bin
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --write-page 1 /tmp/page1.bin

locktest:
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --write-lock 0x00
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --read-lock
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --erase-locked
	../app/_bin/avrhvpp -p /dev/ttyACM0 -d updi -t $(TARGET) --read-lock
