#ifndef COMMANDS_H
#define COMMANDS_H

#include <stdint.h>

enum Command {
	CMD_CLEAR = 0x40,
	CMD_DEBUG = 0x4f,
	CMD_READ_SIGNATURE = 0x50,
	CMD_READ_FUSE = 0x51,
	CMD_READ_LOCK = 0x52,
	CMD_READ_PAGE = 0x53,
	CMD_CHIP_ERASE = 0x60,
	CMD_WRITE_FUSE = 0x61,
	CMD_WRITE_LOCK = 0x62,
	CMD_WRITE_PAGE = 0x63,
	CMD_ADDR0 = 0x70,
	CMD_ADDR1 = 0x71,
	CMD_DATA0 = 0x72,
	CMD_DATA1 = 0x73,
	CMD_LATCH_FLASH_PARAMS = 0x7d,
	CMD_READ_16BYTES = 0x7f,
	CMD_LOAD_7BITS = 0x80,
	
	_CMD_DUMP_BUFFER = 0x100,
	_CMD_WRITE_HEX = 0x101,
	_CMD_READ_HEX = 0x102,
	_CMD_DETECT = 0x103,
};

enum class ProtocolError : uint8_t {
	None = 0x00,
	InvalidArgument = 0x01,
	ProgrammingMode = 0x02,
	ReadyTimeout = 0x03,
	VerifyMismatch = 0x04,
	NotBlank = 0x05,
	UnknownCommand = 0x06,
};

constexpr uint16_t FlashBufferBytes = 256;
constexpr uint8_t FlashParameterBytes = 4;

// Wire format: word_address low/high, byte_count low/high.
struct FlashParameters {
	uint16_t word_address;
	uint16_t byte_count;

	constexpr bool valid() const
	{
		return byte_count > 0 && byte_count <= FlashBufferBytes
			&& byte_count % 2 == 0
			&& (uint32_t)word_address + byte_count / 2 <= 0x10000UL;
	}
};

#endif // COMMANDS_H
