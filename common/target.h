#ifndef TARGET_H
#define TARGET_H

#include <stdint.h>

struct ATmega328P {
	enum {
		Signature0 = 0x1e,
		Signature1 = 0x95,
		Signature2 = 0x0f,
		FlashPageBytes = 128,
		FlashPageWords = FlashPageBytes / 2,
		FlashPageCount = 256,
	};

	static constexpr uint16_t flash_word_address(uint16_t page, uint8_t word)
	{
		return page * FlashPageWords + word;
	}

	static constexpr uint32_t flash_bytes()
	{
		return (uint32_t)FlashPageBytes * FlashPageCount;
	}
};

typedef ATmega328P Target;

static_assert(Target::flash_word_address(0, 0) == 0x0000, "invalid flash address");
static_assert(Target::flash_word_address(3, 0) == 0x00c0, "invalid flash address");
static_assert(Target::flash_word_address(4, 0) == 0x0100, "invalid flash address");
static_assert(Target::flash_word_address(255, 63) == 0x3fff, "invalid flash address");
static_assert(Target::flash_bytes() == 32768, "invalid flash size");

#endif // TARGET_H
