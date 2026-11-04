#ifndef TARGET_H
#define TARGET_H

#include <stdint.h>

enum TargetCapability : uint8_t {
	CapabilityFlash = 1 << 0,
	CapabilityFuseLow = 1 << 1,
	CapabilityFuseHigh = 1 << 2,
	CapabilityFuseExtended = 1 << 3,
	CapabilityLockBits = 1 << 4,
	CapabilityChipErase = 1 << 5,
};

struct TargetDefinition {
	char const *display_name;
	uint8_t signature[3];
	uint16_t flash_page_bytes;
	uint16_t flash_page_count;
	uint8_t capabilities;

	constexpr uint16_t flash_page_words() const
	{
		return flash_page_bytes / 2;
	}

	constexpr uint32_t flash_bytes() const
	{
		return (uint32_t)flash_page_bytes * flash_page_count;
	}

	constexpr uint32_t flash_word_address(uint16_t page, uint16_t word) const
	{
		return (uint32_t)page * flash_page_words() + word;
	}

	constexpr bool supports(TargetCapability capability) const
	{
		return (capabilities & capability) == capability;
	}

	constexpr bool matches_signature(uint8_t signature0, uint8_t signature1, uint8_t signature2) const
	{
		return signature[0] == signature0
			&& signature[1] == signature1
			&& signature[2] == signature2;
	}
};

constexpr TargetDefinition TargetATmega8 = {
	"ATmega8",
	{0x1e, 0x93, 0x07},
	64,
	128,
	CapabilityFlash
		| CapabilityFuseLow
		| CapabilityFuseHigh
		| CapabilityLockBits
		| CapabilityChipErase,
};

constexpr TargetDefinition TargetATmega328P = {
	"ATmega328P",
	{0x1e, 0x95, 0x0f},
	128,
	256,
	CapabilityFlash
		| CapabilityFuseLow
		| CapabilityFuseHigh
		| CapabilityFuseExtended
		| CapabilityLockBits
		| CapabilityChipErase,
};

inline TargetDefinition const *find_target_by_signature(
	uint8_t signature0,
	uint8_t signature1,
	uint8_t signature2)
{
	if (TargetATmega8.matches_signature(signature0, signature1, signature2)) {
		return &TargetATmega8;
	}
	if (TargetATmega328P.matches_signature(signature0, signature1, signature2)) {
		return &TargetATmega328P;
	}
	return 0;
}

static_assert(TargetATmega8.flash_page_bytes <= 256, "ATmega8 flash page does not fit in buffer");
static_assert(TargetATmega8.flash_page_bytes % 2 == 0, "invalid ATmega8 flash page size");
static_assert(TargetATmega8.flash_page_words() == 32, "invalid ATmega8 flash page size");
static_assert(TargetATmega8.flash_word_address(0, 0) == 0x0000, "invalid ATmega8 flash address");
static_assert(TargetATmega8.flash_word_address(127, 31) == 0x0fff, "invalid ATmega8 flash address");
static_assert(TargetATmega8.flash_bytes() == 8192, "invalid ATmega8 flash size");
static_assert(!TargetATmega8.supports(CapabilityFuseExtended), "ATmega8 must not have extended fuse");

static_assert(TargetATmega328P.flash_page_bytes <= 256, "ATmega328P flash page does not fit in buffer");
static_assert(TargetATmega328P.flash_page_bytes % 2 == 0, "invalid ATmega328P flash page size");
static_assert(TargetATmega328P.flash_page_words() == 64, "invalid ATmega328P flash page size");
static_assert(TargetATmega328P.flash_word_address(0, 0) == 0x0000, "invalid ATmega328P flash address");
static_assert(TargetATmega328P.flash_word_address(3, 0) == 0x00c0, "invalid ATmega328P flash address");
static_assert(TargetATmega328P.flash_word_address(4, 0) == 0x0100, "invalid ATmega328P flash address");
static_assert(TargetATmega328P.flash_word_address(255, 63) == 0x3fff, "invalid ATmega328P flash address");
static_assert(TargetATmega328P.flash_bytes() == 32768, "invalid ATmega328P flash size");
static_assert(TargetATmega328P.supports(CapabilityFuseExtended), "ATmega328P must have extended fuse");

#endif // TARGET_H
