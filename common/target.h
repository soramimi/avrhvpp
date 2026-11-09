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

struct IspParameters {
	uint8_t device_code;
	uint16_t eeprom_bytes;
	uint8_t eeprom_page_bytes;
	uint8_t pagel;
	uint8_t bs2;
	bool poll_ready;
	uint8_t write_wait_ms;
	uint8_t erase_wait_ms;
	uint8_t fuse_masks[3];
	uint8_t fuse_write_masks[3];
	uint8_t lock_mask;
};

struct TargetDefinition {
	char const *display_name;
	uint8_t signature[3];
	uint16_t flash_page_bytes;
	uint16_t flash_page_count;
	uint8_t capabilities;
	IspParameters isp;

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
	{0x70, 512, 4, 0xd7, 0xc2, false, 10, 20, {0xff, 0xff, 0}, {0xff, 0xdf, 0}, 0x3f},
};

constexpr TargetDefinition TargetATmega48 = {
	"ATmega48",
	{0x1e, 0x92, 0x05},
	64,
	64,
	CapabilityFlash
		| CapabilityFuseLow
		| CapabilityFuseHigh
		| CapabilityFuseExtended
		| CapabilityLockBits
		| CapabilityChipErase,
	{0x59, 256, 4, 0xd7, 0xc2, true, 0, 0, {0xff, 0xff, 0x01}, {0xff, 0xdf, 0x01}, 0x03},
};

constexpr TargetDefinition TargetATmega88 = {
	"ATmega88",
	{0x1e, 0x93, 0x0a},
	64,
	128,
	CapabilityFlash
		| CapabilityFuseLow
		| CapabilityFuseHigh
		| CapabilityFuseExtended
		| CapabilityLockBits
		| CapabilityChipErase,
	{0x73, 512, 4, 0xd7, 0xc2, true, 0, 0, {0xff, 0xff, 0x07}, {0xff, 0xdf, 0x07}, 0x3f},
};

constexpr TargetDefinition TargetATmega168 = {
	"ATmega168",
	{0x1e, 0x94, 0x06},
	128,
	128,
	CapabilityFlash
		| CapabilityFuseLow
		| CapabilityFuseHigh
		| CapabilityFuseExtended
		| CapabilityLockBits
		| CapabilityChipErase,
	{0x86, 512, 4, 0xd7, 0xc2, true, 0, 0, {0xff, 0xff, 0x07}, {0xff, 0xdf, 0x07}, 0x3f},
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
	{0x86, 1024, 4, 0xd7, 0xc2, true, 0, 0, {0xff, 0xff, 0x07}, {0xff, 0xdf, 0x07}, 0x3f},
};

inline TargetDefinition const *find_target_by_signature(
	uint8_t signature0,
	uint8_t signature1,
	uint8_t signature2)
{
	if (TargetATmega8.matches_signature(signature0, signature1, signature2)) {
		return &TargetATmega8;
	}
	if (TargetATmega48.matches_signature(signature0, signature1, signature2)) {
		return &TargetATmega48;
	}
	if (TargetATmega88.matches_signature(signature0, signature1, signature2)) {
		return &TargetATmega88;
	}
	if (TargetATmega168.matches_signature(signature0, signature1, signature2)) {
		return &TargetATmega168;
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

static_assert(TargetATmega48.flash_page_bytes <= 256, "ATmega48 flash page does not fit in buffer");
static_assert(TargetATmega48.flash_page_bytes % 2 == 0, "invalid ATmega48 flash page size");
static_assert(TargetATmega48.flash_page_words() == 32, "invalid ATmega48 flash page size");
static_assert(TargetATmega48.flash_word_address(0, 0) == 0, "invalid ATmega48 flash address");
static_assert(TargetATmega48.flash_word_address(7, 31) == 0x00ff, "invalid ATmega48 flash boundary");
static_assert(TargetATmega48.flash_word_address(8, 0) == 0x0100, "invalid ATmega48 flash boundary");
static_assert(TargetATmega48.flash_word_address(63, 31) == 0x07ff, "invalid ATmega48 last flash address");
static_assert(TargetATmega48.flash_bytes() == 4096, "invalid ATmega48 flash size");
static_assert(TargetATmega48.supports(CapabilityFuseExtended), "ATmega48 must have extended fuse");

static_assert(TargetATmega88.flash_page_bytes <= 256, "ATmega88 flash page does not fit in buffer");
static_assert(TargetATmega88.flash_page_bytes % 2 == 0, "invalid ATmega88 flash page size");
static_assert(TargetATmega88.flash_page_words() == 32, "invalid ATmega88 flash page size");
static_assert(TargetATmega88.flash_word_address(0, 0) == 0, "invalid ATmega88 flash address");
static_assert(TargetATmega88.flash_word_address(7, 31) == 0x00ff, "invalid ATmega88 flash boundary");
static_assert(TargetATmega88.flash_word_address(8, 0) == 0x0100, "invalid ATmega88 flash boundary");
static_assert(TargetATmega88.flash_word_address(127, 31) == 0x0fff, "invalid ATmega88 last flash address");
static_assert(TargetATmega88.flash_bytes() == 8192, "invalid ATmega88 flash size");
static_assert(TargetATmega88.supports(CapabilityFuseExtended), "ATmega88 must have extended fuse");

static_assert(TargetATmega168.flash_page_bytes <= 256, "ATmega168 flash page does not fit in buffer");
static_assert(TargetATmega168.flash_page_bytes % 2 == 0, "invalid ATmega168 flash page size");
static_assert(TargetATmega168.flash_page_words() == 64, "invalid ATmega168 flash page size");
static_assert(TargetATmega168.flash_word_address(0, 0) == 0, "invalid ATmega168 flash address");
static_assert(TargetATmega168.flash_word_address(3, 63) == 0x00ff, "invalid ATmega168 flash boundary");
static_assert(TargetATmega168.flash_word_address(4, 0) == 0x0100, "invalid ATmega168 flash boundary");
static_assert(TargetATmega168.flash_word_address(127, 63) == 0x1fff, "invalid ATmega168 last flash address");
static_assert(TargetATmega168.flash_bytes() == 16384, "invalid ATmega168 flash size");
static_assert(TargetATmega168.supports(CapabilityFuseExtended), "ATmega168 must have extended fuse");

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
