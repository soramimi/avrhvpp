#ifndef TARGET_H
#define TARGET_H

#include <stdint.h>

enum TargetCapability : uint16_t {
	CapabilityFlash = 1 << 0,
	CapabilityFuseLow = 1 << 1,
	CapabilityFuseHigh = 1 << 2,
	CapabilityFuseExtended = 1 << 3,
	CapabilityLockBits = 1 << 4,
	CapabilityChipErase = 1 << 5,
	CapabilityHvpp = 1 << 6,
	CapabilityUpdi = 1 << 7,
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

struct UpdiFuse {
	char const *name;
	uint8_t offset;
	uint8_t mask;
	uint8_t write_mask;
	bool (*valid_value)(uint8_t) = nullptr;
};

struct UpdiParameters {
	uint16_t sigrow_address = 0;
	uint32_t flash_address = 0;
	uint16_t nvmctrl_address = 0;
	uint16_t fuse_address = 0;
	UpdiFuse const *fuses = nullptr;
	uint8_t fuse_count = 0;
	uint16_t lock_address = 0;
	uint32_t unlocked_value = 0;
	uint8_t lock_bytes = 1;
	uint8_t nvm_version = 0;
	uint8_t pdicfg_offset = 0;
};

struct TargetDefinition {
	char const *display_name;
	uint8_t signature[3];
	uint16_t flash_page_bytes;
	uint16_t flash_page_count;
	uint8_t capabilities;
	IspParameters isp;
	UpdiParameters updi;

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
		| CapabilityChipErase
		| CapabilityHvpp,
	{0x70, 512, 4, 0xd7, 0xc2, false, 10, 20, {0xff, 0xff, 0}, {0xff, 0xdf, 0}, 0x3f},
	{}
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
		| CapabilityChipErase
		| CapabilityHvpp,
	{0x59, 256, 4, 0xd7, 0xc2, true, 0, 0, {0xff, 0xff, 0x01}, {0xff, 0xdf, 0x01}, 0x03},
	{}
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
		| CapabilityChipErase
		| CapabilityHvpp,
	{0x73, 512, 4, 0xd7, 0xc2, true, 0, 0, {0xff, 0xff, 0x07}, {0xff, 0xdf, 0x07}, 0x3f},
	{}
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
		| CapabilityChipErase
		| CapabilityHvpp,
	{0x86, 512, 4, 0xd7, 0xc2, true, 0, 0, {0xff, 0xff, 0x07}, {0xff, 0xdf, 0x07}, 0x3f},
	{}
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
		| CapabilityChipErase
		| CapabilityHvpp,
	{0x86, 1024, 4, 0xd7, 0xc2, true, 0, 0, {0xff, 0xff, 0x07}, {0xff, 0xdf, 0x07}, 0x3f},
	{}
};

// These targets are supported through AVRISP only by this programmer.
constexpr TargetDefinition TargetATtiny2313 = {
	"ATtiny2313",
	{0x1e, 0x91, 0x0a},
	32,
	64,
	CapabilityFlash
		| CapabilityFuseLow
		| CapabilityFuseHigh
		| CapabilityFuseExtended
		| CapabilityLockBits
		| CapabilityChipErase,
	{0x23, 128, 4, 0xd4, 0xd6, true, 0, 0, {0xff, 0xff, 0x01}, {0xff, 0xdf, 0x01}, 0x03},
	{},
};

constexpr TargetDefinition TargetATtiny13A = {
	"ATtiny13A",
	{0x1e, 0x90, 0x07},
	32,
	32,
	CapabilityFlash
		| CapabilityFuseLow
		| CapabilityFuseHigh
		| CapabilityLockBits
		| CapabilityChipErase,
	// SPIEN is bit 7 of the low fuse; high-fuse bits 7..5 are reserved.
	{0x14, 64, 4, 0, 0, true, 0, 0, {0xff, 0x1f, 0}, {0x7f, 0x1f, 0}, 0x03},
	{},
};

// wip: experimental UPDI support

constexpr bool attiny202_watchdog_config(uint8_t value)
{
	return (value & 0x0f) <= 0x0b && (value >> 4) <= 0x0b;
}

constexpr bool attiny202_bod_config(uint8_t value)
{
	uint8_t level = value >> 5;
	return (level == 0 || level == 2 || level == 7) && (value & 0x03) != 0x03;
}

constexpr bool attiny202_oscillator_config(uint8_t value)
{
	return (value & 0x03) == 1 || (value & 0x03) == 2;
}

constexpr bool attiny202_flash_boundary(uint8_t value)
{
	return value <= 8; // Flash is 2 KiB, in units of 256 bytes for these fuses.
}

constexpr UpdiFuse ATtiny202Fuses[] = {
	{"WDTCFG", 0x00, 0xff, 0xff, attiny202_watchdog_config},
	{"BODCFG", 0x01, 0xff, 0xff, attiny202_bod_config},
	{"OSCCFG", 0x02, 0x83, 0x03, attiny202_oscillator_config}, // Preserve OSCLOCK.
	{"SYSCFG0", 0x05, 0xcd, 0xc1}, // Preserve RSTPINCFG.
	{"SYSCFG1", 0x06, 0x07, 0x07},
	{"APPEND", 0x07, 0xff, 0xff, attiny202_flash_boundary},
	{"BOOTEND", 0x08, 0xff, 0xff, attiny202_flash_boundary},
};

constexpr TargetDefinition TargetATtiny202 = {
	"ATtiny202",
	{0x1e, 0x91, 0x23},
	64,
	32,
	CapabilityUpdi | CapabilityFlash | CapabilityLockBits | CapabilityChipErase,
	{0, 0, 0, 0, 0, false, 0, 0, {0, 0, 0}, {0, 0, 0}, 0},
	{0x1100, 0x8000, 0x1000, 0x1280, ATtiny202Fuses, sizeof(ATtiny202Fuses) / sizeof(ATtiny202Fuses[0]), 0x128a, 0xc5},
};

constexpr bool avr64d_bod_config(uint8_t value)
{
	return (value >> 5) <= 3 && (value & 0x03) != 0x03;
}

constexpr bool avr64d_oscillator_config(uint8_t value)
{
	return (value & 0x07) <= 1;
}

constexpr bool avr64d_flash_boundary(uint8_t value)
{
	return value <= 128; // Sizes are in 512-byte Flash pages.
}

constexpr UpdiFuse AVR64DDFuses[] = {
	{"WDTCFG", 0x00, 0xff, 0xff, attiny202_watchdog_config},
	{"BODCFG", 0x01, 0xff, 0xff, avr64d_bod_config},
	{"OSCCFG", 0x02, 0x07, 0x07, avr64d_oscillator_config},
	{"SYSCFG0", 0x05, 0xf9, 0xe1}, // Preserve UPDIPINCFG and RSTPINCFG.
	{"SYSCFG1", 0x06, 0x1f, 0x07}, // Preserve MVIO configuration.
	{"CODESIZE", 0x07, 0xff, 0xff, avr64d_flash_boundary},
	{"BOOTSIZE", 0x08, 0xff, 0xff, avr64d_flash_boundary},
};

constexpr UpdiFuse AVR64DUFuses[] = {
	AVR64DDFuses[0],
	AVR64DDFuses[1],
	AVR64DDFuses[2],
	{"SYSCFG0", 0x05, 0xfb, 0xe3}, // Preserve UPDIPINCFG and RSTPINCFG.
	{"SYSCFG1", 0x06, 0x0f, 0x0f},
	AVR64DDFuses[5],
	AVR64DDFuses[6],
	{"PDICFGL", 0x0a, 0xf3, 0},
	{"PDICFGH", 0x0b, 0xff, 0},
};

constexpr UpdiParameters AVR64DDUpdi = {
	0x1100, 0x800000, 0x1000, 0x1050, AVR64DDFuses,
	sizeof(AVR64DDFuses) / sizeof(AVR64DDFuses[0]), 0x1040, 0x5cc5c55c, 4, 2
};

constexpr UpdiParameters AVR64DUUpdi = {
	0x1080, 0x800000, 0x1000, 0x1050, AVR64DUFuses,
	sizeof(AVR64DUFuses) / sizeof(AVR64DUFuses[0]), 0x1040, 0x5cc5c55c, 4, 4, 0x0a
};

constexpr TargetDefinition TargetAVR64DD28 = {
	"AVR64DD28", {0x1e, 0x96, 0x1b}, 512, 128,
	CapabilityUpdi | CapabilityFlash | CapabilityLockBits | CapabilityChipErase,
	{}, AVR64DDUpdi,
};

constexpr TargetDefinition TargetAVR64DD32 = {
	"AVR64DD32", {0x1e, 0x96, 0x1a}, 512, 128,
	CapabilityUpdi | CapabilityFlash | CapabilityLockBits | CapabilityChipErase,
	{}, AVR64DDUpdi,
};

constexpr TargetDefinition TargetAVR64DU32 = {
	"AVR64DU32", {0x1e, 0x96, 0x21}, 512, 128,
	CapabilityUpdi | CapabilityFlash | CapabilityLockBits | CapabilityChipErase,
	{}, AVR64DUUpdi,
};

//

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
	if (TargetATtiny2313.matches_signature(signature0, signature1, signature2)) {
		return &TargetATtiny2313;
	}
	if (TargetATtiny13A.matches_signature(signature0, signature1, signature2)) {
		return &TargetATtiny13A;
	}
	if (TargetATtiny202.matches_signature(signature0, signature1, signature2)) {
		return &TargetATtiny202;
	}
	if (TargetAVR64DD28.matches_signature(signature0, signature1, signature2)) {
		return &TargetAVR64DD28;
	}
	if (TargetAVR64DD32.matches_signature(signature0, signature1, signature2)) {
		return &TargetAVR64DD32;
	}
	if (TargetAVR64DU32.matches_signature(signature0, signature1, signature2)) {
		return &TargetAVR64DU32;
	}
	return 0;
}

static_assert(TargetATtiny2313.flash_page_bytes == 32, "invalid ATtiny2313 flash page size");
static_assert(TargetATtiny2313.flash_bytes() == 2048, "invalid ATtiny2313 flash size");
static_assert(TargetATtiny2313.flash_word_address(63, 15) == 0x03ff, "invalid ATtiny2313 last flash address");
static_assert(TargetATtiny2313.supports(CapabilityFuseExtended), "ATtiny2313 must have extended fuse");
static_assert(!TargetATtiny2313.supports(CapabilityHvpp), "ATtiny2313 is AVRISP-only here");

static_assert(TargetATtiny13A.flash_page_bytes == 32, "invalid ATtiny13A flash page size");
static_assert(TargetATtiny13A.flash_bytes() == 1024, "invalid ATtiny13A flash size");
static_assert(TargetATtiny13A.flash_word_address(31, 15) == 0x01ff, "invalid ATtiny13A last flash address");
static_assert(!TargetATtiny13A.supports(CapabilityFuseExtended), "ATtiny13A must not have extended fuse");
static_assert(!TargetATtiny13A.supports(CapabilityHvpp), "ATtiny13A is AVRISP-only here");

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
