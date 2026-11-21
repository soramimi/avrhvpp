#include "AvrUpdi.h"
#include "Connection.h"
#include "../common/commands.h"
#include "../common/target.h"
#include <algorithm>
#include <cstdio>
#include <cstring>

namespace {
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

constexpr uint8_t Synch = 0x55;
constexpr uint8_t Ack = 0x40;
constexpr uint8_t StatusA = 0x00;
constexpr uint8_t CtrlA = 0x02;
constexpr uint8_t CtrlB = 0x03;
constexpr uint8_t KeyStatus = 0x07;
constexpr uint8_t ResetRequest = 0x08;
constexpr uint8_t SystemStatus = 0x0b;
constexpr uint8_t Locked = 0x01;
constexpr uint8_t NvmProgramming = 0x08;
constexpr uint8_t SystemReset = 0x20;
constexpr uint8_t NvmKeyAccepted = 0x10;
constexpr uint8_t EraseKeyAccepted = 0x08;
constexpr uint8_t NvmBusy = 0x03;
constexpr uint8_t P0NvmWriteError = 0x04;
constexpr uint8_t DxNvmWriteError = 0x70;
constexpr uint16_t NvmStatus = 0x02;
constexpr uint16_t P4NvmStatus = 0x06;
constexpr uint16_t NvmData = 0x06;
constexpr uint16_t NvmAddress = 0x08;
constexpr uint8_t WritePage = 0x01;
constexpr uint8_t ClearPageBuffer = 0x04;
constexpr uint8_t ChipErase = 0x05;
constexpr uint8_t WriteFuse = 0x07;
constexpr uint8_t NoCommand = 0x00;
constexpr uint8_t FlashWrite = 0x02;
constexpr uint8_t EepromEraseWrite = 0x13;
constexpr uint8_t DxChipErase = 0x20;
constexpr auto ResponseTimeout = 10ms;
constexpr auto StatusTimeout = 100ms;
constexpr auto EraseTimeout = 500ms;
constexpr int SibSize = 32;

bool fail(char const *message)
{
	fprintf(stderr, "%s\n", message);
	return false;
}

int remaining_ms(Clock::time_point deadline)
{
	return static_cast<int>(std::chrono::ceil<std::chrono::milliseconds>(deadline - Clock::now()).count());
}
}

AvrUpdi::AvrUpdi(Connection *conn, TargetDefinition const &target)
	: conn_(conn), target_(target)
{
}

bool AvrUpdi::write(uint8_t const *data, int size)
{
	while (size > 0) {
		int sent = conn_->write(data, size);
		if (sent <= 0 || sent > size) {
			wire_ok_ = false;
			return fail("failed to send UPDI request");
		}
		data += sent;
		size -= sent;
	}
	return true;
}

bool AvrUpdi::send(uint8_t const *data, int size)
{
	// The longest outgoing frame is SYNCH, KEY and eight key bytes.
	uint8_t encoded[20];
	for (int i = 0; i < size; ++i) {
		encoded[2 * i] = CMD_LOAD_7BITS | (data[i] & 0x7f);
		encoded[2 * i + 1] = data[i] & 0x80 ? CMD_UPDI_SEND1 : CMD_UPDI_SEND0;
	}
	return write(encoded, 2 * size);
}

bool AvrUpdi::receive(uint8_t *data, int size, Deadline deadline)
{
	while (size > 0) {
		int ms = remaining_ms(deadline);
		if (ms <= 0) return false;
		int received = conn_->read(data, size, ms);
		if (received <= 0 || received > size) return false;
		data += received;
		size -= received;
	}
	return true;
}

bool AvrUpdi::drain(Deadline deadline)
{
	uint8_t data[64];
	for (int ms = remaining_ms(deadline); ms > 0; ms = remaining_ms(deadline)) {
		int received = conn_->read(data, sizeof(data), ms);
		if (received <= 0) return received == 0;
	}
	return true;
}

bool AvrUpdi::exchange(uint8_t const *request, int size, uint8_t *reply, int reply_size, Deadline deadline)
{
	uint8_t echo[10];
	return send(request, size)
		&& receive(echo, size, deadline)
		&& memcmp(echo, request, size) == 0
		&& receive(reply, reply_size, deadline);
}

int AvrUpdi::load(uint32_t addr, Deadline deadline)
{
	bool wide = target_.updi.nvm_version != 0;
	uint8_t request[] = {Synch, uint8_t(wide ? 0x08 : 0x04), uint8_t(addr), uint8_t(addr >> 8), uint8_t(addr >> 16)};
	uint8_t value;
	return exchange(request, wide ? 5 : 4, &value, 1, deadline) ? value : -1;
}

bool AvrUpdi::store(uint32_t addr, uint8_t const *data, int size)
{
	bool wide = target_.updi.nvm_version != 0;
	uint8_t opcode = wide ? 0x48 : 0x44;
	uint8_t request[] = {Synch, uint8_t(opcode | (size - 1)), uint8_t(addr), uint8_t(addr >> 8), uint8_t(addr >> 16)};
	uint8_t ack;
	if (!exchange(request, wide ? 5 : 4, &ack, 1, Clock::now() + ResponseTimeout) || ack != Ack) {
		// The target may be waiting for the data phase of STS.
		wire_ok_ = false;
		return fail("UPDI store address was not acknowledged");
	}
	if (!exchange(data, size, &ack, 1, Clock::now() + ResponseTimeout) || ack != Ack) {
		return fail("UPDI store data was not acknowledged; write outcome may be unknown");
	}
	return true;
}

bool AvrUpdi::store_byte(uint32_t addr, uint8_t value)
{
	return store(addr, &value, 1);
}

int AvrUpdi::read_cs(uint8_t addr, Deadline deadline)
{
	uint8_t request[] = {Synch, uint8_t(0x80 | addr)};
	uint8_t value;
	return exchange(request, sizeof(request), &value, 1, deadline) ? value : -1;
}

bool AvrUpdi::write_cs(uint8_t addr, uint8_t value)
{
	uint8_t request[] = {Synch, uint8_t(0xc0 | addr), value};
	return exchange(request, sizeof(request), nullptr, 0, Clock::now() + ResponseTimeout);
}

bool AvrUpdi::initialize_link()
{
	uint8_t command = CMD_UPDI_BREAK;
	for (int attempt = 0; attempt <= 10; ++attempt) {
		// The bridge holds BREAK for 50 ms. Send SYNCH shortly after it releases the line.
		if (!write(&command, 1) || !drain(Clock::now() + 51ms)) break;
		if (!write_cs(CtrlB, 0x08) || !write_cs(CtrlA, 0x80)) {
			if (!wire_ok_) break;
			continue;
		}
		if (read_cs(StatusA, Clock::now() + ResponseTimeout) < 0) {
			if (!wire_ok_) break;
			continue;
		}
		int status = read_cs(SystemStatus, Clock::now() + ResponseTimeout);
		if (status >= 0) {
			locked_ = status & Locked;
			return true;
		}
		if (!wire_ok_) break;
	}
	return fail("UPDI link initialization failed");
}

bool AvrUpdi::read_sib(uint8_t *data)
{
	uint8_t request[] = {Synch, 0xe6};
	return exchange(request, sizeof(request), data, SibSize, Clock::now() + StatusTimeout)
		|| fail("failed to read UPDI System Information Block");
}

bool AvrUpdi::send_key(char const (&key)[9], uint8_t status_mask)
{
	uint8_t request[] = {Synch, 0xe0, 0, 0, 0, 0, 0, 0, 0, 0};
	for (int i = 0; i < 8; ++i) request[i + 2] = key[7 - i];
	if (!exchange(request, sizeof(request), nullptr, 0, Clock::now() + ResponseTimeout)) {
		return fail("failed to verify UPDI key echo");
	}
	int status = read_cs(KeyStatus, Clock::now() + ResponseTimeout);
	return (status >= 0 && (status & status_mask)) || fail("UPDI key was not accepted");
}

bool AvrUpdi::reset_target()
{
	bool asserted = write_cs(ResetRequest, 0x59);
	bool released = wire_ok_ && write_cs(ResetRequest, 0x00);
	return (asserted && released) || fail("failed to verify UPDI reset request or release");
}

bool AvrUpdi::wait_system_status(uint8_t mask, uint8_t value, Deadline deadline)
{
	while (Clock::now() < deadline) {
		int status = read_cs(SystemStatus, std::min(deadline, Clock::now() + ResponseTimeout));
		if (status < 0) return fail("failed to read UPDI system status");
		if ((status & mask) == value) return true;
	}
	return fail("UPDI system status timed out");
}

bool AvrUpdi::enter_programming_mode()
{
	reset_needed_ = true;
	if (!send_key("NVMProg ", NvmKeyAccepted) || !reset_target()
		|| !wait_system_status(SystemReset | NvmProgramming | Locked, NvmProgramming, Clock::now() + StatusTimeout)) return false;
	programming_ = true;
	return true;
}

bool AvrUpdi::begin()
{
	bridge_attempted_ = true;
	if (!conn_->enter_avrupdi_mode()) return fail("failed to enter programmer UPDI mode");
	bridge_entered_ = true;
	return initialize_link();
}

bool AvrUpdi::finish()
{
	if (!bridge_attempted_) return true;
	bool finished = true;
	if (bridge_entered_) {
		if (wire_ok_) {
			if (!drain(Clock::now() + ResponseTimeout)) finished = false;
			if (reset_needed_ && !reset_target()) finished = false;
			if (!wire_ok_ || !write_cs(CtrlB, 0x0c)) finished = false; // CCDETDIS | UPDIDIS
		} else {
			finished = false;
		}
		if (!finished) fail("failed to verify target UPDI cleanup");
	}
	if (!conn_->sync()) {
		fail("failed to return programmer to STK500 mode");
		finished = false;
	}
	bridge_attempted_ = false;
	bridge_entered_ = false;
	return finished;
}

bool AvrUpdi::memory_access() const
{
	return (!locked_ && programming_) || fail("UPDI memory access is unavailable while the target is locked or outside NVM programming mode");
}

bool AvrUpdi::read_signature(uint8_t *signature)
{
	if (locked_) return fail("target is locked; UPDI signature row is unavailable");
	if (!programming_ && !enter_programming_mode()) return false;
	for (int i = 0; i < 3; ++i) {
		int byte = load(target_.updi.sigrow_address + i, Clock::now() + ResponseTimeout);
		if (byte < 0) return fail("failed to read UPDI signature");
		signature[i] = static_cast<uint8_t>(byte);
	}
	return true;
}

bool AvrUpdi::verify_identity()
{
	if (identity_verified_) return true;
	uint8_t signature[3];
	if (!read_signature(signature)) return false;
	if (!target_.matches_signature(signature[0], signature[1], signature[2])) {
		fprintf(stderr, "UPDI signature %02x%02x%02x does not match %s\n",
			signature[0], signature[1], signature[2], target_.display_name);
		return false;
	}
	identity_verified_ = true;
	return true;
}

bool AvrUpdi::read_lock(bool *locked)
{
	int status = read_cs(SystemStatus, Clock::now() + ResponseTimeout);
	if (status < 0) return fail("failed to read UPDI lock status");
	locked_ = status & Locked;
	*locked = locked_;
	return true;
}

bool AvrUpdi::read_fuse(UpdiFuse const &fuse, uint8_t *value)
{
	if (!verify_identity() || !memory_access()) return false;
	int byte = load(target_.updi.fuse_address + fuse.offset, Clock::now() + ResponseTimeout);
	if (byte < 0) return fail("failed to read UPDI fuse");
	*value = static_cast<uint8_t>(byte);
	return true;
}

bool AvrUpdi::wait_nvm()
{
	auto deadline = Clock::now() + StatusTimeout;
	auto address = target_.updi.nvmctrl_address + (target_.updi.nvm_version == 4 ? P4NvmStatus : NvmStatus);
	uint8_t error_mask = target_.updi.nvm_version == 0 ? P0NvmWriteError : DxNvmWriteError;
	while (Clock::now() < deadline) {
		int status = load(address,
			std::min(deadline, Clock::now() + ResponseTimeout));
		if (status < 0) return fail("failed to read UPDI NVM status");
		if (status & NvmBusy) continue;
		return !(status & error_mask) || fail("UPDI NVM controller reported a write error");
	}
	return fail("UPDI NVM controller timed out");
}

bool AvrUpdi::issue_nvm_command(uint8_t command)
{
	return store_byte(target_.updi.nvmctrl_address, command) && wait_nvm();
}

bool AvrUpdi::program_fuse(uint16_t address, uint8_t value)
{
	if (target_.updi.nvm_version != 0) return program_eeprom_cells(address, &value, 1);
	uint16_t nvm = target_.updi.nvmctrl_address;
	// Load ADDRL and ADDRH with separate byte stores before issuing WFU.
	return wait_nvm() && store_byte(nvm + NvmAddress, uint8_t(address))
		&& store_byte(nvm + NvmAddress + 1, uint8_t(address >> 8))
		&& store_byte(nvm + NvmData, value) && issue_nvm_command(WriteFuse);
}

bool AvrUpdi::program_eeprom_cells(uint16_t address, uint8_t const *data, uint8_t count)
{
	uint16_t nvm = target_.updi.nvmctrl_address;
	if (!wait_nvm() || !store_byte(nvm, EepromEraseWrite)) return false;
	for (uint8_t i = 0; i < count; ++i) {
		if (!store_byte(address + i, data[i]) || !wait_nvm()) return false;
	}
	return store_byte(nvm, NoCommand);
}

bool AvrUpdi::write_lock(uint32_t value)
{
	if (value == target_.updi.unlocked_value) return fail("UPDI lock cannot be cleared using --write-lock; use --erase-locked to erase and unlock");
	if (!verify_identity() || !memory_access()) return false;
	auto const &updi = target_.updi;
	if (updi.pdicfg_offset) {
		uint16_t address = updi.fuse_address + updi.pdicfg_offset;
		int low = load(address, Clock::now() + ResponseTimeout);
		int high = load(address + 1, Clock::now() + ResponseTimeout);
		if (low < 0 || high < 0) return fail("failed to read UPDI PDICFG before locking");
		uint16_t config = (uint16_t(high) << 8) | uint16_t(low);
		// On AVR DU a previously programmed PDICFG can disable chip erase on reset.
		if ((config >> 4) == 0xb45 && (config & 0x03) != 0x03) {
			return fail("UPDI lock refused: PDICFG would restrict programming access after reset");
		}
	}
	if (updi.lock_bytes == 1) {
		if (value > 0xff || !program_fuse(updi.lock_address, uint8_t(value))) return false;
	} else {
		uint8_t key[] = {uint8_t(value), uint8_t(value >> 8), uint8_t(value >> 16), uint8_t(value >> 24)};
		if (!program_eeprom_cells(updi.lock_address, key, sizeof(key))) return false;
	}
	if (!reset_target()) return false;
	reset_needed_ = false;
	programming_ = false;
	identity_verified_ = false;
	// Once the lock takes effect, only UPDI control/status space is accessible.
	if (!wait_system_status(SystemReset | Locked, Locked, Clock::now() + StatusTimeout)) {
		return fail("failed to confirm UPDI lock after reset; check with --read-lock");
	}
	locked_ = true;
	return true;
}

bool AvrUpdi::write_fuse(UpdiFuse const &fuse, uint8_t value)
{
	if (!fuse.write_mask) return fail("UPDI fuse is read-only");
	if (fuse.valid_value && !fuse.valid_value(value)) return fail("invalid UPDI fuse value");
	uint8_t current;
	if (!read_fuse(fuse, &current)) return false;
	if ((current ^ value) & (fuse.mask & ~fuse.write_mask)) return fail("requested change to a protected UPDI fuse bit");
	value = (current & ~fuse.write_mask) | (value & fuse.write_mask);
	if (value == current) return true;

	if (!program_fuse(target_.updi.fuse_address + fuse.offset, value)) return false;
	uint8_t actual;
	return read_fuse(fuse, &actual)
		&& (((value ^ actual) & fuse.mask) == 0 || fail("UPDI fuse verification failed"));
}

bool AvrUpdi::page_address(uint16_t page, uint32_t *address) const
{
	if (page >= target_.flash_page_count) return fail("UPDI Flash page is out of range");
	uint32_t base = target_.updi.flash_address + uint32_t(page) * target_.flash_page_bytes;
	uint32_t limit = target_.updi.nvm_version == 0 ? 0x10000UL : 0x1000000UL;
	if (base + target_.flash_page_bytes > limit) return fail("UPDI Flash page exceeds the target address space");
	*address = base;
	return true;
}

bool AvrUpdi::read_flash_block(uint32_t address, uint8_t *data, int size)
{
	// ST pointer (24-bit), followed by REPEAT and LD with post-increment.
	uint8_t pointer[] = {Synch, 0x6a, uint8_t(address), uint8_t(address >> 8), uint8_t(address >> 16)};
	uint8_t ack;
	if (!exchange(pointer, sizeof(pointer), &ack, 1, Clock::now() + ResponseTimeout) || ack != Ack) return false;
	while (size > 0) {
		// Keep each burst below the bridge's 128-byte receive ring.
		int count = std::min(size, 64);
		uint8_t repeat[] = {Synch, 0xa0, uint8_t(count - 1)};
		uint8_t read[] = {Synch, 0x24};
		if (!exchange(repeat, sizeof(repeat), nullptr, 0, Clock::now() + ResponseTimeout)
			|| !exchange(read, sizeof(read), data, count, Clock::now() + StatusTimeout)) {
			wire_ok_ = false;
			return false;
		}
		data += count;
		size -= count;
	}
	return true;
}

bool AvrUpdi::read_page(uint16_t page, std::vector<uint8_t> *data)
{
	uint32_t address;
	if (!page_address(page, &address) || !verify_identity() || !memory_access()) return false;
	data->resize(target_.flash_page_bytes);
	if (target_.updi.nvm_version != 0) {
		return read_flash_block(address, data->data(), target_.flash_page_bytes) || fail("failed to read UPDI Flash page");
	}
	for (auto &byte : *data) {
		int value = load(address++, Clock::now() + ResponseTimeout);
		if (value < 0) return fail("failed to read UPDI Flash page");
		byte = static_cast<uint8_t>(value);
	}
	return true;
}

bool AvrUpdi::write_page(uint16_t page, uint8_t const *data, size_t size)
{
	uint32_t address;
	if (size != target_.flash_page_bytes || !page_address(page, &address)) return false;
	std::vector<uint8_t> current;
	if (!read_page(page, &current)) return false;
	if (std::equal(current.begin(), current.end(), data)) return true;
	if (std::any_of(current.begin(), current.end(), [](uint8_t byte) { return byte != 0xff; })) {
		return fail("UPDI Flash page is not blank and differs from the input");
	}
	bool buffered = target_.updi.nvm_version == 0;
	auto nvm = target_.updi.nvmctrl_address;
	if (!wait_nvm()) return false;
	if (buffered ? !issue_nvm_command(ClearPageBuffer) : !store_byte(nvm, FlashWrite)) return false;
	for (size_t offset = 0; offset < size; offset += 2) {
		if (!buffered && data[offset] == 0xff && data[offset + 1] == 0xff) continue;
		if (!store(address + offset, data + offset, 2)) return false;
	}
	if (buffered ? !issue_nvm_command(WritePage) : !(wait_nvm() && store_byte(nvm, NoCommand))) return false;
	if (!read_page(page, &current)) return false;
	return std::equal(current.begin(), current.end(), data) || fail("UPDI Flash page verification failed");
}

bool AvrUpdi::verify_erased()
{
	std::vector<uint8_t> data;
	for (uint16_t page = 0; page < target_.flash_page_count; ++page) {
		if (!read_page(page, &data)) return false;
		if (std::any_of(data.begin(), data.end(), [](uint8_t byte) { return byte != 0xff; })) {
			return fail("UPDI chip erase verification failed; Flash is not blank");
		}
	}
	return true;
}

bool AvrUpdi::chip_erase()
{
	bool buffered = target_.updi.nvm_version == 0;
	if (!verify_identity() || !wait_nvm() || !issue_nvm_command(buffered ? ChipErase : DxChipErase)) return false;
	if (!buffered && !store_byte(target_.updi.nvmctrl_address, NoCommand)) return false;
	bool locked;
	return read_lock(&locked) && (!locked || fail("target is still locked after chip erase")) && verify_erased();
}

bool AvrUpdi::erase_locked()
{
	if (!locked_) return fail("--erase-locked requires a locked UPDI target");
	uint8_t sib[SibSize];
	if (!read_sib(sib)) return false;
	// SIB identifies the family and NVM protocol, but cannot confirm the chip model.
	uint8_t version = target_.updi.nvm_version;
	if (memcmp(sib + 8, "P:", 2) != 0 || sib[10] != '0' + version
		|| (version == 0 && memcmp(sib, "tinyAVR", 7) != 0)) {
		return fail("locked target does not report the expected UPDI NVM protocol");
	}
	fprintf(stderr, "Erasing locked target: its signature cannot be checked first; Flash and EEPROM%s will be erased.\n",
		version == 4 ? " and Boot Row" : "");
	if (!send_key("NVMErase", EraseKeyAccepted) || !reset_target()
		|| !wait_system_status(SystemReset | Locked, 0, Clock::now() + EraseTimeout)) return false;
	locked_ = false;
	return (verify_identity() && verify_erased()) || fail("target was unlocked; post-erase verification failed");
}

bool AvrUpdi::debug(std::string *text)
{
	uint8_t sib[SibSize];
	uint8_t signature[3];
	if (!read_sib(sib) || !read_signature(signature)) return false;
	char value[32];
	snprintf(value, sizeof(value), "signature: 0x%02x%02x%02x\n", signature[0], signature[1], signature[2]);
	*text += "SIB: ";
	auto end = std::find(sib, sib + sizeof(sib), uint8_t(0));
	text->append(reinterpret_cast<char const *>(sib), end - sib);
	*text += '\n';
	*text += value;
	return true;
}
