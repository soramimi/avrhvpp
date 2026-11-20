#include "AvrIsp.h"

#include "Connection.h"



bool AvrIsp::write_request()
{
	auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_);
	while (sent_ < request_.size()) {
		if (std::chrono::steady_clock::now() >= deadline) {
			error_ = "request timeout";
			return false;
		}
		int remaining = (int)(request_.size() - sent_);
		int n = conn_->write(request_.data() + sent_, remaining);
		if (n <= 0 || n > remaining) {
			error_ = "write failed";
			return false;
		}
		sent_ += n;
	}
	return true;
}

bool AvrIsp::read_exact(uint8_t *data, int size, std::chrono::steady_clock::time_point deadline)
{
	while (size > 0) {
		auto remain = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count();
		if (remain <= 0) {
			error_ = "response timeout";
			return false;
		}
		int n = conn_->read(data, size, (int)remain);
		if (n < 0 || n > size) {
			error_ = "read failed";
			return false;
		}
		data += n;
		size -= n;
	}
	return true;
}

bool AvrIsp::discard_input()
{
	uint8_t bytes[64];
	for (int total = 0; total < 1024;) {
		int n = conn_->read(bytes, sizeof(bytes), 0);
		if (n == 0) return true;
		if (n < 0 || n > (int)sizeof(bytes)) {
			error_ = "read failed";
			return false;
		}
		total += n;
	}
	error_ = "unexpected input during synchronization";
	return false;
}

bool AvrIsp::exchange(uint8_t command, const uint8_t *arguments, int argument_size, uint8_t *data, int size, int response_timeout)
{
	request_.assign(1, command);
	if (argument_size) request_.insert(request_.end(), arguments, arguments + argument_size);
	request_.push_back(EndOfPacket);
	sent_ = 0;
	if (!write_request()) return false;
	auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(response_timeout < 0 ? timeout_ : response_timeout);
	uint8_t status;
	if (!read_exact(&status, 1, deadline)) return false;
	if (status != InSync) {
		error_ = "expected STK_INSYNC";
		return false;
	}
	if (!read_exact(data, size, deadline) || !read_exact(&status, 1, deadline)) return false;
	if (status != Ok) {
		error_ = "expected STK_OK";
		return false;
	}
	return true;
}

bool AvrIsp::fail(const char *operation)
{
	fprintf(stderr, "AVRISP %s: %s\n", operation, error_);
	failed_ = true;
	return false;
}

bool AvrIsp::recover_request()
{
	if (!usable_) return false;
	if (sent_ == 0 || sent_ == request_.size()) return true;
	// Only these two-byte control requests can be safely completed.
	if (request_.size() == 2 && (request_[0] == GetSync || request_[0] == EnterProgramming || request_[0] == LeaveProgramming || request_[0] == ReadSignature)) {
		if (write_request()) return true;
	}
	error_ = "incomplete request; programming mode release cannot be confirmed; connection closed";
	fail("finish session");
	conn_->close();
	usable_ = false;
	return false;
}

bool AvrIsp::execute(const char *operation, uint8_t command, const uint8_t *arguments, int argument_size, uint8_t *data, int size, int response_timeout)
{
	if (failed_) return false;
	if ((argument_size && !arguments) || (size && !data)) {
		error_ = "missing request or response buffer";
		return fail(operation);
	}
	if (!active_) {
		error_ = "no active programming session";
		return fail(operation);
	}
	return exchange(command, arguments, argument_size, data, size, response_timeout) || fail(operation);
}

bool AvrIsp::configure_device()
{
	auto const &isp = target_->isp;
	uint16_t page_bytes = target_->flash_page_bytes;
	uint32_t flash_bytes = target_->flash_bytes();
	uint8_t parameters[] = {
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																  isp.device_code, 0, 0, 1, 1, 1, 1,
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																  (uint8_t)(target_->supports(CapabilityFuseExtended) ? 3 : 2),
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																  0xff, 0xff, 0xff, 0xff,
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																  (uint8_t)(page_bytes >> 8), (uint8_t)page_bytes,
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																  (uint8_t)(isp.eeprom_bytes >> 8), (uint8_t)isp.eeprom_bytes,
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																  (uint8_t)(flash_bytes >> 24), (uint8_t)(flash_bytes >> 16),
																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																  (uint8_t)(flash_bytes >> 8), (uint8_t)flash_bytes,
	};
	uint8_t extended[] = {5, isp.eeprom_page_bytes, isp.pagel, isp.bs2, 1};
	return (exchange(SetDevice, parameters, sizeof(parameters), nullptr, 0)
			&& exchange(SetDeviceExtended, extended, sizeof(extended), nullptr, 0)) || fail("configure device");
}

bool AvrIsp::validate_page(uint16_t page)
{
	if (failed_) return false;
	if (!target_ || page >= target_->flash_page_count) {
		error_ = "page requires a configured target and an in-range page number";
		return fail("Flash parameters");
	}
	return true;
}

bool AvrIsp::require_capability(TargetCapability capability)
{
	if (failed_) return false;
	if (!target_ || !target_->supports(capability)) {
		error_ = "operation requires a configured, supported target";
		return fail("target capabilities");
	}
	return true;
}

bool AvrIsp::wait_ready(bool erase)
{
	if (failed_) return false;
	if (!target_->isp.poll_ready) {
		msleep(erase ? target_->isp.erase_wait_ms : target_->isp.write_wait_ms);
		return true;
	}
	auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ready_timeout_);
	while (true) {
		auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count();
		if (remaining <= 0) break;
		uint8_t instruction[] = {0xf0, 0, 0, 0}, result;
		int response_timeout = remaining < timeout_ ? (int)remaining : timeout_;
		if (!execute("poll RDY/BSY", Universal, instruction, sizeof(instruction), &result, 1, response_timeout)) return false;
		if (std::chrono::steady_clock::now() >= deadline) break;
		if (!(result & 1)) return true;
		msleep(1);
	}
	error_ = "RDY/BSY timeout";
	return fail("wait for completion");
}

AvrIsp::AvrIsp(Connection *conn, int timeout, int ready_timeout)
	: conn_(conn)
	, timeout_(timeout)
	, ready_timeout_(ready_timeout)
{
}

AvrIsp::~AvrIsp() { if (begun_ && !finished_) finish(); }

bool AvrIsp::begin(const TargetDefinition *target)
{
	if (begun_ || failed_ || finished_) {
		error_ = "session already started or failed";
		return fail("begin session");
	}
	begun_ = true;
	target_ = target;
	if (target && (target->isp.device_code >= 0xe0 || !target->flash_page_count
				   || target->flash_page_bytes < 2 || target->flash_page_bytes > FlashBufferBytes
				   || target->flash_page_bytes % 2 || target->flash_bytes() > 131072UL
				   || (!target->isp.poll_ready && (!target->isp.write_wait_ms || !target->isp.erase_wait_ms)))) {
		error_ = "invalid target parameters";
		return fail("configure device");
	}
	bool synced = false;
	for (int attempt = 0; attempt < 3; attempt++) {
		if (!discard_input()) break;
		if (exchange(GetSync, nullptr, 0, nullptr, 0)) {
			synced = true;
			conn_->mode_ = Connection::STK500;
			break;
		}
		if (!recover_request()) return false;
	}
	if (!synced) return fail("synchronize");
	if (target && !configure_device()) return false;
	enter_attempted_ = true;
	if (!exchange(EnterProgramming, nullptr, 0, nullptr, 0)) return fail("enter programming mode");
	active_ = true;
	return true;
}

bool AvrIsp::finish()
{
	if (finished_) return !failed_;
	finished_ = true;
	active_ = false;
	if (!recover_request()) return false;
	if (failed_ && !discard_input()) fail("discard failed response");
	if (enter_attempted_) {
		if (!exchange(LeaveProgramming, nullptr, 0, nullptr, 0)) {
			fail("leave programming mode");
			if (recover_request()) {
				discard_input();
				if (!exchange(LeaveProgramming, nullptr, 0, nullptr, 0)) {
					fail("retry leaving programming mode");
					error_ = "programming mode release cannot be confirmed; connection closed";
					fail("finish session");
					conn_->close();
					usable_ = false;
				}
			}
		}
	}
	return !failed_;
}

bool AvrIsp::read_signature(uint8_t *signature)
{
	return execute("read signature", ReadSignature, nullptr, 0, signature, 3);
}

bool AvrIsp::universal(const uint8_t *instruction, uint8_t *result)
{
	return execute("Universal", Universal, instruction, 4, result, 1);
}

bool AvrIsp::read_fuse(uint8_t fuse, uint8_t *value)
{
	if (fuse >= 3) {
		error_ = "invalid fuse index";
		return fail("read fuse");
	}
	uint8_t instruction[] = {(uint8_t)(fuse == 1 ? 0x58 : 0x50), (uint8_t)(fuse == 0 ? 0 : 0x08), 0, 0};
	return universal(instruction, value);
}

bool AvrIsp::read_lock(uint8_t *value)
{
	uint8_t instruction[] = {0x58, 0, 0, 0};
	return universal(instruction, value);
}

bool AvrIsp::write_fuse(uint8_t fuse, uint8_t value)
{
	if (fuse > 2) {
		error_ = "invalid fuse index";
		return fail("write fuse");
	}
	TargetCapability capability = fuse == 0 ? CapabilityFuseLow : fuse == 1 ? CapabilityFuseHigh : CapabilityFuseExtended;
	if (!require_capability(capability)) return false;
	uint8_t mask = target_->isp.fuse_masks[fuse];
	uint8_t current;
	if (!read_fuse(fuse, &current)) return false;
	if ((current ^ value) & mask & (uint8_t)~target_->isp.fuse_write_masks[fuse]) {
		error_ = "requested fuse change is not possible through ISP (SPIEN)";
		return fail("write fuse");
	}
	value |= (uint8_t)~mask;
	if (!((current ^ value) & mask)) return true;
	uint8_t const commands[] = {0xa0, 0xa8, 0xa4};
	uint8_t instruction[] = {0xac, commands[fuse], 0, value}, result;
	if (!universal(instruction, &result) || !wait_ready() || !read_fuse(fuse, &current)) return false;
	if ((current ^ value) & mask) {
		error_ = "fuse read-back verification failed";
		return fail("write fuse");
	}
	return true;
}

bool AvrIsp::write_lock(uint8_t value)
{
	if (!require_capability(CapabilityLockBits)) return false;
	uint8_t current, mask = target_->isp.lock_mask;
	if (!read_lock(&current)) return false;
	value |= (uint8_t)~mask;
	if (value & (uint8_t)~current & mask) {
		error_ = "lock bits cannot change from 0 to 1 without an explicit chip erase";
		return fail("write lock bits");
	}
	if (!((current ^ value) & mask)) return true;
	uint8_t instruction[] = {0xac, 0xe0, 0, value}, result;
	if (!universal(instruction, &result) || !wait_ready() || !read_lock(&current)) return false;
	if ((current ^ value) & mask) {
		error_ = "lock bits read-back verification failed";
		return fail("write lock bits");
	}
	return true;
}

bool AvrIsp::chip_erase()
{
	if (!require_capability(CapabilityChipErase)) return false;
	uint8_t instruction[] = {0xac, 0x80, 0, 0}, result, lock;
	if (!universal(instruction, &result) || !wait_ready(true) || !read_lock(&lock)) return false;
	if ((lock & target_->isp.lock_mask) != target_->isp.lock_mask) {
		error_ = "lock bits remain programmed after chip erase";
		return fail("chip erase");
	}
	std::vector<uint8_t> page(target_->flash_page_bytes);
	for (uint16_t i = 0; i < target_->flash_page_count; ++i) {
		if (!read_page(i, page.data())) return false;
		for (uint8_t byte : page) {
			if (byte != 0xff) {
				error_ = "Flash is not blank after chip erase";
				return fail("chip erase");
			}
		}
	}
	return true;
}

bool AvrIsp::write_page(uint16_t page, const uint8_t *data, size_t size)
{
	if (!require_capability(CapabilityFlash) || !validate_page(page)) return false;
	if (!data || size != target_->flash_page_bytes) {
		error_ = "data length does not match target page size";
		return fail("write page");
	}
	std::vector<uint8_t> current(size);
	if (!read_page(page, current.data())) return false;
	if (!memcmp(data, current.data(), size)) return true;
	for (uint8_t byte : current) {
		if (byte != 0xff) {
			error_ = "Flash page is not blank and does not match the input";
			return fail("write page");
		}
	}
	if (!program_page(page, data, size) || !wait_ready() || !read_page(page, current.data())) return false;
	if (memcmp(data, current.data(), size)) {
		error_ = "Flash page read-back verification failed";
		return fail("write page");
	}
	return true;
}

bool AvrIsp::load_address(uint16_t word_address)
{
	uint8_t arguments[] = {(uint8_t)word_address, (uint8_t)(word_address >> 8)};
	return execute("load address", LoadAddress, arguments, sizeof(arguments), nullptr, 0);
}

bool AvrIsp::read_page(uint16_t page, uint8_t *data)
{
	if (!validate_page(page)) return false;
	uint16_t size = target_->flash_page_bytes;
	uint8_t arguments[] = {(uint8_t)(size >> 8), (uint8_t)size, 'F'};
	return load_address((uint16_t)target_->flash_word_address(page, 0))
		   && execute("read page", ReadPage, arguments, sizeof(arguments), data, size);
}

bool AvrIsp::program_page(uint16_t page, const uint8_t *data, size_t size)
{
	if (!validate_page(page)) return false;
	if (!data || size != target_->flash_page_bytes) {
		error_ = "data length does not match target page size";
		return fail("program page");
	}
	std::vector<uint8_t> arguments = {(uint8_t)(size >> 8), (uint8_t)size, 'F'};
	arguments.insert(arguments.end(), data, data + size);
	return load_address((uint16_t)target_->flash_word_address(page, 0))
		   && execute("program page", ProgramPage, arguments.data(), (int)arguments.size(), nullptr, 0);
}

