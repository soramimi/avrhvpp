#ifndef AVRISP_H
#define AVRISP_H

#include "../common/commands.h"
#include "target.h"
#include "main.h"
#include <vector>
#include <chrono>

class Connection;

class AvrIsp {
private:
	enum : uint8_t {
		GetSync = 0x30,
		SetDevice = 0x42,
		SetDeviceExtended = 0x45,
		EnterProgramming = 0x50,
		LeaveProgramming = 0x51,
		LoadAddress = 0x55,
		Universal = 0x56,
		ProgramPage = 0x64,
		ReadPage = 0x74,
		ReadSignature = 0x75,
		EndOfPacket = 0x20,
		InSync = 0x14,
		Ok = 0x10,
	};
	Connection *conn_;
	int timeout_;
	int ready_timeout_;
	char const *error_ = nullptr;
	TargetDefinition const *target_ = nullptr;
	std::vector<uint8_t> request_;
	size_t sent_ = 0;
	bool begun_ = false, active_ = false, enter_attempted_ = false;
	bool failed_ = false, finished_ = false, usable_ = true;
	
	bool write_request();
	
	bool read_exact(uint8_t *data, int size, std::chrono::steady_clock::time_point deadline);
	
	bool discard_input();
	
	bool exchange(uint8_t command, uint8_t const *arguments, int argument_size, uint8_t *data, int size, int response_timeout = -1);
	
	bool fail(char const *operation);
	
	bool recover_request();
	
	bool execute(char const *operation, uint8_t command, uint8_t const *arguments, int argument_size, uint8_t *data, int size, int response_timeout = -1);
	
	bool configure_device();
	
	bool validate_page(uint16_t page);
	bool require_capability(TargetCapability capability);
	
	bool wait_ready(bool erase = false);
	
public:
	explicit AvrIsp(Connection *conn, int timeout = COMMAND_TIMEOUT_MS, int ready_timeout = ISP_READY_TIMEOUT_MS);
	AvrIsp(AvrIsp const &) = delete;
	AvrIsp &operator=(AvrIsp const &) = delete;
	~AvrIsp();
	
	bool begin(TargetDefinition const *target = nullptr);
	
	bool finish();
	
	bool read_signature(uint8_t *signature);
	
	bool universal(uint8_t const *instruction, uint8_t *result);
	
	bool read_fuse(uint8_t fuse, uint8_t *value);
	
	bool read_lock(uint8_t *value);
	
	bool write_fuse(uint8_t fuse, uint8_t value);
	
	bool write_lock(uint8_t value);
	
	bool chip_erase();
	
	bool write_page(uint16_t page, uint8_t const *data, size_t size);
	
	bool load_address(uint16_t word_address);
	
	bool read_page(uint16_t page, uint8_t *data);
	
	// This only confirms transport status. The caller must wait and verify the target.
	bool program_page(uint16_t page, uint8_t const *data, size_t size);
};

class AvrispFlash {
private:
	AvrIsp *programmer_;
	Target const &target_;
public:
	AvrispFlash(AvrIsp *programmer, Target const &target) : programmer_(programmer), target_(target) {}
	bool read_page(uint16_t page, std::vector<uint8_t> *data)
	{
		data->resize(target_.definition().flash_page_bytes);
		return programmer_->read_page(page, data->data());
	}
	bool write_page(uint16_t page, uint8_t const *data, size_t size) { return programmer_->write_page(page, data, size); }
	bool chip_erase() { return programmer_->chip_erase(); }
};


#endif // AVRISP_H
