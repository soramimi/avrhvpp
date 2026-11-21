#ifndef AVRUPDI_H
#define AVRUPDI_H

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

class Connection;
struct TargetDefinition;
struct UpdiFuse;

class AvrUpdi {
private:
	using Deadline = std::chrono::steady_clock::time_point;

	Connection *conn_;
	TargetDefinition const &target_;
	bool bridge_attempted_ = false;
	bool bridge_entered_ = false;
	bool wire_ok_ = true;
	bool reset_needed_ = false;
	bool programming_ = false;
	bool identity_verified_ = false;
	bool locked_ = false;

	bool write(uint8_t const *data, int size);
	bool send(uint8_t const *data, int size);
	bool receive(uint8_t *data, int size, Deadline deadline);
	bool drain(Deadline deadline);
	bool exchange(uint8_t const *request, int size, uint8_t *reply, int reply_size, Deadline deadline);
	bool initialize_link();
	bool read_sib(uint8_t *data);

	int load(uint32_t addr, Deadline deadline);
	bool store(uint32_t addr, uint8_t const *data, int size);
	bool store_byte(uint32_t addr, uint8_t value);
	int read_cs(uint8_t addr, Deadline deadline);
	bool write_cs(uint8_t addr, uint8_t value);
	bool send_key(char const (&key)[9], uint8_t status_mask);
	bool reset_target();
	bool wait_system_status(uint8_t mask, uint8_t value, Deadline deadline);
	bool enter_programming_mode();
	bool verify_identity();
	bool memory_access() const;
	bool wait_nvm();
	bool issue_nvm_command(uint8_t command);
	bool program_fuse(uint16_t address, uint8_t value);
	bool program_eeprom_cells(uint16_t address, uint8_t const *data, uint8_t count);
	bool read_flash_block(uint32_t address, uint8_t *data, int size);
	bool verify_erased();
	bool page_address(uint16_t page, uint32_t *address) const;

public:
	AvrUpdi(Connection *conn, TargetDefinition const &target);
	bool begin();
	bool finish();
	bool read_signature(uint8_t *signature);
	bool read_lock(bool *locked);
	bool write_lock(uint32_t value);
	bool read_fuse(UpdiFuse const &fuse, uint8_t *value);
	bool write_fuse(UpdiFuse const &fuse, uint8_t value);
	bool read_page(uint16_t page, std::vector<uint8_t> *data);
	bool write_page(uint16_t page, uint8_t const *data, size_t size);
	bool chip_erase();
	bool erase_locked();
	bool debug(std::string *text);
};

#endif // AVRUPDI_H
