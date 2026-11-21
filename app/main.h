#ifndef MAIN_H
#define MAIN_H

#include <cstdint>
#include <cstring>
#include <string_view>
#include <vector>
#include <cstdint>
#include "../common/commands.h"
#include <string>

class Target;
class Connection;

constexpr int BITBANG_MODE_TIMEOUT_MS = 10;
constexpr int PIN_IO_TIMEOUT_MS = 10;
constexpr int MODE_ENTER_TIMEOUT_MS = 100;
constexpr int COMMAND_TIMEOUT_MS = 1000;
constexpr int PAGE_WRITE_TIMEOUT_MS = 2000;
constexpr int ISP_READY_TIMEOUT_MS = 100;

struct Operation {
	Command command;
	uint16_t data = 0;
	uint8_t value = 0;
	uint32_t updi_lock_value = 0;
	std::string path;
	std::vector<uint8_t> bytes;
	std::vector<uint8_t> page_mask;
	bool chip_erase = false;
	Target const *target = nullptr;
};


static inline bool operator == (std::vector<char> const &a, std::string_view b)
{
	if (a.size() != b.size()) return false;
	return memcmp(a.data(), b.data(), a.size()) == 0;
}

void msleep(int ms);

void write_command(Connection *conn, uint8_t command);
void write_encoded_byte(Connection *conn, uint8_t value, uint8_t command0, uint8_t command1);

#endif // MAIN_H
