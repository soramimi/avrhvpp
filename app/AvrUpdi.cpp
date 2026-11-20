#include "AvrUpdi.h"
#include <cstdio>
#include <chrono>
#include "Connection.h"
#include "../common/commands.h"
#include "main.h"

void AvrUpdi::PrintByte(uint8_t c)
{
	if (offset_ & 0xf) {
		putchar(' ');
	} else {
		if (offset_ != 0) putchar('\n');
		printf("%04x: ", (unsigned int)offset_);
	}
	offset_++;
	printf("%02x", (unsigned int)c);
}

int AvrUpdi::receive(void *ptr, int len, int timeout_ms)
{
	auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
	int offset = 0;
	while (len > 0) {
		auto remaining = std::chrono::ceil<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count();
		if (remaining <= 0) break;
		int n = conn_->read((uint8_t *)ptr + offset, len, (int)remaining);
		if (n > 0) {
			offset += n;
			len -= n;
		}
		if (n <= 0) break;
	}
	return offset;
}

void AvrUpdi::receive_all(int timeout_ms)
{
	while (1) {
		uint8_t tmp[256];
		int n = conn_->read(tmp, sizeof(tmp), timeout_ms);
		if (n <= 0) break;
		for (int i = 0; i < n; i++) {
			PrintByte(tmp[i]);
		}
	}
}

void AvrUpdi::send_break()
{
	write_command(conn_, CMD_UPDI_BREAK);
	receive_all(100);
}

void AvrUpdi::send_byte(uint8_t c)
{
	write_command(conn_, CMD_LOAD_7BITS | (c & 0x7f));
	write_command(conn_, (c & 0x80) ? CMD_UPDI_SEND1 : CMD_UPDI_SEND0);
}

void AvrUpdi::send_bytes(void *p, int n)
{
	uint8_t *ptr = (uint8_t *)p;
	for (int i = 0; i < n; i++) {
		send_byte(ptr[i]);
	}
}

int AvrUpdi::default_timeout() const
{
	return 10;
}

int AvrUpdi::updi_LDS(uint16_t addr)
{
	uint8_t tx[] = {0x55, 0x04, (uint8_t)addr, (uint8_t)(addr >> 8)}; // SYNCH, LDS with a 16-bit direct address (low byte first).
	uint8_t rx[5];
	send_bytes(tx, 4);
	int n = receive(rx, sizeof(rx), default_timeout());
	if (n == 5 && memcmp(tx, rx, 4) == 0) { // Check that the first 4 bytes match the transmitted data.
		return rx[4];
	}
	return -1;
}

int AvrUpdi::updi_LDCS(uint8_t addr, int timeout_ms)
{
	if (addr > 0x0f) return -1;
	uint8_t tx[] = {0x55, (uint8_t)(0x80 | addr)}; // SYNCH, LDCS with a 4-bit register address.
	uint8_t rx[3];
	send_bytes(tx, sizeof(tx));
	int n = receive(rx, sizeof(rx), timeout_ms);
	if (n == 3 && memcmp(tx, rx, sizeof(tx)) == 0) {
		return rx[sizeof(tx)];
	}
	return -1;
}

bool AvrUpdi::updi_STCS(uint8_t addr, uint8_t value)
{
	if (addr > 0x0f) return false;
	uint8_t tx[] = {0x55, (uint8_t)(0xc0 | addr), value};
	uint8_t rx[3];
	send_bytes(tx, sizeof(tx));
	// STCS has no target response. Check the local echo of the three sent bytes.
	int n = receive(rx, sizeof(rx), default_timeout());
	return n == 3 && memcmp(tx, rx, sizeof(tx)) == 0;
}

int AvrUpdi::read_control_status(uint8_t addr)
{
	int c = updi_LDCS(addr, default_timeout());
	if (c < 0) {
		fprintf(stderr, "failed to read UPDI register 0x%02x\n", (unsigned int)addr);
	}
	return c;
}

bool AvrUpdi::write_control_status(uint8_t addr, uint8_t value)
{
	if (!updi_STCS(addr, value)) {
		fprintf(stderr, "failed to verify STCS echo for UPDI register 0x%02x\n", (unsigned int)addr);
		return false;
	}
	return true;
}

bool AvrUpdi::configure_and_check_updi_link()
{
	if (!write_control_status(0x03, 0x08)) return false; // CTRLB.CCDETDIS
	if (!write_control_status(0x02, 0x80)) return false; // CTRLA.IBDLY
	if (read_control_status(0x00) < 0) return false; // STATUSA, read the UPDI revision field (UPDIREV).
	return read_control_status(0x0b) >= 0; // ASI_SYS_STATUS, read the current system state.
}

bool AvrUpdi::wait_for_reset(int timeout_ms)
{
	constexpr uint8_t ASI_SYS_STATUS = 0x0b;
	constexpr uint8_t SYSRST = 0x20;
	for (int i = 0; i < timeout_ms; i++) {
		int status = updi_LDCS(ASI_SYS_STATUS, 1);
		if (status < 0) continue;
		if (!(status & SYSRST)) return true;
	}
	return false;
}

void AvrUpdi::setup(Connection *conn)
{
	conn_ = conn;
}

bool AvrUpdi::cmd_debug()
{
	// Check that the UPDI link is functional by sending a break and configuring the UPDI interface. Retry up to 10 times if the link is not functional.
	
	{
		int i = 0;
		while (1) {
			send_break();
			receive_all(default_timeout());
			if (configure_and_check_updi_link()) break;
			if (i < 10) {
				i++;
			} else {
				fprintf(stderr, "UPDI link is not functional; check the target device and connection\n");
			}
		}
	}
	
	// for AVR64DD28
	
	// Request the 32-byte System Information Block (SIB) to verify that the UPDI link is functional.
	{
		send_byte(0x55); // SYNCH frame before each UPDI instruction.
		send_byte(0xe6); // KEY with SIB=1 and SIZE_C=2, request the 32-byte System Information Block.
		receive_all(default_timeout());
	}
	
	if (!configure_and_check_updi_link()) return false;
	
	// Request the 64-bit NVMPROG key to enable flash programming on devices that require it.
	
	{
		
		if (!write_control_status(0x08, 0x59)) { // ASI_RESET_REQ, assert system reset.
			write_control_status(0x08, 0x00); // The reset may have been asserted despite the missing echo.
			return false;
		}
		
		send_byte(0x55); // SYNCH frame before each UPDI instruction.
		send_byte(0xe0); // KEY with SIB=0 and SIZE_C=0, receive the 64-bit NVMPROG key.
		
		// Send the ASCII key "NVMProg " least-significant byte first.
		static constexpr std::string_view key = "NVMProg ";
		for (int i = key.size(); i > 0; i--) {
			send_byte(key[i - 1]);
		}
		receive_all(default_timeout());
		
		constexpr uint8_t NVMPROG = 1 << 4;
		int key_status = read_control_status(0x07); // ASI_KEY_STATUS
		if (key_status < 0 || !(key_status & NVMPROG)) {
			if (key_status >= 0) fprintf(stderr, "UPDI NVMPROG key was not accepted\n");
			write_control_status(0x08, 0x00); // Release the reset held during the key transfer.
			return false;
		}
	}
	
	// Request a system reset to ensure that the device is in a known state.
	
	{
		bool reset_requested = write_control_status(0x08, 0x59); // ASI_RESET_REQ, assert system reset.
		// Release reset even if the echo of the reset assertion was lost.
		bool reset_released = write_control_status(0x08, 0x00);
		if (!reset_requested || !reset_released) return false;
		int timeout_ms = 100;
		if (!wait_for_reset(timeout_ms)) {
			fprintf(stderr, "UPDI reset did not complete within %d ms\n", timeout_ms);
			return false;
		}
	}
	
	// Read the 24-bit device signature from the signature row.
	
	uint32_t sig = 0;
	{
		uint8_t tmp[3];
		uint16_t addr = 0x1100; // SIGROW.DEVICEID0
		int i = 0;
		for (i = 0; i < 3; i++) {
			int c = updi_LDS(addr++); // SIGROW.DEVICEID0, SIGROW.DEVICEID1, SIGROW.DEVICEID2
			if (c < 0) break;
			tmp[i] = c;
		}
		if (i == 3) {
			sig = (tmp[0] << 16) | (tmp[1] << 8) | tmp[2];
		}
	}
	
	putchar('\n');
	if ((sig >> 16) == 0x1e) { // AVR signature prefix
		printf("signature: 0x%06x\n", sig);
	} else {
		fprintf(stderr, "failed to read signature bytes\n");
		return false;
	}
	
	return true;
}
