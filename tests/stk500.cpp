// Constructed STK500v1 fixtures from PLAN.md (AVR061 and avrdude v8.1 stk500.c).
// Run the unmodified firmware parser, USB buffering and SPI transfer code.
#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <vector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#define main firmware_main
#include "../firmware-atmega32u2/main.cpp"
#undef main
#pragma GCC diagnostic pop

uint8_t DDRB, DDRC, DDRD, PORTB, PORTC, PORTD, PINC;
uint8_t SPCR, SPSR, CLKPR, MCUCR, TCCR0B, TIMSK0;
FakeSpiRegister SPDR;
uint8_t fake_pinb() { return PORTB | 8; }
uint8_t fake_pind() { return 0x1e; }
void waitloop(unsigned int) {}
void msleep(unsigned int) {}

struct TargetFixture {
	uint8_t code;
	std::array<uint8_t, 3> signature;
	uint16_t page_bytes, pages, eeprom_bytes;
	uint8_t fuse_bytes, extended_mask, lock_mask;
	bool poll;
	uint8_t pagel = 0xd7, bs2 = 0xc2;
};
static TargetFixture const targets[] = {
	{0x70, {{0x1e, 0x93, 0x07}}, 64, 128, 512, 2, 0, 0x3f, false},
	{0x59, {{0x1e, 0x92, 0x05}}, 64, 64, 256, 3, 1, 3, true},
	{0x73, {{0x1e, 0x93, 0x0a}}, 64, 128, 512, 3, 7, 0x3f, true},
	{0x86, {{0x1e, 0x94, 0x06}}, 128, 128, 512, 3, 7, 0x3f, true},
	{0x86, {{0x1e, 0x95, 0x0f}}, 128, 256, 1024, 3, 7, 0x3f, true},
	{0x23, {{0x1e, 0x91, 0x0a}}, 32, 64, 128, 3, 1, 3, true, 0xd4, 0xd6},
	{0x14, {{0x1e, 0x90, 0x07}}, 32, 32, 64, 2, 0, 3, true, 0, 0},
};
static TargetFixture const *target;
static std::array<uint8_t, 32768> memory;
static std::array<uint8_t, 256> page_buffer;
static std::array<uint8_t, 3> fuses;
static uint8_t lock_bits;
static std::array<uint8_t, 4> instruction;
static unsigned int spi_index, low_word, commits, enables, erases;
static uint8_t spi_reply;
static uint64_t elapsed_us, busy_until;
static std::vector<std::array<uint8_t, 4>> spi_log;
static std::deque<uint8_t> input;
static std::vector<uint8_t> output;
static uint8_t usb_chunk;

void fake_delay_us(double us) { elapsed_us += (uint64_t)us; }
void usb_init() {}
uint8_t is_usb_configured() { return 1; }
uint8_t usb_read_available_()
{
	// Fail a fixture underrun instead of spinning forever in firmware getch().
	assert(!input.empty() || data_rx_buffer_n > 0);
	return (uint8_t)std::min(input.size(), size_t(255));
}
uint8_t usb_read_byte_()
{
	assert(!input.empty());
	uint8_t value = input.front(); input.pop_front(); return value;
}
uint8_t usb_data_rx(uint8_t *data, uint8_t length)
{
	length = (uint8_t)std::min({size_t(length), size_t(usb_chunk), input.size()});
	for (uint8_t i = 0; i < length; ++i) data[i] = usb_read_byte_();
	return length;
}
void usb_data_tx(uint8_t const *data, uint8_t length)
{
	assert(!(PORTC & 4)); // ISP must never enable +12V.
	output.insert(output.end(), data, data + length);
}

static uint8_t transfer_instruction()
{
	spi_log.push_back(instruction);
	uint8_t command = instruction[0], second = instruction[1], value = instruction[3];
	unsigned int word = ((unsigned int)second << 8) | instruction[2];
	unsigned int size = target->page_bytes * target->pages;
	if (command == 0xf0) {
		assert(target->poll);
		return elapsed_us < busy_until ? 1 : 0;
	}
	assert(elapsed_us >= busy_until);
	if (command == 0xac && second == 0x53) { ++enables; return 0; }
	if (command == 0x30) return target->signature.at(instruction[2]);
	if (command == 0x50 && second == 0) return fuses[0];
	if (command == 0x58) return second == 8 ? fuses[1] : lock_bits;
	if (command == 0x50 && second == 8) { assert(target->extended_mask); return fuses[2]; }
	if (command == 0x20 || command == 0x28) {
		unsigned int address = word * 2 + (command == 0x28);
		assert(address < size); return memory[address];
	}
	if (command == 0x40 || command == 0x48) {
		assert(word * 2 < size);
		unsigned int offset = (word % (target->page_bytes / 2)) * 2;
		if (command == 0x40) { low_word = word; page_buffer[offset] = value; }
		else { assert(low_word == word); page_buffer[offset + 1] = value; }
		return 0;
	}
	if (command == 0x4c) {
		assert(word % (target->page_bytes / 2) == 0 && word * 2 + target->page_bytes <= size);
		for (unsigned int i = 0; i < target->page_bytes; ++i) memory[word * 2 + i] &= page_buffer[i];
		page_buffer.fill(0xff); ++commits;
		busy_until = elapsed_us + (target->poll && target->signature[1] != 0x95 ? 2600 : 4500);
		return 0;
	}
	assert(command == 0xac);
	if (second == 0x80) {
		memory.fill(0xff); lock_bits = 0xff; ++erases;
		busy_until = elapsed_us + (target->poll && target->signature[1] != 0x95 ? 10500 : 9000);
	} else if (second == 0xe0) {
		lock_bits &= value;
		busy_until = elapsed_us + 4500;
	} else {
		assert(second == 0xa0 || second == 0xa8 || second == 0xa4);
		unsigned int fuse = second == 0xa0 ? 0 : second == 0xa8 ? 1 : 2;
		assert(fuse != 2 || target->extended_mask);
		bool tiny13 = target->signature[1] == 0x90;
		unsigned int spien_fuse = tiny13 ? 0 : 1;
		uint8_t spien_mask = tiny13 ? 0x80 : 0x20;
		if (fuse == spien_fuse) value = (value & ~spien_mask) | (fuses[fuse] & spien_mask);
		uint8_t mask = fuse == 2 ? target->extended_mask : fuse == 1 && tiny13 ? 0x1f : 0xff;
		value |= (uint8_t)~mask;
		fuses[fuse] = value;
		busy_until = elapsed_us + 4500;
	}
	return 0;
}

void FakeSpiRegister::operator=(uint8_t value)
{
	assert(SPCR == 0x53 && (DDRB & 1) && !(PORTB & 1));
	instruction[spi_index++] = value;
	spi_reply = spi_index == 3 && instruction[0] == 0xac && instruction[1] == 0x53 ? 0x53 : 0;
	if (spi_index == 4) { spi_reply = transfer_instruction(); spi_index = 0; }
	SPSR |= 0x80;
}
FakeSpiRegister::operator uint8_t() const { return spi_reply; }

static void reset(TargetFixture const &fixture, uint8_t chunk)
{
	clear_buffers(); input.clear(); output.clear();
	target = &fixture; usb_chunk = chunk;
	memory.fill(0xff); page_buffer.fill(0xff); fuses = {{0x62, 0xd9, 0xfd}}; lock_bits = 0xff;
	if (target->signature[1] == 0x90) fuses = {{0x6a, 0xff, 0xff}};
	elapsed_us = busy_until = 0; spi_index = commits = enables = erases = 0; spi_log.clear();
	pmode = false; here = 0; param = {}; error = 0;
	reset_mode(Programmer::ISP);
}

static void expect(std::vector<uint8_t> const &request, std::vector<uint8_t> const &reply)
{
	assert(input.empty() && data_rx_buffer_n == 0 && data_tx_buffer_n == 0);
	output.clear(); input.insert(input.end(), request.begin(), request.end());
	unsigned int calls = 0;
	while (!input.empty() || data_rx_buffer_n) {
		assert(++calls <= request.size());
		isp_poll();
	}
	usb_poll_tx();
	assert(output == reply && spi_index == 0);
}

static void configure()
{
	unsigned int size = target->page_bytes * target->pages;
	expect({0x41, 0x81, 0x20}, {0x14, 1, 0x10});
	expect({0x41, 0x82, 0x20}, {0x14, 18, 0x10});
	expect({0x42, target->code, 0, 0, 1, 1, 1, 1, target->fuse_bytes,
		0xff, 0xff, 0xff, 0xff, (uint8_t)(target->page_bytes >> 8), (uint8_t)target->page_bytes,
		(uint8_t)(target->eeprom_bytes >> 8), (uint8_t)target->eeprom_bytes,
		0, 0, (uint8_t)(size >> 8), (uint8_t)size, 0x20}, {0x14, 0x10});
	expect({0x45, 5, 4, target->pagel, target->bs2, 1, 0x20}, {0x14, 0x10});
	assert(param.devicecode == target->code && param.pagesize == target->page_bytes
		&& param.flashsize == size && param.eepromsize == target->eeprom_bytes && !rst_active_high);
	expect({0x50, 0x20}, {0x14, 0x10});
	assert(pmode && SPCR == 0x53);
}

static void universal(std::array<uint8_t, 4> const &command, uint8_t value)
{
	expect({0x56, command[0], command[1], command[2], command[3], 0x20}, {0x14, value, 0x10});
}

static void wait_ready()
{
	if (!target->poll) { fake_delay_us(20000); return; }
	while (elapsed_us < busy_until) {
		universal({{0xf0, 0, 0, 0}}, 1);
		fake_delay_us(1000);
	}
	universal({{0xf0, 0, 0, 0}}, 0);
}

static void load_address(uint16_t address)
{
	expect({0x55, (uint8_t)address, (uint8_t)(address >> 8), 0x20}, {0x14, 0x10});
	assert(here == address);
}

static void page_transfer(uint16_t start, uint16_t length)
{
	std::vector<uint8_t> data(length);
	uint8_t const values[] = {0, 0x20, 0xff, 0x14, 0x10, 0x50, 0x51, 0xac};
	for (size_t i = 0; i < data.size(); ++i) data[i] = values[i % sizeof(values)];
	load_address(start);
	std::vector<uint8_t> request = {0x64, (uint8_t)(length >> 8), (uint8_t)length, 'F'};
	request.insert(request.end(), data.begin(), data.end()); request.push_back(0x20);
	unsigned int before = commits;
	expect(request, {0x14, 0x10});
	assert(commits - before == length / target->page_bytes && here == start + length / 2);
	assert(elapsed_us >= busy_until); // The firmware's 10 ms commit delay must remain.
	load_address(start);
	std::vector<uint8_t> reply = {0x14}; reply.insert(reply.end(), data.begin(), data.end()); reply.push_back(0x10);
	before = spi_log.size();
	expect({0x74, (uint8_t)(length >> 8), (uint8_t)length, 'F', 0x20}, reply);
	assert(here == start + length / 2 && spi_log.size() - before == length);
	for (unsigned int i = 0; i < length; ++i) {
		auto const &instruction = spi_log[before + i];
		unsigned int word = start + i / 2;
		assert(instruction == (std::array<uint8_t, 4>{{(uint8_t)(i % 2 ? 0x28 : 0x20), (uint8_t)(word >> 8), (uint8_t)word, 0}}));
	}
}

int main()
{
	for (auto const &fixture : targets) {
		for (uint8_t chunk : {1, 7, 16}) {
			reset(fixture, chunk);
			expect({0x30, 0x20}, {0x14, 0x10});
			expect({0x31, 0x20}, {0x14, 'A', 'V', 'R', ' ', 'I', 'S', 'P', 0x10});
			expect({0x41, 0x80, 0x20}, {0x14, 2, 0x10});
			expect({0x41, 0x93, 0x20}, {0x14, 'S', 0x10});
			configure();
			assert(enables == 1);
			expect({0x75, 0x20}, {0x14, fixture.signature[0], fixture.signature[1], fixture.signature[2], 0x10});
			for (uint8_t i = 0; i < 3; ++i) universal({{0x30, 0, i, 0}}, fixture.signature[i]);
			universal({{0x50, 0, 0, 0}}, fuses[0]);
			universal({{0x58, 8, 0, 0}}, fuses[1]);
			if (fixture.extended_mask) universal({{0x50, 8, 0, 0}}, 0xfd);
			universal({{0x58, 0, 0, 0}}, 0xff);
			page_transfer(0, fixture.page_bytes);
			page_transfer(256 - fixture.page_bytes / 2, fixture.page_bytes * 2);
			page_transfer((fixture.pages - 1) * (fixture.page_bytes / 2), fixture.page_bytes);
			universal({{0xac, 0xa0, 0, 0x63}}, 0); wait_ready();
			universal({{0x50, 0, 0, 0}}, 0x63);
			uint8_t high = fixture.signature[1] == 0x90 ? 0xe9 : 0xc9;
			universal({{0xac, 0xa8, 0, high}}, 0); wait_ready();
			universal({{0x58, 8, 0, 0}}, high);
			if (fixture.extended_mask) {
				uint8_t value = (uint8_t)~fixture.extended_mask;
				universal({{0xac, 0xa4, 0, value}}, 0); wait_ready();
				universal({{0x50, 8, 0, 0}}, value);
			}
			universal({{0xac, 0xe0, 0, 0xfc}}, 0); wait_ready();
			universal({{0x58, 0, 0, 0}}, 0xfc);
			auto saved = fuses;
			universal({{0xac, 0x80, 0, 0}}, 0); wait_ready();
			configure(); // avrdude re-initializes after erase; ENTER remains idempotent.
			assert(erases == 1 && enables == 1 && fuses == saved && lock_bits == 0xff);
			assert(std::all_of(memory.begin(), memory.end(), [](uint8_t value) { return value == 0xff; }));
			expect({0x51, 0x20}, {0x14, 0x10});
			assert(!pmode && SPCR == 0 && (PORTB & 1) && !(DDRB & 6));
			// Preserve existing framing errors without adding new firmware validation.
			auto count = spi_log.size();
			expect({0x75, 0x21}, {0x15});
			expect({0x52, 0x20}, {0x12});
			expect({0x20}, {0x15});
			assert(spi_log.size() == count);
			expect({0x30, 0x20}, {0x14, 0x10});
			// GET_SYNC must still select STK500 from either proprietary mode.
			for (auto const &mode : {std::vector<uint8_t>{'.', 'A', 'V', 'R', 'H', 'V', 'P', 'P', '.'},
				std::vector<uint8_t>{'.', 'B', 'I', 'T', 'B', 'A', 'N', 'G', '.'}}) {
				std::vector<uint8_t> reply(mode.begin() + 1, mode.end() - 1); reply.push_back('\r'); reply.push_back('\n');
				expect(mode, reply);
				assert(programmer_mode_ != Programmer::ISP);
				expect({0x30, 0x20}, {0x14, 0x10});
				assert(programmer_mode_ == Programmer::ISP && !(PORTC & 4));
			}
		}
	}
	puts("STK500v1 firmware compatibility fixtures passed without hardware.");
}
