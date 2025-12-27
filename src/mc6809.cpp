/**
* Copyright (c) 2025 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sys/stat.h>

#include <cctype>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>
//#include <cstdint>

#include "pico/aon_timer.h"
#include "pico/assert.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/vreg.h"
#include "bsp/board_api.h"

#include "tusb.h"
#include "usb.h"
#include "mc6809.h"
#include "processor.h"
#include "srec.h"
#include "uart.h"
#include "build_time.h"

// PIO program headers will be generated from the .pio file
#include "mc6809.pio.h"

// I'm really pushing my luck with this.
#define SYS_CLK_DEFAULT 360000000

// Default pico uart for command-line interactive access
#define CONSOLE uart0
#define CONSOLE_BAUDRATE 115200

volatile bool verbose = false;

#define BUS_CLOCK_PIO pio0

static pio_dma pio_clock = {
	BUS_CLOCK_PIO,
	0,
	-1,
	-1,
	IO_IRQ_BANK0
};

#define BUS_READ_PIO pio0

static pio_dma piodma_read = {
	BUS_READ_PIO,
	0,
	0,
	1,
	DMA_IRQ_0
};

#define BUS_WRITE_PIO pio1

static pio_dma piodma_write = {
	BUS_WRITE_PIO,
	0,
	4,
	5,
	DMA_IRQ_1
};

// 64-byte memory-mapped registers
// There are 2 blocks, one for the Read DMA and the other for the Write DMA.
// This allows for software enforcement of read-only areas and for device
// registers to share status and control at the same address.
// The alignment is there to allow the base address to have the lower 6 bits
// all zeroes, so that the rest can be used as DMA prefixes without faffing
// around with pointer arithmetic.

registers &reg = registers::getInstance();

void
registers::copy_out(uint8_t *dst, uint16_t addr, uint16_t len)
{
	for (uint16_t i = 0; i < len; i++)
		dst[i] = read_registers[addr + i];
}


void
registers::copy_in(uint16_t addr, const uint8_t *src, uint16_t len)
{
	for (uint16_t i = 0u; i < len; i++)
		read_registers[addr + i] = src[i];
}

// Keep track of which task was interrupted
#define MAX_INT_NEST_DEPTH 16u
static volatile uint8_t task_stack[MAX_INT_NEST_DEPTH];

static void
dump_registers()
{
	static uint8_t printables[16];

	printf("Write block:\n");
	for (uint16_t i = 0u; i < 4u; i++) {
		registers::copy_out(printables, i*16u, 16u);
		printf("%04X:", REGISTER_BASE + i*16u);
		for (uint16_t j = 0u; j < 16u; j++) {
			auto ch = static_cast<unsigned char>(printables[j]) & 0x7F;
			printf(" %02X", printables[j]);
			if (j == 7u)
				printf(" ");
			printables[j] = std::isprint(ch) ? static_cast<char>(ch) : '.';
		}
		printf(" |%.16s|\n", reinterpret_cast<char *>(printables));
	}
	printf("Read block:\n");
	for (uint16_t i = 0u; i < 4u; i++) {
		registers::copy_out(printables, i*16u, 16u);
		printf("%04X:", static_cast<uint16_t>(REGISTER_BASE + i*16u));
		for (uint16_t j = 0u; j < 16u; j++) {
			auto ch = static_cast<unsigned char>(printables[j]) & 0x7F;
			printf(" %02X", printables[j]);
			if (j == 7u)
				printf(" ");
			printables[j] = std::isprint(ch) ? static_cast<char>(ch) : '.';
		}
		printf(" |%.16s|\n", reinterpret_cast<char *>(printables));
	}
	printf("--\n");
}

//	BS_RUNNING,    BS_IRQ,         BS_SYNC,   BS_HALT,   BS_RUNNING_RESET ....
static const run_state run_state_table[RUN_STATE_COUNT][BUS_STATE_COUNT] = {
	{RS_STARTED, RS_INTERRUPTED, RS_SYNCED, RS_HALTED, RS_STOPPED, RS_STOPPED, RS_STOPPED, RS_STOPPED}, // RS_STARTED
	{RS_STARTED, RS_INTERRUPTED, RS_SYNCED, RS_HALTED, RS_STOPPED, RS_STOPPED, RS_STOPPED, RS_STOPPED}, // RS_INTERRUPTED
	{RS_STARTED, RS_INTERRUPTED, RS_SYNCED, RS_HALTED, RS_STOPPED, RS_STOPPED, RS_STOPPED, RS_STOPPED}, // RS_HALTED
	{RS_STARTED, RS_INTERRUPTED, RS_SYNCED, RS_HALTED, RS_STOPPED, RS_STOPPED, RS_STOPPED, RS_STOPPED}, // RS_SYNCED
	{RS_STARTED, RS_INTERRUPTED, RS_SYNCED, RS_HALTED, RS_STOPPED, RS_STOPPED, RS_STOPPED, RS_STOPPED}, // RS_STOPPED
};

static volatile bool q_falling_edge = false;

void __isr
__time_critical_func(gpio_clock_eq_irq_handler())
{
	gpio_put(GPIO_TRACE_C, true); // OINQUE DEBUG to time this function on the scope
	if (gpio_get_irq_event_mask(GPIO_Q) & GPIO_IRQ_EDGE_RISE) {
		gpio_acknowledge_irq(GPIO_Q, GPIO_IRQ_EDGE_RISE);
		uint32_t bus_pins = gpio_get_all();
		ba_bs_u new_bus_state{};
		new_bus_state.byte = ((bus_pins >> GPIO_BS) & 0b00000011u);
		new_bus_state.bit.RESET = RESET_IS_ASSERTED;
		MC6809.unpack_busy_lic_avma(); // Previous cycle's advance notice
#ifdef DEBUG
		MC6809.count_lic();
#endif
		if (MC6809.bus_state.state != new_bus_state.state) {
			MC6809.old_bus_state.state = MC6809.bus_state.state;
			MC6809.bus_state.state = new_bus_state.state;
			MC6809.run_state = run_state_table[MC6809.run_state][MC6809.bus_state.state];
			if (new_bus_state.state == BS_IRQ) {
				auto eirq = static_cast<interrupt>((bus_pins >> (GPIO_A_BASE + 1)) & 0b111u);
				if (eirq < INTERRUPT_RESET && MC6809.task_stack_ptr < MAX_INT_NEST_DEPTH)
					task_stack[MC6809.task_stack_ptr++] = MC6809.task;
			}
		}
//	} else if (gpio_get_irq_event_mask(GPIO_Q) & GPIO_IRQ_EDGE_FALL) {
	} else {
		q_falling_edge = true;
		gpio_acknowledge_irq(GPIO_Q, GPIO_IRQ_EDGE_FALL);
		// These 3 signals are all predictive; they indicate the status of the _next_ memory access
		MC6809.set_busy_lic_avma(static_cast<uint8_t>((gpio_get_all() >> GPIO_BUSY) & 0b00000111));
	}
	gpio_put(GPIO_TRACE_C, false); // OINQUE DEBUG to time this function on the scope
}

// ISR for the R/!W bus read loop and DMA chain
// The PIO mediates the signalling
// There are 2 DMA transfers:
//   1) accept reg + A5-0
//   2) return the byte at that address
//
// This reacts to a read after the fact. Consider breaking
// the DMA chain and deciding what to return on the fly.

static volatile bool read_irq_received = false;
static volatile uint8_t read_location;

void __isr
__time_critical_func(dma_bus_read_irq_handler())
{
	gpio_put(GPIO_TRACE_D, true); // OINQUE DEBUG to time this function on the scope
	// if (dma_channel_get_irq0_status(piodma_read.data_channel)) {
	dma_channel_acknowledge_irq0(piodma_read.data_channel);
	if (!MC6809.bus_state.bit.RESET && MC6809.get_vma()) {
		const uintptr_t read_addr = dma_channel_hw_addr(piodma_read.data_channel)->read_addr;
		read_location = read_addr & 0x0000003Fu;
		read_irq_received = true;
#ifdef DEBUG_NOT_NOW
		if (trace_pos < trace_size - 1) {
			trace[trace_pos][0] = read_location;
			trace[trace_pos][1] = *reinterpret_cast<volatile std::uint8_t *>(read_addr);
			trace[trace_pos][2] = TRACE_READ | (MC6809.get_busy_lic_avma() << 8) | (MC6809.bus_state.byte << 4) | MC6809.run_state;
			trace[trace_pos][3] = 0u;
			trace_pos++;
		}
#endif
	}
	//}
	gpio_put(GPIO_TRACE_D, false); // OINQUE DEBUG to time this function on the scope
}

// ISR for the R/!W bus write loop and DMA chain
// The PIO mediates the signalling
// There are 2 DMA transfers:
//   1) accept write_registers + A5-0
//   2) accept the byte to be written to that address

static volatile bool write_irq_received = false;
static volatile uint8_t write_location;

void __isr
__time_critical_func(dma_bus_write_irq_handler())
{
	gpio_put(GPIO_TRACE_E, true); // OINQUE DEBUG to time this function on the scope
	// if (dma_channel_get_irq1_status(piodma_write.data_channel)) {
	dma_channel_acknowledge_irq1(piodma_write.data_channel);
	if (!MC6809.bus_state.bit.RESET) {
		const uintptr_t written_addr = dma_channel_hw_addr(piodma_write.data_channel)->write_addr;
		write_location = written_addr & 0x0000003Fu;
		write_irq_received = true;
#ifdef DEBUG_NOT_NOW
		if (trace_pos < trace_size - 1) {
			trace[trace_pos][0] = write_location;
			trace[trace_pos][1] = *reinterpret_cast<volatile std::uint8_t *>(written_addr);
			trace[trace_pos][2] = TRACE_WRITE | (MC6809.busy_lic_avma.byte << 8) | (MC6809.bus_state.byte << 4) | MC6809.run_state;
			trace[trace_pos][3] = 0u;
			trace_pos++;
		}
#endif
	}
	// }
	gpio_put(GPIO_TRACE_E, false); // OINQUE DEBUG to time this function on the scope
}

#define SNIPPET_NAMES_INC 1
#include "api.inc"
#undef  SNIPPET_NAMES_INC

enum snippet {
	foreach_snippet(enum_list)
};

static const char *snippet_string[] = {
	foreach_snippet(enum_list_strings)
};

static uint8_t snippet_code[SNIPPET_LEN][16u] = {
#define SNIPPET_INITIALISERS_INC 1
#include "api.inc"
#undef  SNIPPET_INITIALISERS_INC
};

static void
snippet_copy(const snippet snippet_num)
{
	registers::copy_in(REGISTER_SNIPPET_OFFSET, snippet_code[snippet_num], 16u);
	reg[REGISTER_VECTOR_RESET_OFFSET] = REGISTER_SNIPPET;
}

enum copy_type {
	OUTWARDS,
	INWARDS
};

static void
copy(const copy_type direction, const uint16_t count, const uint16_t address, uint8_t *data)
{
	uint16_t start = address;
	const uint16_t end = address + count;
	uint16_t pos = 0u;
	const char *s = nullptr;

	if (direction == OUTWARDS) {
		snippet_copy(COPY_OUT);
		s = "Copying out %u bytes to %04X\n";
	} else {
		snippet_copy(COPY_IN);
		s = "Copying in %u bytes from %04X\n";
	}
	while (start != end) {
		const uint8_t len = MIN(16u, end - start);
		if (verbose)
			printf(s, len, start);
		reg[REGISTER_SNIPPET_OFFSET + 4u] = start;
		reg[REGISTER_SNIPPET_OFFSET + 7u] = len;
		if (direction == OUTWARDS) {
			registers::copy_in(REGISTER_BUFFER_OFFSET, data + pos, len);
			MC6809.start_with_timeout(4u);
		} else {
			MC6809.start_with_timeout(4u);
			registers::copy_out(data + pos, REGISTER_BUFFER_OFFSET, len);
		}
		start += len;
		pos += len;
	}
}

#if 1
#define CHUNK_NAMES_INC 1
#include "api.inc"
#undef  CHUNK_NAMES_INC

enum chunk {
	foreach_chunk(enum_list)
};

static const char *chunk_string[] = {
	foreach_chunk(enum_list_strings)
};

#define CHUNK_INITIALISERS_INC 1
#include "api.inc"
#undef  CHUNK_INITIALISERS_INC
#endif

#ifdef DEBUG
static volatile unsigned UART_CONTROL_COUNT = 0u, UART_TX_DATA_COUNT = 0u;
static volatile unsigned UART_STATUS_COUNT = 0u, UART_RX_DATA_COUNT = 0u;
#endif

static void
peripheral_clear(const bool sysvectors)
{
	registers::clear();
	for (uint16_t i = 0u; i < 16u; i++)
		reg[REGISTER_DEVICES_OFFSET + i] = static_cast<uint8_t>(0xFFu);
	if (sysvectors) {
		snippet_copy(SYNC);
		for (uint16_t i = 0u; i < 16u; i += 2)
			reg[REGISTER_VECTORS_OFFSET + i] = REGISTER_SNIPPET;
	}
#ifdef DEBUG
	UART_CONTROL_COUNT = 0u;
	UART_TX_DATA_COUNT = 0u;
	UART_STATUS_COUNT = 0u;
	UART_RX_DATA_COUNT = 0u;
#endif

}

void
sysreq_initialise()
{
}

void
sysreq_process()
{
	if (reg[SYSTEM_REQUEST_RESPONSE] == RESPONSE_BUSY) {
		switch (static_cast<uint8_t>(reg[SYSTEM_REQUEST])) {
		default:
			// TODO: MarkM - Bad Juju - investigate/report
			break;
		case REQUEST_TIME: {
			tm tm{};
			union {
				struct {
					uint8_t year;
					uint8_t month;
					uint8_t day;
					uint8_t hour;
					uint8_t minute;
					uint8_t second;
					uint8_t day_of_week;
				} time;
				uint8_t bytes[7];
			} guest_time{};
			if (aon_timer_get_time_calendar(&tm)) {
				guest_time.time.year = tm.tm_year;
				guest_time.time.month = tm.tm_mon + 1u;
				guest_time.time.day = tm.tm_mday;
				guest_time.time.hour = tm.tm_hour;
				guest_time.time.minute = tm.tm_min;
				guest_time.time.second = tm.tm_sec;
				guest_time.time.day_of_week = tm.tm_wday;
				registers::copy_in(REGISTER_BUFFER_OFFSET, guest_time.bytes, MIN(sizeof(guest_time), 16u));
				reg[SYSTEM_REQUEST_RESPONSE] = RESPONSE_DONE;
			} else
				reg[SYSTEM_REQUEST_RESPONSE] = RESPONSE_UNAVAILABLE;
			break;
		}
		case REQUEST_SETTIME: {
			tm tm{};
			union {
				struct {
					uint8_t year;
					uint8_t month;
					uint8_t day;
					uint8_t hour;
					uint8_t minute;
					uint8_t second;
					uint8_t day_of_week;
				} time;
				uint8_t bytes[7];
			} guest_time{};
			registers::copy_out(guest_time.bytes, REGISTER_BUFFER, MIN(sizeof(guest_time), 16u));
			tm.tm_year = guest_time.time.year;
			tm.tm_mon = guest_time.time.month;
			tm.tm_mday = guest_time.time.day;
			tm.tm_hour = guest_time.time.hour;
			tm.tm_min = guest_time.time.minute;
			tm.tm_sec = guest_time.time.second;
			tm.tm_wday = guest_time.time.day_of_week;
			if (aon_timer_set_time_calendar(&tm))
				reg[SYSTEM_REQUEST_RESPONSE] = RESPONSE_DONE;
			else
				reg[SYSTEM_REQUEST_RESPONSE] = RESPONSE_UNAVAILABLE;
			break;
		}
		case REQUEST_MOUNT: {
			reg[SYSTEM_REQUEST_RESPONSE] = RESPONSE_UNAVAILABLE;
			break;
		}
		case REQUEST_UNMOUNT: {
			reg[SYSTEM_REQUEST_RESPONSE] = RESPONSE_UNAVAILABLE;
			break;
		}
		}
	}
}

emulated_uart fast_serial;

static void
print_time()
{
	tm tm{};
	if (aon_timer_get_time_calendar(&tm)) // gives POSIX timespec
		printf("%04d-%02d-%02d %02d:%02d:%02d UTC\n",
		       tm.tm_year + 1900,
		       tm.tm_mon + 1,
		       tm.tm_mday,
		       tm.tm_hour,
		       tm.tm_min,
		       tm.tm_sec);
	else
		printf("Unable to get calendar time\n");
}

enum e_commands {
	VERBOSE, EXAMINE, MODIFY, STATUS, SNIPPET, VECTOR, FILL, TIME, GO, TASK, IRQ, TRACE, RUN, STOP, FREQ, RESET, USB, S19, NONE
};

struct s_commands {
	const char *str;
	enum e_commands com;
} commands[] = {
	{ "verbose", VERBOSE },
	{ "examine", EXAMINE },
	{ "modify", MODIFY },
	{ "status", STATUS },
	{ "snippet", SNIPPET },
	{ "vector", VECTOR },
	{ "fill", FILL },
	{ "time", TIME },
	{ "go", GO },
	{ "task", TASK },
	{ "irq", IRQ },
	{ "trace", TRACE },
	{ "run", RUN },
	{ "stop", STOP },
	{ "freq", FREQ },
	{ "reset", RESET },
	{ "usb", USB },
	{ "S19", NONE },
	{ nullptr, NONE }
};

#define BUFFER_SIZE 256u
void
process_command(char *buf)
{
	static uint8_t buffer[BUFFER_SIZE];
	uint16_t buflen = strlen(buf);
	char *tokenlist[64], *ptr = buf + buflen - 1, *token;

	while (isspace(*buf) && buflen > 0u) {
		buf++;
		buflen--;
	}
	if (buflen == 0u)
		return;
	while (isspace(*ptr) && buflen > 0u) {
		*ptr-- = '\0';
		buflen--;
	}
	ptr = buf;
	uint16_t tokencount = 0u;
	while ((token = strsep(&ptr, " \t\n\r")) != nullptr) {
		if (strlen(token) != 0u)
			tokenlist[tokencount++] = token;
	}
	e_commands command = NONE;
	if (tokenlist[0][0] == 'S')
		command = S19;
	else {
		for (int i = 0; ; i++) {
			if (commands[i].com == NONE)
				break;
			if (strncasecmp(tokenlist[0], commands[i].str, strlen(tokenlist[0])) == 0) {
				if (command == NONE)
					command = commands[i].com;
				else {
					printf("Ambiguous command \"%s\" - \"%s\" vs \"%s\"\n", tokenlist[0], commands[command].str, commands[i].str);
					return;
				}
			}
		}
	}
	if (command == NONE) {
		printf("Unknown command \"%s\"\n", tokenlist[0]);
		return;
	}
	switch (command) {
	default:
		printf("Default case found. Why? Fix it!\n");
		break;
	case VERBOSE: {
		if (tokencount != 1u) {
			printf("Usage:\n\n> verbose\n\n");
			return;
		}
		verbose = !verbose;
		printf("Verbose mode %s\n", verbose ? "on" : "off");
	}
	break;
	case EXAMINE: {
		if (tokencount != 3u) {
			printf("Usage:\n\n> examine <start> <end>\n\n");
			return;
		}
		uint32_t hexnum = hex(tokenlist[1], 0);
		if (hexnum >= REGISTER_BASE) {
			printf("Invalid hex start address '%s' (valid is 0 .. %04X)\n",
			       tokenlist[1],
			       REGISTER_BASE - 1);
			return;
		}
		const uint16_t start = hexnum;
		hexnum = hex(tokenlist[2], 0);
		if (hexnum == 0 || hexnum > REGISTER_BASE || hexnum <= start) {
			printf("Invalid hex end address '%s' (valid is 1 .. %04X, and greater than the start)\n",
			       tokenlist[2],
			       REGISTER_BASE);
			return;
		}
		if (!MC6809.assert_stopped())
			return;
		MC6809.preserve_state();
		const uint16_t end = hexnum;
		const uint16_t display_start = start & 0xFFF0u;
		const uint16_t display_end = ((end - 1u) | 0x000Fu) + 1u;
		absolute_time_t next_usb = get_absolute_time();
		for (uint16_t loc = display_start; loc < display_end; loc += 16u) {
			copy(INWARDS, 16u, loc, buffer);
			printf("%04X:", loc);
			for (uint16_t i = 0; i < 16u; i++) {
				if (loc + i >= start && loc + i < end)
					printf(" %02X", buffer[i]);
				else
					printf("   ");
				if (i == 7u)
					printf(" ");
			}
			printf(" |");
			for (uint16_t i = 0; i < 16u; i++) {
				if (loc + i >= start && loc + i < end) {
					auto ch = buffer[i] & 0x7F;
					printf("%c", isprint(ch) ? ch  : '.');
				} else
					printf(" ");
				if (i == 7u)
					printf("|");
			}
			printf("|\n");
			// Keep cranking the USB handle
			// Run TinyUSB service task every 1 ms (non-blocking)
			if (absolute_time_diff_us(get_absolute_time(), next_usb) <= 0) {
				usb_poll();
				next_usb = make_timeout_time_ms(1u);
			}
			//sysreq_process();
		}
		MC6809.restore_state();
	}
	break;
	case MODIFY: {
		if (tokencount < 3u || tokencount > 67u) {
			printf("Usage:\n\n> modify <start> <byte> [ <byte> ... ] (maximum 64 bytes)\n\n");
			return;
		}
		uint32_t hexnum = hex(tokenlist[1], 0);
		if (hexnum > 0xFFFE) {
			printf("Invalid hex start address '%s'\n", tokenlist[1]);
			return;
		}
		const uint16_t start = hexnum;
		for (int i = 2; i < tokencount; i++) {
			hexnum = hex(tokenlist[i], 0);
			if (hexnum > 0xFF) {
				printf("Invalid byte '%s'\n", tokenlist[i]);
				return;
			}
			buffer[i - 2] = hexnum;
		}
		if (start < REGISTER_BASE) {
			if (start + tokencount - 2u > REGISTER_BASE) {
				printf("Overflow into reg region by %u bytes - truncating overflow\n",
				       start + tokencount - 2u - REGISTER_BASE);
				copy(OUTWARDS, REGISTER_BASE - start, start, buffer);
			} else
				copy(OUTWARDS, tokencount - 2, start, buffer);
		} else {
			printf("Modifying system registers, so provoking callbacks\n");
			for (uint16_t i = 0u; i < tokencount - 2u; i++) {
				registers::write(start + i) = buffer[i];
				write_location = (start + i) & 0x3F;
				write_irq_received = true;
				printf("%04X: %02X %02X %u\n", ADDRESS(start + i), static_cast<uint8_t>(registers::write(start + i)), static_cast<uint8_t>(reg[start + i]), write_irq_received);
			}
		}
	}
	break;
	case STATUS: {
		if (tokencount != 1u) {
			printf("Usage:\n\n> status\n\n");
			return;
		}
		// correction factor tweeked from empirical measurement
		constexpr float conversion_factor = ((2.5334f*2.0f)/5.0f)*3.3f/(1 << 12);
		adc_select_input(7);
		uint16_t raw_vcc = adc_read();
		printf("External Vcc = %.1f V\n", raw_vcc * conversion_factor * 2.0f);
		adc_select_input(8);
		uint16_t raw_temp = adc_read();
		float voltage_temp;
		voltage_temp = conversion_factor * raw_temp;
		float temperature = 27.0f - (voltage_temp - 0.706f)/0.001721f;
		printf("Pico2 temperature = %.1f C\n", temperature);
		printf("Run state: %u = %s\n", MC6809.run_state, run_state_string[MC6809.run_state]);
		printf("Bus State: %u = %s\t(previous is %u = %s)\n",
		       MC6809.bus_state.state,
		       ba_bs_string[MC6809.bus_state.state],
		       MC6809.old_bus_state.state,
		       ba_bs_string[MC6809.old_bus_state.state]);
		printf("Task: %u\n", MC6809.task);
		printf("Interrupt task history:");
		for (uint16_t i = 0; i < MC6809.task_stack_ptr; i++)
			printf(" %u", task_stack[i]);
		printf("\n");
		printf("POWER pin: %s\n", POWER_IS_ASSERTED ? "HIGH" : "low");
		printf("RESET pin: %s\n", !RESET_IS_ASSERTED ? "high" : "LOW");
		printf(" HALT pin: %s\n", !HALT_IS_ASSERTED ? "high" : "LOW");
#ifdef DEBUG
		{
			static uint16_t last_LIC = 0u;
			printf("LIC count: %lu\n", MC6809.get_lic_count());
			if (MC6809.run_state == RS_STARTED) {
				uint8_t LIC = MC6809.get_lic_count();
				if (last_LIC != LIC)
					last_LIC = LIC;
				else
					printf("WARNING: LIC count is same as last time (%u), but the run state is %s\n", LIC, run_state_string[MC6809.run_state]);
			}
		}
#endif
		printf("Register and emulated RAM block:\n");
		dump_registers();
		printf("Console:\n");
		printf("Interrupt status: %02X\n", fast_serial.interrupt_status());
		printf("UART: TxQueue: %u\n", fast_serial.receive_level());
		printf("UART: RxQueue: %u\n", fast_serial.transmit_level());
#ifdef DEBUG
		printf("UART: Control: %5u\tTx: %5u\n", UART_CONTROL_COUNT, UART_TX_DATA_COUNT);
		printf("UART: Status:  %5u\tRx: %5u\n", UART_STATUS_COUNT, UART_RX_DATA_COUNT);
#endif
	}
	break;
	case SNIPPET: {
		if (tokencount > 18u || tokencount < 2u) {
			printf("Usage:\n\n> snippet <byte> [ <byte> ... ]\n\n");
			return;
		}
		for (uint16_t i = 1u; i < tokencount; i++) {
			const uint32_t hexnum = hex(tokenlist[i], 0);
			if (hexnum > 0xFF) {
				printf("Invalid byte '%s'\n", tokenlist[i]);
				return;
			}
			buffer[i - 1u] = hexnum;
		}
		printf("%04X:", 0xFFE0u);
		for (uint16_t i = 0; i < tokencount - 1u; i++) {
			reg[REGISTER_SNIPPET_OFFSET + i] = buffer[i];
			printf(" %02X", static_cast<uint8_t>(reg[REGISTER_SNIPPET_OFFSET + i]));
		}
		printf("\n");
		reg[REGISTER_VECTOR_RESET_OFFSET] = REGISTER_SNIPPET;
		MC6809.start();
	}
	break;
	case VECTOR: {
		static constexpr uint16_t vector_count = 8u;
		uint16_t temp[vector_count];
		if (tokencount != vector_count + 1u) {
			printf("Usage:\n\n> vector <vec0> ... <vec7>\n\n");
			return;
		}
		for (int i = 0; i < vector_count; i++) {
			const uint32_t hexnum = hex(tokenlist[i], 0);
			if (hexnum > 0xFFFF) {
				printf("Invalid address '%s'\n", tokenlist[i]);
				return;
			}
			temp[i] = hexnum;
		}
		printf("%04X:", 0xFFF0u);
		for (uint16_t i = 0; i < 8; i++) {
			reg[REGISTER_VECTORS_OFFSET + 2u*i] = temp[i];
			printf(" %04X", static_cast<uint16_t>(reg[REGISTER_VECTORS_OFFSET + 2u*i]));
		}
		printf("\n");
	}
	break;
	case FILL: {
		if (tokencount != 3u && tokencount != 4u) {
			printf("Usage:\n\n> fill <start> <end> [ <value> ]\n\n");
			return;
		}
		uint32_t hexnum = hex(tokenlist[1], 0);
		if (hexnum >= 0xFFFF) {
			printf("Invalid start address '%s'\n", tokenlist[1]);
			return;
		}
		uint16_t start = hexnum;
		hexnum = hex(tokenlist[2], 0);
		if (hexnum > 0xFFFF || hexnum <= start) {
			printf("Invalid end address '%s'\n", tokenlist[2]);
			return;
		}
		uint16_t end = hexnum;
		uint8_t filler;
		if (tokencount > 3u) {
			hexnum = hex(tokenlist[3], 0);
			if (hexnum > 0xFF) {
				printf("Invalid fill byte '%s'\n", tokenlist[3]);
				return;
			}
			filler = hexnum;
		} else
			filler = 0x00u;
		if (!MC6809.assert_stopped())
			return;
		snippet_copy(BLOCK_FILL);
		reg[REGISTER_SNIPPET_OFFSET + 1u] = filler;
		reg[REGISTER_SNIPPET_OFFSET + 3u] = start;
		reg[REGISTER_SNIPPET_OFFSET + 8u] = end;
		MC6809.start_with_timeout(10000u, true);
	}
	break;
	case TIME: {
		if (tokencount > 7u) {
			printf("Usage:\n\n> time [ <Year> [ <Mon> [ <Day> [ <Hour> [ <Min> [ <Sec> ] ] ] ] ] ]\n\n");
			return;
		}
		// 1️⃣ Define the current calendar time (UTC)
		tm tm = {
			.tm_sec = 0,
			.tm_min = 0,
			.tm_hour = 0,
			.tm_mday = 1,
			.tm_mon = 0, // (0 = Jan)
			.tm_year = 2000 - 1900,
			.tm_isdst = 0,
		};
		int num;
		if (tokencount >= 2) {
			num = strtol(tokenlist[1], nullptr, 10);
			if (num < 2025 || num > 2061) {
				printf("Invalid year '%s'\n", tokenlist[1]);
				return;
			}
			tm.tm_year = num - 1900;
		}
		if (tokencount >= 3) {
			num = strtol(tokenlist[2], nullptr, 10);
			if (num < 0 || num > 11) {
				printf("Invalid month '%s'\n", tokenlist[2]);
				return;
			}
			tm.tm_mon = num;
		}
		if (tokencount >= 4) {
			num = strtol(tokenlist[3], nullptr, 10);
			if (num < 1 || num > 31) {
				printf("Invalid day '%s'\n", tokenlist[3]);
				return;
			}
			tm.tm_mday = num;
		}
		if (tokencount >= 5) {
			num = strtol(tokenlist[4], nullptr, 10);
			if (num < 0 || num > 23) {
				printf("Invalid hour '%s'\n", tokenlist[4]);
				return;
			}
			tm.tm_hour = num;
		}
		if (tokencount >= 6) {
			num = strtol(tokenlist[5], nullptr, 10);
			if (num < 0 || num > 59) {
				printf("Invalid minute '%s'\n", tokenlist[5]);
				return;
			}
			tm.tm_min = num;
		}
		if (tokencount == 7) {
			num = strtol(tokenlist[6], nullptr, 10);
			if (num < 0 || num > 59) {
				printf("Invalid second '%s'\n", tokenlist[6]);
				return;
			}
			tm.tm_sec = num;
		}
		if (tokencount > 1)
			aon_timer_set_time_calendar(&tm);
		print_time();
	}
	break;
	case GO: {
		if (tokencount >= 3u) {
			printf("Usage:\n\n> go [ <start> ]\n\n");
			return;
		}
		if (tokencount == 2) {
			const uint32_t hexnum = hex(tokenlist[1], 0);
			if (hexnum > 0xFFFF) {
				printf("Invalid hex start address '%s'\n", tokenlist[1]);
				return;
			}
			const uint16_t start = hexnum;
			reg[REGISTER_VECTOR_RESET_OFFSET] = start;
		}
		MC6809.start();
	}
	break;
	case TASK: {
		uint8_t task = MC6809.task;
		if (tokencount > 2u) {
			printf("Usage:\n\n> task [ <tasknum> ]\n\n");
			return;
		}
		if (tokencount == 2u) {
			const uint32_t hexnum = hex(tokenlist[1], 0);
			if (hexnum > 15u) {
				printf("Invalid hex task number '%s'\n", tokenlist[1]);
				return;
			}
			task = hexnum;
			task = MC6809.task_change(task);
		}
		printf("Task = %u\n", task);
	}
	break;
	case IRQ: {
		if (tokencount != 2u) {
			printf("Usage:\n\n> irq 1|2|3|4|5\n\n");
			return;
		}
		const uint32_t hexnum = hex(tokenlist[1], 0);
		if (hexnum < 1 || hexnum > 5) {
			printf("Bad IRQ test ID '%s'\n", tokenlist[1]);
			return;
		}
		if (!MC6809.assert_stopped())
			return;
		for (int i = 0; i < 16; i++) {
			registers::write(REGISTER_BUFFER_OFFSET + i) = static_cast<uint8_t>(0x00);
			reg[REGISTER_BUFFER_OFFSET + i] = static_cast<uint8_t>(0x00);
		}
		for (int i = 0; i < 16; i += 2)
			reg[REGISTER_VECTORS_OFFSET + i] = static_cast<uint16_t>(0x0000);
		if (hexnum == 1) {
			printf("SWI Test\n");
			snippet_copy(TEST_SWI);
			reg[REGISTER_VECTOR_SWI_OFFSET] = static_cast<uint16_t>(REGISTER_SNIPPET + 14);
			MC6809.start();
			printf("SWI: Check that the MC6809 has finished and that the stack frame is OK\n");
		} else if (hexnum == 2) {
			printf("NMI Test\n");
			uint32_t timeout = 0u;
			snippet_copy(TEST_NMI);
			reg[REGISTER_VECTOR_NMI_OFFSET] = static_cast<uint16_t>(REGISTER_SNIPPET + 15);
			MC6809.start();
			sleep_us(100u);
			ASSERT_NMI;
			while (MC6809.bus_state.state != BS_SYNC) {
				sleep_us(1);
				if (++timeout > 1000000u)
					break;
			}
			DEASSERT_NMI;
			printf("Timeout counter went to %u iterations\n", timeout);
			printf("NMI: Check that the MC6809 has finished and that the stack frame is OK\n");
		} else if (hexnum == 3) {
			printf("IRQ Test\n");
			uint32_t timeout = 0u;
			snippet_copy(TEST_IRQ);
			reg[REGISTER_VECTOR_IRQ_OFFSET] = static_cast<uint16_t>(REGISTER_SNIPPET + 15);
			MC6809.start();
			sleep_us(100u);
			ASSERT_IRQ;
			while (MC6809.bus_state.state != BS_SYNC) {
				sleep_us(1);
				if (++timeout > 1000000u)
					break;
			}
			DEASSERT_IRQ;
			printf("Timeout counter went to %u iterations\n", timeout);
			printf("IRQ: Check that the MC6809 has finished and that the stack frame is OK\n");
		} else if (hexnum == 4) {
			printf("FIRQ Test\n");
			uint32_t timeout = 0u;
			snippet_copy(TEST_FIRQ);
			reg[REGISTER_VECTOR_FIRQ_OFFSET] = static_cast<uint16_t>(REGISTER_SNIPPET + 15);
			MC6809.start();
			sleep_us(100u);
			ASSERT_FIRQ;
			while (MC6809.bus_state.state != BS_SYNC) {
				sleep_us(1);
				if (++timeout > 1000000u)
					break;
			}
			DEASSERT_FIRQ;
			printf("Timeout counter went to %u iterations\n", timeout);
			printf("FIRQ: Check that the MC6809 has finished and that the stack frame is OK\n");
		} else {
			printf("RTI Test\n");
			static constexpr uint16_t stack_len = 16u;
			static uint8_t stack[stack_len] = {
				0xD0, 0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89, 0x90, 0xFF, 0xE6
			};
			for (uint16_t i = 0; i < stack_len; i++)
				reg[REGISTER_BUFFER_OFFSET + 4 + i] = stack[i];
			snippet_copy(TEST_RTI);
			MC6809.start();
			printf("RTI: Check that the MC6809 has finished and that the stack frame is OK\n");
		}
	}
	break;
	case TRACE:
#ifdef DEBUG_NOT_NOW
	{
		if (tokencount > 2u) {
			printf("Usage:\n\n> trace [<len>]\n\n");
			return;
		}
		uint32_t len;
		if (tokencount == 1)
			len = trace_pos;
		else {
			len = strtoul(tokenlist[1], nullptr, 10);
			len = MIN(len, trace_pos);
		}
		if (len > 0)
			for (uint32_t i = 0; i < len; i++) {
				BLA bla{}; bla.byte = static_cast<uint8_t>(trace[i][2] >> 8);
				printf("%04lX %s %02lX %s %s %s %12s %12s %04lX\n",
				       trace[i][0] + 0xFFC0u,
				       (trace[i][2] & TRACE_WRITE) ? "W" : " ",
				       trace[i][1],
				       bla.bit.AVMA ? "AVMA" : "    ",
				       bla.bit.LIC ? "LIC " : "    ",
				       bla.bit.BUSY ? "BUSY" : "    ",
				       ba_bs_string[(trace[i][2] >> 4) & 0x0F],
				       run_state_string[trace[i][2] & 0x0F],
				       trace[i][3]);
			}
	}
	break;
#else
		printf("To enable trace, build with DEBUG defined\n\n");
		break;
#endif
	case RUN: {
		if (tokencount > 3u || tokencount < 1u) {
			printf("Usage:\n\n> run [ <snippetnum>|<chunknum> ] [ ! ]\n\n");
			return;
		}
		if (tokencount == 1u) {
			for (uint16_t i = DAT_INIT; i < SNIPPET_LEN; i++)
				printf("%d: %s\n", i, snippet_string[i]);
			for (uint16_t i = 0; i < CHUNK_LEN; i++)
				printf("%d: %s\n", i + SNIPPET_LEN, chunk_string[i]);
			return;
		}
		bool stop = true;
		if (tokencount == 3u) {
			if (strncmp(tokenlist[2], "!", 1u) == 0)
				stop = false;
		}
		uint32_t hexnum = hex(tokenlist[1], 0);
		snippet snippet_num;
		chunk chunk_num;
		if (hexnum >= SNIPPET_LEN) {
			snippet_num = SNIPPET_LEN;
			hexnum -= SNIPPET_LEN;
			if (hexnum >= CHUNK_LEN) {
				printf("Invalid snippet/chunk index '%s' %x\n", tokenlist[1], hexnum);
				return;
			}
			chunk_num = static_cast<chunk>(hexnum);
		} else
			snippet_num = static_cast<snippet>(hexnum);
		if (MC6809.assert_stopped()) {
			if (snippet_num < SNIPPET_LEN) {
				printf("Running snippet %u = %s\n", snippet_num, snippet_string[snippet_num]);
				snippet_copy(snippet_num);
			} else {
				printf("Running chunk %u = %s\n", chunk_num, chunk_string[chunk_num]);
				copy(OUTWARDS, chunk_len[chunk_num], 0x0130u, chunk_code[chunk_num]);
				reg[REGISTER_VECTOR_RESET_OFFSET] = CHUNK_ADDRESS;
			}
		} else
			return;
		if (stop)
			MC6809.start_with_timeout(1000u, true);
		else
			MC6809.start();
	}
	break;
	case STOP: {
		if (tokencount < 1u || tokencount > 2u) {
			printf("Usage:\n\n> stop [halt|restart|stop]\n\n");
			return;
		}
		if (tokencount == 1u)
			MC6809.stop();
		else {
			if (strcasecmp(tokenlist[1], "halt") == 0)
				MC6809.halt();
			else if (strcasecmp(tokenlist[1], "restart") == 0)
				MC6809.release();
			else if (strcasecmp(tokenlist[1], "stop") == 0)
				MC6809.stop();
			else
				printf("Usage:\n\n> stop [halt|restart|stop]\n\n");
		}
	}
	break;
	case FREQ: {
		if (tokencount != 2u) {
			printf("Usage:\n\n> freq <freqMHz>\n\n");
			return;
		}
		const float mhz = strtof(tokenlist[1], nullptr);
		if (mhz == 0.0f) {
			printf("Invalid frequency '%s'\n", tokenlist[1]);
			return;
		}
		if (mhz < 0.8f || mhz > 3.0f) {
			printf("Frequency %f out of range [0.8 .. 3.0]\n", mhz);
			return;
		}
		MC6809.set_e_frequency(mhz, pio_clock);
		printf("Set E to %.1f MHz\n", MC6809.get_e_frequency());
	}
	break;
	case RESET: {
		if (tokencount != 1u) {
			printf("Usage:\n\n> reset\n\n");
			return;
		}
		peripheral_clear(false);
	}
	break;
	case USB: {
		if (tokencount < 2u) {
			printf("Usage:\n\n> usb ... \n\n");
			return;
		}
		for (unsigned i = 1; i < tokencount; i++) {
			snprintf(reinterpret_cast<char *>(buffer), 64u, "%s\r\n", tokenlist[i]);
			usb_cdc_write(buffer, strlen(reinterpret_cast<char *>(buffer)));
		}
	}
	break;
	case S19: {
		if (tokencount != 1u) {
			printf("Usage:\n\nThis should be a single valid S0, S1, S5 or S9 record\n\n");
			return;
		}
		uint8_t count;
		uint16_t address;
		uint8_t checksum;
		if (process_srecord(tokenlist[0], &count, &address, buffer, &checksum)) {
			if (tokenlist[0][1] == '0') {
				printf("S0 data = \"");
				for (uint16_t i = 0; i < count - 3; i++) {
					const uint8_t c = buffer[i];
					printf("%c", isprint(c) ? c : ' ');
				}
				printf("\"\n");
			} else if (tokenlist[0][1] == '1') {
				if (verbose) {
					printf("S1 address = %04X, length = %u\n", address, count);
					printf("%04X:", address);
					for (uint16_t i = 0; i < count - 3; i++)
						printf(" %02X", buffer[i]);
					printf("\n");
				}
				if (address < 0xFE00) // Don't mess with the DAT RAM or the Pico-supplied registers
					copy(OUTWARDS, count - 3u, address, buffer);
				else if (address >= 0xFFF0) // Allow the reset and interrupt vectors to be set up
					for (uint16_t i = 0; i < count - 3; i++)
						reg[address + i] = buffer[i];
			} else if (tokenlist[0][1] == '5') {
				if (verbose)
					printf("S5 record count = %u\n", address);
			} else if (tokenlist[0][1] == '9') {
				if (verbose)
					printf("S9 start = %04X\n", address);
				reg[REGISTER_VECTOR_RESET_OFFSET] = address;
			}
		} else
			printf("Invalid S record\n");
	}
	break;
	}
}

static volatile bool core1_initialised = false;

#define COMMAND_BUFFER_LEN 128u

static constexpr char prompt[] =  ">>> ";

static void
poll_console()
{
	static char buf[COMMAND_BUFFER_LEN];
	static int idx = 0;
	static bool prompted = false;
	absolute_time_t next_usb = get_absolute_time();

	while (!core1_initialised) {
		sleep_ms(1u);
		usb_poll();
	}

	print_time();

	for (;;) {
		if (!prompted) {
			printf(prompt);
			prompted = true;
		}
		const int ch = getchar_timeout_us(0);
		if (ch != PICO_ERROR_TIMEOUT) {
			if (ch == '\r' || ch == '\n') {
				putchar('\n');
				buf[idx] = '\0';
				process_command(buf);
				idx = 0;
				prompted = false;
			} else if (ch == '\b') {
				if (idx > 0) {
					putchar('\b');
					putchar(' ');
					putchar('\b');
					buf[idx--] = '\0';
				}
			} else if (idx < COMMAND_BUFFER_LEN - 1) {
				if (ch >= ' ' && ch < 0x7F) {
					putchar(ch);
					buf[idx++] = ch;
				}
			}
		}


		// Run TinyUSB service task every 1 ms (non-blocking)
		if (absolute_time_diff_us(get_absolute_time(), next_usb) <= 0) {
			usb_poll();
			next_usb = make_timeout_time_ms(1u);
		}
		MC6809.uart_task();
		//fast_serial.task(registers::write(CONSOLE_CONTROL), &reg[CONSOLE_STATUS], &reg[CONSOLE_RX_DATA]);
		//sysreq_process();
	}
}

void
init_pins_range(const uint8_t base, const uint8_t count)
{
	for (uint8_t i = 0; i < count; ++i)
		gpio_init(base + i);
}

static uint16_t tick_timer[2] = {0u, 0u};
static bool tick_enabled[2] = {false, false};
static bool tick_irq[2] = {false, false};

void
tick_initialise()
{
	reg[SYSTEM_TIMER_STATUS] = static_cast<uint8_t>(0u);
	tick_timer[0] = 0u;
	reg[SYSTEM_TIMER_COUNTERS + 0u] = tick_timer[0];
	tick_timer[1] = 0u;
	reg[SYSTEM_TIMER_COUNTERS + 2u] = tick_timer[1];
	tick_enabled[0] = false;
	tick_enabled[1] = false;
	tick_irq[0] = false;
	tick_irq[1] = false;
}

void
tick_task()
{
	for (uint16_t i = 0; i < 2; i++) {
		if (tick_timer[i] && tick_enabled[i]) {
			reg[SYSTEM_TIMER_COUNTERS + 2*i] = --tick_timer[i];
			if (tick_timer[i] == 0) {
				tick_irq[i] = registers::write(SYSTEM_TIMER_CONTROL) >> (i*4) & TIMER_IRQ;
				if (tick_irq[i])
					reg[SYSTEM_TIMER_STATUS] |= static_cast<uint8_t>((TIMER_IRQ << (4 * i)));
			}
		}
	}
}

interrupt
tick_has_interrupt()
{
	for (uint16_t i = 0; i < 2; i++)
		if (tick_irq[i] && tick_enabled[i])
			return INTERRUPT_IRQ;
	return INTERRUPT_NONE;
}

static void
__no_inline_not_in_flash_func(main_core1)()
{
	printf("Pico2 MC6809E core1 process starting\n");

	init_pins_range(GPIO_TRACE_G, 4); // OINQUE DEBUG trace pins - isr set
	gpio_set_dir_out_masked64(0b1111LLu << GPIO_TRACE_G);

	MC6809.init();
	printf("Pico2 MC6809E guest initialised\n");

	fast_serial.reset();
	printf("Pico2 MC6809E console initialised/reset\n");

	pio_sm_set_enabled(pio_clock.pio, pio_clock.sm, true);
	printf("Pico2 MC6809E E/Q clocks started\n");
	MC6809.set_e_frequency(GUEST_CLK_DEFAULT, pio_clock);
	printf("Pico2 MC6809E E/Q clock set to %.1f MHz\n", MC6809.get_e_frequency());

	gpio_add_raw_irq_handler(GPIO_Q, gpio_clock_eq_irq_handler);
	gpio_set_irq_enabled(GPIO_Q, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
	irq_set_priority(pio_clock.irq, 1);
	irq_set_enabled(pio_clock.irq, true);
	printf("Pico2 MC6809E BA/BS and BUSY/LIC/AVMA bus observer started\n");

	irq_set_exclusive_handler(piodma_read.irq, dma_bus_read_irq_handler);
	irq_set_priority(piodma_read.irq, 0);
	irq_set_enabled(piodma_read.irq, true);
	dma_channel_set_irq0_enabled(piodma_read.data_channel, true);
	dma_channel_start(piodma_read.address_channel);
	pio_sm_put(piodma_read.pio, piodma_read.sm, registers::read_address() >> 6); // Six bits for A0-5
	printf("Pico2 MC6809E Main read DMA chain configured\n");

	irq_set_exclusive_handler(piodma_write.irq, dma_bus_write_irq_handler);
	irq_set_priority(piodma_write.irq, 0);
	irq_set_enabled(piodma_write.irq, true);
	dma_channel_set_irq1_enabled(piodma_write.data_channel, true);
	dma_channel_start(piodma_write.address_channel);
	pio_sm_put(piodma_write.pio, piodma_write.sm, registers::write_address() >> 6); // Six bits for A0-5
	printf("Pico2 MC6809E Main write DMA chain configured\n");

	pio_sm_set_enabled(piodma_read.pio, piodma_read.sm, true);
	pio_sm_set_enabled(piodma_write.pio, piodma_write.sm, true);
	printf("Pico2 MC6809E read pio%d/sm%d and write pio%d/sm%d started\n",
	       PIO_NUM(piodma_read.pio),
	       piodma_read.sm,
	       PIO_NUM(piodma_write.pio),
	       piodma_write.sm);

	sysreq_initialise();
	tick_initialise();
	absolute_time_t next_tick = get_absolute_time();

	printf("Pico2 MC6809E core1 process started\n");

	core1_initialised = true;

	for (;;) {
		if (absolute_time_diff_us(get_absolute_time(), next_tick) <= 0) {
			// tick_task(); // handles system millisecond clock ticks
			next_tick = make_timeout_time_ms(1u);
		}

		// For actions that may take more than one E clock cycle, provide
		// e.g. an "Avail" bit, and set this when the action is done.

		if (q_falling_edge) {
			q_falling_edge = false;
			static uint16_t interrupt_refcount[NUM_INTERRUPTS];
			interrupt eirq;
			//gpio_put(GPIO_TRACE_F, true);
			fast_serial.task();
			interrupt_refcount[INTERRUPT_NONE] = 0;
			interrupt_refcount[INTERRUPT_NMI] = 0;
			interrupt_refcount[INTERRUPT_FIRQ] = 0;
			interrupt_refcount[INTERRUPT_IRQ] = 0;
			// FOREACH interrupt_source:
			eirq = tick_has_interrupt();
			interrupt_refcount[eirq]++;
			eirq = fast_serial.has_interrupt();
#ifdef DEBUG_NOT_NOW
			if (eirq != INTERRUPT_NONE)
				printf("i:%u\n", eirq);
#endif
			interrupt_refcount[eirq]++;
			// ENDFOR
			if (interrupt_refcount[INTERRUPT_NMI] > 0)
				MC6809.assert_interrupt(INTERRUPT_NMI);
			else
				MC6809.deassert_interrupt(INTERRUPT_NMI);
			if (interrupt_refcount[INTERRUPT_FIRQ] > 0)
				MC6809.assert_interrupt(INTERRUPT_FIRQ);
			else
				MC6809.deassert_interrupt(INTERRUPT_FIRQ);
			if (interrupt_refcount[INTERRUPT_IRQ] > 0)
				MC6809.assert_interrupt(INTERRUPT_IRQ);
			else
				MC6809.deassert_interrupt(INTERRUPT_IRQ);
			//gpio_put(GPIO_TRACE_F, false);
		}

		if (write_irq_received) {
			//gpio_put(GPIO_TRACE_F, true);
			write_irq_received = false;
			const uint8_t loc = write_location;
			const uint8_t written_byte = (uint8_t)registers::write(loc);
			switch (loc) {
			default:
				break;

			case SYSTEM_TASK: // Task reg for DAT/MMU
				// TODO: MarkM - Don't allow tasks other than zero to do this!
				reg[SYSTEM_TASK] = MC6809.task_change(written_byte);
				break;

			case SYSTEM_REQUEST:
				if (reg[SYSTEM_REQUEST_RESPONSE] == RESPONSE_BUSY) {
					// ignore request
					registers::write(SYSTEM_REQUEST) = (uint8_t)reg[SYSTEM_REQUEST];
					break;
				}
				switch (written_byte) {
				default:
					registers::write(SYSTEM_REQUEST) = reg[SYSTEM_REQUEST] = REQUEST_NULL;
					reg[SYSTEM_REQUEST_RESPONSE] = RESPONSE_UNKNOWN;;
					break;
				case REQUEST_TIME:
				case REQUEST_SETTIME:
				case REQUEST_MOUNT:
				case REQUEST_UNMOUNT:
					reg[SYSTEM_REQUEST_STATUS] = registers::write(SYSTEM_REQUEST);
					reg[SYSTEM_REQUEST_RESPONSE] = RESPONSE_BUSY;
					break;
				case REQUEST_FLOAT:
					switch (registers::write(SYSTEM_SUBREQUEST)) {
					default:
						reg[SYSTEM_REQUEST_STATUS] = RESPONSE_UNKNOWN;
						reg[SYSTEM_REQUEST_RESPONSE] = RESPONSE_NULL;
						break;
					case FLOAT_ADD: {
						float a = reg[REGISTER_BUFFER_OFFSET + 0];
						float b = reg[REGISTER_BUFFER_OFFSET + 4];
						float c = a + b;
						reg[REGISTER_BUFFER_OFFSET + 0] = c;
						reg[SYSTEM_REQUEST_STATUS] = RESPONSE_DONE;
						reg[SYSTEM_REQUEST_RESPONSE] = static_cast<uint8_t>(c < 0.0f ? 0b1000 : (c == 0.0f ? 0b0100 : 0b0000));
						break;
					}
					case FLOAT_SUBTRACT: {
						float a = reg[REGISTER_BUFFER_OFFSET + 0];
						float b = reg[REGISTER_BUFFER_OFFSET + 4];
						float c = a - b;
						reg[REGISTER_BUFFER_OFFSET + 0] = c;
						reg[SYSTEM_REQUEST_STATUS] = RESPONSE_DONE;
						reg[SYSTEM_REQUEST_RESPONSE] = static_cast<uint8_t>(c < 0.0f ? 0b1000 : (c == 0.0f ? 0b0100 : 0b0000));
						break;
					}
					case FLOAT_MULTIPLY: {
						float a = reg[REGISTER_BUFFER_OFFSET + 0];
						float b = reg[REGISTER_BUFFER_OFFSET + 4];
						float c = a*b;
						reg[REGISTER_BUFFER_OFFSET + 0] = c;
						reg[SYSTEM_REQUEST_STATUS] = RESPONSE_DONE;
						reg[SYSTEM_REQUEST_RESPONSE] = static_cast<uint8_t>(c < 0.0f ? 0b1000 : (c == 0.0f ? 0b0100 : 0b0000));
						break;
					}
					case FLOAT_DIVIDE: {
						float a = reg[REGISTER_BUFFER_OFFSET + 0];
						float b = reg[REGISTER_BUFFER_OFFSET + 4];
						float c = a/b;
						reg[SYSTEM_REQUEST_STATUS] = RESPONSE_DONE;
						reg[SYSTEM_REQUEST_RESPONSE] = static_cast<uint8_t>(c < 0.0f ? 0b1000 : (c == 0.0f ? 0b0100 : 0b0000));
						break;
					}
					}
					break;
				}
				break;

			case SYSTEM_SUBREQUEST:
				// TODO: MarkM - implement finer-grained sysreqs
				break;

			case SYSTEM_TIMER_CONTROL:
				reg[SYSTEM_TIMER_STATUS] = static_cast<uint8_t>(0x00u);
				for (uint16_t i = 0; i < 2; i++)
					tick_irq[i] = false;
				break;

			case SYSTEM_TIMER_COUNTERS + 0:
				break;
			case SYSTEM_TIMER_COUNTERS + 1:
				tick_timer[0] = registers::write(SYSTEM_TIMER_COUNTERS);

			case SYSTEM_TIMER_COUNTERS + 2:
				break;
			case SYSTEM_TIMER_COUNTERS + 3:
				tick_timer[1] = registers::write(SYSTEM_TIMER_COUNTERS + 2u);
				break;

			case CONSOLE_CONTROL: // UART command
#ifdef DEBUG
				if (written_byte & CONSOLE_C_RESET) {
					UART_CONTROL_COUNT = 0u;
					UART_TX_DATA_COUNT = 0u;
					UART_STATUS_COUNT = 0u;
					UART_RX_DATA_COUNT = 0u;
				}
				UART_CONTROL_COUNT++;
#endif
				fast_serial.guest_control();
				break;

			case CONSOLE_TX_DATA: // UART tx data
#ifdef DEBUG
				UART_TX_DATA_COUNT++;
#endif
				fast_serial.guest_transmit();
				break;

			case REGISTER_BUFFER_OFFSET + 0x00:
			case REGISTER_BUFFER_OFFSET + 0x01:
			case REGISTER_BUFFER_OFFSET + 0x02:
			case REGISTER_BUFFER_OFFSET + 0x03:
			case REGISTER_BUFFER_OFFSET + 0x04:
			case REGISTER_BUFFER_OFFSET + 0x05:
			case REGISTER_BUFFER_OFFSET + 0x06:
			case REGISTER_BUFFER_OFFSET + 0x07:
			case REGISTER_BUFFER_OFFSET + 0x08:
			case REGISTER_BUFFER_OFFSET + 0x09:
			case REGISTER_BUFFER_OFFSET + 0x0A:
			case REGISTER_BUFFER_OFFSET + 0x0B:
			case REGISTER_BUFFER_OFFSET + 0x0C:
			case REGISTER_BUFFER_OFFSET + 0x0D:
			case REGISTER_BUFFER_OFFSET + 0x0E:
			case REGISTER_BUFFER_OFFSET + 0x0F:
				// This is a shared block of emulated RAM for
				// moving around blocks of data. Copy the byte from
				// the WRITEs location to its READs mirror.
				reg[loc] = written_byte;
				break;

			case REGISTER_SNIPPET_OFFSET + 0x00:
			case REGISTER_SNIPPET_OFFSET + 0x01:
			case REGISTER_SNIPPET_OFFSET + 0x02:
			case REGISTER_SNIPPET_OFFSET + 0x03:
			case REGISTER_SNIPPET_OFFSET + 0x04:
			case REGISTER_SNIPPET_OFFSET + 0x05:
			case REGISTER_SNIPPET_OFFSET + 0x06:
			case REGISTER_SNIPPET_OFFSET + 0x07:
			case REGISTER_SNIPPET_OFFSET + 0x08:
			case REGISTER_SNIPPET_OFFSET + 0x09:
			case REGISTER_SNIPPET_OFFSET + 0x0A:
			case REGISTER_SNIPPET_OFFSET + 0x0B:
			case REGISTER_SNIPPET_OFFSET + 0x0C:
			case REGISTER_SNIPPET_OFFSET + 0x0D:
			case REGISTER_SNIPPET_OFFSET + 0x0E:
			case REGISTER_SNIPPET_OFFSET + 0x0F:
			// Use this block as write-only memory for debugging. The MC6809
			// may write useful debugging info here.

			case REGISTER_VECTORS_OFFSET + 0x00:
			case REGISTER_VECTORS_OFFSET + 0x01:
			case REGISTER_VECTORS_OFFSET + 0x02:
			case REGISTER_VECTORS_OFFSET + 0x03:
			case REGISTER_VECTORS_OFFSET + 0x04:
			case REGISTER_VECTORS_OFFSET + 0x05:
			case REGISTER_VECTORS_OFFSET + 0x06:
			case REGISTER_VECTORS_OFFSET + 0x07:
			case REGISTER_VECTORS_OFFSET + 0x08:
			case REGISTER_VECTORS_OFFSET + 0x09:
			case REGISTER_VECTORS_OFFSET + 0x0A:
			case REGISTER_VECTORS_OFFSET + 0x0B:
			case REGISTER_VECTORS_OFFSET + 0x0C:
			case REGISTER_VECTORS_OFFSET + 0x0D:
			case REGISTER_VECTORS_OFFSET + 0x0E:
			case REGISTER_VECTORS_OFFSET + 0x0F:
				// Use this block as write-only memory for debugging. The MC6809
				// may write useful debugging info here.
				break;
			}
			//gpio_put(GPIO_TRACE_F, false);
		} // if (write_irq_received)

		if (read_irq_received) {
			//gpio_put(GPIO_TRACE_F, true);
			read_irq_received = false;
			const uint8_t loc = read_location;
			switch (loc) {
			default:
				// The MC6809 got whatever byte was here. By default, do nothing else.
				break;

			case SYSTEM_TASK:
				// Will be set by writing to this address
				break;

			case SYSTEM_REQUEST_RESPONSE:
				// Will be set by the sysreq processor
				break;

			case SYSTEM_TIMER_STATUS:
				break;
			case SYSTEM_TIMER_COUNTERS + 0:
				break;
			case SYSTEM_TIMER_COUNTERS + 1:
				if (reg[SYSTEM_TIMER_COUNTERS] == static_cast<uint16_t>(0u)) {
					tick_irq[0] = false;
					if (registers::write(SYSTEM_TIMER_CONTROL) & TIMER_REPEAT)
						tick_timer[0] = registers::write(SYSTEM_TIMER_COUNTERS);
					reg[SYSTEM_TIMER_STATUS] &= static_cast<uint8_t>(~TIMER_IRQ);
				}
				break;
			case SYSTEM_TIMER_COUNTERS + 2:
				break;
			case SYSTEM_TIMER_COUNTERS + 3:
				if (reg[SYSTEM_TIMER_COUNTERS + 2u] == static_cast<uint16_t>(0u)) {
					tick_irq[1] = false;
					if (registers::write((SYSTEM_TIMER_CONTROL) >> 4) & TIMER_REPEAT)
						tick_timer[1] = registers::write(SYSTEM_TIMER_COUNTERS + 2u);
					reg[SYSTEM_TIMER_STATUS] &= static_cast<uint8_t>(~(TIMER_IRQ << 4));
				}
				break;

			case CONSOLE_STATUS: // UART status
#ifdef DEBUG
				UART_STATUS_COUNT++;
#endif
				fast_serial.guest_status();
				break;

			case CONSOLE_RX_DATA: // UART Rx data
#ifdef DEBUG
				UART_RX_DATA_COUNT++;
#endif
				// The guest already got the byte, now get the next one
				fast_serial.guest_receive();
				break;
			}
			//gpio_put(GPIO_TRACE_F, false);
		} // if (read_irq_received)
	} // for(;;)
}

static void
clock_pio_init()
{
	// SM0: E/Q clock
	const int ofs = pio_add_program(pio_clock.pio, &mc6809_clock_program);
	if (ofs < 0) {
		printf("Pico2 MC6809E insufficient space for E/Q program\n");
		panic("Unable to proceed");
	}
	pio_clock.sm = pio_claim_unused_sm(pio_clock.pio, false);
	if (pio_clock.sm < 0) {
		printf("Pico2 MC6809E unable to claim unused SM for E/Q program\n");
		panic("Unable to proceed");
	}
	mc6809_clock_program_init(pio_clock.pio, pio_clock.sm, ofs);

	printf("Pico2 MC6809E E/Q clocks pio%d sm%d configured\n", PIO_NUM(BUS_CLOCK_PIO), pio_clock.sm);
}

static void
bus_pio_dma_init()
{
	// Pins mc6809_bus
	init_pins_range(GPIO_A_BASE, 6); // A0..5 inputs
	gpio_set_dir_in_masked(0b111111u << GPIO_A_BASE);

	gpio_init(GPIO_RW);
	gpio_set_dir(GPIO_RW, GPIO_IN);
	init_pins_range(GPIO_D_BASE, 8); // D0..7 inputs initially (inputs/tri-state)
	gpio_set_dir_in_masked(0b11111111u << GPIO_D_BASE);

	gpio_init(GPIO_CS);
	gpio_set_dir(GPIO_CS, GPIO_IN);

	// Bus status
	init_pins_range(GPIO_BS, 2); // BS, BA inputs
	gpio_set_dir_in_masked(0b11u << GPIO_BS);
	init_pins_range(GPIO_BUSY, 3); // BUSY, LIC, AVMA
	gpio_set_dir_in_masked(0b111u << GPIO_BUSY);

	// DMA for main MC6809 bus read loop
	if (dma_channel_is_claimed(piodma_read.address_channel)) {
		printf("Pico2 MC6809E DMA channel %d for bus_read address is already claimed\n",
		       piodma_read.address_channel);
		panic("Unable to proceed");
	}
	dma_channel_claim(piodma_read.address_channel);
	if (dma_channel_is_claimed(piodma_read.data_channel)) {
		printf("Pico2 MC6809E DMA channel %d for bus_read data is already claimed\n", piodma_read.data_channel);
		panic("Unable to proceed");
	}
	dma_channel_claim(piodma_read.data_channel);

	// SM: Bus read responder
	const int ofs_read_bus = pio_add_program(piodma_read.pio, &mc6809_bus_read_program);
	if (ofs_read_bus < 0) {
		printf("Pico2 MC6809E insufficient space for bus_read program\n");
		panic("Unable to proceed");
	}
	piodma_read.sm = pio_claim_unused_sm(piodma_read.pio, true);
	if (piodma_read.sm < 0) {
		printf("Pico2 MC6809E unable to claim unused SM for bus_read program\n");
		panic("Unable to proceed");
	}
	mc6809_bus_read_program_init(piodma_read.pio, piodma_read.sm, ofs_read_bus);
	printf("Pico2 MC6809E read pio%u sm%d configured\n", PIO_NUM(piodma_read.pio), piodma_read.sm);
	// DMA move the requested memory data to PIO for output
	dma_channel_config read_out_dma = dma_channel_get_default_config(piodma_read.data_channel);
	channel_config_set_high_priority(&read_out_dma, true);
	channel_config_set_dreq(&read_out_dma, pio_get_dreq(piodma_read.pio, piodma_read.sm, true));
	channel_config_set_read_increment(&read_out_dma, false);
	channel_config_set_write_increment(&read_out_dma, false);
	channel_config_set_transfer_data_size(&read_out_dma, DMA_SIZE_8);
	channel_config_set_chain_to(&read_out_dma, piodma_read.address_channel);
	dma_channel_configure(
		piodma_read.data_channel,
		&read_out_dma,
		&piodma_read.pio->txf[piodma_read.sm],	// dst
		nullptr,				// src - will be overwritten by the below DMA
		dma_encode_transfer_count(1),
		false);
	printf("Pico2 MC6809E DMA read_out channel configured\n");
	// DMA move address from PIO into the data DMA config
	dma_channel_config read_address_dma = dma_channel_get_default_config(piodma_read.address_channel);
	channel_config_set_high_priority(&read_address_dma, true);
	channel_config_set_dreq(&read_address_dma, pio_get_dreq(piodma_read.pio, piodma_read.sm, false));
	channel_config_set_read_increment(&read_address_dma, false);
	channel_config_set_write_increment(&read_address_dma, false);
	channel_config_set_transfer_data_size(&read_address_dma, DMA_SIZE_32);
	channel_config_set_chain_to(&read_address_dma, piodma_read.data_channel);
	dma_channel_configure(
		piodma_read.address_channel,
		&read_address_dma,
		&dma_channel_hw_addr(piodma_read.data_channel)->read_addr,	// dst
		&piodma_read.pio->rxf[piodma_read.sm],				// src
		dma_encode_transfer_count(1),
		false);
	printf("Pico2 MC6809E DMA read_address channel configured\n");

	// SM: Bus write responder
	const int ofs_write_bus = pio_add_program(piodma_write.pio, &mc6809_bus_write_program);
	if (ofs_write_bus < 0) {
		printf("Pico2 MC6809E insufficient space for bus_write program\n");
		panic("Unable to proceed");
	}
	piodma_write.sm = pio_claim_unused_sm(piodma_write.pio, true);
	if (piodma_write.sm < 0) {
		printf("Pico2 MC6809E unable to claim unused SM for bus_write program\n");
		panic("Unable to proceed");
	}
	mc6809_bus_write_program_init(piodma_write.pio, piodma_write.sm, ofs_write_bus);
	printf("Pico2 MC6809E write pio%u sm%d configured\n", PIO_NUM(piodma_write.pio), piodma_write.sm);
	// DMA for main MC6809 bus write loop
	if (dma_channel_is_claimed(piodma_write.address_channel)) {
		printf("Pico2 MC6809E DMA channel %d for bus_write address is already claimed\n",
		       piodma_write.address_channel);
		panic("Unable to proceed");
	}
	dma_channel_claim(piodma_write.address_channel);
	if (dma_channel_is_claimed(piodma_write.data_channel)) {
		printf("Pico2 MC6809E DMA channel %d for bus_write data is already claimed\n",
		       piodma_write.data_channel);
		panic("Unable to proceed");
	}
	dma_channel_claim(piodma_write.data_channel);
	dma_channel_config write_in_dma = dma_channel_get_default_config(piodma_write.data_channel);
	channel_config_set_high_priority(&write_in_dma, true);
	channel_config_set_dreq(&write_in_dma, pio_get_dreq(piodma_write.pio, piodma_write.sm, false));
	channel_config_set_read_increment(&write_in_dma, false);
	channel_config_set_write_increment(&write_in_dma, false);
	channel_config_set_transfer_data_size(&write_in_dma, DMA_SIZE_8);
	channel_config_set_chain_to(&write_in_dma, piodma_write.address_channel);
	dma_channel_configure(
		piodma_write.data_channel,
		&write_in_dma,
		nullptr,					// dst - will be overwritten by the below DMA
		&piodma_write.pio->rxf[piodma_write.sm],	// src
		dma_encode_transfer_count(1),
		false);
	printf("Pico2 MC6809E DMA write_in channel configured\n");
	// DMA move address from PIO into the write data DMA config
	dma_channel_config write_address_dma = dma_channel_get_default_config(piodma_write.address_channel);
	channel_config_set_high_priority(&write_address_dma, true);
	channel_config_set_dreq(&write_address_dma, pio_get_dreq(piodma_write.pio, piodma_write.sm, false));
	channel_config_set_read_increment(&write_address_dma, false);
	channel_config_set_write_increment(&write_address_dma, false);
	channel_config_set_transfer_data_size(&write_address_dma, DMA_SIZE_32);
	channel_config_set_chain_to(&write_address_dma, piodma_write.data_channel);
	dma_channel_configure(
		piodma_write.address_channel,
		&write_address_dma,
		&dma_channel_hw_addr(piodma_write.data_channel)->write_addr,	// dst
		&piodma_write.pio->rxf[piodma_write.sm],			// src
		dma_encode_transfer_count(1),
		false);
	printf("Pico2 MC6809E DMA write_address channel configured\n");
}

#if 0
static void
measure_freqs()
{
	uint32_t f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY)/1000;
	uint32_t f_pll_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_USB_CLKSRC_PRIMARY)/1000;
	uint32_t f_rosc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_ROSC_CLKSRC);
	uint32_t f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS)/1000;
	uint32_t f_clk_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI)/1000;
	uint32_t f_clk_usb = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB)/1000;
	uint32_t f_clk_adc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC)/1000;
#ifdef CLOCKS_FC0_SRC_VALUE_CLK_RTC
	uint32_t f_clk_rtc = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC);
#endif
	printf("pll_sys  = %d MHz\n", f_pll_sys);
	printf("pll_usb  = %d MHz\n", f_pll_usb);
	printf("rosc     = %d kHz\n", f_rosc);
	printf("clk_sys  = %d MHz\n", f_clk_sys);
	printf("clk_peri = %d MHz\n", f_clk_peri);
	printf("clk_usb  = %d MHz\n", f_clk_usb);
	printf("clk_adc  = %d MHz\n", f_clk_adc);
#ifdef CLOCKS_FC0_SRC_VALUE_CLK_RTC
	printf("clk_rtc  = %d kHz\n", f_clk_rtc);
#endif
// Can't measure clk_ref/xosc as it is the ref
}
#endif

int
main()
{
	vreg_set_voltage(VREG_VOLTAGE_1_50);
	sleep_ms(5);

	stdio_init_all();

	printf("\n\nPico2 MC6809E bus supervisor %s\n", DEVICE_VERSION);
	printf("Pico2 MC6809E built on %s %s\n", __DATE__, __TIME__);
#ifdef DEBUG
	printf("Pico2 MC6809E build with DEBUG defined\n");
#endif

	printf("Pico2 MC6809E raising sys_clock to %d MHz ... ", SYS_CLK_DEFAULT/1000000);
	if (set_sys_clock_hz(SYS_CLK_DEFAULT, false)) {
		stdio_init_all();
		uart_init(CONSOLE, CONSOLE_BAUDRATE);
		printf("successful!\n");
	} else {
		printf("unsuccessful! Reverting to %d MHz ... ", SYS_CLK_HZ/1000000);
		set_sys_clock_hz(SYS_CLK_HZ, true);
		stdio_init_all();
		uart_init(CONSOLE, CONSOLE_BAUDRATE);
		printf("successful!\n");
	}
	printf("Pico2 MC6809E console baudrate set to %d\n", CONSOLE_BAUDRATE);

	adc_init();
	adc_gpio_init(GPIO_POWER_ADC);
	adc_set_temp_sensor_enabled(true);
	// Enable external electronics' power
	gpio_init(GPIO_POWER);
	gpio_set_dir(GPIO_POWER, GPIO_OUT);
	POWER_ASSERT;

	// Assert reset until the guest is needed
	gpio_init(GPIO_RESET);
	gpio_set_dir(GPIO_RESET, GPIO_OUT);
	RESET_ASSERT;

	// Hold !halt high until needed
	gpio_init(GPIO_HALT);
	gpio_set_dir(GPIO_HALT, GPIO_OUT);
	HALT_DEASSERT;

	// The 3 interrupt outputs are going to pretend to be open-collector by
	// either going high impedance (becoming an input) or driving low
	// (becoming an output). Macros will be used to avoid confusion.
	gpio_init(GPIO_NMI);
	gpio_put(GPIO_NMI, false);
	DEASSERT_NMI;
	gpio_init(GPIO_FIRQ);
	gpio_put(GPIO_FIRQ, false);
	DEASSERT_FIRQ;
	gpio_init(GPIO_IRQ);
	gpio_put(GPIO_IRQ, false);
	DEASSERT_IRQ;

	sleep_ms(10);
	printf("Pico2 MC6809E external power on.\n");

	usb_init();
	printf("Pico2 MC6809E initialised USB\n");

	clock_pio_init();
	bus_pio_dma_init();
	peripheral_clear(true);
	printf("Pico2 MC6809E emulated RAM and reg initialised\n");

	multicore_launch_core1(main_core1);

	tm tm = build_time_tm();
	aon_timer_stop();
	aon_timer_start_calendar(&tm);

	printf("Pico2 MC6809E ready!\n");

	poll_console();
}
