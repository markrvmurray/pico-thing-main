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
//#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/vreg.h"
#include "bsp/board_api.h"

#include "tusb.h"
#include "usb.h"
#include "registers.h"
#include "pico_thing.h"
#include "mc6809.h"
#include "srec.h"
#include "mc6850.h"
#include "tick_timer.h"
#include "build_time.h"
#include "command_parser.h"

/* Flex/bison generated scanner API (yy_buffer_state is flex's internal type) */
extern "C" {
struct yy_buffer_state;
typedef struct yy_buffer_state *YY_BUFFER_STATE;
extern YY_BUFFER_STATE cmd_yy_scan_string(const char *);
extern void            cmd_yy_delete_buffer(YY_BUFFER_STATE);
extern int             cmd_yyparse(parsed_cmd_t *);
extern void            cmd_yy_reset_str_pool(void);
}

// PIO program headers will be generated from the .pio file
#include <cstdint>

#include "pico_thing.pio.h"

// I'm really pushing my luck with this.
#define SYS_CLK_DEFAULT 320000000

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

// PIO-driven task pins (pio2, GPIO base 16, pins 40-45)
static PIO task_pio;
static uint task_pio_sm;

static inline void
task_pins_update(uint8_t task)
{
	uint32_t pio_word = task | ((task == 0u) ? (1u << NUM_TASK_PINS) : 0u);
	pio_sm_put(task_pio, task_pio_sm, pio_word);
}

// Keep track of which task was interrupted
#define MAX_INT_NEST_DEPTH 16u
static volatile uint8_t task_stack[MAX_INT_NEST_DEPTH];


//	BS_RUNNING,    BS_IRQ,         BS_SYNC,   BS_HALT,   BS_RUNNING_RESET ....
static const run_state run_state_table[RUN_STATE_COUNT][BUS_STATE_COUNT] = {
	{RS_STARTED, RS_INTERRUPTED, RS_SYNCED, RS_HALTED, RS_STOPPED, RS_STOPPED, RS_STOPPED, RS_STOPPED}, // RS_STARTED
	{RS_STARTED, RS_INTERRUPTED, RS_SYNCED, RS_HALTED, RS_STOPPED, RS_STOPPED, RS_STOPPED, RS_STOPPED}, // RS_INTERRUPTED
	{RS_STARTED, RS_INTERRUPTED, RS_SYNCED, RS_HALTED, RS_STOPPED, RS_STOPPED, RS_STOPPED, RS_STOPPED}, // RS_HALTED
	{RS_STARTED, RS_INTERRUPTED, RS_SYNCED, RS_HALTED, RS_STOPPED, RS_STOPPED, RS_STOPPED, RS_STOPPED}, // RS_SYNCED
	{RS_STARTED, RS_INTERRUPTED, RS_SYNCED, RS_HALTED, RS_STOPPED, RS_STOPPED, RS_STOPPED, RS_STOPPED}, // RS_STOPPED
};

static volatile bool q_rising_edge = false;
static volatile bool q_falling_edge = false;
static volatile bool core1_initialised = false;

mc6850 fast_serial(CONSOLE_CONTROL, CONSOLE_TX_DATA);
mc6850 aux_serial(AUX_CONTROL, AUX_TX_DATA);

// Cached interrupt state — updated by for(;;) loop's has_interrupt() calls,
// applied to GPIO pins by the Q-rising ISR.  Decouples IRQ delivery from
// for(;;) loop throughput.  Packed into a single byte so the ISR does one
// volatile read (not three) — the init ISR budget is extremely tight.
static volatile uint8_t pending_interrupts = 0u;
static constexpr uint8_t PEND_NMI  = 1u << 0;
static constexpr uint8_t PEND_FIRQ = 1u << 1;
static constexpr uint8_t PEND_IRQ  = 1u << 2;

// 50 Hz tick timer (IRQ) + E-cycle countdown (NMI).
static tick_timer system_tick(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);

static volatile uint32_t core1_qrise_count = 0u;
static volatile uint32_t core1_irq_assert_count = 0u;
static volatile uint32_t core1_loop_count = 0u;
static volatile uint32_t core1_write_ring_count = 0u;
static volatile uint32_t core1_read_irq_count = 0u;
static volatile uint8_t core1_loop_phase = 0u;
static volatile uint8_t core1_last_write_loc = 0u;
static volatile uint8_t core1_last_read_loc = 0u;
static volatile uint32_t core1_isr_count = 0u;	// incremented in GPIO ISR

// Stack watermark for Core 1 — paint stack with 0xDEADBEEF, then scan
// from the bottom to find the high-water mark.
extern "C" uint32_t __StackOneBottom;	// linker symbol
extern "C" uint32_t __StackOneTop;	// linker symbol

void __isr
__time_critical_func(gpio_clock_eq_irq_handler())
{
#ifdef DEBUG
	gpio_put(GPIO_TRACE_EQ_IRQ, true); // scope timing
#endif
	core1_isr_count++;
	if (gpio_get_irq_event_mask(GPIO_Q) & GPIO_IRQ_EDGE_RISE) {
		q_rising_edge = true;
		gpio_acknowledge_irq(GPIO_Q, GPIO_IRQ_EDGE_RISE);
		bus_pins bus_pins{};
		bus_pins.word = gpio_get_all();
		// BUSY/LIC/AVMA are stable at Q-rising (this is the latest valid capture point;
		// by Q-falling the 6809 has already updated them for the next cycle). Unpack
		// immediately so that vma is valid before the DMA read ISR fires (~20 cycles later).
		MC6809.set_busy_lic_avma(static_cast<uint8_t>(bus_pins.bits.BUSY_LIC_AVMA));
		ba_bs_u new_bus_state{};
		new_bus_state.byte = ((bus_pins.bits.BS_BA) & 0b00000011u);
		new_bus_state.bit.RESET = RESET_IS_ASSERTED;
#ifdef DEBUG
		MC6809.count_lic();
#endif
		if (MC6809.bus_state.state != new_bus_state.state) {
			MC6809.old_bus_state.state = MC6809.bus_state.state;
			MC6809.bus_state.state = new_bus_state.state;
			MC6809.run_state = run_state_table[MC6809.run_state][MC6809.bus_state.state];
			if (new_bus_state.state == BS_IRQ) {
				auto eirq = static_cast<interrupt>(bus_pins.bits.A >> 1);
				if (eirq < INTERRUPT_RESET && MC6809.task_stack_ptr < MAX_INT_NEST_DEPTH)
					task_stack[MC6809.task_stack_ptr.fetch_add(1u, std::memory_order_relaxed)] = MC6809.task;
			}
		}
		// E-cycle countdown NMI timer
		if (system_tick.tick())
			pending_interrupts |= PEND_NMI;
		// Assert/deassert all interrupt pins from cached pending state.
		// NMI is edge-triggered (one-shot): assert once and clear flag.
		// IRQ/FIRQ are level-triggered: assert when pending, deassert
		// when not.  The DMA read ISR clears PEND_IRQ after
		// rx_read_fast_path() and timer reads, keeping stale assertion
		// windows short.
		{
			const uint8_t pend = pending_interrupts;
			if (pend & PEND_NMI) {
				ASSERT_NMI;
				pending_interrupts = pend & ~PEND_NMI;
			}
			if (pend & PEND_FIRQ) ASSERT_FIRQ; else DEASSERT_FIRQ;
			if (pend & PEND_IRQ)  ASSERT_IRQ;  else DEASSERT_IRQ;
		}
//	} else if (gpio_get_irq_event_mask(GPIO_Q) & GPIO_IRQ_EDGE_FALL) {
	} else {
		q_falling_edge = true;
		gpio_acknowledge_irq(GPIO_Q, GPIO_IRQ_EDGE_FALL);
		bus_pins bus_pins{};
		bus_pins.word = gpio_get_all();
		// Grab the current 8 D pins if this is a read cycle, and the LIC timing lets us
		// know it is an instruction, and it is the last byte of the snippet block.
		// I want to catch RTI (0x3B).
		if (MC6809.get_lic() && bus_pins.bits.RW && !bus_pins.bits.CS && bus_pins.bits.A == ((REGISTER_SNIPPET + REGISTER_SNIPPET_LEN - 1) & registers::register_mask) && bus_pins.bits.D == 0x3Bu)
			MC6809.apply_rti();
		// BUSY/LIC/AVMA are captured at Q-rising (the latest valid point for the current
		// cycle). By Q-falling the 6809 has already updated them for the next cycle, so
		// we do NOT capture them here.
	}
#ifdef DEBUG
	gpio_put(GPIO_TRACE_EQ_IRQ, false); // scope timing
#endif
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
// Dedicated lossless counter for CONSOLE_RX_DATA reads.
// The shared read_irq_received slot can be overwritten before Core 1 processes
// it (e.g. a CONSOLE_STATUS read ISR fires right after a CONSOLE_RX_DATA read
// ISR), which would silently skip guest_receive() and cause byte duplication.
// A wrapping counter survives any number of overwrites of the shared slot.
static volatile uint8_t rx_read_count = 0;
static          uint8_t rx_processed_count = 0;
static volatile uint8_t aux_rx_read_count = 0;
static          uint8_t aux_rx_processed_count = 0;

void __isr
__time_critical_func(dma_bus_read_irq_handler())
{
#ifdef DEBUG
	gpio_put(GPIO_TRACE_READ_IRQ, true); // scope timing
#endif
	//if (dma_channel_get_irq0_status(piodma_read.data_channel)) {
	dma_channel_acknowledge_irq0(piodma_read.data_channel);
	if (!MC6809.bus_state.bit.RESET && MC6809.get_vma()) {
		const uintptr_t read_addr = dma_channel_hw_addr(piodma_read.data_channel)->read_addr;
		const uint8_t loc = read_addr & registers::register_mask;
		if (loc == CONSOLE_RX_DATA) {
			// Use a dedicated wrapping counter so that a rapid CONSOLE_STATUS
			// ISR cannot overwrite this notification before Core 1 processes it.
			// A single read_irq_received flag would be silently lost if a second
			// ISR fired before Core 1 drained it, causing guest_receive() to be
			// skipped and RDRF to stay set with a stale byte.
			rx_read_count++;
			// Fast-path: clear RDRF/IRQ immediately so the next 6809 status
			// read sees the change.  guest_receive() will stage the next byte
			// (if any) and re-set RDRF on the next for(;;) iteration.
			fast_serial.rx_read_fast_path();
			// Do NOT clear PEND_IRQ here — the for(;;) loop recomputes it
			// from all device has_interrupt() calls each Q-rising edge.
			// Clearing here would briefly deassert /IRQ even if another
			// device still has a pending interrupt.
		} else if (loc == AUX_RX_DATA) {
			aux_rx_read_count++;
			aux_serial.rx_read_fast_path();
		} else if (loc == SYSTEM_TIMER_STATUS) {
			// Reading the timer status register acknowledges the IRQ.
			system_tick.acknowledge();
		} else {
			read_location = loc;
			read_irq_received = true;
		}
#ifdef DEBUG_NOT_NOW
		if (trace_pos < trace_size - 1) {
			trace[trace_pos][0] = loc;
			trace[trace_pos][1] = *reinterpret_cast<volatile std::uint8_t *>(read_addr);
			trace[trace_pos][2] = TRACE_READ | (MC6809.get_busy_lic_avma() << 8) | (MC6809.bus_state.byte << 4) | MC6809.run_state;
			trace[trace_pos][3] = 0u;
			trace_pos++;
		}
#endif
	}
	//}
#ifdef DEBUG
	gpio_put(GPIO_TRACE_READ_IRQ, false); // scope timing
#endif
}

// ISR for the R/!W bus write loop and DMA chain
// The PIO mediates the signalling
// There are 2 DMA transfers:
//   1) accept write_registers + A5-0
//   2) accept the byte to be written to that address

// Ring buffer for DMA write events.  The ISR pushes {location, value}
// entries; the Core 1 for(;;) loop drains them.  This replaces the old
// single-slot write_location / write_irq_received pair, which silently
// dropped writes when back-to-back 6809 writes arrived faster than the
// for(;;) loop could process them.
struct write_entry {
	uint8_t location;
	uint8_t value;
};
static constexpr uint8_t WRITE_RING_SIZE = 32u;  // must be power of 2
static constexpr uint8_t WRITE_RING_MASK = WRITE_RING_SIZE - 1u;
static volatile write_entry write_ring[WRITE_RING_SIZE];
static volatile uint8_t write_ring_head = 0u;    // written by ISR
static volatile uint8_t write_ring_tail = 0u;    // read by for(;;) loop
#ifdef DEBUG
static uint32_t write_ring_overflow = 0u;
#endif

void __isr
__time_critical_func(dma_bus_write_irq_handler())
{
#ifdef DEBUG
	gpio_put(GPIO_TRACE_WRITE_IRQ, true); // scope timing
#endif
	// if (dma_channel_get_irq1_status(piodma_write.data_channel)) {
	dma_channel_acknowledge_irq1(piodma_write.data_channel);
	if (!MC6809.bus_state.bit.RESET) {
		const uintptr_t written_addr = dma_channel_hw_addr(piodma_write.data_channel)->write_addr;
		const uint8_t wloc = written_addr & registers::register_mask;
		const uint8_t wval = *reinterpret_cast<volatile std::uint8_t *>(written_addr);
		// Special-case any writes that need to be handled before the next guest
		// instruction picks up an incorrect read. I.e., these need to be _fast_.
		if (wloc == SYSTEM_TASK) {
			const uint8_t new_task = MC6809.task_change(wval);
			reg[SYSTEM_TASK] = new_task;
			task_pins_update(new_task);
		}
		// CONSOLE_CONTROL / AUX_CONTROL: full guest_control() is deferred
		// to the write_ring so mode changes are sequenced after pending TX
		// data.  But RTS must take effect immediately — otherwise
		// has_interrupt() can stage another byte in the window between the
		// 6809 deasserting RTS and the for(;;) loop processing the write.
		else if (wloc == CONSOLE_CONTROL)
			fast_serial.control_written_isr(wval);
		else if (wloc == AUX_CONTROL)
			aux_serial.control_written_isr(wval);
		else if (wloc == CONSOLE_TX_DATA)
			fast_serial.write_received();	// clear TDRE now; for(;;) restores it after guest_transmit()
		else if (wloc == AUX_TX_DATA)
			aux_serial.write_received();
		else if (wloc == SYSTEM_TIMER_CONTROL_13)
			system_tick.control(wval);
		// The first 16 bytes of emulated registers are devices. The rest are RAM as far as the guest
		// CPU is concerned.
		else if (wloc >= REGISTER_BUFFER_OFFSET && wloc < REGISTER_BUFFER_OFFSET + REGISTER_BUFFER_LEN*3)
			reg[wloc] = wval;
		// Queue for the for(;;) loop to do full dispatch.
		const uint8_t h = write_ring_head;
		const uint8_t next_h = (h + 1u) & WRITE_RING_MASK;
		if (next_h == write_ring_tail) {
			// Ring full — drop oldest entry so the newest is never lost.
			write_ring_tail = (write_ring_tail + 1u) & WRITE_RING_MASK;
#ifdef DEBUG
			write_ring_overflow++;
#endif
		}
		write_ring[h].location = wloc;
		write_ring[h].value = wval;
		write_ring_head = next_h;
#ifdef DEBUG_NOT_NOW
		if (trace_pos < trace_size - 1) {
			trace[trace_pos][0] = wloc;
			trace[trace_pos][1] = wval;
			trace[trace_pos][2] = TRACE_WRITE | (MC6809.busy_lic_avma.byte << 8) | (MC6809.bus_state.byte << 4) | MC6809.run_state;
			trace[trace_pos][3] = 0u;
			trace_pos++;
		}
#endif
	}
	// }
#ifdef DEBUG
	gpio_put(GPIO_TRACE_WRITE_IRQ, false); // scope timing
#endif
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

static bool
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
			if (!MC6809.start_with_timeout(4u)) {
				printf("Snippet execution failed — CPU did not respond\n");
				return false;
			}
		} else {
			if (!MC6809.start_with_timeout(4u)) {
				printf("Snippet execution failed — CPU did not respond\n");
				return false;
			}
			registers::copy_out(data + pos, REGISTER_BUFFER_OFFSET, len);
		}
		start += len;
		pos += len;
	}
	return true;
}

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

#ifdef DEBUG
static volatile unsigned ACIA_CONTROL_COUNT = 0u, ACIA_TX_DATA_COUNT = 0u;
static volatile unsigned ACIA_STATUS_COUNT = 0u, ACIA_RX_DATA_COUNT = 0u;
static volatile unsigned AUX_ACIA_CONTROL_COUNT = 0u, AUX_ACIA_TX_DATA_COUNT = 0u;
static volatile unsigned AUX_ACIA_STATUS_COUNT = 0u, AUX_ACIA_RX_DATA_COUNT = 0u;
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
	ACIA_CONTROL_COUNT = 0u;
	ACIA_TX_DATA_COUNT = 0u;
	ACIA_STATUS_COUNT = 0u;
	ACIA_RX_DATA_COUNT = 0u;
	AUX_ACIA_CONTROL_COUNT = 0u;
	AUX_ACIA_TX_DATA_COUNT = 0u;
	AUX_ACIA_STATUS_COUNT = 0u;
	AUX_ACIA_RX_DATA_COUNT = 0u;
#endif

	// Reset emulated peripherals (as a real !RESET would)
	fast_serial.reset();
	aux_serial.reset();
	system_tick.reset();
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
		case REQUEST_SET_TIME: {
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

/* cmd_parse() - glue between process_command() and the flex/bison parser */
bool
cmd_parse(char *input, parsed_cmd_t *out)
{
	out->tag = CMD_NONE;
	cmd_yy_reset_str_pool();
	YY_BUFFER_STATE buf = cmd_yy_scan_string(input);
	int rc = cmd_yyparse(out);
	cmd_yy_delete_buffer(buf);
	/* YYERROR in grammar actions does not invoke yyerror(), so cmd.tag may
	 * remain CMD_NONE on failure.  Normalise to CMD_SYNTAX_ERROR. */
	if (rc != 0 && out->tag == CMD_NONE)
		out->tag = CMD_SYNTAX_ERROR;
	return rc == 0;
}

static volatile interrupt console_interrupt = INTERRUPT_NONE;
// Tracks which interrupt caused the current break (INTERRUPT_NONE = no active break).
// Set by CMD_BREAK on success; cleared by CMD_BREAK_RETURN.
static interrupt break_active = INTERRUPT_NONE;

static void
console_set_interrupt(interrupt eirq)
{
	console_interrupt = eirq;
}

static interrupt
console_has_interrupt()
{
	return console_interrupt;
}

void
process_command(char *buf)
{
	static uint8_t buffer[256];
	parsed_cmd_t cmd;

	/* Skip blank lines */
	while (isspace(static_cast<unsigned char>(*buf)))
		buf++;
	if (*buf == '\0')
		return;

	if (!cmd_parse(buf, &cmd))
		return; /* error message already printed by parser */

	switch (cmd.tag) {

	case CMD_POWER:
		// TODO: MarkM: Start powered-down, and refactor for user-controlled power-on and -off.
		break;

	case CMD_VERBOSE:
		verbose = !verbose;
		printf("Verbose mode %s\n", verbose ? "on" : "off");
		break;

	case CMD_STATUS: {
#ifdef NO_ADC_YET
		constexpr float conversion_factor = ((2.5334f*2.0f)/5.0f)*3.3f/(1 << 12);
		adc_select_input(7);
		uint16_t raw_vcc = adc_read();
		printf("External Vcc = %.1f V\n", 2.0f * conversion_factor * static_cast<float>(raw_vcc));
		adc_select_input(8);
		uint16_t raw_temp = adc_read();
		float voltage_temp = conversion_factor * static_cast<float>(raw_temp);
		float temperature = 27.0f - (voltage_temp - 0.706f)/0.001721f;
		printf("Pico2 temperature = %.1f C\n", temperature);
#endif
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
		printf("  NMI pin: %s\n", NMI_IS_ASSERTED ? "OUTPUT LOW" : "hi-z");
		printf(" FIRQ pin: %s\n", FIRQ_IS_ASSERTED ? "OUTPUT LOW" : "hi-z");
		printf("  IRQ pin: %s\n", IRQ_IS_ASSERTED ? "OUTPUT LOW" : "hi-z");
#ifdef DEBUG
		static uint32_t last_LIC = 0u;
		printf("LIC count: %lu\n", MC6809.get_lic_count());
		if (MC6809.is_started()) {
			uint32_t LIC = MC6809.get_lic_count();
			if (last_LIC != LIC)
				last_LIC = LIC;
			else
				printf("WARNING: LIC count is same as last time (%u), but the run state is %s\n", LIC, run_state_string[MC6809.run_state]);
		}
		printf("RTI count: %lu\n", MC6809.get_rti_count());
#endif
		printf("Register and emulated RAM block:\n");
		registers::dump();
		printf("Tick timer: enabled=%d  irq_pending=%d\n",
		       (int)system_tick.is_enabled(), (int)system_tick.is_irq_pending());
		printf("Core1 loop: %lu  q_rise: %lu  irq_assert: %lu\n",
		       core1_loop_count, core1_qrise_count, core1_irq_assert_count);
		printf("Core1 write_ring: %lu  read_irq: %lu\n",
		       core1_write_ring_count, core1_read_irq_count);
		printf("Core1 phase: %u  last_write: %02X  last_read: %02X  isr: %lu\n",
		       core1_loop_phase, core1_last_write_loc, core1_last_read_loc,
		       core1_isr_count);
		{
			// Scan stack watermark from bottom to find high-water mark
			volatile uint32_t *bottom = &__StackOneBottom;
			volatile uint32_t *top = &__StackOneTop;
			volatile uint32_t *p = bottom;
			while (p < top && *p == 0xDEADBEEFu)
				p++;
			uint32_t stack_total = (top - bottom) * sizeof(uint32_t);
			uint32_t stack_free = (p - bottom) * sizeof(uint32_t);
			printf("Core1 stack: %lu total, %lu free (min), %lu used (max)\n",
			       stack_total, stack_free, stack_total - stack_free);
		}
		printf("Console ACIA:\n");
		printf("Interrupt status: %02X\n", fast_serial.interrupt_status());
		printf("TxQueue: %u\n", fast_serial.receive_level());
		printf("RxQueue: %u\n", fast_serial.transmit_level());
#ifdef DEBUG
		printf("Register access counts\n");
		printf("Control: %5u\tTx: %5u\n", ACIA_CONTROL_COUNT, ACIA_TX_DATA_COUNT);
		printf("Status:  %5u\tRx: %5u\n", ACIA_STATUS_COUNT, ACIA_RX_DATA_COUNT);
#endif
		printf("Aux ACIA:\n");
		printf("Interrupt status: %02X\n", aux_serial.interrupt_status());
		printf("TxQueue: %u\n", aux_serial.receive_level());
		printf("RxQueue: %u\n", aux_serial.transmit_level());
#ifdef DEBUG
		printf("Register access counts\n");
		printf("Control: %5u\tTx: %5u\n", AUX_ACIA_CONTROL_COUNT, AUX_ACIA_TX_DATA_COUNT);
		printf("Status:  %5u\tRx: %5u\n", AUX_ACIA_STATUS_COUNT, AUX_ACIA_RX_DATA_COUNT);
#endif
		break;
	}

	case CMD_RESET:
		peripheral_clear(false);
		break;

	case CMD_EXAMINE_DAT: {
		const uint16_t start = 0xFD00u;
		const uint16_t end   = 0xFFC0u;
		if (!MC6809.assert_stopped())
			break;
		MC6809.preserve_state();
		const uint16_t display_start = start & 0xFFF0u;
		const uint16_t display_end   = ((end - 1u) | 0x000Fu) + 1u;
		absolute_time_t next_usb = get_absolute_time();
		for (uint16_t loc = display_start; loc < display_end; loc += 16u) {
			if (!copy(INWARDS, 16u, loc, buffer))
				break;
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
					auto ch = static_cast<unsigned char>(buffer[i]) & 0x7Fu;
					printf("%c", isprint(ch) ? ch : '.');
				} else
					printf(" ");
				if (i == 7u)
					printf("|");
			}
			printf("|\n");
			if (absolute_time_diff_us(get_absolute_time(), next_usb) <= 0) {
				usb_poll();
				next_usb = make_timeout_time_ms(1u);
			}
		}
		MC6809.restore_state();
		break;
	}

	case CMD_EXAMINE: {
		const uint16_t start = cmd.examine.start;
		const uint16_t end   = cmd.examine.end;
		if (!MC6809.assert_stopped())
			break;
		MC6809.preserve_state();
		const uint16_t display_start = start & 0xFFF0u;
		const uint16_t display_end   = ((end - 1u) | 0x000Fu) + 1u;
		absolute_time_t next_usb = get_absolute_time();
		for (uint16_t loc = display_start; loc < display_end; loc += 16u) {
			if (!copy(INWARDS, 16u, loc, buffer))
				break;
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
					auto ch = static_cast<unsigned char>(buffer[i]) & 0x7Fu;
					printf("%c", isprint(ch) ? ch : '.');
				} else
					printf(" ");
				if (i == 7u)
					printf("|");
			}
			printf("|\n");
			if (absolute_time_diff_us(get_absolute_time(), next_usb) <= 0) {
				usb_poll();
				next_usb = make_timeout_time_ms(1u);
			}
		}
		MC6809.restore_state();
		break;
	}

	case CMD_MODIFY: {
		const uint16_t start = cmd.modify.start;
		const uint8_t  count = cmd.modify.count;
		memcpy(buffer, cmd.modify.data, count);
		if (start < REGISTER_BASE) {
			if (!MC6809.assert_stopped())
				break;
			if (static_cast<uint32_t>(start) + count > REGISTER_BASE) {
				printf("Overflow into reg region by %u bytes - truncating overflow\n",
				       start + count - REGISTER_BASE);
				(void)copy(OUTWARDS, REGISTER_BASE - start, start, buffer);
			} else
				(void)copy(OUTWARDS, count, start, buffer);
		} else {
			printf("Modifying system registers, so provoking callbacks\n");
			for (uint16_t i = 0u; i < count; i++) {
				const uint8_t loc = (start + i) & registers::register_mask;
				registers::write(loc) = buffer[i];
				// Inject into the write ring so the for(;;) loop dispatches callbacks.
				const uint8_t h = write_ring_head;
				const uint8_t next_h = (h + 1u) & WRITE_RING_MASK;
				if (next_h == write_ring_tail)
					write_ring_tail = (write_ring_tail + 1u) & WRITE_RING_MASK;
				write_ring[h].location = loc;
				write_ring[h].value = buffer[i];
				write_ring_head = next_h;
				printf("%04X: %02X %02X\n",
				       static_cast<uint16_t>(start + i),
				       static_cast<uint8_t>(registers::write(loc)),
				       static_cast<uint8_t>(reg[start + i]));
			}
		}
		break;
	}

	case CMD_MODIFY_READ: {
		const uint16_t start = cmd.modify.start;
		const uint8_t  count = cmd.modify.count;
		memcpy(buffer, cmd.modify.data, count);
		if (start < REGISTER_BASE) {
			printf("mo! only applies to system registers ($%04X+)\n", REGISTER_BASE);
		} else {
			printf("Modifying read registers directly (no callbacks)\n");
			for (uint16_t i = 0u; i < count; i++) {
				reg[start + i] = buffer[i];
				printf("%04X: %02X\n",
				       static_cast<uint16_t>(start + i),
				       static_cast<uint8_t>(reg[start + i]));
			}
		}
		break;
	}

	case CMD_SNIPPET: {
		const uint8_t count = cmd.snippet.count;
		printf("%04X:", 0xFFE0u);
		for (uint16_t i = 0; i < count; i++) {
			reg[REGISTER_SNIPPET_OFFSET + i] = cmd.snippet.data[i];
			printf(" %02X", static_cast<uint8_t>(reg[REGISTER_SNIPPET_OFFSET + i]));
		}
		printf("\n");
		reg[REGISTER_VECTOR_RESET_OFFSET] = REGISTER_SNIPPET;
		MC6809.start();
		break;
	}

	case CMD_LOOP:
		snippet_copy(LOOP_RW);
		reg[REGISTER_SNIPPET_OFFSET + 0u] = cmd.loop.opcode;
		reg[REGISTER_SNIPPET_OFFSET + 1u] = cmd.loop.address;
		MC6809.start();
		break;

	case CMD_VECTOR:
		printf("%04X:", 0xFFF0u);
		for (uint16_t i = 0; i < 8u; i++) {
			reg[REGISTER_VECTORS_OFFSET + 2u*i] = cmd.vector.vec[i];
			printf(" %04X", static_cast<uint16_t>(reg[REGISTER_VECTORS_OFFSET + 2u*i]));
		}
		printf("\n");
		break;

	case CMD_FILL_ALL:
	case CMD_FILL_ALL_VALUE:
	case CMD_FILL:
	case CMD_FILL_VALUE:
		if (!MC6809.assert_stopped())
			break;
		snippet_copy(BLOCK_FILL);
		reg[REGISTER_SNIPPET_OFFSET + 1u] = cmd.fill.value;
		reg[REGISTER_SNIPPET_OFFSET + 3u] = cmd.fill.start;
		reg[REGISTER_SNIPPET_OFFSET + 8u] = cmd.fill.end;
		if (!MC6809.start_with_timeout(10000u, true))
			printf("Fill failed — CPU did not respond\n");
		break;

	case CMD_TIME_QUERY:
		print_time();
		break;

	case CMD_TIME_SET: {
		tm tm = {
			.tm_sec   = 0,
			.tm_min   = 0,
			.tm_hour  = 0,
			.tm_mday  = 1,
			.tm_mon   = 0,
			.tm_year  = 2000 - 1900,
			.tm_isdst = 0,
		};
		tm.tm_year = cmd.time.year - 1900;
		if (cmd.time.count >= 2) tm.tm_mon  = cmd.time.month; /* already 0-based from grammar */
		if (cmd.time.count >= 3) tm.tm_mday = cmd.time.day;
		if (cmd.time.count >= 4) tm.tm_hour = cmd.time.hour;
		if (cmd.time.count >= 5) tm.tm_min  = cmd.time.minute;
		if (cmd.time.count >= 6) tm.tm_sec  = cmd.time.second;
		aon_timer_set_time_calendar(&tm);
		print_time();
		break;
	}

	case CMD_GO:
		fast_serial.reset();
		aux_serial.reset();
		system_tick.reset();
		MC6809.start();
		break;

	case CMD_GO_ADDR:
		fast_serial.reset();
		aux_serial.reset();
		system_tick.reset();
		reg[REGISTER_VECTOR_RESET_OFFSET] = cmd.go.address;
		MC6809.start();
		break;

	case CMD_TASK_QUERY:
		printf("Task = %u\n", MC6809.task);
		break;

	case CMD_TASK_SET: {
		uint8_t task = MC6809.task_change(cmd.task.tasknum);
		task_pins_update(task);
		printf("Task = %u\n", task);
		break;
	}

	case CMD_IRQ: {
		uint32_t timeout = 0u;
		const uint8_t *final_stack = nullptr;
		uint16_t final_stack_length = 0u;
		const char *test_name = nullptr;
		const uint8_t hexnum = cmd.irq.testnum;

		if (!MC6809.assert_stopped())
			break;
		for (int i = 0; i < 16; i++) {
			registers::write(REGISTER_BUFFER_OFFSET + i) = static_cast<uint8_t>(0x00);
			reg[REGISTER_BUFFER_OFFSET + i] = static_cast<uint8_t>(0x00);
		}
		for (int i = 0; i < 16; i += 2)
			reg[REGISTER_VECTORS_OFFSET + i] = static_cast<uint16_t>(0x0000);
		snippet_copy(ZERO);
		if (!MC6809.start_with_timeout(10000u, true)) {
			printf("Zero failed — CPU did not respond\n");
			break;
		}

		if (hexnum == 0) {
			printf("Interrupt strobe test. Hit any key to continue.\n");
			for (;;) {
				const int ch = getchar_timeout_us(0);
				if (ch != PICO_ERROR_TIMEOUT)
					break;
				console_set_interrupt(INTERRUPT_NMI);
				sleep_ms(5u);
				console_set_interrupt(INTERRUPT_NONE);
				sleep_ms(5u);
				console_set_interrupt(INTERRUPT_FIRQ);
				sleep_ms(5u);
				console_set_interrupt(INTERRUPT_NONE);
				sleep_ms(5u);
				console_set_interrupt(INTERRUPT_IRQ);
				sleep_ms(5u);
				console_set_interrupt(INTERRUPT_NONE);
				sleep_ms(5u);
			}
		} else if (hexnum == 1) {
			static constexpr const char *testname = "SWI";
			static constexpr uint16_t stack_result_length = 12u;
			static constexpr uint8_t stack_result[stack_result_length] = {
				0xD0, 0x55, 0xAA, 0x00, 0x66, 0x77, 0x00, 0x00, 0x33, 0xCC, 0xFF, 0xEE
			};
			printf("SWI Test\n");
			snippet_copy(TEST_SWI);
			reg[REGISTER_VECTOR_SWI_OFFSET] = static_cast<uint16_t>(REGISTER_SNIPPET + 14);
			MC6809.start();
			while (!MC6809.is_synced()) { sleep_us(1); if (++timeout > 1000000u) break; }
			MC6809.stop();
			test_name = testname; final_stack_length = stack_result_length; final_stack = stack_result;
		} else if (hexnum == 2) {
			static constexpr const char *testname = "NMI";
			static constexpr uint16_t stack_result_length = 12u;
			static constexpr uint8_t stack_result[stack_result_length] = {
				0xD8, 0x00, 0x00, 0x00, 0xAA, 0xBB, 0x00, 0x00, 0xCC, 0xDD, 0xFF, 0xED
			};
			printf("NMI Test\n");
			snippet_copy(TEST_NMI);
			reg[REGISTER_VECTOR_NMI_OFFSET] = static_cast<uint16_t>(REGISTER_SNIPPET + REGISTER_SNIPPET_LEN - 1);
			MC6809.start();
			sleep_us(100u);
			console_set_interrupt(INTERRUPT_NMI);
			while (!MC6809.is_synced()) { sleep_us(1); if (++timeout > 1000000u) break; }
			MC6809.stop();
			console_set_interrupt(INTERRUPT_NONE);
			test_name = testname; final_stack_length = stack_result_length; final_stack = stack_result;
		} else if (hexnum == 3) {
			static constexpr const char *testname = "IRQ";
			static constexpr uint16_t stack_result_length = 12u;
			static constexpr uint8_t stack_result[stack_result_length] = {
				0xC8, 0x00, 0x00, 0x00, 0xAB, 0xAB, 0x00, 0x00, 0xCD, 0xCD, 0xFF, 0xED
			};
			printf("IRQ Test\n");
			snippet_copy(TEST_IRQ);
			reg[REGISTER_VECTOR_IRQ_OFFSET] = static_cast<uint16_t>(REGISTER_SNIPPET + REGISTER_SNIPPET_LEN - 1);
			MC6809.start();
			sleep_us(100u);
			console_set_interrupt(INTERRUPT_IRQ);
			while (!MC6809.is_synced()) { sleep_us(1); if (++timeout > 1000000u) break; }
			MC6809.stop();
			console_set_interrupt(INTERRUPT_NONE);
			test_name = testname; final_stack_length = stack_result_length; final_stack = stack_result;
		} else if (hexnum == 4) {
			static constexpr const char *testname = "FIRQ";
			static constexpr uint16_t stack_result_length = 3u;
			static constexpr uint8_t stack_result[stack_result_length] = {
				0x1F, 0xFF, 0xED
			};
			printf("FIRQ Test\n");
			snippet_copy(TEST_FIRQ);
			reg[REGISTER_VECTOR_FIRQ_OFFSET] = static_cast<uint16_t>(REGISTER_SNIPPET + REGISTER_SNIPPET_LEN - 1);
			MC6809.start();
			sleep_us(100u);
			console_set_interrupt(INTERRUPT_FIRQ);
			while (!MC6809.is_synced()) { sleep_us(1); if (++timeout > 1000000u) break; }
			MC6809.stop();
			console_set_interrupt(INTERRUPT_NONE);
			test_name = testname; final_stack_length = stack_result_length; final_stack = stack_result;
		} else {
			static constexpr const char *testname = "RTI";
			static constexpr uint16_t stack_result_length = 12u;
			static constexpr uint8_t stack_result[stack_result_length] = {
				0x80, 0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89, 0x90, 0xFF, 0xE7
			};
			printf("RTI Test\n");
			reg[REGISTER_VECTOR_SWI_OFFSET] = static_cast<uint16_t>(REGISTER_SNIPPET + 9);
			for (uint16_t i = 0; i < stack_result_length; i++)
				reg[REGISTER_BUFFER_OFFSET + (16u - stack_result_length) + i] = stack_result[i];
			reg[REGISTER_BUFFER_OFFSET + REGISTER_BUFFER_LEN - 1] = static_cast<uint8_t>(0xE6);
			snippet_copy(TEST_RTI);
			MC6809.start();
			while (!MC6809.is_synced()) { sleep_us(1); if (++timeout > 1000000u) break; }
			MC6809.stop();
			test_name = testname; final_stack_length = stack_result_length; final_stack = stack_result;
		}

		if (final_stack_length) {
			bool success = true;
			for (uint16_t i = 0; i < final_stack_length; i++)
				if (reg[REGISTER_BUFFER_OFFSET + (16u - final_stack_length) + i] != final_stack[i]) {
					printf("mismatch %04X %02X %02X\n",
					       REGISTER_BUFFER_OFFSET + (16u - final_stack_length) + i,
					       static_cast<uint8_t>(reg.write(REGISTER_BUFFER_OFFSET + (16u - final_stack_length) + i)),
					       final_stack[i]);
					success = false;
				}
			if (timeout > 1000000u) {
				printf("Timeout reached\n");
				success = false;
			}
			printf("%s: Timeout counter: %u  Test result: %s\n",
			       test_name, timeout, success ? "PASS" : "FAIL");
		}
		break;
	}

	case CMD_TRACE:
	case CMD_TRACE_LEN:
#ifdef DEBUG_NOT_NOW
	{
		uint32_t len = (cmd.tag == CMD_TRACE_LEN) ? MIN(cmd.trace.length, trace_pos) : trace_pos;
		if (len > 0)
			for (uint32_t i = 0; i < len; i++) {
				BLA bla{}; bla.byte = static_cast<uint8_t>(trace[i][2] >> 8);
				printf("%04lX %s %02lX %s %s %s %12s %12s %04lX\n",
				       trace[i][0] + 0xFFC0u,
				       (trace[i][2] & TRACE_WRITE) ? "W" : " ",
				       trace[i][1],
				       bla.bit.AVMA ? "AVMA" : "    ",
				       bla.bit.LIC  ? "LIC " : "    ",
				       bla.bit.BUSY ? "BUSY" : "    ",
				       ba_bs_string[(trace[i][2] >> 4) & 0x0F],
				       run_state_string[trace[i][2] & 0x0F],
				       trace[i][3]);
			}
		break;
	}
#else
		printf("To enable trace, build with DEBUG defined\n\n");
		break;
#endif

	case CMD_RUN_LIST:
		for (uint16_t i = DAT_INIT; i < SNIPPET_LEN; i++)
			printf("%X: %s\n", i, snippet_string[i]);
		for (uint16_t i = 0; i < CHUNK_LEN; i++)
			printf("%X: %s\n", i + SNIPPET_LEN, chunk_string[i]);
		break;

	case CMD_RUN:
	case CMD_RUN_NOWAIT: {
		const bool stop = (cmd.tag == CMD_RUN);
		uint32_t idx = cmd.run.index;
		snippet snippet_num;
		chunk chunk_num;
		if (idx >= SNIPPET_LEN) {
			snippet_num = SNIPPET_LEN; /* sentinel: it's a chunk */
			idx -= SNIPPET_LEN;
			if (idx >= CHUNK_LEN) {
				printf("Invalid snippet/chunk index\n");
				break;
			}
			chunk_num = static_cast<chunk>(idx);
		} else {
			snippet_num = static_cast<snippet>(idx);
		}
		if (!MC6809.assert_stopped())
			break;
		if (snippet_num < SNIPPET_LEN) {
			printf("Running snippet %u = %s\n", snippet_num, snippet_string[snippet_num]);
			snippet_copy(snippet_num);
		} else {
			printf("Running chunk %u = %s\n", chunk_num, chunk_string[chunk_num]);
			if (!copy(OUTWARDS, chunk_len[chunk_num], 0x0130u, chunk_code[chunk_num]))
				break;
			reg[REGISTER_VECTOR_RESET_OFFSET] = CHUNK_ADDRESS;
		}
		if (stop) {
			if (!MC6809.start_with_timeout(1000u))
				printf("Run failed — CPU did not respond\n");
		} else
			MC6809.start();
		break;
	}

	case CMD_STOP:
		MC6809.stop();
		break;

	case CMD_STOP_HALT:
		MC6809.halt();
		break;

	case CMD_STOP_RESTART:
		MC6809.release();
		break;

	case CMD_STOP_STOP:
		MC6809.stop();
		break;

	case CMD_BREAK: {
		if (break_active != INTERRUPT_NONE) {
			printf("Break already active (use 'break return' first)\n");
			break;
		}
		// NMI break: load the BREAK snippet into the snippet block,
		// point the NMI vector there, and trigger NMI.  The 6809 pushes its
		// full register set onto the stack and the snippet catches the 6809.
		uint32_t timeout = 0u;
		snippet_copy(BREAK);
		for (uint i = 0; i < 16; i++) {
			reg[REGISTER_BUFFER_OFFSET + i] = static_cast<uint8_t>(0x00);
			registers::write(REGISTER_BUFFER_OFFSET + i) = static_cast<uint8_t>(0x00);
		}
		reg[REGISTER_VECTOR_NMI_OFFSET] = static_cast<uint16_t>(REGISTER_SNIPPET);
		printf("NMI break -> %04X\n", REGISTER_SNIPPET);
		console_set_interrupt(INTERRUPT_NMI);
		// Wait for the full 6809 register set to be stacked.
		// TODO: MarkM: this is exactly what I wanted BS_IRQ to help with.
		while ((reg[REGISTER_BUFFER + 10] | reg[REGISTER_BUFFER + 11]) == static_cast<uint8_t>(0x00)) { sleep_us(1); if (++timeout > 1000000u) break; }
		console_set_interrupt(INTERRUPT_NONE);
		if (timeout > 1000000u)
			printf("Timeout. No NMI?\n");
		else {
			break_active = INTERRUPT_NMI;
			uint8_t cc = reg[REGISTER_BUFFER];
			printf("CC: ");
			for (uint i = 0; i < 8; i++) {
				printf("%c", cc & 0x80 ? "EFHINZVC"[i] : '-');
				cc <<= 1;
			}
			printf("\nA: %02X  B: %02X  DP: %02X\nX: %02X%02X  Y: %02X%02X\nU: %02X%02X  PC: %02X%02X\n",
				static_cast<uint8_t>(reg[REGISTER_BUFFER + 1]),
				static_cast<uint8_t>(reg[REGISTER_BUFFER + 2]),
				static_cast<uint8_t>(reg[REGISTER_BUFFER + 3]),
				static_cast<uint8_t>(reg[REGISTER_BUFFER + 4]),
				static_cast<uint8_t>(reg[REGISTER_BUFFER + 5]),
				static_cast<uint8_t>(reg[REGISTER_BUFFER + 6]),
				static_cast<uint8_t>(reg[REGISTER_BUFFER + 7]),
				static_cast<uint8_t>(reg[REGISTER_BUFFER + 8]),
				static_cast<uint8_t>(reg[REGISTER_BUFFER + 9]),
				static_cast<uint8_t>(reg[REGISTER_BUFFER + 10]),
				static_cast<uint8_t>(reg[REGISTER_BUFFER + 11]));
		}
		break;
	}

	case CMD_BREAK_ADDR:
		// TODO: address-based breakpoint at $%04X
		printf("Address breakpoint at $%04X (not yet implemented)\n", cmd.brk.address);
		break;

	case CMD_BREAK_RETURN:
		if (break_active == INTERRUPT_NONE) {
			printf("No active break to return from\n");
			break;
		}
		printf("Returning from %s break\n",
			break_active == INTERRUPT_NMI ? "NMI" :
			break_active == INTERRUPT_SWI ? "SWI" : "???");
		reg[REGISTER_SNIPPET + 14] = static_cast<uint8_t>(0x00);
		break_active = INTERRUPT_NONE;
		break;

	case CMD_FREQ:
		MC6809.set_e_frequency(cmd.freq.mhz, pio_clock);
		printf("Set E to %.1f MHz\n", MC6809.get_e_frequency());
		break;

	case CMD_USB: {
		/* Send each space-separated word as a separate USB CDC write.
		 * Optional port prefix: "usb 2 ..." sends to CDC interface 1. */
		char args[sizeof(cmd.usb.args)];
		memcpy(args, cmd.usb.args, sizeof(args));
		char *ptr = args;
		uint8_t itf = 0;
		if (ptr[0] == '2' && (ptr[1] == ' ' || ptr[1] == '\t')) {
			itf = 1;
			ptr += 2;
		}
		char *word;
		while ((word = strsep(&ptr, " \t")) != nullptr) {
			if (*word == '\0')
				continue;
			snprintf(reinterpret_cast<char *>(buffer), 64u, "%s\r\n", word);
			usb_cdc_write_n(itf, buffer, strlen(reinterpret_cast<char *>(buffer)));
		}
		usb_cdc_flush_n(itf);
		break;
	}

	case CMD_SRECORD: {
		static bool srecord_ignore = false;
		uint8_t count;
		uint16_t address;
		uint8_t checksum;
		const char *line = cmd.srecord.line;
		if (!process_srecord(line, &count, &address, buffer, &checksum)) {
			printf("Invalid S record\n");
			break;
		}
		// S0 and S9 reset the ignore flag (stream boundaries)
		if (line[1] == '0' || line[1] == '9')
			srecord_ignore = false;
		if (srecord_ignore) {
			printf("%s IGNORED\n", line);
			break;
		}
		if (line[1] == '0') {
			printf("S0 data = \"");
			for (uint16_t i = 0; i < count - 3u; i++) {
				const uint8_t c = buffer[i];
				printf("%c", isprint(c) ? c : ' ');
			}
			printf("\"\n");
		} else if (line[1] == '1') {
			if (verbose) {
				printf("S1 address = %04X, length = %u\n", address, count);
				printf("%04X:", address);
				for (uint16_t i = 0; i < count - 3u; i++)
					printf(" %02X", buffer[i]);
				printf("\n");
			}
			if (address + count < 0xFE00u) {
				if (!MC6809.is_stopped()) {
					printf("%s IGNORED (CPU not stopped)\n", line);
					srecord_ignore = true;
				} else if (!copy(OUTWARDS, count - 3u, address, buffer)) {
					printf("%s IGNORED (copy failed)\n", line);
					srecord_ignore = true;
				}
			} else if (address >= 0xFFF0u)
				for (uint16_t i = 0; i < count - 3u; i++)
					reg[address + i] = buffer[i];
		} else if (line[1] == '5') {
			if (verbose)
				printf("S5 record count = %u\n", address);
		} else if (line[1] == '9') {
			if (verbose)
				printf("S9 start = %04X\n", address);
			if (!srecord_ignore)
				reg[REGISTER_VECTOR_RESET_OFFSET] = address;
			srecord_ignore = false;	// end of stream — reset for next load
		}
		break;
	}

	case CMD_NITROS9: {
		if (!MC6809.assert_stopped())
			break;
		if (verbose)
			printf("Initialising DAT\n");
		snippet_copy(DAT_INIT);
		if (!MC6809.start_with_timeout(4u)) {
			printf("DAT_INIT failed — CPU did not respond\n");
			break;
		}
		if (verbose)
			printf("Loading DBGMON v%u.%u.%u to $FC00\n",
			       chunk_code[DBGMON][21], chunk_code[DBGMON][22], chunk_code[DBGMON][23]);
		if (!copy(OUTWARDS, chunk_len[DBGMON], 0xFC00u, chunk_code[DBGMON]))
			break;
		/* Set illegal-opcode vector ($FFF0-$FFF1) to JT.IlOp ($FC12) */
		reg[REGISTER_VECTOR_RESERVED_OFFSET] = static_cast<uint16_t>(0xFC12u);
		if (verbose)
			printf("Loading NITROS9 v%u.%u.%u to $0130\n",
			       chunk_code[NITROS9][3], chunk_code[NITROS9][4], chunk_code[NITROS9][5]);
		if (!copy(OUTWARDS, chunk_len[NITROS9], 0x0130u, chunk_code[NITROS9]))
			break;
		/* Set boot device byte at $0132 (0 = IDE, 1 = DriveWire) */
		uint8_t bdev = cmd.nitros9.boot_device;
		if (!copy(OUTWARDS, 1u, 0x0132u, &bdev))
			break;
		reg[REGISTER_VECTOR_RESET_OFFSET] = CHUNK_ADDRESS;
		printf("Booting NitrOS-9 (%s)\n", bdev ? "DriveWire" : "IDE");
		MC6809.start();
		break;
	}

	default:
		break;
	}
}

#define COMMAND_BUFFER_LEN 256u

static constexpr char prompt[] =  ">>> ";

static void
poll_console()
{
	static char buf[COMMAND_BUFFER_LEN];
	static int idx = 0;
	static bool prompted = false;

	while (!core1_initialised) {
		sleep_ms(1u);
		usb_poll();
	}

	print_time();

	// Software watchdog: detect Core 1 for(;;) loop stalls.
	// Sampled every WATCHDOG_INTERVAL_MS from Core 0.  Requires
	// WATCHDOG_THRESHOLD consecutive misses before alerting, so transient
	// ISR starvation during heavy bus activity (e.g. NitrOS9 boot) doesn't
	// trigger false alarms.
	uint32_t watchdog_last_loop_count = core1_loop_count;
	uint32_t watchdog_last_isr_count = core1_isr_count;
	static constexpr uint32_t WATCHDOG_INTERVAL_MS = 10u;
	static constexpr uint32_t WATCHDOG_THRESHOLD = 10u;	// 10 × 10ms = 100ms
	uint32_t watchdog_miss_count = 0u;
	absolute_time_t watchdog_next = make_timeout_time_ms(WATCHDOG_INTERVAL_MS);
	bool watchdog_stalled = false;

	for (;;) {
		if (absolute_time_diff_us(get_absolute_time(), watchdog_next) <= 0) {
			const uint32_t cur_loop = core1_loop_count;
			const uint32_t cur_isr = core1_isr_count;
			if (cur_loop == watchdog_last_loop_count && core1_initialised) {
				watchdog_miss_count++;
				if (!watchdog_stalled && watchdog_miss_count >= WATCHDOG_THRESHOLD) {
					const bool isr_alive = (cur_isr != watchdog_last_isr_count);
					printf("\n*** Core 1 for(;;) loop stalled (%lums)!\n",
					       watchdog_miss_count * WATCHDOG_INTERVAL_MS);
					if (!isr_alive)
						printf("*** ISRs also stopped — probable hard fault or deadlock\n");
					else
						printf("*** ISRs still running — thread mode starved or hung\n");
					printf("  phase=%u  loop=%lu  isr=%lu\n",
					       core1_loop_phase, cur_loop, cur_isr);
					printf("  last_write=%02X  last_read=%02X\n",
					       core1_last_write_loc, core1_last_read_loc);
					printf("  pending_interrupts=%02X  write_ring: head=%u tail=%u\n",
					       pending_interrupts, write_ring_head, write_ring_tail);
					printf("  ACIA: int_status=%02X  Aux: int_status=%02X\n",
					       fast_serial.interrupt_status(), aux_serial.interrupt_status());
					printf("  Tick: enabled=%d  irq_pending=%d\n",
					       (int)system_tick.is_enabled(), (int)system_tick.is_irq_pending());
					// Stack watermark check
					{
						volatile uint32_t *bottom = &__StackOneBottom;
						volatile uint32_t *top = &__StackOneTop;
						volatile uint32_t *p = bottom;
						while (p < top && *p == 0xDEADBEEFu)
							p++;
						uint32_t used = (top - p) * sizeof(uint32_t);
						uint32_t total = (top - bottom) * sizeof(uint32_t);
						printf("  stack: %lu/%lu bytes used", used, total);
						if (p == bottom)
							printf(" *** STACK OVERFLOW ***");
						printf("\n");
					}
					watchdog_stalled = true;
				}
			} else {
				if (watchdog_stalled) {
					printf("*** Core 1 for(;;) loop resumed after %lums (loop=%lu)\n",
					       watchdog_miss_count * WATCHDOG_INTERVAL_MS,
					       cur_loop);
				}
				watchdog_stalled = false;
				watchdog_miss_count = 0u;
			}
			watchdog_last_loop_count = cur_loop;
			watchdog_last_isr_count = cur_isr;
			watchdog_next = make_timeout_time_ms(WATCHDOG_INTERVAL_MS);
		}

		if (!prompted) {
			printf(prompt);
			prompted = true;
		}
		// Poll USB before reading a character so that all chars delivered
		// by this tud_task() call are visible to getchar_timeout_us()
		// before uart_task() can consume them.  Since tud_task() is the
		// only path that moves data from the USB hardware FIFO into the
		// TinyUSB CDC ring buffer, this guarantees that any char available
		// in the ring buffer after usb_poll() is read by getchar first
		// (advancing idx), causing uart_task() to be suppressed.
		usb_poll();
		const int ch = getchar_timeout_us(0);
		if (ch != PICO_ERROR_TIMEOUT) {
			if (ch == '\r' || ch == '\n') {
				putchar('\n');
				buf[idx] = '\0';
				process_command(buf);
				// Resample watchdog baseline — process_command() may have
				// run a snippet/chunk that legitimately starved the for(;;)
				// loop for its duration.
				watchdog_last_loop_count = core1_loop_count;
				watchdog_last_isr_count = core1_isr_count;
				watchdog_next = make_timeout_time_ms(WATCHDOG_INTERVAL_MS);
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
		MC6809.uart_task(idx > 0);
	}
}

void
init_pins_range(const uint8_t base, const uint8_t count)
{
	for (uint8_t i = 0; i < count; ++i)
		gpio_init(base + i);
}

static void
__no_inline_not_in_flash_func(main_core1)()
{
	printf("Pico2 MC6809E core1 process starting\n");

	init_pins_range(GPIO_TRACE_CORE1, 4); // trace pins - isr set
	gpio_set_dir_out_masked64(0b1111LLu << GPIO_TRACE_CORE1);

	fast_serial.reset();
	aux_serial.reset();
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

	pio_sm_set_enabled(task_pio, task_pio_sm, true);
	printf("Pico2 MC6809E task pio%d/sm%d started\n", PIO_NUM(task_pio), task_pio_sm);

	sysreq_initialise();

	// Start the 50 Hz tick timer (hardware alarm, runs independently of E clock).
	system_tick.start();
	printf("Pico2 MC6809E 50 Hz tick timer started\n");

	printf("Pico2 MC6809E core1 process started\n");

	// Paint the unused portion of Core 1's stack with a canary pattern.
	// SP points to the current top; everything between StackOneBottom and
	// the current SP is unused and can be painted safely.
	{
		volatile uint32_t *bottom = &__StackOneBottom;
		volatile uint32_t *sp;
		__asm volatile ("mov %0, sp" : "=r" (sp));
		// Leave 64 bytes headroom below current SP
		for (volatile uint32_t *p = bottom; p < sp - 16; p++)
			*p = 0xDEADBEEFu;
		uint32_t stack_free = (sp - bottom) * sizeof(uint32_t);
		printf("Pico2 MC6809E core1 stack: %lu bytes total, %lu bytes free at start\n",
		       (&__StackOneTop - &__StackOneBottom) * sizeof(uint32_t),
		       stack_free);
	}

	core1_initialised = true;

	for (;;) {
		core1_loop_phase = 1;
		core1_loop_count++;
		// Interrupts need to be done before the falling edge of Q
		if (q_rising_edge) {
			q_rising_edge = false;
			core1_qrise_count++;
			core1_loop_phase = 2;
			uint32_t interrupt_refcount[NUM_INTERRUPTS] = {};
			interrupt eirq;
			// FOR_EACH interrupt_source:
			core1_loop_phase = 20;
			eirq = console_has_interrupt();
			interrupt_refcount[eirq]++;
			core1_loop_phase = 21;
			eirq = system_tick.has_interrupt();
			interrupt_refcount[eirq]++;
			core1_loop_phase = 22;
			eirq = fast_serial.has_interrupt();
			interrupt_refcount[eirq]++;
			core1_loop_phase = 23;
			eirq = aux_serial.has_interrupt();
			interrupt_refcount[eirq]++;
			core1_loop_phase = 3;
			if (interrupt_refcount[INTERRUPT_IRQ] > 0u)
				core1_irq_assert_count++;
			core1_loop_phase = 30;
			// Update cached state so the Q-rising ISR can assert/deassert
			// interrupt pins.  Single volatile write; the ISR handles all
			// GPIO assertion and deassertion — no apply_interrupts() here.
			pending_interrupts =
				(interrupt_refcount[INTERRUPT_NMI]  > 0u ? PEND_NMI  : 0u) |
				(interrupt_refcount[INTERRUPT_FIRQ] > 0u ? PEND_FIRQ : 0u) |
				(interrupt_refcount[INTERRUPT_IRQ]  > 0u ? PEND_IRQ  : 0u);
			core1_loop_phase = 31;
			// END FOR_EACH
			//gpio_put(GPIO_TRACE_CORE1, false);
		}

		core1_loop_phase = 4;
		while (write_ring_tail != write_ring_head) {
			core1_write_ring_count++;
			//gpio_put(GPIO_TRACE_CORE1, true);
			const uint8_t t = write_ring_tail;
			const uint8_t written_location = write_ring[t].location;
			const uint8_t written_byte = write_ring[t].value;
			core1_last_write_loc = written_location;
			core1_loop_phase = 5;
			write_ring_tail = (t + 1u) & WRITE_RING_MASK;
			switch (written_location) {
			default:
				break;

			case SYSTEM_TASK: // Task reg for DAT/MMU
				// TODO: MarkM - Don't allow tasks other than zero to do this!
				// Handled in DMA write ISR via PIO task_output
				break;

			case SYSTEM_REQUEST:
				if (reg[SYSTEM_REQUEST_RESPONSE] == RESPONSE_BUSY) {
					// ignore request
					registers::write(SYSTEM_REQUEST) = reg[SYSTEM_REQUEST];
					break;
				}
				switch (written_byte) {
				default:
					registers::write(SYSTEM_REQUEST) = reg[SYSTEM_REQUEST] = REQUEST_NULL;
					reg[SYSTEM_REQUEST_RESPONSE] = RESPONSE_UNKNOWN;
					break;
				case REQUEST_TIME:
				case REQUEST_SET_TIME:
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

			case CONSOLE_CONTROL: { // ACIA command
#ifdef DEBUG
				mc6850_control cr = {written_byte};
				if (cr.divide_select == 0b11u) {
					ACIA_CONTROL_COUNT = 0u;
					ACIA_TX_DATA_COUNT = 0u;
					ACIA_STATUS_COUNT = 0u;
					ACIA_RX_DATA_COUNT = 0u;
				}
				ACIA_CONTROL_COUNT++;
#endif
				fast_serial.guest_control(written_byte);
				break;
			}
			case CONSOLE_TX_DATA: // ACIA tx data
#ifdef DEBUG
				ACIA_TX_DATA_COUNT++;
#endif
				fast_serial.guest_transmit(written_byte);
				break;

			case AUX_CONTROL: {
#ifdef DEBUG
				mc6850_control cr = {written_byte};
				if (cr.divide_select == 0b11u) {
					AUX_ACIA_CONTROL_COUNT = 0u;
					AUX_ACIA_TX_DATA_COUNT = 0u;
					AUX_ACIA_STATUS_COUNT = 0u;
					AUX_ACIA_RX_DATA_COUNT = 0u;
				}
				AUX_ACIA_CONTROL_COUNT++;
#endif
				aux_serial.guest_control(written_byte);
				break;
			}
			case AUX_TX_DATA:
#ifdef DEBUG
				AUX_ACIA_TX_DATA_COUNT++;
#endif
				aux_serial.guest_transmit(written_byte);
				break;

			// Tick timer writes are handled directly in the DMA write ISR.

			// Everything else is RAM. See the write ISR
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
				break;
			}
			//gpio_put(GPIO_TRACE_CORE1, false);
		} // while (write_ring not empty)

		// Drain the CONSOLE_RX_DATA counter before checking the shared slot.
		// Each ISR-detected RX data read must call guest_receive() exactly once.
		core1_loop_phase = 6;
		while (rx_processed_count != rx_read_count) {
#ifdef DEBUG
			ACIA_RX_DATA_COUNT++;
#endif
			fast_serial.guest_receive();
			rx_processed_count++;
		}
		while (aux_rx_processed_count != aux_rx_read_count) {
#ifdef DEBUG
			AUX_ACIA_RX_DATA_COUNT++;
#endif
			aux_serial.guest_receive();
			aux_rx_processed_count++;
		}

		core1_loop_phase = 7;
		if (read_irq_received) {
			core1_read_irq_count++;
			//gpio_put(GPIO_TRACE_CORE1, true);
			read_irq_received = false;
			core1_last_read_loc = read_location;
			core1_loop_phase = 8;
			switch (read_location) {
			default:
				// The MC6809 got whatever byte was here. By default, do nothing else.
				break;
			case REGISTER_SNIPPET_OFFSET + 0x0F:
				// Last instruction in the snippet block.
				// If it is RTI (0x3B), then we need to be switching things
				// around in the task scheduling.
				if (MC6809.get_lic())
					if (reg[read_location] == static_cast<uint8_t>(0x3B))
						MC6809.apply_rti();
				break;

			case SYSTEM_TASK:
				// Will be set by writing to this address
				break;

			case SYSTEM_REQUEST_RESPONSE:
				// Will be set by the sysreq processor
				break;

			case CONSOLE_STATUS: // ACIA status
#ifdef DEBUG
				ACIA_STATUS_COUNT++;
#endif
				break;

			case AUX_STATUS:
#ifdef DEBUG
				AUX_ACIA_STATUS_COUNT++;
#endif
				break;

			// CONSOLE_RX_DATA / AUX_RX_DATA handled via rx_read_count above.

			// Tick timer status read is handled directly in the DMA read ISR.

			}
			//gpio_put(GPIO_TRACE_CORE1, false);
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

	// DMA for the main MC6809 bus read loop
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
	// DMA for the main MC6809 bus write loop
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

	// SM: Task pin output driver (pio2 with GPIO base 16 for access to GPIO 40-45)
	task_pio = pio2;
	pio_set_gpio_base(task_pio, 16);
	const int ofs_task = pio_add_program(task_pio, &task_output_program);
	if (ofs_task < 0) {
		printf("Pico2 MC6809E insufficient space for task_output program\n");
		panic("Unable to proceed");
	}
	task_pio_sm = pio_claim_unused_sm(task_pio, true);
	task_output_program_init(task_pio, task_pio_sm, ofs_task);
	printf("Pico2 MC6809E task output pio%u sm%u configured\n", PIO_NUM(task_pio), task_pio_sm);
}

#define MEASURE_FREQS
#ifdef MEASURE_FREQS
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

// Stub: disable PIO state machines for power-down
static void
pio_shutdown()
{
	// TODO: disable and unclaim PIO state machines, DMA channels, IRQs
}

static void
set_power()
{
	// NOTE: Start of power-up process
	POWER_ASSERT;

	// Setup !Reset, !Halt, and the interrupt pins.
	MC6809.init();
	printf("Pico2 MC6809E guest state initialised\n");

	sleep_ms(10);
	printf("Pico2 MC6809E external power on.\n");

	usb_init();
	printf("Pico2 MC6809E initialised USB\n");

	clock_pio_init();
	bus_pio_dma_init();
	peripheral_clear(true);
	printf("Pico2 MC6809E emulated RAM and devices initialised\n");

	multicore_launch_core1(main_core1);
	// NOTE: End of power-up process
}

static void
reset_power()
{
	MC6809.stop();
	printf("Pico2 MC6809E guest stopped\n");

	multicore_reset_core1();
	printf("Pico2 MC6809E core1 stopped\n");

	pio_shutdown();
	printf("Pico2 MC6809E emulated RAM and devices disabled\n");

	POWER_DEASSERT;
	printf("Pico2 MC6809E external power off\n");
}

int
main()
{
	vreg_set_voltage(VREG_VOLTAGE_2_00);
	sleep_ms(5);

	gpio_set_function(GPIO_UART_TX, UART_FUNCSEL_NUM(uart0, GPIO_UART_TX));
	gpio_set_function(GPIO_UART_RX, UART_FUNCSEL_NUM(uart0, GPIO_UART_RX));

	stdio_init_all();

	printf("\n\nPico2 MC6809E bus supervisor %s\n", DEVICE_VERSION);
	printf("Pico2 MC6809E built on %s %s\n", __DATE__, __TIME__);
#ifdef DEBUG
	printf("Pico2 MC6809E build with DEBUG defined\n");
#endif

#ifdef MEASURE_FREQS
	measure_freqs();
#endif
	printf("Pico2 MC6809E raising sys_clock to %d MHz ... ", SYS_CLK_DEFAULT/1000000);
	if (!set_sys_clock_hz(SYS_CLK_DEFAULT, false)) {
		printf("unsuccessful! Reverting to %d MHz ... ", SYS_CLK_HZ/1000000);
		set_sys_clock_hz(SYS_CLK_HZ, true);
	}
	stdio_init_all();
	uart_init(CONSOLE, CONSOLE_BAUDRATE);
	printf("successful!\n");
	printf("Pico2 MC6809E console baudrate set to %d\n", CONSOLE_BAUDRATE);

#ifdef NO_ADC_YET
	adc_init();
	adc_gpio_init(GPIO_POWER_ADC);
	adc_set_temp_sensor_enabled(true);
#endif
	// Enable external electronics' power
	gpio_init(GPIO_POWER);
	gpio_set_dir(GPIO_POWER, GPIO_OUT);
	set_power();

	tm tm = build_time_tm();
	aon_timer_stop();
	aon_timer_start_calendar(&tm);

	printf("Pico2 MC6809E ready!\n");

	poll_console();
}
