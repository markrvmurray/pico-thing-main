/**
* Copyright (c) 2025 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

//#include <sys/stat.h>

#include <cctype>
#include <cinttypes>
#include <cstdio>
//#include <cstring>
//#include <cstdlib>
//#include <ctime>
//#include <cstdint>

#include "pico/aon_timer.h"
#include "pico/assert.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/util/queue.h"

#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/regs/sysinfo.h"
#include "hardware/sync.h"
#include "hardware/vreg.h"
#include "bsp/board_api.h"

#include "tusb.h"
#include "usb.h"
#include "registers.h"
#include "pico_thing.h"
#include "mc6809.h"
#include "uart_pump.h"
#include "srec.h"
#include "mc6850.h"
#include "rail_monitor.h"
#include "tick_timer.h"
#include "build_time.h"
#include "command_parser.h"
#include "status.h"

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
//#include <cstdint>

#include "pico_thing.pio.h"

// I'm really pushing my luck with this.
#define SYS_CLK_DEFAULT 320000000

// The Pico's command console, and the only one: stdio is wired to this UART
// alone, while both USB CDC ports belong to the guest's ACIAs.  So this is
// also the only real baud rate in the firmware - the CDC ports ignore the
// host's line coding, and the guest's divide-select bits pick loopback and
// reset rather than a rate.
// Set from CMake with -DPICO_THING_CONSOLE_BAUD=<baud>; see the note there for
// why the default is 115200 and what a faster rate would cost on input.
#define CONSOLE uart0
#ifndef PICO_THING_CONSOLE_BAUD
#define PICO_THING_CONSOLE_BAUD 115200
#endif
#define CONSOLE_BAUDRATE PICO_THING_CONSOLE_BAUD

volatile bool verbose = false;

// Every PIO program in this firmware, and the block it lives on.  The block is
// not a free choice: pio0 and pio1 must sit at GPIO base 0 to see E, Q, the
// address and data buses and !CS, while pio2 sits at base 16 to reach the task
// pins and the DAT fault signals - so a program goes where its pins are
// visible and where there is instruction space, not where it would read best.
//
// Each program's state machine, instruction offset, DMA channels and interrupt
// line live together in one pio_dma so they cannot drift apart; -1 means not
// claimed, or none of that kind.  See the PIO IRQ flag map further down for
// which flag each program raises.
#define BUS_CLOCK_PIO	pio0		// E/Q quadrature clock
#define BUS_READ_PIO	pio0		// guest reads of the register block
#define BUS_WRITE_PIO	pio1		// guest writes to the register block
#define RTI_DETECT_PIO	pio1		// RTI opcode detector
#define TASK_OUTPUT_PIO	pio2		// task pins, GPIO base 16
#define TRACE_LO_PIO	pio0		// bus trace, GPIO 0-31
#define TRACE_HI_PIO	pio2		// bus trace, GPIO 32-47
#define BUS_FAULT_PIO	pio2		// DAT fault sampler, needs GPIO_FAULT
#define BUS_STATUS_PIO	pio2		// BUSY/LIC/AVMA mirror, GPIO 20-22

//                                    pio             sm  off  addr data  irq
static pio_dma pio_clock     = { BUS_CLOCK_PIO,       -1, -1,  -1,  -1, IO_IRQ_BANK0 };
static pio_dma piodma_read   = { BUS_READ_PIO,        -1, -1,   0,   1, DMA_IRQ_0 };
static pio_dma piodma_write  = { BUS_WRITE_PIO,       -1, -1,   4,   5, DMA_IRQ_1 };
static pio_dma rti_detect    = { RTI_DETECT_PIO,      -1, -1,  -1,  -1, PIO1_IRQ_0 };
static pio_dma task_output   = { TASK_OUTPUT_PIO,     -1, -1,  -1,  -1, -1 };
static pio_dma trace_lo      = { TRACE_LO_PIO,        -1, -1,  -1,   2, -1 };
static pio_dma trace_hi      = { TRACE_HI_PIO,        -1, -1,  -1,   3, -1 };
static pio_dma bus_fault     = { BUS_FAULT_PIO,       -1, -1,  -1,  -1, PIO2_IRQ_0 };
// One channel, in the RP2350's ENDLESS transfer-count mode, so nothing has to
// re-arm it and it never stops of its own accord.  Chaining two channels to
// each other looks like the way to get an endless transfer but is not: see
// bus_pio_dma_init().  It goes in the data field; it carries no address.
static pio_dma bus_status    = { BUS_STATUS_PIO,      -1, -1,  -1,   6, -1 };


static inline void
task_pins_update(uint8_t task)
{
	uint32_t pio_word = task | ((task == 0u) ? (1u << NUM_TASK_PINS) : 0u);
	pio_sm_put(task_output.pio, task_output.sm, pio_word);
}

// task_stack is now inside mc6809 class (mc6809::task_stack[])

// Vector watchpoint — when armed, writes to $FFF0-$FFFF are blocked and trigger NMI.
// The Pico sets the NMI vector to the BREAK snippet before arming.
static volatile bool watchpoint_armed = false;
static uint16_t watchpoint_saved_nmi_vector = 0u;
// Set by the write ISR on a watchpoint hit, consumed by the Q-rising handler.
// The ISR must not touch GPIO_NMI itself: the Q-rising handler drives that pin from
// pending_interrupts on every Q-rising, so a direct ASSERT_NMI is released one
// cycle later — long before the 6809 samples it, and the interrupt is lost.
// A separate flag also avoids racing the read-modify-write on pending_interrupts.
static volatile bool watchpoint_nmi_pending = false;

// Set by the Q-rising handler as it raises the watchpoint's NMI, so the
// console reports only a real hit.  SHARED also carries copies and system
// requests, so bytes left there are no sign that BREAK has run; the handler
// clears the PC slot as it raises the NMI, leaving only BREAK's copy to fill it.
static volatile bool watchpoint_fired = false;

// Set by the DAT fault handler, consumed by the Q-rising handler, for exactly
// the reason watchpoint_nmi_pending exists: the Q-rising handler drives the
// NMI pin from cached state on every pass, so an ISR that asserts the pin
// itself has it released a cycle later and the interrupt is lost.
//
// A separate flag also means the fault handler touches nothing the Q-rising
// handler read-modify-writes, which is what lets it run at a HIGHER priority
// and so set this before the pass that will act on it, rather than waiting
// behind one.  Sampling the fault in PIO moved the latch out of the Q handler
// and left the request behind it; this puts the request back in front.
static volatile bool dat_fault_nmi_request = false;

// How many Q-risings the NMI pin is held low for.  The datasheet asks for a
// pulse width of at least one E cycle, and a one-shot assert - one Q-rising to
// the next - is one E cycle minus however late the two ISR entries were,
// which only just meets it.  NMI is edge-triggered, so a wider pulse costs
// nothing: the edge is latched once and a longer low cannot re-trigger it.
static constexpr uint8_t NMI_ASSERT_CYCLES = 2u;
static volatile uint8_t nmi_assert_cycles = 0u;

// Set by the Q-rising handler on an NMI vector fetch - the bus event BS_IRQ
// reports, and the point by which the 6809 has stacked its whole register
// frame.  CMD_BREAK clears it before raising the NMI and waits on it after, so
// "the break has been taken" comes from the bus rather than from a guess about
// what SHARED holds.
static volatile bool break_vector_fetched = false;

// The register frame a full-frame interrupt stacks and the BREAK snippet copies
// into SHARED: CC, A, B, DP, X, Y, U, PC.
static constexpr uint16_t BREAK_FRAME_LEN = 12u;
// The E (entire) flag in a stacked CC.  NMI, IRQ and SWI all set it before
// pushing, so it marks a frame that has actually arrived.
static constexpr uint8_t CC_ENTIRE = 0x80u;

// Read the frame the BREAK snippet copies into SHARED, once it has landed and
// stopped changing.  The snippet stores one byte per pass of its loop, CC first
// and PC last, so a reader that does not wait for a settled copy can catch a PC
// with only its high byte in.  Two readings a millisecond apart that agree,
// with E set in the stacked CC, mean the copy is complete.
//
// E is the right sentinel and a non-zero PC is not: a guest that breaks at
// $0000 - where a wild jump lands, and so exactly what one breaks to find -
// has a legitimately zero PC and would never satisfy it.
static bool
break_frame_settled(uint8_t *frame, const uint32_t settle_ms)
{
	uint8_t previous[BREAK_FRAME_LEN];

	registers::copy_out(previous, REGISTER_BUFFER_OFFSET, BREAK_FRAME_LEN);
	for (uint32_t ms = 0u; ms < settle_ms; ms++) {
		sleep_ms(1u);
		registers::copy_out(frame, REGISTER_BUFFER_OFFSET, BREAK_FRAME_LEN);
		if ((frame[0] & CC_ENTIRE) != 0u
		    && memcmp(frame, previous, BREAK_FRAME_LEN) == 0)
			return true;
		memcpy(previous, frame, BREAK_FRAME_LEN);
	}
	return false;
}

// Take the watchpoint down and clear the evidence that it fired.  The BREAK
// snippet leaves the stacked frame in SHARED, and anything that finds E set in
// its CC while the watchpoint is armed reads it as a fresh hit - so a report
// that does not clear it gets made again by the next thing to look.  Restoring
// the NMI vector matters too: while armed it points at BREAK rather than at
// wherever the guest had it.
static void
watchpoint_disarm()
{
	if (watchpoint_armed) {
		reg[REGISTER_VECTOR_NMI_OFFSET] = watchpoint_saved_nmi_vector;
		watchpoint_armed = false;
	}
	watchpoint_nmi_pending = false;
	watchpoint_fired = false;
	reg[REGISTER_BUFFER_OFFSET] = static_cast<uint8_t>(0u);
	reg[REGISTER_BUFFER_OFFSET + 10] = static_cast<uint8_t>(0u);
	reg[REGISTER_BUFFER_OFFSET + 11] = static_cast<uint8_t>(0u);
}

// The three open-collector interrupt pins, driven together from the Q-rising
// handler.  Asserting one means enabling its output; the output value is
// already low, so the direction register is the drive.
static constexpr uint32_t INTERRUPT_PIN_MASK =
	(1u << GPIO_NMI) | (1u << GPIO_FIRQ) | (1u << GPIO_IRQ);

// BUSY, LIC and AVMA, sampled at E-falling by the bus_status state machine and
// written here by its DMA, once per E cycle, for ever.  Read at Q-rising - the
// cycle after the sample - the three bits describe the cycle being read.  See
// union bus_status in pico_thing.h for why that is the correct sample point and
// why a reader must take the word once rather than the fields one at a time.
static volatile bus_status_pins bus_status_mirror = { .word = 0u };

static volatile bool q_rising_edge = false;
static volatile bool core1_initialised = false;

mc6850 fast_serial(CONSOLE_CONTROL, CONSOLE_TX_DATA);
mc6850 aux_serial(AUX_CONTROL, AUX_TX_DATA);

// Cached interrupt state — updated by for(;;) loop's has_interrupt() calls,
// applied to GPIO pins by the Q-rising ISR.  Decouples IRQ delivery from
// for(;;) loop throughput.  Packed into a single byte so the ISR does one
// volatile read (not three) — the init ISR budget is extremely tight.
static volatile uint8_t pending_interrupts = 0u;
// An interrupt enum to its pending bit.  Only three of the nine can be
// asserted on a pin; INTERRUPT_NONE, the software interrupts and RESET - which
// no device asks for - contribute nothing.
static constexpr uint8_t PEND_NMI  = 1u << 0;
static constexpr uint8_t PEND_FIRQ = 1u << 1;
static constexpr uint8_t PEND_IRQ  = 1u << 2;

// Interrupt amplification.  acia_torture's handler is entered ~300 times for
// 24 real ACIA receive events, and the question is whether the 6809 really
// takes 300 interrupts (so /IRQ is being held asserted too long) or whether
// something else re-enters the handler.  irq_vectors counts the vector
// fetches the processor actually performs; pend_irq_cycles counts the E cycles
// /IRQ was asserted, so their ratio is the cycles per taken interrupt.
static volatile uint32_t irq_vectors = 0u;
static volatile uint32_t pend_irq_cycles = 0u;

// Single-stepping (TODO 1b).  Both halves of a step MUST happen in the same
// Q-rising pass: clearing the BREAK snippet's spin byte so the guest falls
// through to its RTI, and arming the countdown that will bring it straight
// back.  Driven from Core 0 instead, the two writes are about a millisecond
// apart - a thousand E cycles, some hundreds of instructions of a tight loop -
// and what comes back is not a step at all.  Hence the request flag: Core 0
// asks, the handler does it atomically.
static volatile uint8_t step_arm_cycles = 0u;	// non-zero: release and arm now
static volatile bool step_active = false;	// restore the spin on NMI entry

// LIC-gated stepping, which is what actually steps ONE INSTRUCTION.  A cycle
// count cannot: instructions are 2 to 11 cycles, so the count that advances
// exactly one depends on the lengths of the code being stepped (991fb9d
// measured 29 for a NOP+BRA loop, and the pattern oscillates with those two
// lengths).  LIC does not care: the mirror from 6b77bb3 makes it mean "this
// cycle is an opcode fetch", so raising NMI there leaves it pending through
// the instruction being fetched, and the 6809 finishes that one and takes it.
//
// Three states, because the snippet's own RTI has to be stepped over first.
// The detector fires on the RTI's OPCODE FETCH (LIC=1 then $3B at E-fall), so
// at that moment the RTI has not run yet; the first LIC rising edge after it is
// the RTI finishing, measured on the board at E cycle 15 - a full frame is 15
// cycles.  Arm there and the NMI is pending through exactly one guest
// instruction, which the 6809 completes before taking it.
//
// No cycle count anywhere, which is the point: neither the RTI's length nor the
// stepped instruction's length appears in the rule.  Proved on the board
// against a four-instruction loop of three different lengths.
//
// IT MUST BE THE RAW LIC PIN, NOT THE DMA MIRROR.  Both were swept, offsets 0
// to 2 E cycles, and every mirror variant stepped exactly TWO instructions
// while the raw pin at offset 0 stepped one.  The mirror is written by DMA at
// E-falling and read here a quarter cycle later at Q-rising, so it is a whole
// cycle stale - and a cycle is precisely what the gate cannot spare.  The
// mirror's re-phasing exists to make a trace readable (6b77bb3); acting one
// cycle ahead needs the "in advance" signal as the 6809 drives it.
enum : uint8_t { STEP_IDLE = 0u, STEP_RELEASED = 1u, STEP_AFTER_RTI = 2u };
static volatile uint8_t step_state = STEP_IDLE;
static volatile bool step_request = false;	// Core 0 asks; the handler acts
static volatile bool step_prev_lic = false;	// for the rising-edge test
// Instrument, not mechanism: where the LIC rising edges actually fall after
// the snippet's RTI is fetched, in E cycles from that fetch.  The settle above
// was a guess at "past the RTI", and the board says it is not - every stop came
// back at the same PC, so the NMI is pending before the guest executes
// anything.  Guessing again is no better: log the edges, read which one the
// working cycle count corresponds to, and arm on that edge instead of a count.
// Costs a compare per pass and only while a step test is running.
static constexpr uint8_t STEP_EDGE_MAX = 8u;
static volatile uint8_t step_edge[STEP_EDGE_MAX] = {};
static volatile uint8_t step_edges = 0u;	// how many recorded
static volatile uint8_t step_pass = 0u;		// E cycles since the RTI fetch
static volatile bool step_logging = false;	// armed by the RTI detector
static volatile uint8_t step_armed_edge = 0u;	// edge the LIC gate fired on


static inline uint8_t
pend_bit(const interrupt eirq)
{
	switch (eirq) {
	case INTERRUPT_NMI:	return PEND_NMI;
	case INTERRUPT_FIRQ:	return PEND_FIRQ;
	case INTERRUPT_IRQ:	return PEND_IRQ;
	default:		return 0u;
	}
}

// 50 Hz tick timer (IRQ) + E-cycle countdown (NMI).
// Constructed from getInstance() rather than reg: reg is a namespace-scope
// reference in another translation unit, and the order the two are initialised
// in is unspecified.  The function-local static behind getInstance() is built
// on first use, so there is nothing to get wrong.
static tick_timer system_tick(registers::getInstance(), SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);

static volatile uint32_t core1_qrise_count = 0u;
static volatile uint32_t core1_irq_assert_count = 0u;
static volatile uint32_t core1_loop_count = 0u;
static volatile uint32_t core1_write_ring_count = 0u;
static volatile uint32_t core1_read_irq_count = 0u;
static volatile uint8_t core1_last_write_loc = 0u;
static volatile uint8_t core1_last_read_loc = 0u;
static volatile uint32_t core1_isr_count = 0u;	// incremented in GPIO ISR

// Bus trace capture hardware.  Two PIO state machines sample the bus once per
// E cycle and two DMA channels write straight into MC6809's buffers, so the
// CPU does no per-cycle work at all.  The low half needs a GPIO-base-0 PIO
// (it must see E on GPIO 0 and the whole bus) and the high half needs the
// base-16 one, so they hand off with the RP2350 cross-block IRQ.  PREV/NEXT
// wrap across the three blocks, so pio0's "prev" is pio2 — which matters,
// because pio1 is full to the word (bus_write 19 + rti_detect 13 = 32) and
// pio0 is the only base-0 block with room.
// PIO IRQ flags, which the instruction streams in pico_thing.pio raise and
// this file consumes.  They are named here because the two sides are edited
// independently and a silent mismatch is expensive: a renumbering on
// 2026-10-01 left the RTI detector raising one flag while this file enabled
// and cleared another, which killed RTI detection outright - and with it the
// nesting unwind, so the run state stuck in RS_IN_IRQ - without a word.
//
// Flags are per block.  pio1 and pio2 each have their own set of eight.
//
//   pio1 flag 1  rti_detect        -> PIO1_IRQ_0 -> pio_rti_irq_handler()
//   pio2 flag 0  mc6809_clock's "irq prev set 0", once per E cycle at Q-fall
//                (prev from pio0 wraps to pio2), consumed by bus_fault's wait
//   pio2 flag 1  mc6809_clock's "irq prev set 1", once per E cycle at E-fall,
//                consumed by bus_status's wait.  Nothing else may wait on it:
//                a satisfied wait clears the flag, so a second waiter would
//                steal samples from the BUSY/LIC/AVMA mirror
//   pio2 flag 2  bus_capture_lo -> bus_capture_hi handshake, SM to SM
//   pio2 flag 3  bus_fault, the DAT fault signal to the CPU
static constexpr uint RTI_PIO_IRQ = 1u;
static constexpr pio_interrupt_source RTI_PIO_IRQ_SOURCE = pis_interrupt1;

static constexpr uint BUS_FAULT_PIO_IRQ = 3u;
static constexpr pio_interrupt_source BUS_FAULT_PIO_IRQ_SOURCE = pis_interrupt3;

static constexpr uint TRACE_HANDSHAKE_PIO_IRQ = 2u;

// Set when the capture should wait for its trigger pin rather than start now.
static volatile bool trace_hw_triggered = false;
// Set when a capture is armed but the guest is not running yet.
static volatile bool trace_pending_start = false;
static void trace_hw_start();

// Samples captured so far.  The DMA transfer count is the authority: it counts
// down from trace_size, so this is exact and free.
static uint32_t
trace_hw_count()
{
	if (trace_lo.offset < 0)
		return 0u;
	const uint32_t left = dma_channel_hw_addr(trace_lo.data_channel)->transfer_count;
	return (left > TRACE_SIZE) ? 0u : (TRACE_SIZE - left);
}

static void
trace_hw_stop()
{
	if (trace_lo.offset < 0)
		return;
	dma_channel_abort(trace_lo.data_channel);
	dma_channel_abort(trace_hi.data_channel);
	pio_sm_set_enabled(trace_lo.pio, trace_lo.sm, false);
	pio_sm_set_enabled(trace_hi.pio, trace_hi.sm, false);
}

static void
trace_hw_arm()
{
	if (trace_lo.offset < 0)
		return;
	trace_hw_stop();
	// Drain both FIFOs.  The state machines keep sampling after a completed
	// DMA stops draining them, so without this the next capture would open
	// with the tail of the previous one.
	pio_sm_clear_fifos(trace_lo.pio, trace_lo.sm);
	pio_sm_clear_fifos(trace_hi.pio, trace_hi.sm);
	pio_sm_restart(trace_lo.pio, trace_lo.sm);
	pio_sm_restart(trace_hi.pio, trace_hi.sm);
	pio_interrupt_clear(trace_hi.pio, TRACE_HANDSHAKE_PIO_IRQ);
	// .wrap never revisits the trigger, so the PC has to be put back by hand.
	if (trace_hw_triggered)
		bus_capture_lo_program_rearm(trace_lo.pio, trace_lo.sm,
					     static_cast<uint>(trace_lo.offset));
	else
		bus_capture_lo_program_trigger_now(trace_lo.pio, trace_lo.sm,
						   static_cast<uint>(trace_lo.offset));
	dma_channel_set_write_addr(trace_lo.data_channel, MC6809.trace_base(), false);
	dma_channel_set_write_addr(trace_hi.data_channel, MC6809.trace_hi_base(), false);
	dma_channel_set_trans_count(trace_lo.data_channel, TRACE_SIZE, false);
	dma_channel_set_trans_count(trace_hi.data_channel, TRACE_SIZE, false);
	dma_channel_start(trace_lo.data_channel);
	dma_channel_start(trace_hi.data_channel);
	// Hold the state machines off until the guest is actually running.  They
	// free-run once enabled, so arming against a reset-held bus would spend
	// the whole one-shot on idle cycles before the guest fetched anything —
	// the same trap the old software capture guarded against with a
	// run_state test.  trace_hw_start() is called from the reset-to-running
	// transition in the EQ ISR.
	if (MC6809.get_run_state() >= RS_RUNNING)
		trace_hw_start();
	else
		trace_pending_start = true;
}

// Enable both halves.  They go within a few clocks of each other, far inside
// one E cycle, so they latch onto the same first edge; after that the high
// half is driven by the low half's IRQ and cannot drift.
static void
trace_hw_start()
{
	trace_pending_start = false;
	pio_sm_set_enabled(trace_hi.pio, trace_hi.sm, true);
	pio_sm_set_enabled(trace_lo.pio, trace_lo.sm, true);
}

// Set to zero ONLY for bring-up, and ONLY while !NMI is physically
// disconnected: it suspends the automatic NMIBlock assertion so the sampler
// neither raises the mask nor gates on it, which makes a fault repeatable on a
// scope.  With !NMI wired, a single bad pointer would then NMI-storm the guest
// with nothing to stop it, and SYSTEM_DAT_FAULT would hold the latest fault
// rather than the first.
// Set from CMake with -DPICO_THING_DAT_FAULT_AUTO_BLOCK=0, so bring-up does
// not need a source edit.
#ifndef PICO_THING_DAT_FAULT_AUTO_BLOCK
#define PICO_THING_DAT_FAULT_AUTO_BLOCK 1
#endif
#define DAT_FAULT_AUTO_BLOCK PICO_THING_DAT_FAULT_AUTO_BLOCK

// How many Q-risings to wait before raising NMIBlock after a fault.
//
// The GAL generates the fault NMI as a registered output clocked by Q:
//
//	NMI.D = MMU_dat & Last_Block & !NMIBlock;
//
// so raising the mask in the same handler pass that first sees the fault
// removes the NMI's D term before the 6809 has had a full E cycle to sample
// it, and the interrupt the fault was supposed to raise never arrives.  The
// mask still has to go up - a guest dying of a bad pointer would otherwise
// NMI-storm - but not until the NMI it is masking has been seen.
//
// The window has to outlast the instruction that faulted, not just one E
// cycle.  The 6809 samples !NMI on Q-falling of the cycle before LIC and only
// acts on it at an instruction boundary, so a mask that goes up two cycles
// after the fault can still be up before the boundary arrives - and the
// longest instructions run to about twenty cycles.  The default clears that
// with room to spare.  Set from CMake with
// -DPICO_THING_DAT_FAULT_BLOCK_DELAY=<cycles>; a wider window is also what
// makes the fault NMI visible on a scope.
#ifndef PICO_THING_DAT_FAULT_BLOCK_DELAY
#define PICO_THING_DAT_FAULT_BLOCK_DELAY 32u
#endif
static constexpr uint8_t DAT_FAULT_BLOCK_DELAY = PICO_THING_DAT_FAULT_BLOCK_DELAY;

// Counts down to the NMIBlock assertion above.  Zero means idle, and also
// means the sampler is free to latch the next fault.
static volatile uint8_t dat_fault_block_countdown = 0u;

// Set when a DAT fault is latched, cleared when the resulting NMI takes its
// vector.  Distinguishes a fault NMI from the timer and watchpoint NMIs, which
// must not force a task switch.
static volatile bool dat_fault_nmi_pending = false;

// Stack watermark for Core 1 — paint stack with 0xDEADBEEF, then scan
// from the bottom to find the high-water mark.
extern "C" uint32_t __StackOneBottom;	// linker symbol (NOLINT: the name is the SDK's)
extern "C" uint32_t __StackOneTop;	// linker symbol (NOLINT: the name is the SDK's)

void __isr
__time_critical_func(gpio_q_irq_handler())
{
#ifdef DEBUG
	gpio_put(GPIO_TRACE_C, true); // scope timing
#endif
	// Q-RISING ONLY.  The falling edge used to be enabled as well, purely to
	// poll GPIO_FAULT once per cycle - a million entries a second to read one
	// pin.  The bus_fault state machine on pio2 does that sampling now,
	// clocked by the clock program's "irq prev set 0" at the same Q-falling
	// phase, and raises an interrupt only when a fault is actually there.
	//
	// The event mask is still read and tested rather than assumed: this is a
	// raw handler on IO_IRQ_BANK0, so it is called for any bank 0 interrupt,
	// and only Q-rising belongs to it.  IO_BANK0 sits behind the APB bridge,
	// where each register access costs several cycles against one for SIO, so
	// it is read once and acknowledged once.
	//
	// Acknowledging up front is the safe order: the INTR latch is set by the
	// hardware whenever an edge arrives, so an edge that lands while this
	// pass is still running is latched again and re-enters.  Acknowledging
	// afterwards would clear that latch and lose the edge.
	const uint32_t q_events = gpio_get_irq_event_mask(GPIO_Q);

	gpio_acknowledge_irq(GPIO_Q, q_events & GPIO_IRQ_EDGE_RISE);
	if (q_events & GPIO_IRQ_EDGE_RISE) {
		// Counted here rather than on entry: one increment per E cycle
		// instead of two, and it then measures the same event as
		// core1_qrise, which the for(;;) loop counts when it observes the
		// rising edge.  The pair is the starvation ratio that diagnosed the
		// Core 1 livelock of 2026-09-15 - if this climbs while core1_qrise
		// and core1_loop stand still, the ISRs are eating the loop.
		core1_isr_count++;
		q_rising_edge = true;
		bus_pins bus_pins{};
		bus_pins.word = gpio_get_all();
		// BUSY/LIC/AVMA are stable at Q-rising (this is the latest valid capture point;
		// by Q-falling the 6809 has already updated them for the next cycle). Unpack
		// immediately so that vma is valid before the DMA read ISR fires (~20 cycles later).
		// From the mirror, not from bus_pins: these three are valid only
		// t_CD after Q-rising - 300 ns at 1 MHz - so the snapshot this
		// handler just took of the pins has the PREVIOUS cycle's values.
		// The mirror was sampled at the last E-falling, which describes
		// this cycle.  One load, then decode: see union bus_status_pins.
		const auto bla = static_cast<uint8_t>(bus_status_mirror.word & 0x07u);
		MC6809.set_busy_lic_avma(bla);
		// Keep what we just used, against the row the capture DMA is about
		// to write, so the columns line up.  Only while armed: this reads a
		// DMA register, which is not something to do on every E cycle for
		// nothing.
		if (MC6809.trace_is_armed())
			MC6809.trace_mirror_store(trace_hw_count(), bla);
		auto address_lsb = static_cast<uint8_t>(bus_pins.bits.A);
		const auto new_bs = static_cast<enum bus_state>(bus_pins.bits.BS_BA & 0x03u);
		const bool reset_now = RESET_IS_ASSERTED;
#ifdef DEBUG
		MC6809.count_lic();
#endif
		if (reset_now) {
			if (MC6809.run_state != RS_RESET) {
				MC6809.run_state = RS_RESET;
				MC6809.task_stack_ptr = 0;
			}
			// Keep the reported state honest while the guest is held: this
			// branch used to leave bus_state frozen at whatever it was when
			// the reset pin went low, and status reported that as current.
			// It is done here rather than ahead of the chain so the running
			// path below is left exactly as it was - the budget in this
			// handler is measured in a few instructions, and the cycles
			// spent here are ones where no bus traffic needs servicing.
			if (new_bs != MC6809.bus_state) {
				MC6809.old_bus_state = MC6809.bus_state;
				MC6809.bus_state = new_bs;
			}
		} else if (MC6809.run_state == RS_RESET) {
			MC6809.run_state = RS_RUNNING;
			// The guest has started fetching.  Release a trace that was
			// armed while the bus was still held in reset, so its one-shot
			// is spent on real cycles rather than idle ones.
			if (trace_pending_start)
				trace_hw_start();
		} else if (new_bs != MC6809.bus_state) {
			MC6809.old_bus_state = MC6809.bus_state;
			MC6809.bus_state = new_bs;
			switch (new_bs) {
			case BUS_IRQ: {
				auto eirq = static_cast<interrupt>((address_lsb - 0x30u) >> 1);
				if (eirq <= INTERRUPT_RESET)
					MC6809.trace_irq_event(eirq);	// arm trace if this vector is a configured trigger
				if (eirq < INTERRUPT_RESET) {
					// The depth counts every interrupt; only the task
					// HISTORY is bounded.  These used to share one guard,
					// so at MAX_INT_NEST_DEPTH the depth stopped rising
					// while every RTI kept lowering it - the count went
					// permanently out of step, and nothing said so.
					if (MC6809.task_stack_ptr < mc6809::MAX_INT_NEST_DEPTH)
						MC6809.task_stack[MC6809.task_stack_ptr.fetch_add(
							1u, std::memory_order_relaxed)] = MC6809.task;
					else
						MC6809.depth_overflows++;
					if (MC6809.nesting_depth < 0xFFu)
						MC6809.nesting_depth++;
				}
				// Both the break command and the watchpoint point the NMI
				// vector at the BREAK snippet, and the vector fetch is the
				// point at which the frame is on the stack and the copy is
				// about to run.  This is the bus telling the console the
				// break has been taken.
				if (eirq == INTERRUPT_IRQ)
					irq_vectors++;
				if (eirq == INTERRUPT_NMI) {
					break_vector_fetched = true;
					// The guest is entering BREAK again, so it is
					// long past the spin it fell through: put the
					// offset back before it gets there.
					if (step_active)
						reg[REGISTER_SNIPPET + 14] =
							static_cast<uint8_t>(0xFEu);
				}
				// DELIBERATELY NOT UNIVERSAL (the owner, 2026-10-04).
				// The brief once asked for every interrupt to switch to
				// task 0 on entry; this switches only a DAT fault.  So
				// every other handler runs under the INTERRUPTED task's
				// own mapping and must switch for itself if it needs
				// system memory - the DAT only lets task 0 write DAT RAM.
				// Making the switch universal would change the
				// environment every NitrOS-9 handler runs in, so the risk
				// is in the guest rather than here, and this is the only
				// arrangement proven to boot an OS.  Asked and answered:
				// leave it.  See also apply_rti(), where the matching
				// decision not to restore the task lives.
				//
				// A DAT fault vectors through NMI, and its handler has to
				// reach the DAT RAM to reclaim the dead process's pages —
				// which the GAL only enables for task 0.  (The faulting
				// instruction is gone: the 6809 takes NMI at an instruction
				// boundary, so the access has already completed on garbage
				// and cannot be retried.)  Switch here rather than when the
				// fault was latched, so the rest of that instruction and the
				// CPU's register push both happen under the faulting task's
				// own mapping, and the faulting task is on the task stack
				// before the handler runs.
				if (eirq == INTERRUPT_NMI && dat_fault_nmi_pending) {
					dat_fault_nmi_pending = false;
					const uint8_t supervisor = MC6809.task_change(0u);
					reg[SYSTEM_TASK] = supervisor;
					task_pins_update(supervisor);
				}
				// Not DEBUG-only: the interrupt tests deassert on this
				// count, so it has to be maintained in every build.
				if (eirq <= INTERRUPT_RESET)
					MC6809.count_irq_ack(eirq);
				// Reset fetches its vector through here too, and that is the
				// guest starting rather than the guest taking an interrupt.
				// Counting it as one stuck: nesting_depth is not incremented
				// for reset - the task stack push above is guarded the same
				// way - so apply_rti() had nothing to unwind, BUS_RUNNING
				// deliberately will not undo RS_IN_IRQ, and every guest read
				// as being in an interrupt from the moment it started.
				if (eirq < INTERRUPT_RESET)
					MC6809.run_state = RS_IN_IRQ;
				break;
			}
			case BUS_SYNC:  MC6809.run_state = RS_SYNCED; break;
			case BUS_HALT:  MC6809.run_state = RS_HALTED; break;
			case BUS_RUNNING:
				if (MC6809.run_state != RS_IN_IRQ)
					MC6809.run_state = RS_RUNNING;
				break;
			default:
				break;
			}
		}
		// Deferred NMIBlock assertion after a DAT fault, giving the NMI the
		// GAL raised time to be seen before its D term is taken away.
		if (dat_fault_block_countdown > 0u && --dat_fault_block_countdown == 0u) {
			gpio_put(GPIO_NMI_BLOCK, true);
			reg[SYSTEM_DAT_FAULT] |= DAT_FAULT_NMI_BLOCK;
		}
		// Deferred task restore after RTI detection
		MC6809.tick_rti_countdown();
		// LIC-gated step.  Release and gate are both here for the same
		// reason the cycle-count path is: from Core 0 the two would be a
		// millisecond apart, which is not a step.
		{
			// The raw pin, deliberately: see the note on the states.
			const bool lic_now = gpio_get(GPIO_LIC);
			if (step_request) {
				// Zeroing the BRA's offset turns the snippet's spin into
				// "branch to the next byte", which is the RTI.
				reg[REGISTER_SNIPPET + 14] = static_cast<uint8_t>(0x00);
				step_state = STEP_RELEASED;
				step_request = false;
			}
			if (step_logging) {
				if (step_pass < 0xFFu)
					step_pass++;
				if (lic_now && !step_prev_lic
				    && step_edges < STEP_EDGE_MAX)
					step_edge[step_edges++] = step_pass;
			}
			if (step_state == STEP_AFTER_RTI
			    && lic_now && !step_prev_lic) {
				// The RTI has ended and the guest's own
				// instruction is this cycle.  PEND_NMI reaches the
				// pin later in this same pass, so it is pending
				// through that one instruction.
				pending_interrupts |= PEND_NMI;
				step_armed_edge = step_edges;
				step_state = STEP_IDLE;
			}
			step_prev_lic = lic_now;
		}
		// A step asked for: release the guest and arm, in this one pass.
		// Ordered before the tick below so the cycle armed here is not
		// immediately spent.
		if (step_arm_cycles != 0u) {
			// Zeroing the BRA's offset turns the snippet's spin into
			// "branch to the next byte", which is the RTI.
			reg[REGISTER_SNIPPET + 14] = static_cast<uint8_t>(0x00);
			system_tick.arm_countdown(step_arm_cycles);
			step_arm_cycles = 0u;
		}
		// E-cycle countdown NMI timer: $FFC8 bits 5:1 are a 5-bit count,
		// decremented here once per Q-rising, and the pass it reaches zero
		// on raises NMI.  So "LDA #(1<<1); STA $FFC8" means NMI after one
		// E cycle - the single-stepping primitive.
		//
		// This was disabled by an #if 0 from e6c56df (March 2026) with no
		// reason recorded.  Re-enabled 2026-10-04 and proved on the board;
		// if it misbehaves, the first suspect is that PEND_NMI is shared
		// with the DAT fault, the watchpoint and break, all of which funnel
		// through the one nmi_assert_cycles pulse below - two requests in
		// the same pass collapse into a single pulse, and only break has a
		// way (break_vector_fetched) to confirm its own was the one taken.
		if (system_tick.tick())
			pending_interrupts |= PEND_NMI;

		// Assert/deassert all interrupt pins from cached pending state.
		// NMI is edge-triggered (one-shot): assert once and clear flag.
		// IRQ/FIRQ are level-triggered: assert when pending, deassert
		// when not.  The DMA read ISR clears PEND_IRQ after
		// rx_read_fast_path() and timer reads, keeping stale assertion
		// windows short.
		//
		// The three pins are driven with one pair of SIO writes rather than
		// an ASSERT/DEASSERT each: the pins are open-collector, so the
		// direction register IS the drive, and gpio_set_dir_masked() sets the
		// output-enables of all three from one mask.  Three stores become
		// two, and they are unconditional, so nothing here depends on a
		// shadow copy staying in step with mc6809::init() and deinit(),
		// which drive the same pins from Core 0.
		{
			const uint8_t pend = pending_interrupts;
			if (pend & PEND_IRQ)
				pend_irq_cycles++;
			uint32_t oe = 0u;
			if ((pend & PEND_NMI) || watchpoint_nmi_pending
			    || dat_fault_nmi_request) {
				if (watchpoint_nmi_pending) {
					reg[REGISTER_BUFFER_OFFSET] = static_cast<uint8_t>(0u);
					reg[REGISTER_BUFFER_OFFSET + 10] = static_cast<uint8_t>(0u);
					reg[REGISTER_BUFFER_OFFSET + 11] = static_cast<uint8_t>(0u);
					watchpoint_fired = true;
				}
				nmi_assert_cycles = NMI_ASSERT_CYCLES;
				pending_interrupts = pend & ~PEND_NMI;
				watchpoint_nmi_pending = false;
				dat_fault_nmi_request = false;
			}
			// Hold it for the full width, then let it go.  Each request
			// restarts the count, so a second NMI during the pulse does not
			// shorten it.
			if (nmi_assert_cycles > 0u) {
				oe |= 1u << GPIO_NMI;
				nmi_assert_cycles--;
			}
			if (pend & PEND_FIRQ)
				oe |= 1u << GPIO_FIRQ;
			if (pend & PEND_IRQ)
				oe |= 1u << GPIO_IRQ;
			gpio_set_dir_masked(INTERRUPT_PIN_MASK, oe);
		}
	}
#ifdef DEBUG
	gpio_put(GPIO_TRACE_C, false); // scope timing
#endif
}

// ISR for the RTI opcode detector PIO program.
// Fires when the PIO detects LIC=1 followed by D[7:0]=$3B at E-fall.
void __isr
__time_critical_func(pio_rti_irq_handler())
{
	pio_interrupt_clear(rti_detect.pio, RTI_PIO_IRQ);
	MC6809.apply_rti();
	// During a LIC-gated step this is the BREAK snippet's own RTI: from the
	// next opcode fetch onwards the guest is running its own code again.
	if (step_state == STEP_RELEASED)
		step_state = STEP_AFTER_RTI;
	// Both gates log, so the cycle count that works can be read against the
	// edges it falls between.
	if (step_active) {
		step_pass = 0u;
		step_edges = 0u;
		step_armed_edge = 0u;
		step_logging = true;
	}
}

// ISR for the DAT fault sampler PIO program.
//
// Fires when bus_fault finds GPIO_FAULT high at Q-falling, which means the
// guest has read or written an 8kB block the DAT does not map.  The flag is
// cleared first: it is a latch, and a handler that returns without clearing it
// re-enters for ever.
//
// Deliberately NOT a re-test of the FAULT pin.  The sampler established the
// fault at Q-falling, the point where the GAL's registered output and the live
// bus pins both describe the same cycle; by the time this runs the pin may
// have gone low again, and a re-test would throw a real fault away.
//
// The task and page come from the sampler's own `in pins 16`, so they are the
// values at the fault rather than whatever the bus has moved on to.  R/!W
// cannot: it is GPIO 2, outside pio2's base-16 window, so the direction bit is
// read live here.  The record therefore has mixed provenance - page and task
// latched, direction sampled a little later - which is worth knowing when a
// fault's R/!W bit looks wrong.
void __isr
__time_critical_func(pio_bus_fault_irq_handler())
{
	pio_interrupt_clear(bus_fault.pio, BUS_FAULT_PIO_IRQ);

	// Drain the FIFO and keep the FIRST word.  More than one can be waiting
	// if the guest faulted again before this ran, and first-fault-wins
	// applies to the sample exactly as it does to the record.  Draining the
	// rest also means no stale word is left to be read as the next fault.
	high_bus_pins high_pins{};
	bool sampled = false;
	while (!pio_sm_is_rx_fifo_empty(bus_fault.pio, static_cast<uint>(bus_fault.sm))) {
		const uint32_t word = pio_sm_get(bus_fault.pio, static_cast<uint>(bus_fault.sm));
		if (!sampled) {
			high_pins.word = word;
			sampled = true;
		}
	}
	// Belt and braces: the flag cannot be raised without the push that
	// precedes it, but a live read beats recording nothing.
	if (!sampled)
		high_pins.word = sio_hw->gpio_hi_in;

	// First fault wins.  The countdown stands in for the mask until it
	// expires, so a second fault inside the window cannot overwrite the
	// first, and NMIBlock rejects everything after that until the guest
	// clears it.
	if (dat_fault_block_countdown != 0u)
		return;
	if (DAT_FAULT_AUTO_BLOCK && high_pins.bits.NMI_BLOCK)
		return;

	// Stop the interrupts rather than filter them.  A guest dying of a bad
	// pointer faults on every cycle, and rejecting a million interrupts a
	// second here would cost more than the Q-falling poll this replaced.
	// The source goes back on when the guest clears SYSTEM_DAT_FAULT, or at
	// the next reset - the same two places that lower NMIBlock.
	pio_set_irq0_source_enabled(bus_fault.pio, BUS_FAULT_PIO_IRQ_SOURCE, false);

	// Arm the mask rather than raising it here: see the note by
	// DAT_FAULT_BLOCK_DELAY.
	if constexpr (DAT_FAULT_AUTO_BLOCK)
		dat_fault_block_countdown = DAT_FAULT_BLOCK_DELAY;
	// Tell the BUS_IRQ case that the NMI now on its way is a fault, so it
	// can drop the guest into task 0 for the handler.
	dat_fault_nmi_pending = true;
	// And ask for that NMI rather than relying on the GAL's.
	//
	// The GAL raises one - NMI.D = MMU_dat & Last_Block & !NMIBlock - but it
	// lasts only as long as the faulting access is on the bus, because the D
	// term goes false the moment the access ends.  That puts the whole pulse
	// inside the faulting instruction's own last cycle, and the 6809 samples
	// !NMI on Q-falling of the cycle BEFORE the last one, so it arrives a
	// cycle too late for the boundary it caused and is gone before the next.
	//
	// Going through pending_interrupts asserts from the next Q-rising and
	// holds the pin from the Q-rising handler, which is the path the
	// watchpoint and the irq tests already use.  This handler runs at the
	// same priority as that one and on the same core, so neither can preempt
	// the other part-way through this read-modify-write.
	dat_fault_nmi_request = true;
	// Nothing else writes bits 2-6.  The NMIBlock bit is not set here: it
	// mirrors the pin, and the pin is not up yet.
	reg[SYSTEM_DAT_FAULT] = static_cast<uint8_t>(
		DAT_FAULT_FAULT
		| (gpio_get(GPIO_RW) ? DAT_FAULT_RW : 0u));
	// The task pins are the ones the DAT hardware itself saw, not
	// MC6809.task.
	reg[SYSTEM_DAT_FAULT_LOC] = static_cast<uint8_t>(
		(high_pins.bits.TASK << DAT_FAULT_LOC_TASK_SHIFT)
		| high_pins.bits.A_HI);
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

// What the guest was SERVED on each console ACIA status read, newest last.
// The counters showed RDRF staged and cleared identically whether or not a
// host is attached, so the question is no longer "what cleared it" but "what
// did the handler actually see, and in what order".  This answers that: it is
// the byte the read DMA handed over, recorded in the ISR that saw the access.
// Only the reads served with S_IRQ set are the handler's - the rest are the
// polled output loop waiting on TDRE, and there are over a thousand of those.
// The FIRST ones are kept, not the last, because test 3 is the first interrupt
// the suite ever takes.
// Console status reads, counted IN THE ISR because the generic notification
// path cannot see them.  The ISR handles CONSOLE_STATUS in a branch of its own
// and that branch does not set read_irq_received, so the Core 1 switch which
// increments the other three access counters is never reached for this address
// - which left st's console "Status" column a permanent zero from 1527983
// until this was noticed on 2026-10-04.  AUX_STATUS has no such branch and so
// counted all along, which is what made the asymmetry visible: an identical
// flowtest run reported 8193 writes against 8390 status reads on the aux port
// and 8193 against 0 on the console.
//
// A plain counter rather than the read_irq_received flag, for the same reason
// rx_read_count is one: that flag carries a single address, so two reads
// arriving before Core 1 drains it are counted once.  The aux column still has
// that flaw and is a lower bound; this one is exact.
static volatile unsigned CONSOLE_STATUS_READS = 0u;

#define STATUS_SEEN_LEN 16u
static volatile uint8_t status_seen[STATUS_SEEN_LEN];
static volatile uint32_t status_seen_n = 0u;	// reads with S_IRQ set
static volatile uint32_t status_irq_rdrf = 0u;	// ... and S_RDRF set too: correct
static volatile uint32_t status_irq_nordrf = 0u;	// ... but S_RDRF CLEAR: the fault

void __isr
__time_critical_func(dma_bus_read_irq_handler())
{
#ifdef DEBUG
	gpio_put(GPIO_TRACE_C, true); // scope timing
#endif
	dma_channel_acknowledge_irq0(piodma_read.data_channel);
	if (MC6809.run_state >= RS_RUNNING && MC6809.get_vma()) {
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
		} else if (loc == CONSOLE_STATUS) {
			CONSOLE_STATUS_READS++;
			// registers::read() is what the DMA served from.
			const uint8_t served =
				*reinterpret_cast<volatile uint8_t *>(
					registers::read_address() + CONSOLE_STATUS);
			if (served & 0x80u) {		// S_IRQ
				if (status_seen_n < STATUS_SEEN_LEN)
					status_seen[status_seen_n] = served;
				status_seen_n++;
				if (served & 0x01u)	// S_RDRF
					status_irq_rdrf++;
				else
					status_irq_nordrf++;
			}
		} else if (loc == AUX_RX_DATA) {
			aux_rx_read_count++;
			aux_serial.rx_read_fast_path();
		} else if (loc == SYSTEM_TIMER_STATUS) {
			// Reading the timer status register acknowledges the IRQ.
			system_tick.acknowledge();
		} else if (loc == SYSTEM_DAT_FAULT) {
			// Reading the fault register clears the fault and the R/!W
			// latched with it — the DMA has already handed the pre-clear byte
			// to the PIO, so the guest sees the fault and the clear lands
			// before its next read.  NMIBlock (bit 7) is live state and
			// persists.  SYSTEM_DAT_FAULT_LOC is left alone: it only means
			// anything while a fault is latched, and the next fault
			// overwrites it, so the guest can read the two in either order.
			reg[SYSTEM_DAT_FAULT] &= static_cast<uint8_t>(~DAT_FAULT_LATCHED);
		} else {
			read_location = loc;
			read_irq_received = true;
		}
	}
#ifdef DEBUG
	gpio_put(GPIO_TRACE_C, false); // scope timing
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

// Register-block writes from the console, for Core 1 to carry out as though
// the guest had made them.  A queue rather than the write ring, whose one
// producer has to be the write ISR: Core 1 takes these off with its interrupts
// disabled and runs each through bus_write_followup(), so the two never
// interleave.  Long enough for the largest modify, 255 bytes.
// A guest write to SYSTEM_TASK that no OS should be making: from a non-zero
// task, to a DIFFERENT non-zero task - a user process moving itself into
// someone else's mapping.  This counts rather than blocks, deliberately.
//
// The obvious rule, "only task 0 may write $FFC0", is wrong for this platform.
// The NitrOS-9 Level 2 kernel runs in the fixed $E000-$FDFF window that is
// mapped into every task, so it writes $FFC0 with the INTERRUPTED task still
// in force on every IRQ, FIRQ and NMI entry, and twice on every system call
// from user state (krn.asm KrnFasterClrXxxx, MapT0, SWICall).  A task-0 guard
// would reject all of those and break the boot.  Nor can a bad write be
// refused once identified: no kernel site reads $FFC0 back to check it took,
// so a dropped entry write leaves the handler running with the user's block 0
// as its direct page, and a dropped return write resumes the user in task 0's
// map - the "guest runs wild" failure of the 2026-09-16 bring-up.
//
// What every legitimate write has in common is its value: 0, or the task
// already in force.  Anything else is the guest doing something no kernel path
// does, so it is worth seeing - `st` reports the count and the last one.
static volatile uint32_t task_write_odd_count = 0u;
static volatile uint8_t task_write_odd_from = 0u;
static volatile uint8_t task_write_odd_to = 0u;

static constexpr uint32_t CONSOLE_WRITE_QUEUE_LEN = 256u;
static queue_t console_write_queue;
static volatile uint32_t console_writes_done = 0u;	// counted by Core 1

// Everything a write into the register block sets off, whoever made it: the
// fast paths that must act before the guest's next instruction, then an entry
// in the write ring for Core 1's loop to finish.  The DMA write ISR calls it
// for guest writes; Core 1 calls it for console writes with its interrupts
// off, so the two cannot interleave.  The byte is already in the write side.
static void
__time_critical_func(bus_write_followup)(const uint8_t wloc, const uint8_t wval)
{
	// Special-case any writes that need to be handled before the next guest
	// instruction picks up an incorrect read. I.e., these need to be _fast_.
	if (wloc == SYSTEM_TASK) {
		const uint8_t old_task = MC6809.task_get();
		const uint8_t new_task = MC6809.task_change(wval);
		if (old_task != 0u && new_task != 0u && new_task != old_task) {
			task_write_odd_count++;
			task_write_odd_from = old_task;
			task_write_odd_to = new_task;
		}
		reg[SYSTEM_TASK] = new_task;
		task_pins_update(new_task);
	}
	else if (wloc == SYSTEM_RTI_SIGNAL)
		// For a guest that signals its own RTIs instead of being watched; see
		// pico_thing.h.  The rti_detect PIO program catches every RTI opcode
		// fetch by itself, so a guest that does both has each RTI counted
		// twice.  Nothing in this tree writes it.
		MC6809.apply_rti();
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
	else if (wloc == SYSTEM_INT_DEPTH)
		// The OS telling us where it really is; see the note in
		// pico_thing.h for why this exists at all.
		MC6809.declare_depth(wval);
	else if (wloc == SYSTEM_WATCHPOINT) {
		// Arm/disarm deferred to write ring handler (needs snippet_code[])
	}
	else if (wloc == SYSTEM_TRACE) {
		// USim TraceCtl convention: non-zero arms a one-shot bus trace, zero stops.
		if (wval)
			MC6809.trace_arm();
		else
			MC6809.trace_stop();
	}
	else if (wloc == SYSTEM_DAT_FAULT) {
		// Bit 7 (NMIBlock) drives the hardware fault-NMI mask pin, and the
		// guest reads its state back from the same bit.  This has to happen
		// here rather than in the write ring, so that a guest masking faults
		// is protected before its next instruction can fault.  Bits 0 and
		// 1 (Fault and R/!W) are owned by the Q-rising sampler and are
		// read-only, so writes leave them alone.
		const bool block = (wval & DAT_FAULT_NMI_BLOCK) != 0u;
		// An explicit write wins over a pending automatic assertion:
		// leaving the countdown armed would raise the mask again a
		// cycle or two after the guest had just cleared it.
		dat_fault_block_countdown = 0u;
		gpio_put(GPIO_NMI_BLOCK, block);
		// Lowering the mask is the guest saying it has dealt with the
		// fault, so the sampler's interrupt goes back on here - it was
		// switched off when the fault was latched, to keep a fault storm
		// from becoming an interrupt storm.  Any queued samples belong to
		// the fault just handled and must not be read as the next one.
		if (!block) {
			pio_sm_clear_fifos(bus_fault.pio, static_cast<uint>(bus_fault.sm));
			pio_interrupt_clear(bus_fault.pio, BUS_FAULT_PIO_IRQ);
			pio_set_irq0_source_enabled(bus_fault.pio,
						    BUS_FAULT_PIO_IRQ_SOURCE, true);
		}
		if (block)
			reg[SYSTEM_DAT_FAULT] |= DAT_FAULT_NMI_BLOCK;
		else
			reg[SYSTEM_DAT_FAULT] &= static_cast<uint8_t>(~DAT_FAULT_NMI_BLOCK);
	}
	// The first 16 bytes of emulated registers are devices. The rest are RAM as far as the guest
	// CPU is concerned. When watchpoint is armed, writes to snippet space ($FFE0-$FFEF) and
	// vector area ($FFF0-$FFFF) are blocked and trigger NMI — protects both the BREAK handler
	// and the vectors it guards.
	else if (watchpoint_armed && wloc >= REGISTER_SNIPPET_OFFSET)
		watchpoint_nmi_pending = true;
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
}

void __isr
__time_critical_func(dma_bus_write_irq_handler())
{
#ifdef DEBUG
	gpio_put(GPIO_TRACE_D, true); // scope timing
#endif
	dma_channel_acknowledge_irq1(piodma_write.data_channel);
	if (MC6809.run_state >= RS_RUNNING) {
		const uintptr_t written_addr = dma_channel_hw_addr(piodma_write.data_channel)->write_addr;
		const uint8_t wloc = written_addr & registers::register_mask;
		const uint8_t wval = *reinterpret_cast<volatile std::uint8_t *>(written_addr);
		bus_write_followup(wloc, wval);
	}
#ifdef DEBUG
	gpio_put(GPIO_TRACE_D, false); // scope timing
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

static const uint8_t snippet_code[SNIPPET_LEN][16u] = {
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

// Copies in after which the Pico's two views of SHARED disagreed: the read
// buffer, filled by the write interrupt handler, and the write buffer, filled
// directly by DMA as the processor writes.  Diagnostic - it says which side
// of the Pico a bad byte came from.
static uint32_t copy_buffer_disagreements = 0u;

static bool
copy(const copy_type direction, const uint16_t count, const uint16_t address, const uint8_t *data)
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
			registers::copy_out(const_cast<uint8_t *>(data) + pos, REGISTER_BUFFER_OFFSET, len);
			uint8_t written[REGISTER_BUFFER_LEN];
			registers::copy_out_write(written, REGISTER_BUFFER_OFFSET, len);
			if (std::memcmp(written, data + pos, len) != 0)
				copy_buffer_disagreements++;
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
static volatile unsigned ACIA_RX_DATA_COUNT = 0u;
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
	ACIA_RX_DATA_COUNT = 0u;
	CONSOLE_STATUS_READS = 0u;
	AUX_ACIA_CONTROL_COUNT = 0u;
	AUX_ACIA_TX_DATA_COUNT = 0u;
	AUX_ACIA_STATUS_COUNT = 0u;
	AUX_ACIA_RX_DATA_COUNT = 0u;
#endif

	// Reset tick timer (ACIA resets are owned by Core 1)
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
			reg[SYSTEM_REQUEST_RESPONSE] = RESPONSE_UNKNOWN;
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

// The DS12885's registers, for the rtc command.  The clock is kept in binary
// and 24-hour mode.  The DS12885 has no century register - that is the
// DS12C887's - so the year is two digits and taken to be 20xx.
enum rtc_register : uint8_t {
	RTC_SECONDS,
	RTC_SECONDS_ALARM,
	RTC_MINUTES,
	RTC_MINUTES_ALARM,
	RTC_HOURS,
	RTC_HOURS_ALARM,
	RTC_DAY,		// day of the week, 1 being Sunday
	RTC_DATE,
	RTC_MONTH,
	RTC_YEAR,
	RTC_A,			// the time, alarm and date registers are the ten below this
	RTC_B,
	RTC_C,
	RTC_D,
	RTC_CLOCK_REGISTERS	// $00-$0D, what RTC_READ fetches
};
static constexpr uint8_t RTC_A_DV_MASK = 0x70u;
static constexpr uint8_t RTC_A_DV_RUN = 0x20u;	// 010, the only divider pattern that keeps time
static constexpr uint8_t RTC_B_SET = 0x80u;
static constexpr uint8_t RTC_B_MODE = 0x06u;	// DM set for binary, 24/12 set for 24-hour
static constexpr uint8_t RTC_D_VRT = 0x80u;

static const char *const rtc_day_name[] = {
	"?", "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"
};

// Month is 1-12.
static int
days_in_month(const int year, const int month)
{
	static constexpr int days[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
	const bool leap = (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
	return days[month - 1] + ((month == 2 && leap) ? 1 : 0);
}

// Day of the week as the DS12885 counts it, 1 being Sunday, by Sakamoto's
// method.  Month is 1-12.
static uint8_t
rtc_day_of_week(const int year, const int month, const int day)
{
	static constexpr int offset[] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
	const int y = year - (month < 3 ? 1 : 0);
	return static_cast<uint8_t>((y + y/4 - y/100 + y/400 + offset[month - 1] + day) % 7 + 1);
}

// Fetch registers $00-$0D with RTC_READ, which waits out an update and
// retakes any pass one lands in, so they are a single instant.  False, with
// the reason printed, if the chunk cannot run or the RTC does not answer.
static bool
rtc_read(uint8_t r[RTC_CLOCK_REGISTERS])
{
	if (!MC6809.assert_stopped())
		return false;
	if (!copy(OUTWARDS, chunk_len[RTC_READ], CHUNK_ADDRESS, chunk_code[RTC_READ]))
		return false;
	reg[REGISTER_VECTOR_RESET_OFFSET] = CHUNK_ADDRESS;
	if (!MC6809.start_with_timeout(100u)) {
		printf("RTC is not answering - its update-in-progress bit never cleared\n");
		return false;
	}
	registers::copy_out(r, REGISTER_BUFFER_OFFSET, RTC_CLOCK_REGISTERS);
	return true;
}

// The RAM and DAT test, for the ramtest command.
//
// Physical memory is 256 pages of 8kB, 64 to each chip select the RAM glue
// decodes: RAM0 and RAM1 are the two chips on the board, RAM2 and RAM3 come
// out on the expansion connector.  Pages $00-$FE are reached by remapping
// task zero's slot 0; page $FF cannot be put in a slot - a slot holding it
// faults - and is reached only through the fixed window at $E000-$FDFF.
// Every fill and check runs from the Pico's own window, so nothing under
// test is also holding the code or the stack doing the testing.
//
// The order matters.  Each chip select has its data lines proven first,
// then its address lines, and only then its storage: a stuck data bit
// otherwise passes itself off as one page holding another's data, and a
// floating bus as a chip storing garbage.  Each stage reads only what the
// stages before it have already made trustworthy.
static constexpr uint8_t RAM_LAST_DAT_PAGE = 0xFEu;
static constexpr uint8_t RAM_FIXED_PAGE = 0xFFu;
static constexpr uint16_t RAM_WINDOW = 0x0000u;
static constexpr uint16_t RAM_WINDOW_END = 0x2000u;
static constexpr uint16_t RAM_FIXED_START = 0xE000u;
static constexpr uint16_t RAM_FIXED_END = 0xFE00u;	// the DAT RAM follows
static constexpr uint16_t RAM_DAT_BASE = 0xFE00u;
static constexpr uint16_t RAM_DAT_ENTRIES = 256u;
static constexpr uint8_t RAM_TASKS = 32u;
static constexpr uint8_t RAM_SLOTS = 7u;		// slot 7 is the fixed window
static constexpr uint16_t RAM_REPORT_LIMIT = 8u;
static constexpr uint8_t RAM_ATTEMPTS = 3u;
static constexpr uint8_t RAM_CONFIRM_READS = 3u;
static constexpr uint8_t RAM_DATA_BITS = 8u;
static constexpr uint8_t RAM_OFFSET_LINES = 13u;	// A0-A12, within a page
static constexpr uint8_t RAM_PAGE_LINES = 6u;		// PA13-PA18, within a chip
static constexpr uint8_t RAM_MARKER = 0xAAu;
static constexpr uint8_t RAM_ANTI_MARKER = 0x55u;
static constexpr uint64_t RAM_PROGRESS_US = 30u * 1000u * 1000u;

// Patch points in the RAM_FILL snippet, and the opcodes that go in them.
static constexpr uint16_t RAM_FILL_START = 1u;
static constexpr uint16_t RAM_FILL_ADD = 5u;
static constexpr uint16_t RAM_FILL_PAGE = 6u;
static constexpr uint16_t RAM_FILL_STORE = 7u;
static constexpr uint16_t RAM_FILL_END = 10u;
static constexpr uint8_t OP_ADDA_IMM = 0x8Bu;
static constexpr uint8_t OP_ADDB_IMM = 0xCBu;
static constexpr uint8_t OP_STA_IDX = 0xA7u;
static constexpr uint8_t OP_STB_IDX = 0xE7u;

static constexpr struct ram_chip {
	const char *name;
	uint8_t first;
	uint8_t last;
	bool on_board;
} ram_chips[] = {
	{ "RAM0", 0x00u, 0x3Fu, true },
	{ "RAM1", 0x40u, 0x7Fu, true },
	{ "RAM2", 0x80u, 0xBFu, false },
	{ "RAM3", 0xC0u, 0xFFu, false },	// includes page $FF, the fixed window
};
static constexpr uint8_t RAM_CHIPS = sizeof ram_chips / sizeof ram_chips[0];

enum ram_page_state : uint8_t {
	RAM_GOOD,
	RAM_BAD,
	RAM_ALIASED,	// holds exactly what was written to another page
	RAM_SILENT,	// nothing answering
	RAM_SKIPPED,	// its chip failed the data or address lines
	RAM_STATES
};

enum ram_stage : uint8_t {
	STAGE_GOOD,
	STAGE_BAD,
	STAGE_SILENT,
	STAGE_NOT_RUN,
};

enum ram_fault_kind : uint8_t {
	FAULT_DATA_READS_1,
	FAULT_DATA_READS_0,
	FAULT_DATA_SHORT,
	FAULT_DATA_OTHER,
	FAULT_ADDR_STUCK,		// the line's marker and the base share a cell
	FAULT_ADDR_MAYBE_SHORTED,	// more than one did: they may be shorted instead
	FAULT_ADDR_SHORT,
};

struct ram_fault {
	ram_fault_kind kind;
	uint8_t a;
	uint8_t b;
};

struct ram_faults {
	ram_fault list[16];
	uint8_t count;
};

static uint16_t ram_retries;		// snippet runs that failed and were tried again
static uint16_t ram_transients;		// mismatches that went away when read again
static uint16_t ram_write_buffer_right;	// mismatches whose write buffer held the right value
static uint16_t ram_recovered;		// checks that passed once the page was filled again
static int16_t ram_window_page = -1;	// what slot 0 maps now, or -1 when not known
static uint64_t ram_started_us;
static uint64_t ram_progress_due_us;
static uint8_t ram_sig_page[256];	// the page each signature belongs to

// Run a snippet operation, trying again when the processor does not reach
// SYNC.  A lost run says nothing about the memory, so it is counted and
// reported rather than ending the test; only repeated failure ends it.
template <typename F>
static bool
ram_try(F &&op)
{
	for (uint8_t attempt = 0u; attempt < RAM_ATTEMPTS; attempt++) {
		if (op())
			return true;
		ram_retries++;
	}
	return false;
}

// A line every half minute or so while the long passes run, so the test is
// seen to be alive without the console filling up.
static void
ram_progress(const char *doing, const unsigned page)
{
	const uint64_t now = time_us_64();
	if (now < ram_progress_due_us)
		return;
	ram_progress_due_us = now + RAM_PROGRESS_US;
	const unsigned secs = static_cast<unsigned>((now - ram_started_us) / 1000000u);
	printf("    ... %s page %02X, %u:%02u in\n", doing, page, secs / 60u, secs % 60u);
}

// Each page's signature, added to the address byte by the storage passes: a
// fixed scramble of the page number, a permutation because the multiplier is
// odd.  With the plain page number, pages n and n+2^k differed by one bit in
// every byte, so any fault forcing that bit read exactly like the other
// page's data.
static uint8_t
ram_sig(const uint8_t page)
{
	return static_cast<uint8_t>(page * 0x9Du + 0x5Bu);
}

// The value a storage pass leaves at an address: its low or high byte, plus
// the page's signature.
static uint8_t
ram_pattern(const uint32_t address, const bool high, const uint8_t page)
{
	return static_cast<uint8_t>((high ? (address >> 8) : address) + ram_sig(page));
}

// Map a page into slot 0 unless it is there already.
static bool
ram_select(const uint8_t page)
{
	if (ram_window_page == page)
		return true;
	ram_window_page = -1;
	if (!ram_try([page] { return copy(OUTWARDS, 1u, RAM_DAT_BASE, &page); }))
		return false;
	ram_window_page = page;
	return true;
}

static bool
ram_poke(const uint8_t page, const uint16_t offset, const uint8_t value)
{
	return ram_select(page)
	    && ram_try([&] { return copy(OUTWARDS, 1u, static_cast<uint16_t>(RAM_WINDOW + offset), &value); });
}

static bool
ram_peek(const uint8_t page, const uint16_t offset, uint8_t &value)
{
	return ram_select(page)
	    && ram_try([&] { return copy(INWARDS, 1u, static_cast<uint16_t>(RAM_WINDOW + offset), &value); });
}

static void
ram_fault_add(ram_faults &f, const ram_fault_kind kind, const uint8_t a, const uint8_t b)
{
	for (uint8_t i = 0u; i < f.count; i++)
		if (f.list[i].kind == kind
		    && ((f.list[i].a == a && f.list[i].b == b) || (f.list[i].a == b && f.list[i].b == a)))
			return;
	if (f.count < sizeof f.list / sizeof f.list[0])
		f.list[f.count++] = ram_fault{ kind, a, b };
}

// Address lines are numbered 0-18: A0-A12 within a page, then PA13-PA18.
static void
ram_fault_print(const ram_fault &f)
{
	const char *const line_a = f.a < RAM_OFFSET_LINES ? "A" : "PA";
	const char *const line_b = f.b < RAM_OFFSET_LINES ? "A" : "PA";

	switch (f.kind) {
	case FAULT_DATA_READS_1:
		printf("      D%u reads 1\n", f.a);
		break;
	case FAULT_DATA_READS_0:
		printf("      D%u reads 0\n", f.a);
		break;
	case FAULT_DATA_SHORT:
		printf("      D%u and D%u shorted together\n", f.a, f.b);
		break;
	case FAULT_DATA_OTHER:
		printf("      wrote %02X, read %02X\n", f.a, f.b);
		break;
	case FAULT_ADDR_STUCK:
		printf("      %s%u stuck or open\n", line_a, f.a);
		break;
	case FAULT_ADDR_MAYBE_SHORTED:
		printf("      (the lines above may instead be shorted to one another)\n");
		break;
	case FAULT_ADDR_SHORT:
		printf("      %s%u and %s%u shorted together\n", line_a, f.a, line_b, f.b);
		break;
	}
}

// Stage 1, the data lines: walking ones and walking zeros written to and read
// back from one address, so nothing here depends on an address line.  Every
// read returning the same value is nothing answering.  Otherwise a bit that
// never changes is a line stuck at that level, and a walking bit that drags
// another bit with it is a pair of lines shorted together.  False only when
// the processor could not run a copy.
static bool
ram_test_data_lines(const ram_chip &c, ram_faults &faults, ram_stage &result)
{
	uint8_t wrote[2u * RAM_DATA_BITS];
	uint8_t seen[2u * RAM_DATA_BITS];

	for (uint8_t i = 0u; i < RAM_DATA_BITS; i++) {
		wrote[i] = static_cast<uint8_t>(1u << i);
		wrote[RAM_DATA_BITS + i] = static_cast<uint8_t>(~(1u << i));
	}
	bool one_value = true;
	uint8_t always_set = 0xFFu;
	uint8_t always_clear = 0xFFu;
	for (uint8_t i = 0u; i < 2u * RAM_DATA_BITS; i++) {
		if (!ram_poke(c.first, 0u, wrote[i]) || !ram_peek(c.first, 0u, seen[i]))
			return false;
		one_value = one_value && seen[i] == seen[0];
		always_set &= seen[i];
		always_clear &= static_cast<uint8_t>(~seen[i]);
	}
	if (one_value) {
		result = STAGE_SILENT;
		return true;
	}

	const uint8_t before = faults.count;
	const uint8_t stuck = always_set | always_clear;
	for (uint8_t bit = 0u; bit < RAM_DATA_BITS; bit++) {
		if (always_set & (1u << bit))
			ram_fault_add(faults, FAULT_DATA_READS_1, bit, 0u);
		if (always_clear & (1u << bit))
			ram_fault_add(faults, FAULT_DATA_READS_0, bit, 0u);
	}
	bool explained = true;
	for (uint8_t i = 0u; i < 2u * RAM_DATA_BITS; i++) {
		const uint8_t bit = static_cast<uint8_t>(i % RAM_DATA_BITS);
		const uint8_t diff = static_cast<uint8_t>((seen[i] ^ wrote[i]) & ~stuck);
		const uint8_t partners = static_cast<uint8_t>(diff & ~(1u << bit));
		for (uint8_t other = 0u; other < RAM_DATA_BITS; other++)
			if (partners & (1u << other))
				ram_fault_add(faults, FAULT_DATA_SHORT, bit, other);
		if (diff != 0u && partners == 0u)
			explained = false;
	}
	if (!explained && faults.count == before)
		for (uint8_t i = 0u; i < 2u * RAM_DATA_BITS; i++)
			if (seen[i] != wrote[i]) {
				ram_fault_add(faults, FAULT_DATA_OTHER, wrote[i], seen[i]);
				break;
			}
	result = faults.count == before ? STAGE_GOOD : STAGE_BAD;
	return true;
}

// Stage 2, the address lines, run once the data lines are good.  A marker
// goes at offset 0 of the chip's first page, at each power-of-two offset in
// that page - one for each of A0-A12 - and at the start of each power-of-two
// page within the chip - one for each of PA13-PA18.
//
// The complement written at the base and turning up at a line's location
// means the two share a cell: that line is stuck or open.  Which way it is
// stuck cannot be told from here - forced high or forced low, the base and
// the line's location land on the same cell either way - and it does not
// matter for finding it.  Two lines pulled low by a short to each other both
// land on the base too, so when more than one line does, a short is noted as
// the other possibility.  Once no line shares the base, the complement
// written at one line's location and turning up at another's means those two
// lines are shorted, pulling each other high.  In the manner of Barr's
// address-bus test.
static bool
ram_test_address_lines(const ram_chip &c, ram_faults &faults, ram_stage &result)
{
	struct {
		uint8_t page;
		uint16_t offset;
	} loc[1u + RAM_OFFSET_LINES + RAM_PAGE_LINES];
	static constexpr uint8_t locations = sizeof loc / sizeof loc[0];
	uint8_t v = 0u;

	loc[0] = { c.first, 0u };
	for (uint8_t i = 0u; i < RAM_OFFSET_LINES; i++)
		loc[1u + i] = { c.first, static_cast<uint16_t>(1u << i) };
	for (uint8_t j = 0u; j < RAM_PAGE_LINES; j++)
		loc[1u + RAM_OFFSET_LINES + j] = { static_cast<uint8_t>(c.first + (1u << j)), 0u };

	const uint8_t before = faults.count;
	for (const auto &l : loc)
		if (!ram_poke(l.page, l.offset, RAM_MARKER))
			return false;

	if (!ram_poke(loc[0].page, loc[0].offset, RAM_ANTI_MARKER))
		return false;
	uint8_t sharing_base = 0u;
	for (uint8_t k = 1u; k < locations; k++) {
		if (!ram_peek(loc[k].page, loc[k].offset, v))
			return false;
		if (v == RAM_ANTI_MARKER) {
			ram_fault_add(faults, FAULT_ADDR_STUCK, static_cast<uint8_t>(k - 1u), 0u);
			sharing_base++;
		}
	}
	if (sharing_base > 1u)
		ram_fault_add(faults, FAULT_ADDR_MAYBE_SHORTED, 0u, 0u);
	if (!ram_poke(loc[0].page, loc[0].offset, RAM_MARKER))
		return false;

	// A line sharing the base already moves every write that crosses it, so
	// the pairwise checks would only restate it.
	if (faults.count == before) {
		for (uint8_t k = 1u; k < locations; k++) {
			if (!ram_poke(loc[k].page, loc[k].offset, RAM_ANTI_MARKER))
				return false;
			for (uint8_t j = 1u; j < locations; j++) {
				if (j == k)
					continue;
				if (!ram_peek(loc[j].page, loc[j].offset, v))
					return false;
				if (v != RAM_MARKER)
					ram_fault_add(faults, FAULT_ADDR_SHORT,
						      static_cast<uint8_t>(k - 1u), static_cast<uint8_t>(j - 1u));
			}
			if (!ram_poke(loc[k].page, loc[k].offset, RAM_MARKER))
				return false;
		}
	}
	result = faults.count == before ? STAGE_GOOD : STAGE_BAD;
	return true;
}

static bool
ram_fill(const uint16_t start, const uint16_t end, const bool high, const uint8_t page)
{
	return ram_try([=] {
		snippet_copy(RAM_FILL);
		reg[REGISTER_SNIPPET_OFFSET + RAM_FILL_START] = start;
		reg[REGISTER_SNIPPET_OFFSET + RAM_FILL_ADD] = high ? OP_ADDA_IMM : OP_ADDB_IMM;
		reg[REGISTER_SNIPPET_OFFSET + RAM_FILL_PAGE] = ram_sig(page);
		reg[REGISTER_SNIPPET_OFFSET + RAM_FILL_STORE] = high ? OP_STA_IDX : OP_STB_IDX;
		reg[REGISTER_SNIPPET_OFFSET + RAM_FILL_END] = end;
		return MC6809.start_with_timeout(1000u);
	});
}

struct ram_finding {
	ram_page_state state;
	uint16_t offset;	// the first wrong byte, from the start of the page
	uint8_t wrote;
	uint8_t seen;
	uint8_t other_page;	// RAM_ALIASED: the page whose data it holds
};

// Check [start, end) against the pattern, sixteen bytes at a time, stopping
// at the first block with a mismatch.  Only the low-byte pass can say more
// than bad, since only its pattern changes from byte to byte.  A block that
// reads one value throughout is nothing answering - the bus floating - even
// when that value happens to equal one of the pattern's bytes; a working chip
// cannot return a constant here.  A block wrong throughout by one constant
// holds another page's pattern, and names it; wrong throughout any other way,
// it matches no page at all.  False only when the processor could not run
// the copy.
static bool
ram_check(const uint16_t start, const uint16_t end, const bool high, const uint8_t page,
	  ram_finding &f)
{
	uint8_t buf[16];

	f = ram_finding{ RAM_GOOD, 0u, 0u, 0u, 0u };
	for (uint32_t a = start; a < end; a += sizeof buf) {
		const uint16_t n = static_cast<uint16_t>(MIN(sizeof buf, end - a));
		uint16_t wrong = 0u;
		bool one_delta = true;
		bool one_value = true;
		uint8_t delta = 0u;
		// A mismatch is read again before it is believed.  One that goes
		// away was the read, not the memory, and is only counted - along
		// with whether the Pico's write buffer, which DMA fills directly,
		// had the right bytes all along.
		for (uint8_t attempt = 0u; attempt < RAM_CONFIRM_READS; attempt++) {
			if (!ram_try([&] { return copy(INWARDS, n, static_cast<uint16_t>(a), buf); }))
				return false;
			delta = static_cast<uint8_t>(buf[0] - ram_pattern(a, high, page));
			wrong = 0u;
			one_delta = true;
			one_value = true;
			bool write_buffer_right = true;
			uint8_t written[16];
			registers::copy_out_write(written, REGISTER_BUFFER_OFFSET, n);
			for (uint16_t i = 0u; i < n; i++) {
				const uint8_t want = ram_pattern(a + i, high, page);
				one_delta = one_delta && static_cast<uint8_t>(buf[i] - want) == delta;
				one_value = one_value && buf[i] == buf[0];
				write_buffer_right = write_buffer_right && written[i] == want;
				if (buf[i] != want && wrong++ == 0u) {
					f.offset = static_cast<uint16_t>(a + i - start);
					f.wrote = want;
					f.seen = buf[i];
				}
			}
			if (wrong == 0u) {
				if (attempt != 0u)
					ram_transients++;
				break;
			}
			if (write_buffer_right)
				ram_write_buffer_right++;
		}
		if (wrong == 0u)
			continue;
		f.state = RAM_BAD;
		if (!high) {
			if (one_value) {
				f.state = RAM_SILENT;
			} else if (wrong == n) {
				if (one_delta) {
					f.state = RAM_ALIASED;
					f.other_page = ram_sig_page[static_cast<uint8_t>(ram_sig(page) + delta)];
				} else {
					f.state = RAM_SILENT;
				}
			}
		}
		return true;
	}
	return true;
}

// Consecutive pages that fail the same way, reported as one line.
struct ram_run {
	ram_page_state state;
	uint16_t first;
	uint16_t last;
	uint8_t delta;		// RAM_ALIASED: the other page, less this one
};

static void
ram_run_flush(ram_run &run)
{
	if (run.state == RAM_ALIASED || run.state == RAM_SILENT) {
		const bool one = run.first == run.last;
		if (one)
			printf("    page %02X", run.first);
		else
			printf("    pages %02X-%02X", run.first, run.last);
		if (run.state == RAM_SILENT)
			printf(": nothing answering sensibly\n");
		else if (one)
			printf(": holds the data written to page %02X\n",
			       static_cast<uint8_t>(run.first + run.delta));
		else
			printf(": hold the data written to pages %02X-%02X\n",
			       static_cast<uint8_t>(run.first + run.delta),
			       static_cast<uint8_t>(run.last + run.delta));
	}
	run.state = RAM_GOOD;
}

// Stage 3, one storage pass over every page whose chip passed stages 1 and 2.
// Every page is written before any is read back, so a page answering at
// another page's address shows up as that page's values rather than passing.
// Pages already found wrong are left out; the high-byte pass exists for
// A8-A12, which the low byte cannot see.
static bool
ram_pass(const bool high, ram_page_state state[256], uint16_t &reported)
{
	printf("  %s storage pass: each address's %s byte plus its page's signature\n",
	       high ? "second" : "first", high ? "high" : "low");
	for (uint16_t page = 0u; page <= RAM_LAST_DAT_PAGE; page++) {
		if (state[page] != RAM_GOOD)
			continue;
		ram_progress("filling", page);
		if (!ram_select(static_cast<uint8_t>(page))) {
			printf("  aborted mapping page %02X for the fill\n", page);
			return false;
		}
		if (!ram_fill(RAM_WINDOW, RAM_WINDOW_END, high, static_cast<uint8_t>(page))) {
			printf("  aborted filling page %02X\n", page);
			return false;
		}
	}
	if (state[RAM_FIXED_PAGE] == RAM_GOOD
	    && !ram_fill(RAM_FIXED_START, RAM_FIXED_END, high, RAM_FIXED_PAGE)) {
		printf("  aborted filling the fixed window\n");
		return false;
	}

	ram_run run{ RAM_GOOD, 0u, 0u, 0u };
	for (uint16_t page = 0u; page <= RAM_FIXED_PAGE; page++) {
		if (state[page] != RAM_GOOD)
			continue;
		ram_progress("checking", page);
		const bool fixed = page == RAM_FIXED_PAGE;
		if (!fixed && !ram_select(static_cast<uint8_t>(page))) {
			ram_run_flush(run);
			printf("  aborted mapping page %02X for the check\n", page);
			return false;
		}
		ram_finding f{};
		if (!ram_check(fixed ? RAM_FIXED_START : RAM_WINDOW, fixed ? RAM_FIXED_END : RAM_WINDOW_END,
			       high, static_cast<uint8_t>(page), f)) {
			ram_run_flush(run);
			printf("  aborted checking page %02X\n", page);
			return false;
		}
		state[page] = f.state;
		if (f.state == RAM_ALIASED || f.state == RAM_SILENT) {
			const uint8_t delta = static_cast<uint8_t>(f.other_page - page);
			if (run.state == f.state && page == run.last + 1u
			    && (f.state != RAM_ALIASED || delta == run.delta)) {
				run.last = page;
			} else {
				ram_run_flush(run);
				run = ram_run{ f.state, page, page, delta };
			}
			continue;
		}
		ram_run_flush(run);
		if (f.state == RAM_BAD && reported++ < RAM_REPORT_LIMIT)
			printf("    page %02X offset %04X: wrote %02X, read %02X\n",
			       page, f.offset, f.wrote, f.seen);
	}
	ram_run_flush(run);

	// Memory can be overwritten after the pass wrote it without the cell
	// being faulty - by a lost snippet run, whose processor runs wild until
	// the Pico resets it, or by stray writes while it is parked in SYNC with
	// its buses released.  So each plain bad page is filled and checked once
	// more on its own: an overwritten page passes, a faulty one fails again.
	// Pages holding another page's data, or answering nothing, are not
	// retried - a stray write does not produce those.
	for (uint16_t page = 0u; page <= RAM_FIXED_PAGE; page++) {
		if (state[page] != RAM_BAD)
			continue;
		const bool fixed = page == RAM_FIXED_PAGE;
		const uint16_t start = fixed ? RAM_FIXED_START : RAM_WINDOW;
		const uint16_t end = fixed ? RAM_FIXED_END : RAM_WINDOW_END;
		if ((!fixed && !ram_select(static_cast<uint8_t>(page)))
		    || !ram_fill(start, end, high, static_cast<uint8_t>(page))) {
			printf("  aborted filling page %02X again\n", page);
			return false;
		}
		ram_finding again{};
		if (!ram_check(start, end, high, static_cast<uint8_t>(page), again)) {
			printf("  aborted checking page %02X again\n", page);
			return false;
		}
		if (again.state == RAM_GOOD) {
			state[page] = RAM_GOOD;
			ram_recovered++;
			printf("    page %02X: good when filled and checked again\n", page);
		}
	}
	return true;
}

// The DAT RAM itself, all 256 entries, written and read back in both
// polarities.  Only slot 7's addresses are touched while it holds these,
// so entries that would fault or map nothing are harmless - with one
// exception, which sets the order.  A processor parked in SYNC floats its
// address bus towards $0000, and the glue decodes that as a slot 0 access
// through task zero's entry 0; left holding $FF, that entry would raise a
// DAT fault on every park.  So the inverted pass goes first and the
// straight pass last, leaving entry 0 mapping page 0.
static bool
ram_test_dat_storage(bool &good)
{
	uint8_t wrote[RAM_DAT_ENTRIES];
	uint8_t seen[RAM_DAT_ENTRIES];
	uint16_t reported = 0u;

	good = true;
	for (uint8_t pass = 0u; pass < 2u; pass++) {
		const bool invert = pass == 0u;
		for (uint16_t e = 0u; e < RAM_DAT_ENTRIES; e++)
			wrote[e] = static_cast<uint8_t>(invert ? ~e : e);
		if (!ram_try([&] { return copy(OUTWARDS, RAM_DAT_ENTRIES, RAM_DAT_BASE, wrote); })
		    || !ram_try([&] { return copy(INWARDS, RAM_DAT_ENTRIES, RAM_DAT_BASE, seen); }))
			return false;
		for (uint16_t e = 0u; e < RAM_DAT_ENTRIES; e++) {
			if (seen[e] == wrote[e])
				continue;
			good = false;
			if (reported++ < RAM_REPORT_LIMIT)
				printf("    DAT entry $%02X (task %u slot %u): wrote %02X, read %02X\n",
				       e, e >> 3, e & 7u, wrote[e], seen[e]);
		}
	}
	ram_window_page = -1;
	return true;
}

// Translation through every task's row and every remappable slot.  Each
// entry names a page that differs from the entry one DAT address line away -
// bit 7 folds in $55 - and the second storage pass left each page's first
// bytes holding its signature, so a read through the wrong entry, or with a
// task or A13-A15 line wrong, reaches a page holding the wrong signature.
// Uses only pages $00-$7F, the ones on the board.
static bool
ram_test_translation(bool &good)
{
	uint8_t map[RAM_DAT_ENTRIES];
	uint8_t buf[16];
	uint16_t reported = 0u;

	for (uint16_t e = 0u; e < RAM_DAT_ENTRIES; e++)
		map[e] = static_cast<uint8_t>((e & 0x7Fu) ^ ((e & 0x80u) ? 0x55u : 0x00u));
	MC6809.stop();
	task_pins_update(MC6809.task_change(0u));
	ram_window_page = -1;
	if (!ram_try([&] { return copy(OUTWARDS, RAM_DAT_ENTRIES, RAM_DAT_BASE, map); }))
		return false;

	good = true;
	for (uint8_t task = 0u; task < RAM_TASKS; task++) {
		MC6809.stop();
		task_pins_update(MC6809.task_change(task));
		for (uint8_t slot = 0u; slot < RAM_SLOTS; slot++) {
			const uint8_t page = map[task * 8u + slot];
			const uint8_t want = ram_sig(page);
			int16_t wrong_value = -1;
			// Read again before believing a mismatch, as the storage check does.
			for (uint8_t attempt = 0u; attempt < RAM_CONFIRM_READS; attempt++) {
				if (!ram_try([&] { return copy(INWARDS, sizeof buf, static_cast<uint16_t>(slot << 13), buf); }))
					return false;
				wrong_value = -1;
				for (const uint8_t b : buf)
					if (b != want) {
						wrong_value = b;
						break;
					}
				if (wrong_value < 0) {
					if (attempt != 0u)
						ram_transients++;
					break;
				}
			}
			if (wrong_value < 0)
				continue;
			// A runaway write may have trampled the page since the storage
			// pass left its signature there.  Fill it again from task zero,
			// put entry 0 back, and read it once more through this task's
			// entry before believing the translation is wrong.
			MC6809.stop();
			task_pins_update(MC6809.task_change(0u));
			ram_window_page = -1;
			if (!ram_select(page) || !ram_fill(RAM_WINDOW, RAM_WINDOW_END, true, page)
			    || !ram_try([&] { return copy(OUTWARDS, 1u, RAM_DAT_BASE, &map[0]); }))
				return false;
			ram_window_page = -1;
			MC6809.stop();
			task_pins_update(MC6809.task_change(task));
			if (!ram_try([&] { return copy(INWARDS, sizeof buf, static_cast<uint16_t>(slot << 13), buf); }))
				return false;
			wrong_value = -1;
			for (const uint8_t b : buf)
				if (b != want) {
					wrong_value = b;
					break;
				}
			if (wrong_value < 0) {
				ram_recovered++;
				printf("    task %u slot %u: good once page %02X was filled again\n", task, slot, page);
				continue;
			}
			good = false;
			if (reported++ < RAM_REPORT_LIMIT)
				printf("    task %u slot %u: expected page %02X, reached page %02X\n",
				       task, slot, page, ram_sig_page[static_cast<uint8_t>(wrong_value)]);
		}
	}
	MC6809.stop();
	task_pins_update(MC6809.task_change(0u));
	return true;
}

// Report each chip select's storage.  A chip that failed its data or address
// lines was not given a storage test and is reported as such; the board's
// two chips must be good throughout; an expansion chip may be good or absent,
// but one answering for some pages and not others is broken.
static bool
ram_summary(const ram_page_state state[256], const ram_stage data[RAM_CHIPS],
	    const ram_stage address[RAM_CHIPS], bool &on_board_good)
{
	bool good = true;

	on_board_good = true;
	for (uint8_t i = 0u; i < RAM_CHIPS; i++) {
		const ram_chip &c = ram_chips[i];
		printf("  %s pages %02X-%02X: ", c.name, c.first, c.last);
		bool chip_good = true;
		if (data[i] == STAGE_SILENT) {
			printf("nothing answering%s\n", c.on_board ? "" : " - nothing fitted?");
			chip_good = !c.on_board;
		} else if (data[i] != STAGE_GOOD || address[i] != STAGE_GOOD) {
			printf("%s lines faulty, storage not tested\n", data[i] != STAGE_GOOD ? "data" : "address");
			chip_good = false;
		} else {
			uint16_t counts[RAM_STATES] = { 0u, 0u, 0u, 0u, 0u };
			for (uint16_t page = c.first; page <= c.last; page++)
				counts[state[page]]++;
			const uint16_t pages = static_cast<uint16_t>(c.last - c.first + 1u);
			if (counts[RAM_GOOD] == pages) {
				printf("all %u good\n", pages);
			} else {
				printf("%u good, %u bad, %u holding another page's data, %u not answering\n",
				       counts[RAM_GOOD], counts[RAM_BAD], counts[RAM_ALIASED], counts[RAM_SILENT]);
				chip_good = false;
			}
		}
		if (!chip_good) {
			good = false;
			if (c.on_board)
				on_board_good = false;
		}
	}
	return good;
}

static bool
ram_test()
{
	static ram_page_state state[256];
	ram_stage data[RAM_CHIPS];
	ram_stage address[RAM_CHIPS];
	uint16_t reported = 0u;
	bool good = true;
	bool part_good = true;

	ram_started_us = time_us_64();
	ram_progress_due_us = ram_started_us + RAM_PROGRESS_US;
	ram_retries = 0u;
	ram_transients = 0u;
	ram_write_buffer_right = 0u;
	ram_recovered = 0u;
	copy_buffer_disagreements = 0u;
	ram_window_page = -1;
	for (uint16_t p = 0u; p < 256u; p++)
		ram_sig_page[ram_sig(static_cast<uint8_t>(p))] = static_cast<uint8_t>(p);
	MC6809.stop();
	task_pins_update(MC6809.task_change(0u));

	printf("  DAT RAM, all 256 entries both ways up\n");
	if (!ram_test_dat_storage(part_good))
		return false;
	good = good && part_good;

	// Stages 1 and 2 for each chip select, before any storage is trusted.
	// A stage that finds a fault is run a second time, and only faults the
	// second run finds too are believed.
	printf("  data and address lines, chip select by chip select\n");
	for (uint8_t i = 0u; i < RAM_CHIPS; i++) {
		const ram_chip &c = ram_chips[i];
		ram_faults faults{};
		address[i] = STAGE_NOT_RUN;
		if (!ram_test_data_lines(c, faults, data[i])) {
			printf("  aborted testing %s's data lines\n", c.name);
			return false;
		}
		if (data[i] == STAGE_BAD) {
			ram_faults again{};
			if (!ram_test_data_lines(c, again, data[i])) {
				printf("  aborted testing %s's data lines\n", c.name);
				return false;
			}
			if (data[i] != STAGE_BAD)
				ram_transients++;
			faults = again;
		}
		if (data[i] == STAGE_GOOD) {
			if (!ram_test_address_lines(c, faults, address[i])) {
				printf("  aborted testing %s's address lines\n", c.name);
				return false;
			}
			if (address[i] == STAGE_BAD) {
				ram_faults again{};
				if (!ram_test_address_lines(c, again, address[i])) {
					printf("  aborted testing %s's address lines\n", c.name);
					return false;
				}
				if (address[i] != STAGE_BAD)
					ram_transients++;
				faults = again;
			}
		}
		printf("  %s pages %02X-%02X: ", c.name, c.first, c.last);
		if (data[i] == STAGE_SILENT)
			printf("nothing answering%s\n", c.on_board ? "" : " - nothing fitted?");
		else if (data[i] == STAGE_BAD)
			printf("data lines faulty - address and storage tests skipped\n");
		else if (address[i] == STAGE_BAD)
			printf("data lines good, address lines faulty - storage test skipped\n");
		else
			printf("data lines good, address lines good\n");
		for (uint8_t f = 0u; f < faults.count; f++)
			ram_fault_print(faults.list[f]);

		const ram_page_state initial = data[i] == STAGE_SILENT ? RAM_SILENT
			: (data[i] == STAGE_GOOD && address[i] == STAGE_GOOD) ? RAM_GOOD : RAM_SKIPPED;
		for (uint16_t page = c.first; page <= c.last; page++)
			state[page] = initial;
	}

	// Stage 3.  A summary after each pass, so that a later abort cannot hide
	// what the first pass had already found.
	bool on_board_good = true;
	for (uint8_t pass = 0u; pass < 2u; pass++) {
		if (!ram_pass(pass == 1u, state, reported))
			return false;
		const bool summary_good = ram_summary(state, data, address, on_board_good);
		if (pass == 1u)
			good = summary_good && good;
	}

	if (on_board_good) {
		printf("  DAT translation, %u tasks through %u slots each\n", RAM_TASKS, RAM_SLOTS);
		if (!ram_test_translation(part_good))
			return false;
		good = good && part_good;
	} else {
		printf("  DAT translation not tested - it needs pages 00-7F good\n");
		good = false;
	}
	return good;
}

// The synctest command: how often a snippet run fails to finish.  Three
// phases of the same number of runs: the SYNC snippet alone, touching nothing
// outside the Pico's window; sixteen-byte copies from on-board page $00; and
// the same from expansion page $C0, when anything answers there.  A copy that
// never reaches SYNC is counted apart from one that reaches it with the wrong
// bytes, so the processor misreading its instructions can be told from it
// misreading RAM.  A lost run may have trampled the bytes being copied, so
// they are written again after one.
static constexpr uint8_t SYNCTEST_ON_BOARD_PAGE = 0x00u;
static constexpr uint8_t SYNCTEST_EXPANSION_PAGE = 0xC0u;

static void
synctest_report(const char *what, const uint32_t runs, const uint32_t lost, const uint32_t wrong)
{
	printf("  %-24s %" PRIu32 " runs: %" PRIu32 " did not finish, %" PRIu32 " finished with wrong bytes\n",
	       what, runs, lost, wrong);
}

// One copy of sixteen bytes from the start of slot 0, quietly - copy() would
// print on every failure, and failures are what is being counted.
static bool
synctest_copy_in(uint8_t buf[REGISTER_BUFFER_LEN])
{
	snippet_copy(COPY_IN);
	reg[REGISTER_SNIPPET_OFFSET + 4u] = RAM_WINDOW;
	reg[REGISTER_SNIPPET_OFFSET + 7u] = static_cast<uint8_t>(REGISTER_BUFFER_LEN);
	if (!MC6809.start_with_timeout(4u))
		return false;
	registers::copy_out(buf, REGISTER_BUFFER_OFFSET, REGISTER_BUFFER_LEN);
	return true;
}

static bool
synctest_place(const uint8_t page, const uint8_t pattern[REGISTER_BUFFER_LEN])
{
	MC6809.stop();
	task_pins_update(MC6809.task_change(0u));
	return ram_try([&] { return copy(OUTWARDS, 1u, RAM_DAT_BASE, &page); })
	    && ram_try([&] { return copy(OUTWARDS, REGISTER_BUFFER_LEN, RAM_WINDOW, pattern); });
}

static void
synctest_copies(const char *what, const uint8_t page, const uint32_t runs,
		const uint8_t pattern[REGISTER_BUFFER_LEN])
{
	uint8_t buf[REGISTER_BUFFER_LEN];

	if (!synctest_place(page, pattern)) {
		printf("  %-24s could not place the pattern - skipped\n", what);
		return;
	}
	if (synctest_copy_in(buf) && std::memcmp(buf, pattern, sizeof buf) != 0) {
		bool one_value = true;
		for (const uint8_t b : buf)
			one_value = one_value && b == buf[0];
		printf("  %-24s %s - skipped\n", what,
		       one_value ? "nothing answering" : "does not hold the pattern");
		return;
	}
	uint32_t lost = 0u;
	uint32_t wrong = 0u;
	for (uint32_t n = 0u; n < runs; n++) {
		if (!synctest_copy_in(buf)) {
			lost++;
			(void) synctest_place(page, pattern);
		} else if (std::memcmp(buf, pattern, sizeof buf) != 0) {
			wrong++;
			(void) synctest_place(page, pattern);
		}
	}
	synctest_report(what, runs, lost, wrong);
}

static void
synctest(const uint32_t runs)
{
	uint8_t pattern[REGISTER_BUFFER_LEN];

	// Each bit takes both values somewhere across the sixteen bytes.
	for (uint8_t i = 0u; i < sizeof pattern; i++)
		pattern[i] = static_cast<uint8_t>(0xA5u ^ (i * 0x11u));
	printf("Snippet reliability at E = %.1f MHz, %" PRIu32 " runs a phase\n",
	       static_cast<double>(MC6809.get_e_frequency()), runs);

	uint32_t lost = 0u;
	for (uint32_t n = 0u; n < runs; n++) {
		snippet_copy(SYNC);
		if (!MC6809.start_with_timeout(4u))
			lost++;
	}
	synctest_report("SYNC snippet alone", runs, lost, 0u);
	synctest_copies("copies from page 00", SYNCTEST_ON_BOARD_PAGE, runs, pattern);
	synctest_copies("copies from page C0", SYNCTEST_EXPANSION_PAGE, runs, pattern);
}

/* cmd_parse() - glue between process_command() and the flex/bison parser */
bool
cmd_parse(const char *input, parsed_cmd_t *out)
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
// The guest's own NMI vector, kept while a break points that vector at the
// BREAK snippet.  Without it the command leaves the vector redirected for
// good - on a timeout, and after "break return" - so the guest's next NMI
// lands in a snippet that copies twelve bytes and spins instead of in its own
// handler, and a DAT fault would go there too.  The watchpoint has always
// saved and restored this across arm and disarm; break never did.
static uint16_t break_saved_nmi_vector = 0u;

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

// Raw ADC counts to engineering units.  Shared by the human-readable and the
// JSON status output so the constants cannot drift apart.
//
// Both conversions turn on the reference the ADC measures against, which is
// ADC_AVDD - nominally 3.3V, in practice whatever the board delivers.  The
// datasheet is blunt about what that costs: at 3.3V a reading of 891 is
// 20.1C, and a reference one percent lower makes the same reading 24.3C.  A
// reference reading low inflates every voltage and deflates every
// temperature, which is exactly the pair of symptoms a board shows when this
// is left at its nominal value.  Measure ADC_AVDD and set it here.
// Derived rather than probed, 2026-10-01: pin 53 has no test point, so this
// comes from the divider midpoint instead - 2.5014V metered at GPIO 46 against
// a quiet mean of 3158.88 counts gives 2.5014 x 4096 / 3158.88 = 3.2435.  The
// quiet mean is the one to use: halted and unpowered agreed to 0.01 counts
// (3158.89 and 3158.88, SD 1.6 and 1.2), while the guest serving the bus reads
// 3533 with SD 49 - see the serving artefact note.  The previous 3.2464, taken
// on pin 53 in September, left the supply reading 5 mV high; this leaves it
// 0.1 mV high.
static constexpr float ADC_VREF_VOLTS = 3.2435f;	// 2.5014V at GPIO 46 / 3158.88 counts, quiet

// What the divider feeding GPIO 46 actually divides the 5V line by.  It is a
// per-board figure: the first version 2 board measured 1.9407, the replacement
// measures 2.0001, from 5.0005V at the rail against 2.5001V at GPIO 46.  Unlike
// the reference it affects only the supply reading, so the two can be told
// apart by whether the die temperature moves with it.
static constexpr float BUS_DIVIDER_RATIO = 2.0000f;	// 5.0027V rail / 2.5014V pin, 2026-10-01

// The converter's zero error, in counts, subtracted before anything is scaled.
// It is not a per-channel figure: the same offset appears on the supply
// divider and on the die sensor, which is what distinguishes it from a
// reference error - a reference error scales, an offset shifts, and no single
// reference value fits both channels while one offset fits both.
//
// Still zero on the replacement version 2 board, and deliberately so rather
// than by measurement, because one pin voltage cannot separate an offset from
// a reference error: 2.5014V at GPIO 46 should read 3155.9 counts against the
// old reference and reads 3158.88, which fits either a 0.09% lower reference
// or a +3 count offset equally well.  The die channel cannot break the tie
// either - 39.3C one way, 40.3C the other, both plausible.  With nothing to
// choose between them, the reference carries it and this stays at zero, which
// is the fewer moving parts.  Separating them needs a second, different pin
// voltage.
//
// The first board needed 54 counts, turning its 928 into 31.8C against a 25.0C
// ambient, so this is a per-board figure and a real offset can be large.
static constexpr float ADC_OFFSET_COUNTS = 0.0f;	// not distinguishable from the reference at one point

// Counts to the voltage on the pin: a 12-bit conversion against the reference,
// with the converter's own zero error taken out first.
static inline float
adc_pin_volts(const uint16_t raw)
{
	return (static_cast<float>(raw) - ADC_OFFSET_COUNTS)*ADC_VREF_VOLTS/4096.0f;
}

static inline float
adc_rail_volts(const uint16_t raw)
{
	return adc_pin_volts(raw)*BUS_DIVIDER_RATIO;
}

static inline float
// Vbe is 0.706V at 27C with a slope of -1.721mV per degree.  These are the
// RP2350's own figures, not carried over from the RP2040: section 12.4.6 of
// the RP2350 datasheet gives the same two numbers.
adc_die_celsius(const uint16_t raw)
{
	return 27.0f - (adc_pin_volts(raw) - 0.706f)/0.001721f;
}

// --- Supply monitor -----------------------------------------------------
//
// Sampled from a hardware alarm rather than from either core's loop.  The
// console loop stalls for as long as whatever command it is running takes,
// and Core 1's loop is the bus path and has no business blocking on an ADC
// conversion, so neither can promise to look at the rail often enough.  The
// alarm fires on Core 0 whatever those loops are doing.
//
// It is also the only thing that touches the ADC.  There is one input mux,
// so a sample taken anywhere else could be preempted between selecting the
// channel and reading it, and both readings would be wrong.  Everything else
// reads the counts cached here.

// How much external 5V has to be there before the load switch is closed.
// Set from CMake with -DPICO_THING_POWER_RAIL_MIN_VOLTS=<volts>; the fallback
// here is for anything that compiles this without the build system.  A
// nominal 5V supply may sag 10%, so the value sits just above that: high
// enough to reject an absent or failing supply, low enough to accept a
// working one under load.
#ifndef PICO_THING_POWER_RAIL_MIN_VOLTS
#define PICO_THING_POWER_RAIL_MIN_VOLTS 4.5f
#endif
static constexpr float POWER_RAIL_MIN_VOLTS = PICO_THING_POWER_RAIL_MIN_VOLTS;
// It doubles as the "healthy" line for the baseline below: only a sample at
// or above this may seed or move the baseline, so a sag cannot drag its own
// reference down, and a USB-powered start with no +VDC - the Pico runs fine
// on VBUS through D109 - never establishes one to fall from.

// Below this the supply is no longer usable and the load switch is dropped.
// Set from CMake with -DPICO_THING_POWER_RAIL_DROP_VOLTS=<volts>.
#ifndef PICO_THING_POWER_RAIL_DROP_VOLTS
#define PICO_THING_POWER_RAIL_DROP_VOLTS 4.0f
#endif
static constexpr float POWER_RAIL_DROP_VOLTS = PICO_THING_POWER_RAIL_DROP_VOLTS;

// How far below its own slow baseline +5VP has to fall to count as low.  This
// is the PRIMARY detector; POWER_RAIL_DROP_VOLTS above is only the backstop
// for a baseline that never established.  Relative rather than absolute
// because both ADC channels read 0.5 to 0.7V high whenever the Pico serves
// the bus, and that inflation cancels in a difference but corrupts an
// absolute threshold completely.  Set with
// -DPICO_THING_POWER_RAIL_FALL_VOLTS=<volts>.
#ifndef PICO_THING_POWER_RAIL_FALL_VOLTS
#define PICO_THING_POWER_RAIL_FALL_VOLTS 0.40f
#endif
static constexpr float POWER_RAIL_FALL_VOLTS = PICO_THING_POWER_RAIL_FALL_VOLTS;

// Healthy samples needed before a baseline is trusted, and the divisor of the
// running average once it is.  At RAIL_MONITOR_MS that is about a second to
// establish and five seconds of memory - slow enough that a real collapse,
// which is milliseconds, trips long before the baseline can follow it down.
static constexpr uint32_t RAIL_GOOD_SAMPLES = 50u;
static constexpr uint32_t RAIL_BASELINE_DIV = 250u;

// How long the rail is given to charge the board's bulk capacitance before
// the monitor is allowed to judge it.  Set from CMake with
// -DPICO_THING_POWER_RAIL_SETTLE_MS=<milliseconds>.
#ifndef PICO_THING_POWER_RAIL_SETTLE_MS
#define PICO_THING_POWER_RAIL_SETTLE_MS 250u
#endif
static constexpr uint32_t RAIL_SETTLE_MS = PICO_THING_POWER_RAIL_SETTLE_MS;

// How settled Core 1 has to be before the startup commands run: core1_initialised
// is set before its loop starts, but the emulated devices need a few turns of
// that loop to reach a sane state.
static constexpr uint32_t CORE1_START_LOOPS = 100u;

// And how long it is given to get there.  Generous rather than tight: with the
// guest held in reset the bus still runs, and the interrupt load that puts on
// Core 1 has been seen to hold its loop down to tens of turns a second for the
// first couple of seconds.  This is a backstop against a Core 1 that will never
// arrive, not a judgement on one that is merely slow.
static constexpr uint32_t CORE1_START_TIMEOUT_MS = 10000u;

static constexpr uint32_t RAIL_MONITOR_MS = 20u;
// A supply that is genuinely gone stays gone.  Requiring several samples in a
// row rides out the dip a bus cycle or an inrush transient puts on the rail,
// the same way the Core 1 watchdog waits for consecutive misses.
static constexpr uint32_t RAIL_LOW_SAMPLES = 3u;

static repeating_timer rail_timer;
static volatile uint16_t rail_raw = 0u;		// counts, external 5V through the divider
static volatile uint16_t rail_trip_raw = 0u;	// the sample that opened the switch
static volatile uint16_t die_raw = 0u;		// counts, internal temperature sensor
static volatile uint32_t rail_low_count = 0u;	// consecutive samples below the cutout
static volatile float rail_baseline = 0.0f;	// slow average of a healthy +5VP
static volatile uint32_t rail_good_count = 0u;	// consecutive healthy samples seen
static volatile uint64_t rail_settle_until = 0u;	// us since boot; no judging before this
static volatile bool rail_sampled = false;	// at least one pass has run
static volatile bool rail_collapsed = false;	// set here, acted on by the console loop

// These readings are only true while the Pico is not serving the bus.
//
// adcdump on the replacement version 2 board (September 2026) found both
// channels right, to within a count or two, with the guest halted or
// unpowered: 3158 counts on the supply, where the metered 5.0033V rail
// predicts 3158, and 865 on the die, 39C.  Whenever the Pico answers bus
// reads - the guest held in reset, running, or faulting - both read high:
// the supply by 350 to 420 counts, 0.55 to 0.67V at the rail, and the die by
// about 125 counts, which reads nearly 60 degrees colder than it is.  The
// reference was metered at the same time and had not moved.
//
// That is a shift, not scatter, so the median of several samples once planned
// here would not remove it.  Nor is it one converter offset: the channels move
// by different amounts, and by how much depends on how closely the samples
// follow one another.  It behaves like interference from serving the bus,
// some of it reaching the converter inside the chip, and it grew as parts
// were fitted.  The cause is not yet found.
//
// For the cutout this errs in the unsafe direction.  A high reading cannot
// trip it falsely, but it can hide a real sag: while the bus is being served,
// the rail may fall to about 3.4V before the reading drops below 4.0V.
static bool
power_rail_monitor(repeating_timer *)
{
#if PICO_THING_BOARD_VERSION >= 2
	// Every conversion here follows a mux change, and the sample capacitor
	// has to charge to the new channel through whatever impedance drives
	// it.  The first conversion after a switch is thrown away so the second
	// starts from a settled cap.  This matters most for the die sensor: it
	// is a biased diode with a far higher source impedance than a resistive
	// divider, and it is preceded by a channel sitting three times higher.
	adc_select_input(GPIO_POWER_ADC - ADC_BASE_PIN);
	(void) adc_read();
	const uint16_t raw = adc_read();
	rail_raw = raw;
	// Opening the switch is a single gpio_put, which is all that can safely
	// be done from an alarm callback - and it is the part that matters.
	// Stopping the PIOs, the DMA and Core 1 needs the console loop, so that
	// is left to it.
	// The decision itself is rail_judge(), in rail_monitor.h, so that ctest
	// can cover the sags, recoveries, glitches and the settle window
	// without collapsing a real rail - see tests/test_rail_monitor.cpp.
	// The volts are computed unconditionally now rather than only when
	// armed and settled: one float divide every 20 ms, against a decision
	// that is finally testable.
	static constexpr rail_limits rail_cfg = {
		POWER_RAIL_DROP_VOLTS,
		POWER_RAIL_FALL_VOLTS,
		POWER_RAIL_MIN_VOLTS,
		RAIL_LOW_SAMPLES,
		RAIL_GOOD_SAMPLES,
		RAIL_BASELINE_DIV,
	};
	const rail_state rs = {
		POWER_IS_ASSERTED && !rail_collapsed,
		time_us_64(),
		rail_settle_until,
		adc_rail_volts(raw),
		rail_low_count,
		rail_baseline,
		rail_good_count,
	};
	const rail_judgement rj = rail_judge(rs, rail_cfg);
	rail_low_count = rj.low_count;
	rail_baseline = rj.baseline;
	rail_good_count = rj.good_count;
	if (rj.trip) {
		// Keep the sample that decided it.  By the time the console loop
		// reports this the switch is open, the load is gone and the rail
		// has recovered, so a reading taken then says nothing about why.
		rail_trip_raw = raw;
		POWER_DEASSERT;
		rail_collapsed = true;
	}
#endif
	// The die sensor is internal, on the channel above the pins
	adc_select_input(NUM_ADC_CHANNELS - 1);
	(void) adc_read();
	die_raw = adc_read();
	rail_sampled = true;
	return true;
}

static void
power_rail_monitor_start()
{
	add_repeating_timer_ms(static_cast<int32_t>(RAIL_MONITOR_MS),
			       power_rail_monitor, nullptr, &rail_timer);
	while (!rail_sampled)		// so the first power-on has a reading to judge
		tight_loop_contents();
}

// The flow-control state of one ACIA's CDC port.  The UART console has no
// flow control at all and needs none: it is a terminal, and a dropped
// character costs a smudged display.  The CDC ports carry a guest's real
// output and DriveWire's disk traffic, so the rule there is that either the
// port is not in use, or nothing is lost.
static void
status_gather_flow(acia_status_t *a, const uint8_t port)
{
	const uart_port_state &st = uart_port_get(port);

	a->dtr = usb_cdc_connected_n(port);
	a->host_rts = usb_cdc_host_rts_n(port);
	a->honour_host_rts = st.honour_host_rts;
	a->held = st.tx_count > 0u && st.tx_index < st.tx_count;
}

// Where a staged byte's RDRF went.  Three places clear it and the counts tell
// them apart - which is the only way to answer why acia_torture test 3 fails
// while a host is attached and passes while it is not.
static void
status_print_rdrf(const char *name, const mc6850::rdrf_why &w)
{
	printf("    %-7s RDRF: staged %" PRIu32 "  cleared by: guest-read %" PRIu32
	       "  RTS-drop %" PRIu32 "  reset %" PRIu32 "   host-CTS changes %"
	       PRIu32 "\n",
	       name, w.staged, w.rx_read, w.rts_drop, w.reset, w.cts_change);
}

static void
status_print_flow(const char *name, const acia_status_t *a)
{
	printf("    %-7s port: %s", name, a->dtr ? "open" : "CLOSED - output discarded");
	if (a->dtr) {
		printf(", host RTS %s", a->host_rts ? "high" : "LOW");
		if (!a->honour_host_rts)
			printf(" (ignored)");
		else if (!a->host_rts)
			printf(" - output held, guest stalled");
		if (a->held)
			printf(", bytes waiting");
	}
	printf("\n");
}

void
status_gather(status_snapshot_t *s)
{
	s->run_state = MC6809.get_run_state();
	s->bus_state = MC6809.get_bus_state();
	s->old_bus_state = MC6809.get_old_bus_state();
	s->task = MC6809.get_task();
	s->task_write_odd_count = task_write_odd_count;
	s->task_write_odd_from = task_write_odd_from;
	s->task_write_odd_to = task_write_odd_to;
	s->nesting_depth = MC6809.get_nesting_depth();
	s->task_stack_ptr = MC6809.get_task_stack_ptr();
	for (uint16_t i = 0; i < s->task_stack_ptr && i < STATUS_MAX_INT_NEST_DEPTH; i++)
		s->task_history[i] = MC6809.get_task_history(i);

	s->power = POWER_IS_ASSERTED;
	// Both readings come from the supply monitor's cache; nothing else may
	// touch the ADC mux.  Version 1 has the power enable on GPIO 46 rather
	// than a divider, so it has no supply reading to report.
#if PICO_THING_BOARD_VERSION >= 2
	s->voltage = rail_raw;
#else
	s->voltage = 0u;           // no bus voltage divider on version 1
#endif
	s->temperature = die_raw;

	// The raw revision rather than rp2350_chip_version(), whose numbering
	// does not follow the stepping names and which asserts on a foreign ID.
	const uint32_t chip_id = *static_cast<io_ro_32 *>(reinterpret_cast<void *>(SYSINFO_BASE + SYSINFO_CHIP_ID_OFFSET));
	s->chip_revision = static_cast<uint8_t>((chip_id & SYSINFO_CHIP_ID_REVISION_BITS) >> SYSINFO_CHIP_ID_REVISION_LSB);
	s->rom_version = rp2350_rom_version();
	const uint32_t package_sel = *static_cast<io_ro_32 *>(reinterpret_cast<void *>(SYSINFO_BASE + SYSINFO_PACKAGE_SEL_OFFSET));
	s->qfn60 = (package_sel & SYSINFO_PACKAGE_SEL_BITS) != 0u;

	// Measured with the frequency counter rather than asked of the SDK:
	// clock_get_hz() returns what the SDK believes, which is exactly what is
	// wrong when something has moved a PLL under it.  Taken here, in the
	// status snapshot, rather than at boot, so it always reports the tree
	// actually running - a boot-time print cannot survive someone adding a
	// clock change after it.  About a millisecond each.
	s->clk_sys_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
	s->clk_peri_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);
	s->clk_usb_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB);
	s->clk_adc_khz = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC);

	s->reset = RESET_IS_ASSERTED;
	s->halt = HALT_IS_ASSERTED;
	s->nmi = NMI_IS_ASSERTED;
	s->firq = FIRQ_IS_ASSERTED;
	s->irq = IRQ_IS_ASSERTED;

#ifdef DEBUG
	static uint32_t last_LIC = 0u;
	s->lic_count = MC6809.get_lic_count();
	s->rail_baseline = rail_baseline;
	s->rail_good = rail_good_count;
	s->busy_lic_avma = MC6809.get_busy_lic_avma();
	s->mirror_raw = bus_status_mirror.word;
	s->mirror_pc = (bus_status.sm < 0) ? 0xFFu : static_cast<uint8_t>(
		pio_sm_get_pc(bus_status.pio, static_cast<uint>(bus_status.sm))
		- static_cast<uint>(bus_status.offset));
	s->mirror_fifo = (bus_status.sm < 0) ? 0xFFu : static_cast<uint8_t>(
		pio_sm_get_rx_fifo_level(bus_status.pio, static_cast<uint>(bus_status.sm)));
	s->mirror_dma_busy = dma_channel_is_busy(static_cast<uint>(bus_status.data_channel));
	// Nobody consumes pio2 flag 1 while the bus_status SM is parked on its
	// wait, so if the clock's "irq prev set 1" is arriving at all the flag
	// must be LATCHED HIGH here.  Reading it clear means the flag never
	// arrives and the fault is the irq routing, not the state machine.
	s->pio2_irq = static_cast<uint8_t>(bus_status.pio->irq & 0xFFu);
	s->clock_pc = (pio_clock.sm < 0) ? 0xFFu : static_cast<uint8_t>(
		pio_sm_get_pc(pio_clock.pio, static_cast<uint>(pio_clock.sm))
		- static_cast<uint>(pio_clock.offset));
	s->rti_count = MC6809.get_rti_count();
	s->last_lic = last_LIC;
	s->lic_stalled = false;
	if (MC6809.is_running()) {
		if (last_LIC != s->lic_count)
			last_LIC = s->lic_count;
		else
			s->lic_stalled = true;
	}
#endif

	registers::copy_out_write(s->write_regs, 0, 64);
	registers::copy_out(s->read_regs, 0, 64);

	s->tick_enabled = system_tick.is_enabled();
	s->tick_irq_pending = system_tick.is_irq_pending();

	s->core1_loop = core1_loop_count;
	s->core1_qrise = core1_qrise_count;
	s->core1_irq_assert = core1_irq_assert_count;
	s->core1_write_ring = core1_write_ring_count;
	s->core1_read_irq = core1_read_irq_count;
	s->core1_last_write = core1_last_write_loc;
	s->core1_last_read = core1_last_read_loc;
	s->core1_isr = core1_isr_count;
	{
		volatile uint32_t *bottom = &__StackOneBottom;
		volatile uint32_t *top = &__StackOneTop;
		volatile uint32_t *p = bottom;
		while (p < top && *p == 0xDEADBEEFu)
			p++;
		s->core1_stack_total = (top - bottom) * sizeof(uint32_t);
		s->core1_stack_free = (p - bottom) * sizeof(uint32_t);
	}

	s->console_acia.interrupt_status = fast_serial.interrupt_status();
	s->console_acia.tx_level = fast_serial.receive_level();
	s->console_acia.rx_level = fast_serial.transmit_level();
	status_gather_flow(&s->console_acia, 0u);
#ifdef DEBUG
	s->console_acia.control_count = ACIA_CONTROL_COUNT;
	s->console_acia.tx_data_count = ACIA_TX_DATA_COUNT;
	s->console_acia.status_count = CONSOLE_STATUS_READS;
	s->console_acia.rx_data_count = ACIA_RX_DATA_COUNT;
#endif

	s->aux_acia.interrupt_status = aux_serial.interrupt_status();
	s->aux_acia.tx_level = aux_serial.receive_level();
	s->aux_acia.rx_level = aux_serial.transmit_level();
	status_gather_flow(&s->aux_acia, 1u);
#ifdef DEBUG
	s->aux_acia.control_count = AUX_ACIA_CONTROL_COUNT;
	s->aux_acia.tx_data_count = AUX_ACIA_TX_DATA_COUNT;
	s->aux_acia.status_count = AUX_ACIA_STATUS_COUNT;
	s->aux_acia.rx_data_count = AUX_ACIA_RX_DATA_COUNT;
#endif
}

static void
status_print_regs(const char *label, const uint8_t *regs)
{
	uint8_t printables[17];
	printf("%s:\n", label);
	for (uint16_t i = 0u; i < 4u; i++) {
		memcpy(printables, regs + i * 16u, 16u);
		printf("%04X:", REGISTER_BASE + i * 16u);
		for (uint16_t j = 0u; j < 16u; j++) {
			auto ch = static_cast<unsigned char>(printables[j]) & 0x7F;
			printf(" %02X", printables[j]);
			if (j == 7u)
				printf(" ");
			printables[j] = std::isprint(ch) ? static_cast<char>(ch) : '.';
		}
		printables[16] = '\0';
		printf(" |%.8s|%.8s|\n", reinterpret_cast<char *>(printables), reinterpret_cast<char *>(printables + 8));
	}
}

// The RP2350 steppings, named by bootrom version.  CHIP_ID.REVISION cannot
// name them on its own: A4 changed only the bootrom, and although the
// datasheet gives its revision as 0x8, an A4 (bootrom 4) on the replacement
// version 2 board reads 0x3, the same as an A3.  The revision is kept here as
// the datasheet's figure, so a chip that disagrees with it shows its own.
struct chip_stepping_t {
	uint8_t rom_version;
	uint8_t revision;
	const char *name;
};

// nullptr for a bootrom version the datasheet does not list.
static const chip_stepping_t *
chip_stepping(const uint8_t rom_version)
{
	static constexpr chip_stepping_t steppings[] = {
		{ 2u, 0x2u, "A2" },
		{ 3u, 0x3u, "A3" },
		{ 4u, 0x8u, "A4" },
	};
	for (const auto &s : steppings)
		if (s.rom_version == rom_version)
			return &s;
	return nullptr;
}

void
status_print(const status_snapshot_t *s)
{
	// 1. Signals — POWER/RESET/HALT lined up with NMI/FIRQ/IRQ
	//    Columns at 0, 19, 38.  Colons at 6, 25, 44.  Values at 8, 27, 46.
	printf("POWER: %-12sRESET: %-12s HALT: %s\n",
	       s->power ? "HIGH" : "low",
	       !s->reset ? "high" : "LOW",
	       !s->halt ? "high" : "LOW");
	// Both ADC channels read high whenever the Pico is answering bus reads,
	// and only RS_HALTED and RS_UNPOWERED have been measured against a
	// meter - those are the two the calibration constants were derived from.
	// Everything else serves the bus, where the supply reads 0.54 to 0.66V
	// high depending on what the guest is doing and the die about 123 counts
	// high, for reasons not yet understood.  Saying so beats printing -18C
	// as though it were a temperature, which is a misreading I made myself
	// before working out what the figures meant.
	const bool adc_quiet = s->run_state == RS_HALTED
			    || s->run_state == RS_UNPOWERED;
	static constexpr const char *adc_warning =
		"   <- the bus is being served; both ADC channels read high";
#if PICO_THING_BOARD_VERSION >= 2
	// The baseline as well as the sample: without it there is no way to see
	// whether the relative detector is armed, which is what the bench test
	// for the cutout has to watch while it sags the sense node at J107.
	if (s->rail_baseline > 0.0f)
		printf("  cutout baseline %.2fV (%" PRIu32 " good)%s\n",
		       (double) s->rail_baseline, s->rail_good,
		       s->rail_good >= 50u ? ", armed"
					   : ", still establishing");
	else
		printf("  cutout baseline not established - absolute floor only\n");
	printf("PICO SUPPLY (+5VP): %.2fV (%u counts)%s\n",
	       adc_rail_volts(s->voltage), s->voltage,
	       adc_quiet ? "" : adc_warning);
#endif
	printf("PICO TEMPERATURE: %.1fC (%u counts)%s\n",
	       adc_die_celsius(s->temperature), s->temperature,
	       adc_quiet ? "" : adc_warning);
	const char part = s->qfn60 ? 'A' : 'B';
	if (const chip_stepping_t *stepping = chip_stepping(s->rom_version)) {
		if (stepping->revision == s->chip_revision)
			printf("PICO CHIP: RP2350%c %s (bootrom %u)\n",
			       part, stepping->name, s->rom_version);
		else
			printf("PICO CHIP: RP2350%c %s (bootrom %u, CHIP_ID revision %X)\n",
			       part, stepping->name, s->rom_version, s->chip_revision);
	} else
		printf("PICO CHIP: RP2350%c, unknown bootrom %u (CHIP_ID revision %X)\n",
		       part, s->rom_version, s->chip_revision);
	// clk_adc is the one with a silent failure: away from 48 MHz the sample
	// capacitor does not finish charging and every conversion reads low, with
	// nothing to say so.  clk_peri carries the console's baud rate.
	printf("CLOCKS: sys %" PRIu32 "  peri %" PRIu32 "  usb %" PRIu32 "  adc %" PRIu32 " kHz%s\n",
	       s->clk_sys_khz, s->clk_peri_khz, s->clk_usb_khz, s->clk_adc_khz,
	       s->clk_adc_khz == 48000u ? "" : "   <- adc is not 48000!");
	printf("  NMI: %-12s FIRQ: %-12s  IRQ: %s\n",
	       s->nmi ? "OUTPUT LOW" : "hi-z",
	       s->firq ? "OUTPUT LOW" : "hi-z",
	       s->irq ? "OUTPUT LOW" : "hi-z");
	printf("\n");

	// 2. Bus state, run state
	printf("Bus State: %-16s  Prev: %-16s\n",
	       bus_state_string[s->bus_state], bus_state_string[s->old_bus_state]);
	printf("Run State: %s  Nesting: %u\n", run_state_string[s->run_state], s->nesting_depth);
	{
		// Only worth a line when something has actually gone wrong, or the
		// OS has had to put us right.  A clean run says nothing.
		const mc6809::depth_errors d = MC6809.get_depth_errors();
		if (d.overflows || d.underflows || d.declares)
			printf("  depth: %" PRIu32 " overflows (history full), %" PRIu32
			       " underflows (RTI at 0), %" PRIu32 " OS declarations\n",
			       d.overflows, d.underflows, d.declares);
	}
	printf("Task: %u  History:", s->task);
	if (s->task_stack_ptr == 0)
		printf(" -");
	else
		for (uint16_t i = 0; i < s->task_stack_ptr; i++)
			printf(" %u", s->task_history[i]);
	printf("  Odd task writes: %" PRIu32, s->task_write_odd_count);
	if (s->task_write_odd_count > 0u)
		printf("   <- last was task %u -> %u!",
		       s->task_write_odd_from, s->task_write_odd_to);
	printf("\n");

	// 3. LIC / RTI (DEBUG only)
#ifdef DEBUG
	// The live bits as well as the count: the count only moves while the guest
	// is running, so under HALT the bits are the only way to see the mirror
	// tracking - and LIC is HIGH when halted at an instruction boundary.
	BLA live{}; live.byte = s->busy_lic_avma;
	printf("LIC: %-10" PRIu32 "  RTI: %-10" PRIu32 "  mirror: %c%c%c\n",
	       s->lic_count, s->rti_count,
	       live.bit.BUSY ? 'B' : '.',
	       live.bit.LIC  ? 'L' : '.',
	       live.bit.AVMA ? 'V' : '.');
	// Which stage is stuck, when the mirror is not tracking: a PC parked on
	// the wait means the clock's E-falling flag is not arriving; a full FIFO
	// means the state machine pushes but the DMA does not drain; an idle DMA
	// means the endless transfer is not running at all.
	printf("mirror raw %08" PRIX32 "  sm pc +%u  rx fifo %u  dma %s\n",
	       s->mirror_raw, s->mirror_pc, s->mirror_fifo,
	       s->mirror_dma_busy ? "busy" : "IDLE");
	printf("pio2 irq flags %02X (flag1 %s)  clock pc +%u\n",
	       s->pio2_irq, (s->pio2_irq & 0x02u) ? "SET" : "clear", s->clock_pc);
	if (s->lic_stalled)
		printf("WARNING: LIC stalled at %" PRIu32 ", run state is %s\n",
		       s->lic_count, run_state_string[s->run_state]);
	printf("\n");
#endif

	// 4. Register hex dump
	printf("Registers:\n");
	status_print_regs("Write block", s->write_regs);
	status_print_regs("Read block", s->read_regs);
	printf("\n");

	// 5. Tick timer + Core 1 table
	printf("Tick:  Enabled: %-3s  IRQ Pending: %-3s\n",
	       s->tick_enabled ? "yes" : "no",
	       s->tick_irq_pending ? "yes" : "no");
	printf("Core 1:\n");
	printf("       Loop     Q Rise   IRQ Asrt   Wrt Ring   Read IRQ        ISR\n");
	printf("  %9" PRIu32 "  %9" PRIu32 "  %9" PRIu32
	       "  %9" PRIu32 "  %9" PRIu32 "  %9" PRIu32 "\n",
	       s->core1_loop, s->core1_qrise, s->core1_irq_assert,
	       s->core1_write_ring, s->core1_read_irq, s->core1_isr);
	printf("  Last Write: %02X   Last Read: %02X\n",
	       s->core1_last_write, s->core1_last_read);
	printf("  Stack Total: %" PRIu32 "  Free (min): %" PRIu32
	       "  Used (max): %" PRIu32 "\n",
	       s->core1_stack_total, s->core1_stack_free,
	       s->core1_stack_total - s->core1_stack_free);
	printf("\n");

	// 6. ACIA table — two data rows
#ifdef DEBUG
	printf("ACIA:        IRQ  Tx Lvl  Rx Lvl  Control  Tx Data   Status  Rx Data\n");
	printf("  Console     %02X  %6u  %6u  %7u  %7u  %7u  %7u\n",
	       s->console_acia.interrupt_status,
	       s->console_acia.tx_level, s->console_acia.rx_level,
	       s->console_acia.control_count, s->console_acia.tx_data_count,
	       s->console_acia.status_count, s->console_acia.rx_data_count);
	status_print_flow("Console", &s->console_acia);
	status_print_rdrf("Console", fast_serial.why_rdrf());
	{
		const uint32_t n = status_seen_n;
		const uint32_t show = (n < STATUS_SEEN_LEN) ? n : STATUS_SEEN_LEN;
		printf("    IRQ taken %" PRIu32 " times; /IRQ asserted %" PRIu32
		       " E cycles (%.1f cycles per taken interrupt)\n",
		       irq_vectors, pend_irq_cycles,
		       irq_vectors ? (double) pend_irq_cycles / (double) irq_vectors
				   : 0.0);
		printf("    Console S_IRQ status reads: %" PRIu32
		       " (with RDRF %" PRIu32 ", WITHOUT RDRF %" PRIu32
		       "); first %" PRIu32 ":", n, status_irq_rdrf,
		       status_irq_nordrf, show);
		for (uint32_t i = 0; i < show; i++)
			printf(" %02X", status_seen[i]);
		printf("\n");
	}
	printf("  Aux         %02X  %6u  %6u  %7u  %7u  %7u  %7u\n",
	       s->aux_acia.interrupt_status,
	       s->aux_acia.tx_level, s->aux_acia.rx_level,
	       s->aux_acia.control_count, s->aux_acia.tx_data_count,
	       s->aux_acia.status_count, s->aux_acia.rx_data_count);
	status_print_flow("Aux", &s->aux_acia);
	status_print_rdrf("Aux", aux_serial.why_rdrf());
#else
	printf("ACIA:        IRQ  Tx Lvl  Rx Lvl\n");
	printf("  Console     %02X  %6u  %6u\n",
	       s->console_acia.interrupt_status,
	       s->console_acia.tx_level, s->console_acia.rx_level);
	status_print_flow("Console", &s->console_acia);
	printf("  Aux         %02X  %6u  %6u\n",
	       s->aux_acia.interrupt_status,
	       s->aux_acia.tx_level, s->aux_acia.rx_level);
	status_print_flow("Aux", &s->aux_acia);
#endif
}

static void
status_json_regs(const char *name, const uint8_t *regs)
{
	printf(R"json("%s":")json", name);
	for (int i = 0; i < 64; i++) {
		if (i > 0) printf(" ");
		printf("%02X", regs[i]);
	}
	printf("\"");
}

static void
status_json_acia(const char *name, const acia_status_t *a)
{
	printf(R"json("%s":{"interrupt_status":%u,"tx_level":%u,"rx_level":%u)json", name,
	       a->interrupt_status, a->tx_level, a->rx_level);
#ifdef DEBUG
	printf(R"json(,"control_count":%u,"tx_data_count":%u,"status_count":%u,"rx_data_count":%u)json",
	       a->control_count, a->tx_data_count, a->status_count, a->rx_data_count);
#endif
	printf("}");
}

void
status_json(const status_snapshot_t *s)
{
	printf(R"json({"run_state":"%s","bus_state":"%s","old_bus_state":"%s")json",
	       run_state_string[s->run_state], bus_state_string[s->bus_state],
	       bus_state_string[s->old_bus_state]);
	printf(R"json(,"task":%u,"nesting_depth":%u,"task_stack_ptr":%u)json", s->task, s->nesting_depth, s->task_stack_ptr);
	printf(R"json(,"task_write_odd":{"count":%)json" PRIu32 R"json(,"from":%u,"to":%u})json",
	       s->task_write_odd_count, s->task_write_odd_from, s->task_write_odd_to);
	printf(R"json(,"task_history":[)json");
	for (uint16_t i = 0; i < s->task_stack_ptr; i++) {
		if (i > 0) printf(",");
		printf("%u", s->task_history[i]);
	}
	printf("]");
	printf(R"json(,"power":%s,"reset":%s,"halt":%s,"nmi":%s,"firq":%s,"irq":%s)json",
	       s->power ? "true" : "false", s->reset ? "true" : "false",
	       s->halt ? "true" : "false", s->nmi ? "true" : "false",
	       s->firq ? "true" : "false", s->irq ? "true" : "false");
	// Raw counts as well as volts/celsius: the conversions are approximate,
	// the counts are what was actually measured.  Version 1 has no bus
	// divider, so its voltage is null rather than a misleading zero.
#if PICO_THING_BOARD_VERSION >= 2
	printf(R"json(,"voltage":%.2f,"voltage_raw":%u)json", adc_rail_volts(s->voltage), s->voltage);
#else
	printf(R"json(,"voltage":null,"voltage_raw":null)json");
#endif
	printf(R"json(,"temperature":%.1f,"temperature_raw":%u)json",
	       adc_die_celsius(s->temperature), s->temperature);
	if (const chip_stepping_t *stepping = chip_stepping(s->rom_version))
		printf(R"json(,"chip_stepping":"%s")json", stepping->name);
	else
		printf(R"json(,"chip_stepping":null)json");
	printf(R"json(,"chip_revision":%u,"rom_version":%u,"chip_package":"%s")json",
	       s->chip_revision, s->rom_version, s->qfn60 ? "QFN-60" : "QFN-80");
	printf(R"(,"clk_sys_khz":%)" PRIu32 R"(,"clk_peri_khz":%)" PRIu32
	       R"(,"clk_usb_khz":%)" PRIu32 R"(,"clk_adc_khz":%)" PRIu32,
	       s->clk_sys_khz, s->clk_peri_khz, s->clk_usb_khz, s->clk_adc_khz);
#ifdef DEBUG
	printf(",\"lic_count\":%" PRIu32 ",\"rti_count\":%" PRIu32 ",\"lic_stalled\":%s",
	       s->lic_count, s->rti_count, s->lic_stalled ? "true" : "false");
#endif
	printf(",");
	status_json_regs("write_regs", s->write_regs);
	printf(",");
	status_json_regs("read_regs", s->read_regs);
	printf(R"json(,"tick_enabled":%s,"tick_irq_pending":%s)json",
	       s->tick_enabled ? "true" : "false",
	       s->tick_irq_pending ? "true" : "false");
	printf(",\"core1_loop\":%" PRIu32 ",\"core1_qrise\":%" PRIu32
	       ",\"core1_irq_assert\":%" PRIu32,
	       s->core1_loop, s->core1_qrise, s->core1_irq_assert);
	printf(R"(,"core1_write_ring":%)" PRIu32 R"(,"core1_read_irq":%)" PRIu32,
	       s->core1_write_ring, s->core1_read_irq);
	printf(R"(,"core1_last_write":%u,"core1_last_read":%u,"core1_isr":%)" PRIu32,
	       s->core1_last_write, s->core1_last_read, s->core1_isr);
	printf(R"(,"core1_stack_total":%)" PRIu32 R"(,"core1_stack_free":%)" PRIu32,
	       s->core1_stack_total, s->core1_stack_free);
	printf(",");
	status_json_acia("console_acia", &s->console_acia);
	printf(",");
	status_json_acia("aux_acia", &s->aux_acia);
	printf("}\n");
}

static bool set_power();
static void reset_power();

void
process_command(char *buf)
{
	static uint8_t buffer[256];
	parsed_cmd_t cmd{};

	/* Skip blank lines */
	while (isspace(static_cast<unsigned char>(*buf)))
		buf++;
	if (*buf == '\0')
		return;

	if (!cmd_parse(buf, &cmd))
		return; /* error message already printed by parser */

	switch (cmd.tag) {

	case CMD_POWER:
		printf("Power is %s\n", POWER_IS_ASSERTED ? "on" : "off");
		break;

	case CMD_POWER_ON:
		if (POWER_IS_ASSERTED) {
			printf("Power is already on\n");
		} else {
			set_power();
		}
		break;

	case CMD_POWER_OFF:
		if (!POWER_IS_ASSERTED) {
			printf("Power is already off\n");
		} else {
			reset_power();
		}
		break;

	case CMD_VERBOSE:
		verbose = !verbose;
		printf("Verbose mode %s\n", verbose ? "on" : "off");
		break;

	case CMD_HELP:
		// Keep this in step with the grammar in command_parser.y and the
		// keyword table in command_lexer.l.  Every command has a two-letter
		// alias except "go"; keywords and hex are case-insensitive.
		printf(
		"\nAll numbers are hex unless stated.  Two-letter aliases in brackets.\n"
		"\n--- CPU ---\n"
		"  run [n] [!]        (ru)  run snippet n; ! leaves the CPU running\n"
		"  go [addr]                start the CPU, optionally at addr\n"
		"  stop [halt|restart](sp)  stop, or halt, or stop-and-restart\n"
		"  reset              (rs)  pulse !RESET\n"
		"  freq <MHz>         (fq)  set the E clock, %.1f to %.1f\n"
		"  task [n]           (ta)  show or set the task register\n"
		"  power [on|off]     (pw)  external power enable\n"
		"\n--- Memory ---\n"
		"  examine <s> <e>    (ex)  dump guest memory from s to e\n"
		"  examine dat        (ex)  dump the DAT RAM at $FE00\n"
		"  modify <a> <b>...  (mo)  write bytes b... from address a\n"
		"  fill <s> <e> [v]   (fi)  fill s..e with v (default 00)\n"
		"  fill all [v]       (fi)  fill $0000-$FDFF\n"
		"  vector [n]         (vc)  show or set the CPU vectors\n"
		"  adcdump                  burst-sample both ADC channels in reset,\n"
		"                           running, faulting, halted and unpowered;\n"
		"                           dumps raw counts as CSV (power-cycles guest)\n"
		"  ramtest                  test every RAM page and the DAT (overwrites RAM)\n"
		"  synctest [n]             count snippet runs that fail (n decimal, default 1000)\n"
		"  snippet [n]        (sn)  list snippets, or copy n to $FFE0\n"
		"\n--- Debug ---\n"
		"  trace on|off       (tr)  arm or disarm the %u-cycle one-shot\n"
		"  trace [start] [len] (tr)  dump the capture; rows are hex, and the\n"
		"                           start defaults to 0.  Row numbers stay\n"
		"                           absolute, so offset dumps line up\n"
		"  trace <vector>     (tr)  arm on swi|swi2|swi3|nmi|firq|irq\n"
		"  break [addr|r]     (br)  breakpoint; r resumes\n"
		"  irq <0-8>          (ir)  interrupt tests; 7 is NMIBlock latency,\n"
		"                           8 is the DAT fault path\n"
		"  loop <type> <addr> (lo)  hammer addr for scope work.  The type is\n"
		"                           CASE-SENSITIVE: lower case is a byte access,\n"
		"                           upper case a word:\n"
		"                             r   LDA (read byte)    R   LDD (read word)\n"
		"                             w   STA (write byte)   W   STD (write word)\n"
		"                             rw  NEG (read-modify-write; RW also works)\n"
		"  status [period]    (st)  system status, repeating every period ms\n"
		"  verbose            (vb)  toggle verbose output\n"
		"\n--- Host ---\n"
		"  time [y m d h m s] (ti)  show or set the clock\n"
		"  rtc [y m d h m s]        show or set the DS12885 real-time clock\n"
		"  rtc set                  set the Pico clock from the RTC\n"
		"  usb [args]         (ub)  USB console control\n"
		"  flow               (fl)  show both CDC ports' flow control\n"
		"  flow <1|2> on|off        honour that port's host RTS, or ignore it\n"
		"  nitros9 [ide|drivewire]  (ni)  boot NitrOS-9; drivewire is also dw\n"
		"  help               (h)   this text\n"
		"\nAn S-record pasted at the prompt is loaded directly.\n\n",
		static_cast<double>(GUEST_CLK_MIN), static_cast<double>(GUEST_CLK_MAX),
		TRACE_SIZE);
		break;

	case CMD_STATUS: {
		status_snapshot_t snap{};
		status_gather(&snap);
		status_print(&snap);
		uint32_t period = cmd.status.period;
		while (period > 0u) {
			for (uint32_t i = 0u; i < period * 1000u; i++) {
				usb_poll();
				if (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT)
					goto status_done;
				mc6809::uart_task(true);
				sleep_ms(1u);
			}
			printf("%s%s", ANSI_CLEAR_SCREEN, ANSI_CURSOR_HOME);
			status_gather(&snap);
			status_print(&snap);
		}
status_done:
		break;
	}

	case CMD_RESET:
		if (!MC6809.assert_powered())
			break;
		peripheral_clear(false);
		break;

	case CMD_EXAMINE_DAT: {
		constexpr uint16_t start = 0xFD00u;
		constexpr uint16_t end   = 0xFFC0u;
		if (!MC6809.assert_stopped())
			break;
		MC6809.preserve_state();
		constexpr uint16_t display_start = start & 0xFFF0u;
		constexpr uint16_t display_end   = ((end - 1u) | 0x000Fu) + 1u;
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
					const int ch = static_cast<unsigned char>(buffer[i]) & 0x7F;
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
			// Read only what was asked for.  The line is laid out on a
			// 16-byte boundary, but reading all of it would touch registers
			// either side of the range - and in I/O space a read is not free:
			// it pops the IDE data register, clears RDRF, acknowledges the
			// timer IRQ, clears a latched DAT fault.  So each line reads just
			// its overlap with start..end.
			const uint32_t line_end = static_cast<uint32_t>(loc) + 16u;
			const uint16_t first = loc > start ? loc : start;
			const uint16_t past = static_cast<uint16_t>(line_end < end ? line_end : end);
			if (!copy(INWARDS, static_cast<uint16_t>(past - first), first,
				  buffer + (first - loc)))
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
					const int ch = static_cast<unsigned char>(buffer[i]) & 0x7F;
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
			if (!MC6809.assert_powered())
				break;
			// Core 1 carries each write out as though the guest had made it,
			// read side and device follow-ups included.  Both sides are shown
			// once it has.
			static constexpr uint32_t MODIFY_TIMEOUT_MS = 100u;
			printf("Modifying system registers as the guest would\n");
			const uint32_t done_before = console_writes_done;
			uint16_t queued = 0u;
			for (; queued < count; queued++) {
				const write_entry w = {
					static_cast<uint8_t>((start + queued) & registers::register_mask),
					buffer[queued]
				};
				if (!queue_try_add(&console_write_queue, &w))
					break;
			}
			if (queued < count)
				printf("Console write queue full - %u of %u bytes queued\n", queued, count);
			const absolute_time_t deadline = make_timeout_time_ms(MODIFY_TIMEOUT_MS);
			while (console_writes_done - done_before < queued
			       && absolute_time_diff_us(get_absolute_time(), deadline) > 0)
				sleep_ms(1u);
			if (console_writes_done - done_before < queued) {
				printf("Core 1 did not carry out the writes within %" PRIu32 "ms\n",
				       MODIFY_TIMEOUT_MS);
				break;
			}
			sleep_ms(1u);	// the ring dispatch follows in the same Core 1 pass
			for (uint16_t i = 0u; i < queued; i++) {
				const uint8_t loc = static_cast<uint8_t>((start + i) & registers::register_mask);
				printf("%04X: %02X %02X\n",
				       static_cast<uint16_t>(start + i),
				       static_cast<uint8_t>(registers::write(loc)),
				       static_cast<uint8_t>(reg[loc]));
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
		if (!MC6809.assert_powered())
			break;
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
		if (!MC6809.assert_powered())
			break;
		snippet_copy(LOOP_RW);
		reg[REGISTER_SNIPPET_OFFSET + 0u] = cmd.loop.opcode;
		reg[REGISTER_SNIPPET_OFFSET + 1u] = cmd.loop.address;
		MC6809.start();
		break;

	case CMD_SYNCTEST:
		if (!MC6809.assert_stopped())
			break;
		synctest(cmd.synctest.runs);
		// Leave task zero and an identity DAT behind, as ramtest does.
		MC6809.stop();
		task_pins_update(MC6809.task_change(0u));
		snippet_copy(DAT_INIT);
		if (!MC6809.start_with_timeout(4u))
			printf("  restoring the identity DAT failed\n");
		break;

	case CMD_RAMTEST: {
		if (!MC6809.assert_stopped())
			break;
		printf("RAM/DAT test - this overwrites all of RAM and takes several minutes\n");
		const bool good = ram_test();
		if (ram_retries != 0u)
			printf("  %u snippet run%s failed and had to be repeated\n",
			       ram_retries, ram_retries == 1u ? "" : "s");
		if (ram_recovered != 0u)
			printf("  %u check%s passed once the page was filled again - overwritten after the test wrote it, not a faulty cell\n",
			       ram_recovered, ram_recovered == 1u ? "" : "s");
		if (ram_transients != 0u)
			printf("  %u mismatch%s went away when read again - not counted as faults\n",
			       ram_transients, ram_transients == 1u ? "" : "es");
		if (ram_write_buffer_right != 0u)
			printf("  %u bad read%s had the right bytes in the Pico's write buffer\n",
			       ram_write_buffer_right, ram_write_buffer_right == 1u ? "" : "s");
		if (copy_buffer_disagreements != 0u)
			printf("  %" PRIu32 " cop%s left the Pico's read and write buffers disagreeing\n",
			       copy_buffer_disagreements, copy_buffer_disagreements == 1u ? "y" : "ies");
		// Leave task zero and an identity DAT behind, whatever happened.
		MC6809.stop();
		task_pins_update(MC6809.task_change(0u));
		snippet_copy(DAT_INIT);
		if (!MC6809.start_with_timeout(4u))
			printf("  restoring the identity DAT failed\n");
		const unsigned secs = static_cast<unsigned>((time_us_64() - ram_started_us) / 1000000u);
		printf("RAM/DAT test: %s (%u:%02u)\n", good ? "PASS" : "FAIL", secs / 60u, secs % 60u);
		break;
	}

	case CMD_RTC_QUERY: {
		uint8_t r[RTC_CLOCK_REGISTERS];
		if (!rtc_read(r))
			break;
		if ((r[RTC_B] & RTC_B_MODE) != RTC_B_MODE) {
			printf("RTC has not been set - it is not in binary 24-hour mode (register B = %02X)\n",
			       r[RTC_B]);
			printf("Set it with rtc <year> <month> <day> <hour> <minute> <second>\n");
		} else {
			const uint8_t day = r[RTC_DAY];
			printf("%04d-%02d-%02d %02d:%02d:%02d %s\n",
			       2000 + r[RTC_YEAR], r[RTC_MONTH], r[RTC_DATE],
			       r[RTC_HOURS], r[RTC_MINUTES], r[RTC_SECONDS],
			       rtc_day_name[(day >= 1u && day <= 7u) ? day : 0u]);
		}
		if ((r[RTC_A] & RTC_A_DV_MASK) != RTC_A_DV_RUN || (r[RTC_B] & RTC_B_SET) != 0u)
			printf("RTC clock is stopped (register A = %02X, B = %02X)\n", r[RTC_A], r[RTC_B]);
		if ((r[RTC_D] & RTC_D_VRT) == 0u)
			printf("RTC battery is absent or exhausted (VRT clear) - the time will not survive power off\n");
		break;
	}

	case CMD_RTC_TO_PICO: {
		// Set the Pico's own clock from the RTC.  Only a time this firmware
		// could have set is trusted: binary 24-hour mode, a running clock,
		// and every field possible.  A clear VRT is only a warning - the
		// time is still right if the board has stayed powered since it was
		// set.
		uint8_t r[RTC_CLOCK_REGISTERS];
		if (!rtc_read(r))
			break;
		if ((r[RTC_B] & RTC_B_MODE) != RTC_B_MODE) {
			printf("RTC has not been set (not in binary 24-hour mode) - Pico clock unchanged\n");
			break;
		}
		if ((r[RTC_A] & RTC_A_DV_MASK) != RTC_A_DV_RUN || (r[RTC_B] & RTC_B_SET) != 0u) {
			printf("RTC clock is stopped, so its time is stale - Pico clock unchanged\n");
			break;
		}
		const int year = 2000 + r[RTC_YEAR];
		const int month = r[RTC_MONTH];
		if (month < 1 || month > 12 || r[RTC_DATE] < 1 || r[RTC_DATE] > days_in_month(year, month)
		    || r[RTC_HOURS] > 23 || r[RTC_MINUTES] > 59 || r[RTC_SECONDS] > 59) {
			printf("RTC holds an impossible date or time - Pico clock unchanged\n");
			break;
		}
		if ((r[RTC_D] & RTC_D_VRT) == 0u)
			printf("RTC battery is absent or exhausted (VRT clear) - using its time anyway\n");
		tm tm = {
			.tm_sec   = r[RTC_SECONDS],
			.tm_min   = r[RTC_MINUTES],
			.tm_hour  = r[RTC_HOURS],
			.tm_mday  = r[RTC_DATE],
			.tm_mon   = month - 1,
			.tm_year  = year - 1900,
			.tm_isdst = 0,
		};
		aon_timer_set_time_calendar(&tm);
		print_time();
		break;
	}

	case CMD_RTC_SET: {
		const int year = cmd.time.year;
		const int month = cmd.time.month + 1;	// the grammar makes it 0-based, for tm
		const int day = cmd.time.day;
		if (day > days_in_month(year, month)) {
			printf("Invalid day %d for %04d-%02d\n", day, year, month);
			break;
		}
		// Registers $00-$09 in order.  The alarm bytes are zeroed, and
		// RTC_SET leaves every interrupt disabled, so no alarm is armed.
		const uint8_t values[RTC_A] = {
			static_cast<uint8_t>(cmd.time.second), 0u,
			static_cast<uint8_t>(cmd.time.minute), 0u,
			static_cast<uint8_t>(cmd.time.hour), 0u,
			rtc_day_of_week(year, month, day),
			static_cast<uint8_t>(day),
			static_cast<uint8_t>(month),
			static_cast<uint8_t>(year % 100),
		};
		if (!MC6809.assert_stopped())
			break;
		if (!copy(OUTWARDS, chunk_len[RTC_SET], CHUNK_ADDRESS, chunk_code[RTC_SET]))
			break;
		// Staged after the copy, which moves the chunk through SHARED.
		registers::copy_in(REGISTER_BUFFER_OFFSET, values, RTC_A);
		reg[REGISTER_VECTOR_RESET_OFFSET] = CHUNK_ADDRESS;
		if (!MC6809.start_with_timeout(100u)) {
			printf("RTC set did not finish - the CPU never reached SYNC\n");
			break;
		}
		printf("RTC set to %04d-%02d-%02d %02d:%02d:%02d %s\n",
		       year, month, day, cmd.time.hour, cmd.time.minute, cmd.time.second,
		       rtc_day_name[rtc_day_of_week(year, month, day)]);
		break;
	}

	case CMD_ADC: {
		// Burst-sample both ADC channels under five guest conditions and dump
		// every reading.  The die temperature has been seen to swing far more
		// than the supply divider while the bus is busy - more than a shared
		// reference error could explain - and one sample per condition cannot
		// tell a noisy channel from a shifted one.  This prints the raw counts
		// and leaves the judgement to whoever reads them.
		//
		// The last two conditions are the quiet ones, there to tell a steady
		// offset from bus activity: halted, the processor has let go of the
		// bus and the Pico answers no reads; unpowered, nothing on the board
		// switches at all.  Unpowering power-cycles the guest, so its RAM is
		// lost.  Readings are buffered and printed at the end, because powering
		// down and back up prints progress that would otherwise split the CSV.
		//
		// ADC_RAM_LOOP is the discriminator the other conditions cannot be.
		// LOOP_RW is five bytes of LDA extended and BRA, and it runs at
		// $FFE0 - inside the Pico's own window - so five of its eight cycles
		// are opcode and operand fetches that the PICO answers, and only the
		// data read reaches RAM.  Reset is the same story at one fetch per
		// cycle, and the faulting loop likewise.  All three therefore serve
		// the bus at about the same rate, which is why their agreeing on an
		// offset proves nothing: it cannot separate "the Pico is serving"
		// from "the Pico is serving at this rate".
		//
		// This condition runs byte-for-byte the same loop from RAM at $2000,
		// reading the same $20D0.  The bus is every bit as busy - same
		// instruction, same address lines, same eight-cycle period - but the
		// Pico drives D0-D7 not once.  If the offset survives, the cause is
		// current through the ground return or something else the bus does,
		// and no amount of serving less will help.  If it vanishes, the cause
		// is the Pico answering reads, and the data-bus drive strength is
		// where to look next.
#if PICO_THING_BOARD_VERSION < 2
		// Version 1 has the power enable on GPIO 46, not a supply divider,
		// so there is no second channel to compare the die sensor against
		// and nothing this would settle.
		printf("No supply divider on version 1 - nothing to compare\n\n");
#else
		if (!MC6809.assert_powered())
			break;
		static constexpr uint32_t ADC_BURST = 64u;
		static constexpr uint8_t ADC_BURST_PAGE = 1u;	// the page condition 2 unmaps
		static constexpr uint32_t ADC_HALT_TIMEOUT_MS = 100u;
		// Discarded conversions before each condition's burst.  The supply
		// channel's first readings come in low and climb - 3210 rising to
		// 3320 over the first twelve samples of the RAM-loop condition,
		// measured 2026-10-01 - which dragged that condition's mean down by
		// about 10 counts.  The die channel is settled from its first sample,
		// so whatever charges up belongs to the divider and the converter
		// rather than to the board, and the guest settling for longer before
		// sampling would not have touched it: the ramp is inside the burst.
		// Running the same mux-and-convert pattern for a while first, and
		// keeping none of it, puts the channel at equilibrium before sample
		// zero.
		static constexpr uint32_t ADC_WARMUP = 64u;
		// LDA $20D0 / BRA -5, assembled by hand: the same instructions
		// LOOP_RW runs, at the same target, but placed in RAM.  $2000 and
		// $20D0 are both in ADC_BURST_PAGE, so one DAT entry maps the lot.
		static constexpr uint16_t ADC_RAM_LOOP_ADDR = 0x2000u;
		static const uint8_t adc_ram_loop[] = {
			0xB6u, 0x20u, 0xD0u,	// LDA $20D0
			0x20u, 0xFBu		// BRA -5, back to $2000
		};
		enum : uint8_t {
			ADC_RESET,		// guest held in reset
			ADC_LOOP,		// LOOP_RW reading a mapped page
			ADC_RAM_LOOP,		// the same loop, but resident in RAM
			ADC_FAULTING,		// LOOP_RW reading an unmapped page
			ADC_HALTED,		// LOOP_RW halted, bus released
			ADC_UNPOWERED,		// guest rail off
			ADC_CONDITIONS
		};
		static uint16_t adc_samples[ADC_CONDITIONS][ADC_BURST][2];

		printf("ADC burst, %" PRIu32 " samples per condition - power-cycles the guest, losing its RAM\n",
		       ADC_BURST);

		// Take the converter off the supply monitor for the duration: there is
		// one input mux, and the two would otherwise fight over it.
		cancel_repeating_timer(&rail_timer);

		const uint16_t dat_entry = static_cast<uint16_t>(0xFE00u + ADC_BURST_PAGE);
		uint8_t dat_saved = 0u;
		bool dat_read = false;

		for (uint8_t cond = 0u; cond < ADC_CONDITIONS; cond++) {
			if (cond == ADC_UNPOWERED) {
				reset_power();
			} else {
				MC6809.stop();
				sleep_ms(1u);
				if (cond == ADC_FAULTING || cond == ADC_HALTED) {
					// DAT RAM is written from task zero, the only task
					// the GAL lets write it.  Unmap the page the loop is
					// about to read for the faulting loop, keeping its
					// entry, and put it back for the halted one, which
					// has to run before it can halt.
					task_pins_update(MC6809.task_change(0u));
					if (cond == ADC_FAULTING) {
						dat_read = copy(INWARDS, 1u, dat_entry, &dat_saved);
						const uint8_t unavailable = 0xFFu;
						(void) copy(OUTWARDS, 1u, dat_entry, &unavailable);
					} else if (dat_read)
						(void) copy(OUTWARDS, 1u, dat_entry, &dat_saved);
					MC6809.stop();
					sleep_ms(1u);
				}
				if (cond == ADC_RAM_LOOP) {
					// Lay the loop down in RAM and start the guest on it
					// rather than on the snippet.  copy() runs a snippet
					// of its own, so this has to be the last thing before
					// the vector is pointed at RAM.
					if (!copy(OUTWARDS, sizeof adc_ram_loop,
						  ADC_RAM_LOOP_ADDR, adc_ram_loop))
						printf("Could not write the RAM loop - "
						       "condition %u is not what it claims\n",
						       ADC_RAM_LOOP);
					MC6809.stop();
					sleep_ms(1u);
					reg[REGISTER_VECTOR_RESET_OFFSET] = ADC_RAM_LOOP_ADDR;
					MC6809.start();
					sleep_ms(2u);	// let the loop get going before sampling
				} else if (cond != ADC_RESET) {
					snippet_copy(LOOP_RW);
					reg[REGISTER_SNIPPET_OFFSET + 0u] = static_cast<uint8_t>(0xB6u);	// LDA extended
					reg[REGISTER_SNIPPET_OFFSET + 1u] =
						static_cast<uint8_t>(ADC_BURST_PAGE << 5);
					// Explicitly, not by leftover: the RAM condition above
					// points this at RAM, and the next condition must not
					// inherit that.
					reg[REGISTER_VECTOR_RESET_OFFSET] = REGISTER_SNIPPET;
					MC6809.start();
					sleep_ms(2u);	// let the loop get going before sampling
				}
				if (cond == ADC_HALTED) {
					MC6809.halt();
					for (uint32_t ms = 0u; !MC6809.is_halted() && ms < ADC_HALT_TIMEOUT_MS; ms++)
						sleep_ms(1u);
					if (!MC6809.is_halted())
						printf("Guest did not halt within %" PRIu32 "ms - sampled anyway\n",
						       ADC_HALT_TIMEOUT_MS);
				}
			}

			// Warm the channel up, keeping nothing: see ADC_WARMUP.  The
			// pattern matches the burst below exactly, so the converter and
			// the mux see the same conditions either side of sample zero.
			for (uint32_t n = 0u; n < ADC_WARMUP; n++) {
				adc_select_input(GPIO_POWER_ADC - ADC_BASE_PIN);
				(void) adc_read();
				(void) adc_read();
				adc_select_input(NUM_ADC_CHANNELS - 1);
				(void) adc_read();
				(void) adc_read();
			}

			for (uint32_t n = 0u; n < ADC_BURST; n++) {
				adc_select_input(GPIO_POWER_ADC - ADC_BASE_PIN);
				(void) adc_read();
				adc_samples[cond][n][0] = static_cast<uint16_t>(adc_read());
				adc_select_input(NUM_ADC_CHANNELS - 1);
				(void) adc_read();
				adc_samples[cond][n][1] = static_cast<uint16_t>(adc_read());
			}

			if (cond == ADC_HALTED) {
				MC6809.stop();
				MC6809.release();
				sleep_ms(1u);
			}
		}

		// Back on as 'power on' would do it, with the supply monitor running
		// first: the rail check that gates the switch judges its reading.
		power_rail_monitor_start();
		sleep_ms(3u * RAIL_MONITOR_MS);
		if (!set_power())
			printf("Guest not powered back on - use 'power on'\n");

		printf("cond 0 = guest in reset, 1 = LOOP_RW at $FFE0 (Pico serves the fetches),\n"
		       "     2 = the same loop in RAM at $2000 (Pico serves nothing),\n"
		       "     3 = faulting loop, 4 = halted, 5 = unpowered\n");
		printf("cond,n,bus,die\n");
		for (uint8_t cond = 0u; cond < ADC_CONDITIONS; cond++)
			for (uint32_t n = 0u; n < ADC_BURST; n++)
				printf("%u,%" PRIu32 ",%u,%u\n", cond, n,
				       adc_samples[cond][n][0], adc_samples[cond][n][1]);
		printf("end\n\n");
#endif
		break;
	}

	case CMD_VECTOR:
		if (!MC6809.assert_powered())
			break;
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
		if (!MC6809.assert_powered())
			break;
		fast_serial.reset();
		aux_serial.reset();
		system_tick.reset();
		MC6809.start();
		break;

	case CMD_GO_ADDR:
		if (!MC6809.assert_powered())
			break;
		fast_serial.reset();
		aux_serial.reset();
		system_tick.reset();
		reg[REGISTER_VECTOR_RESET_OFFSET] = cmd.go.address;
		MC6809.start();
		break;

	case CMD_TASK_QUERY:
		if (!MC6809.assert_powered())
			break;
		printf("Task = %u\n", MC6809.task);
		break;

	case CMD_TASK_SET: {
		if (!MC6809.assert_powered())
			break;
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
#ifdef DEBUG
		uint32_t expected_rti_count = 0u;
		uint32_t expected_irq_ack_count = 0u;
#endif

		if (!MC6809.assert_stopped())
			break;

		// Chunk-based test (irq 6) handles its own setup — skip the
		// ZERO preamble which would wipe the S-record-loaded vectors.
		if (hexnum == 6)
			goto irq_chunk_test;

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
#ifdef DEBUG
		MC6809.clear_rti_count();
		MC6809.clear_irq_ack_count();
#endif

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
#ifdef DEBUG
			expected_irq_ack_count = 1u;
#endif
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
#ifdef DEBUG
			expected_irq_ack_count = 1u;
#endif
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
#ifdef DEBUG
			expected_irq_ack_count = 1u;
#endif
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
#ifdef DEBUG
			expected_irq_ack_count = 1u;
#endif
		} else if (hexnum == 5) {
			static constexpr const char *testname = "RTI";
			static constexpr uint16_t stack_result_length = 12u;
			static constexpr uint8_t stack_result[stack_result_length] = {
				0x80, 0x12, 0x23, 0x34, 0x45, 0x56, 0x67, 0x78, 0x89, 0x90, 0xFF, 0xE7
			};
			printf("RTI Test\n");
#ifdef DEBUG
			expected_rti_count = 2u;
			expected_irq_ack_count = 1u;
#endif
			reg[REGISTER_VECTOR_SWI_OFFSET] = static_cast<uint16_t>(REGISTER_SNIPPET + 9);
			for (uint16_t i = 0; i < stack_result_length; i++)
				reg[REGISTER_BUFFER_OFFSET + (16u - stack_result_length) + i] = stack_result[i];
			reg[REGISTER_BUFFER_OFFSET + REGISTER_BUFFER_LEN - 1] = static_cast<uint8_t>(0xE6);
			snippet_copy(TEST_RTI);
			MC6809.start();
			while (!MC6809.is_synced()) { sleep_us(1); if (++timeout > 1000000u) break; }
			MC6809.stop();
			test_name = testname; final_stack_length = stack_result_length; final_stack = stack_result;
		} else if (hexnum == 7) {
			// NMIBlock write-to-effect latency.  The snippet arms the trace
			// and then sets and clears NMIBlock; we read back how many E
			// cycles passed before the pin followed each write.
			printf("NMIBlock latency test\n");
			snippet_copy(NMI_BLOCK_LAT);
			MC6809.start();
			while (!MC6809.is_synced()) { sleep_us(1); if (++timeout > 1000000u) break; }
			MC6809.stop();

			const uint32_t have = MC6809.trace_count();
			if (have == 0u) {
				printf("  no cycles captured — was a trace already armed?\n\n");
				break;
			}
			// Resolution warning: the trace samples once per E cycle, at
			// E-falling, but the guest's write data is not captured until
			// about that same point — so the pin cannot possibly have moved
			// by the write cycle's own sample, and one cycle is the floor,
			// not a failure.  The GAL's deadline is Q-rising of the NEXT
			// cycle, a quarter cycle after E-fall, which this sampling rate
			// cannot resolve.  So report the count and say what it does and
			// does not settle.
			unsigned found = 0u;
			unsigned prev_block = 0u;
			for (uint32_t i = 0; i < have && found < 2u; i++) {
				bus_pins bp{}; bp.word = MC6809.trace_word(i);
				high_bus_pins hp{}; hp.word = MC6809.trace_hi_word(i);
				if (bp.bits.CS || bp.bits.RW ||
				    bp.bits.A != (SYSTEM_DAT_FAULT & registers::register_mask)) {
					prev_block = hp.bits.NMI_BLOCK;
					continue;
				}
				uint32_t j = i;
				while (j < have) {
					high_bus_pins hn{}; hn.word = MC6809.trace_hi_word(j);
					if (hn.bits.NMI_BLOCK != prev_block)
						break;
					j++;
				}
				found++;
				if (j >= have)
					printf("  %-5s at cycle %" PRIu32 ": pin never followed within the capture\n",
					       prev_block ? "clear" : "set", i);
				else
					printf("  %-5s at cycle %" PRIu32 ": pin followed within %" PRIu32 " E cycle%s%s\n",
					       prev_block ? "clear" : "set", i, j - i,
					       (j - i) == 1u ? "" : "s",
					       (j - i) <= 1u
					         ? " (the floor at this sampling rate)"
					         : " — LATE, at least one cycle beyond the GAL's deadline");
				prev_block = !prev_block;
			}
			if (found < 2u)
				printf("  only %u of 2 writes to $%04X found in %" PRIu32 " cycles\n",
				       found, REGISTER_BASE + SYSTEM_DAT_FAULT, have);
			printf("  one cycle means the pin moved somewhere between E-fall of\n"
			       "  the write and E-fall of the next cycle. The GAL's deadline\n"
			       "  is Q-rise, a quarter cycle into that window, so whether it\n"
			       "  was met needs a scope on NMIBlock against Q.\n\n");
			// Consume the one-shot: with the CPU about to be stopped it can
			// never fill, and an armed trace keeps the E-fall interrupt alive.
			MC6809.trace_stop();
			break;
		} else if (hexnum == 9) {
			// Single step (TODO 1b), composed from parts that already
			// work: the countdown NMI re-enabled in f1b36b7, the BREAK
			// snippet's copy of the stacked frame, break_vector_fetched
			// for "the NMI was taken", and break_frame_settled() for
			// "the copy has finished".
			//
			// The target is a FOUR-instruction loop of three different
			// lengths, and the stepping is judged by whether each stacked
			// PC is the SUCCESSOR of the last one.
			//
			//   $0000  10 CE E0 00  LDS #$E000   NMI is masked until S is
			//   $0004  12           NOP      2c  loaded, so this comes first
			//   $0005  B6 00 20     LDA $0020 5c
			//   $0008  12           NOP      2c
			//   $0009  20 F9        BRA $0004 3c
			//
			// It used to be a two-instruction NOP/BRA loop, judged by the
			// PC ALTERNATING - and that test cannot tell one instruction
			// from any ODD number of them.  It is how 991fb9d came to call
			// a 29-cycle arm "perfect": the board later showed LIC edges at
			// 15 18 20 23 25 28 after the RTI fetch, so arming at 29 steps
			// past SIX instruction boundaries, and alternated purely by
			// parity.  A four-state loop has no such hiding place, and the
			// 5-cycle LDA in the middle means a rule that depends on
			// instruction length cannot pass either.
			printf("Single-step test\n");
			static constexpr uint8_t step_target[] = {
				0x10u, 0xCEu, 0xE0u, 0x00u,
				0x12u, 0xB6u, 0x00u, 0x20u, 0x12u, 0x20u, 0xF9u
			};
			// Successor of each legal PC, one instruction on.
			static constexpr uint16_t STEP_PC[] = {
				0x0004u, 0x0005u, 0x0008u, 0x0009u
			};
			static constexpr uint16_t STEP_NEXT[] = {
				0x0005u, 0x0008u, 0x0009u, 0x0004u
			};
			// 29 E cycles after the release is what 991fb9d swept out as
			// "perfect" on the old two-instruction loop, and it is kept
			// here only as the negative control.  On this loop it fails,
			// which is the point: the board showed LIC edges at
			// 15 18 20 23 25 28 after the RTI fetch, so 29 arms six
			// instruction boundaries downstream.  The old loop could not
			// show that, because any ODD number of instructions alternates
			// between two PCs.
			static constexpr uint8_t STEP_ARM_CYCLES = 29u;
			static constexpr uint STEP_COUNT = 6u;

			MC6809.stop();
			sleep_ms(1u);
			// Before BREAK: copy() runs a snippet of its own.
			if (!copy(OUTWARDS, sizeof step_target, 0x0000u, step_target)) {
				printf("  could not plant the target loop\n");
				break;
			}
			const uint16_t step_saved_nmi =
				static_cast<uint16_t>(reg[REGISTER_VECTOR_NMI_OFFSET]);
			snippet_copy(BREAK);
			printf("  gate PCs                     verdict\n");
			// Pass 0 is the cycle count, kept as the negative control:
			// it should FAIL on this loop, and that failure is the
			// evidence that a count cannot step arbitrary code.
			for (uint pass = 0u; pass < 2u; pass++) {
			const bool use_lic = (pass != 0u);
			const char *step_label = use_lic ? "LIC" : "29";
			reg[REGISTER_VECTOR_NMI_OFFSET] =
				static_cast<uint16_t>(REGISTER_SNIPPET);
			reg[REGISTER_VECTOR_RESET_OFFSET] = static_cast<uint16_t>(0x0000);
			step_active = true;
			break_vector_fetched = false;
			MC6809.start();

			// The first stop is not a step: it is the countdown fired once
			// to get the guest into BREAK in the first place.
			system_tick.arm_countdown(1u);
			uint32_t waited = 0u;
			while (!break_vector_fetched && ++waited <= 1000u)
				sleep_ms(1u);
			bool step_ok = break_vector_fetched;
			if (!step_ok)
				printf("  the guest never reached BREAK - did it load S?\n");

			uint16_t pcs[STEP_COUNT] = {};
			uint n = 0u;
			for (; step_ok && n < STEP_COUNT; n++) {
				uint8_t frame[BREAK_FRAME_LEN];
				(void) break_frame_settled(frame, 50u);
				pcs[n] = static_cast<uint16_t>((frame[10] << 8) | frame[11]);
				break_vector_fetched = false;
				if (use_lic)
					step_request = true;		// gate on the next opcode fetch
				else
					step_arm_cycles = STEP_ARM_CYCLES;
				waited = 0u;
				while (!break_vector_fetched && ++waited <= 1000u)
					sleep_ms(1u);
				if (!break_vector_fetched) {
					printf("  step %u: no NMI came back\n", n + 1u);
					step_ok = false;
				}
			}

			step_active = false;
			step_logging = false;
			reg[REGISTER_SNIPPET + 14] = static_cast<uint8_t>(0xFEu);
			MC6809.stop();
			sleep_ms(1u);

			uint steps_of_one = 0u, wrong = 0u, strays = 0u;
			for (uint i = 0; i < n; i++) {
				uint slot = 4u;
				for (uint k = 0u; k < 4u; k++)
					if (pcs[i] == STEP_PC[k])
						slot = k;
				if (slot == 4u) {
					strays++;
					continue;
				}
				if (i == 0u)
					continue;
				uint prev = 4u;
				for (uint k = 0u; k < 4u; k++)
					if (pcs[i - 1] == STEP_PC[k])
						prev = k;
				if (prev < 4u && pcs[i] == STEP_NEXT[prev])
					steps_of_one++;
				else
					wrong++;
			}
			printf("  %-4s ", step_label);
			for (uint i = 0; i < STEP_COUNT; i++)
				printf(i < n ? "%04X " : ".... ", pcs[i]);
			if (n != STEP_COUNT)
				printf("  stalled after %u\n", n);
			else if (strays)
				printf("  %u outside the loop\n", strays);
			else if (wrong == 0u)
				printf("  ONE INSTRUCTION PER STEP\n");
			else
				printf("  %u/%u not the next instruction\n",
				       wrong, steps_of_one + wrong);
			// Where the LIC rising edges fell on the last step of this
			// gate, in E cycles after the snippet's RTI was fetched.  The
			// count gate arms at STEP_ARM_CYCLES, so the edge it lands
			// just past is the one the LIC gate should use.
			if (!use_lic) {
				printf("       edges after the RTI fetch:");
				for (uint i = 0u; i < step_edges; i++)
					printf(" %u", static_cast<uint>(step_edge[i]));
				printf("\n");
			}
			}	// cycle-count pass, then LIC pass
			reg[REGISTER_VECTOR_NMI_OFFSET] = step_saved_nmi;
			break;
		} else if (hexnum == 8) {
			// DAT fault.  The snippet unmaps one 8kB block of the running
			// task and then reads from it; the fault sampler latches the
			// fault, its direction, and the task and page the DAT saw.
			//
			// Pages 1, 2 and 4 drive one A_HI line high at a time and 6
			// drives two at once, so a line stuck high, stuck low or
			// crossed with another cannot pass all four.  The reported page
			// is the only evidence for those pins, and it is read back from
			// the bus rather than from anything the Pico already knows.
			//
			// Each page is faulted twice, once by a read and once by a
			// write, so the direction the sampler latches is checked against
			// a known answer rather than merely reported.  The later cases
			// fault from a non-zero task: the latched task is read back off
			// the task pins, so a task of zero every time - which is what
			// five lines stuck low would give - is not enough to prove them.
			//
			// A fault is fatal to the access that caused it, so every page
			// and direction needs its own run.
			printf("DAT fault test\n");
			// Extended LDA and STA differ in one bit, take the same operand,
			// and the accumulator already holds the byte a store would write,
			// so the direction of the faulting access is a one-byte patch.
			//
			// The pages walk each A_HI line alone and then two together; the
			// tasks walk each TASK line alone and then all five together.  A
			// line stuck either way, or crossed with its neighbour, fails
			// somewhere in that set.
			static constexpr struct {
				uint8_t task;
				uint8_t page;
				uint8_t opcode;
				char    dir;
				bool    reads;
			} fault_cases[] = {
				{  0u, 1u, 0xB6u, 'R', true  }, {  0u, 1u, 0xB7u, 'W', false },
				{  0u, 2u, 0xB6u, 'R', true  }, {  0u, 2u, 0xB7u, 'W', false },
				{  0u, 4u, 0xB6u, 'R', true  }, {  0u, 4u, 0xB7u, 'W', false },
				{  0u, 6u, 0xB6u, 'R', true  }, {  0u, 6u, 0xB7u, 'W', false },
				{  1u, 1u, 0xB6u, 'R', true  },
				{  2u, 1u, 0xB6u, 'R', true  },
				{  4u, 1u, 0xB6u, 'R', true  },
				{  8u, 1u, 0xB6u, 'R', true  },
				{ 16u, 1u, 0xB6u, 'R', true  },
				{ 31u, 1u, 0xB6u, 'R', true  },
			};
			bool all_pass = true;
			for (const auto &d : fault_cases) {
				const uint8_t page = d.page;
				MC6809.stop();
				sleep_ms(1u);	// let the Q-rising handler see the reset

				// Unmap the block from task zero, which is the only task the
				// GAL lets write DAT RAM - MMU_cs is qualified by Root.  This
				// runs a copy snippet of its own, so it has to finish before
				// the fault snippet is laid down.
				task_pins_update(MC6809.task_change(0u));
				const uint8_t unavailable = 0xFFu;
				if (!copy(OUTWARDS, 1u, static_cast<uint16_t>(0xFE00u + d.task * 8u + page),
					  &unavailable)) {
					printf("  task %2u page %u %c: FAIL  could not write the DAT entry\n",
					       d.task, page, d.dir);
					all_pass = false;
					continue;
				}

				// Then enter the task the fault is to come from.  A reset does
				// not disturb the task, so this survives the start below.
				MC6809.stop();
				sleep_ms(1u);
				task_pins_update(MC6809.task_change(d.task));

				snippet_copy(DAT_FAULT);
				// Only the access itself is patched: the opcode picks the
				// direction, the high byte picks the page.
				reg[REGISTER_SNIPPET_OFFSET + 4u] = d.opcode;
				reg[REGISTER_SNIPPET_OFFSET + 5u] = static_cast<uint8_t>(page << 5);
				// Aim the fault's NMI at the SYNC that ends the snippet, so
				// the handler parks instead of re-entering the code that
				// faulted and faulting again.
				reg[REGISTER_VECTOR_NMI_OFFSET] = static_cast<uint16_t>(REGISTER_SNIPPET + 7u);

				// The mask and the latch are cleared by the reset that
				// start() performs, so there is deliberately no local
				// workaround here: if that ever stops happening this test
				// should fail rather than paper over it.
				const uint32_t nmi_acked = MC6809.get_irq_ack_count(INTERRUPT_NMI);
				MC6809.start();
				uint32_t waited = 0u;
				while (!(static_cast<uint8_t>(reg[SYSTEM_DAT_FAULT]) & DAT_FAULT_FAULT)) {
					sleep_us(10u);
					if (++waited > 100000u)
						break;
				}
				// The latch is not the end of it.  The 6809 has to finish the
				// faulting instruction, run the one after it, and push twelve
				// bytes before it fetches the NMI vector - twenty-odd cycles
				// on a bus trace.  Stopping as soon as the fault appears takes
				// the clock away before any of that happens, which reads as an
				// undelivered NMI when the guest was simply never given time.
				for (uint32_t settle = 0u;
				     settle < 1000u
				     && MC6809.get_irq_ack_count(INTERRUPT_NMI) == nmi_acked;
				     settle++)
					sleep_us(10u);
				// Sampled before the stop, so the task is the one the fault
				// handler left behind rather than anything the stop does.
				const bool delivered =
					MC6809.get_irq_ack_count(INTERRUPT_NMI) > nmi_acked;
				const uint8_t task_after = MC6809.task_get();
				MC6809.stop();

				const uint8_t status = static_cast<uint8_t>(reg[SYSTEM_DAT_FAULT]);
				const uint8_t loc = static_cast<uint8_t>(reg[SYSTEM_DAT_FAULT_LOC]);
				const uint8_t got_page = static_cast<uint8_t>(loc & DAT_FAULT_LOC_PAGE_MASK);
				const uint8_t got_task = static_cast<uint8_t>(
					(loc & DAT_FAULT_LOC_TASK_MASK) >> DAT_FAULT_LOC_TASK_SHIFT);
				const bool raised = (status & DAT_FAULT_FAULT) != 0u;
				const bool got_reads = (status & DAT_FAULT_RW) != 0u;
				// Delivery is checked as well as latching.  The sampler
				// records a fault whether or not the 6809 ever takes the NMI,
				// so a test that stopped at "raised" passed for a long time
				// while no interrupt was reaching the processor at all.  The
				// switch to task zero is a second, independent witness: it
				// happens only on the NMI vector fetch.
				const bool ok = raised && got_page == page && got_task == d.task
						&& got_reads == d.reads
						&& delivered && task_after == 0u;
				printf("  task %2u page %u %c: %-4s  raised=%s  dir=%c  page=%u  task=%u"
				       "  nmi=%s  ->task%u\n",
				       d.task, page, d.dir, ok ? "PASS" : "FAIL", raised ? "yes" : "no",
				       got_reads ? 'R' : 'W', got_page, got_task,
				       delivered ? "yes" : "no", task_after);
				if (!raised)
					printf("    (no fault seen in %" PRIu32 "ms)\n", waited / 100u);
				all_pass = all_pass && ok;
			}
			// Put back the NMI vector and the task, so nothing after this
			// inherits the test's rigging.  The fault handler switches to
			// task zero itself on every fault, but a case that failed to
			// fault would not have, and would leave the guest somewhere odd.
			//
			// The fault mask is deliberately left as the last fault set it.
			// That is not rigging, it is the state a fault is supposed to
			// leave behind - an unhandled fault masks further ones until the
			// guest is reset, and reset is what lowers it.
			reg[REGISTER_VECTOR_NMI_OFFSET] = REGISTER_SNIPPET;
			task_pins_update(MC6809.task_change(0u));
			printf("DAT fault test: %s\n\n", all_pass ? "ALL PASS" : "FAIL");
			break;
		} else {
		irq_chunk_test:
			// Chunk-based interrupt test (irq 6)
			// Requires irq_test.s19 pre-loaded by pasting at console.
			// The chunk sets up its own vectors, runs SWI/SWI2/SWI3,
			// then signals DONE. Pico delivers NMI, FIRQ, IRQ and checks results.
			printf("Interrupt chunk test\n");

			// Load embedded irq_test chunk and set reset vector
			if (!copy(OUTWARDS, chunk_len[IRQ_TEST], CHUNK_ADDRESS, chunk_code[IRQ_TEST]))
				break;
			reg[REGISTER_VECTOR_RESET_OFFSET] = CHUNK_ADDRESS;
#ifdef DEBUG
			MC6809.clear_rti_count();
#endif
			MC6809.clear_irq_ack_count();
			MC6809.start();

			// Wait for READY signal (chunk has installed vectors)
			while (static_cast<uint8_t>(reg[REGISTER_BUFFER_OFFSET + 0]) == 0u) {
				sleep_us(1);
				if (++timeout > 1000000u) break;
			}
			if (timeout > 1000000u) {
				printf("Timeout waiting for READY\n");
				MC6809.stop();
				break;
			}

			// Wait for DONE signal (SWI/SWI2/SWI3 complete, interrupts unmasked)
			while (static_cast<uint8_t>(reg[REGISTER_BUFFER_OFFSET + 7]) == 0u) {
				sleep_us(1);
				if (++timeout > 1000000u) break;
			}
			if (timeout > 1000000u) {
				printf("Timeout waiting for DONE\n");
				MC6809.stop();
				break;
			}

			// Deliver hardware interrupts.  Hold each line until the 6809
			// fetches the matching vector, then release it, the way a real
			// device holds its request until acknowledged.
			//
			// A fixed assertion window races.  Five microseconds is about
			// five bus cycles at 1 MHz, and in that time the request has to
			// reach Core 1's poll of console_has_interrupt(), become a
			// pending bit, be driven onto the pin at the next Q-rising and
			// then survive to the processor's sampling point.  A short pulse
			// can expire unseen, which showed up as FIRQ or IRQ
			// intermittently reporting 0x00 while the rest of the run passed.
			//
			// Releasing at the vector fetch rather than after the handler
			// cannot cause a second entry: the 6809 masks I on IRQ entry and
			// both F and I on FIRQ entry, so nothing can be taken again until
			// the RTI that follows the release.
			static constexpr struct { interrupt intr; const char *name; } deliver[] = {
				{INTERRUPT_NMI,  "NMI"},
				{INTERRUPT_FIRQ, "FIRQ"},
				{INTERRUPT_IRQ,  "IRQ"},
			};
			for (const auto &d : deliver) {
				const uint32_t acked = MC6809.get_irq_ack_count(d.intr);
				console_set_interrupt(d.intr);
				uint32_t waited = 0u;
				while (MC6809.get_irq_ack_count(d.intr) == acked) {
					sleep_us(1);
					if (++waited > 100000u) {
						printf("  %s: timeout waiting for vector fetch\n", d.name);
						break;
					}
				}
				console_set_interrupt(INTERRUPT_NONE);
				sleep_ms(2u);	// let the handler run to its RTI
			}

			MC6809.stop();

			// Check results in SHARED buffer
			static const struct { const char *name; uint8_t offset; uint8_t expected; } checks[] = {
				{"SWI",  1, 0x11},
				{"SWI2", 2, 0x22},
				{"SWI3", 3, 0x33},
				{"NMI",  4, 0xAA},
				{"FIRQ", 5, 0xBB},
				{"IRQ",  6, 0xCC},
			};
			bool success = true;
			for (const auto &c : checks) {
				const uint8_t got = static_cast<uint8_t>(reg[REGISTER_BUFFER_OFFSET + c.offset]);
				const bool ok = (got == c.expected);
				if (!ok) success = false;
				printf("  %s: %s (expected 0x%02X, got 0x%02X)\n",
				       c.name, ok ? "PASS" : "FAIL", c.expected, got);
			}
#ifdef DEBUG
			static const char *irq_names[] = {
				"ILL", "SWI3", "SWI2", "FIRQ", "IRQ", "SWI", "NMI", "RESET", "NONE"
			};
			printf("  IRQ ack: %" PRIu32 " [", MC6809.get_irq_ack_total());
			for (int i = 0; i < NUM_INTERRUPTS; i++) {
				uint32_t c = MC6809.get_irq_ack_count(static_cast<interrupt>(i));
				if (c > 0u)
					printf(" %s=%" PRIu32, irq_names[i], c);
			}
			printf(" ]  RTI: %" PRIu32 "\n", MC6809.get_rti_count());
#endif
			printf("Interrupt test: %s\n", success ? "PASS" : "FAIL");

			// --- Watchpoint test phase ---
			// Set WP_PHASE=1 so the chunk takes the watchpoint path on restart.
			// The chunk code is still in RAM — just restart with the same reset vector.
			printf("Watchpoint test\n");
			reg[REGISTER_BUFFER_OFFSET + 8] = static_cast<uint8_t>(1u);  // S_WP_PHASE
			MC6809.start();

			// Wait for WP_ARMED signal (chunk has armed the watchpoint)
			timeout = 0u;
			while (static_cast<uint8_t>(reg[REGISTER_BUFFER_OFFSET + 9]) != 2u) {
				sleep_us(1);
				if (++timeout > 1000000u) break;
			}
			if (timeout > 1000000u) {
				printf("  Timeout waiting for watchpoint arm\n");
				MC6809.stop();
				break;
			}

			// The chunk will now write to $FFF0, triggering NMI → BREAK.
			// BREAK copies 12 bytes of stack frame to SHARED and spins.
			// Wait for SHARED[0] to become non-zero (BREAK writes CC there).
			sleep_ms(5u);  // give the trigger + NMI + BREAK time to complete

			MC6809.stop();

			// BREAK snippet copies stack[0..11] to SHARED[0..11].
			// Stack frame: CC(0) A(1) B(2) DP(3) X(4-5) Y(6-7) U(8-9) PC(10-11)
			const uint8_t wp_cc = static_cast<uint8_t>(reg[REGISTER_BUFFER_OFFSET + 0]);
			const uint8_t wp_a  = static_cast<uint8_t>(reg[REGISTER_BUFFER_OFFSET + 1]);
			const uint8_t wp_b  = static_cast<uint8_t>(reg[REGISTER_BUFFER_OFFSET + 2]);
			const uint8_t wp_dp = static_cast<uint8_t>(reg[REGISTER_BUFFER_OFFSET + 3]);
			const uint16_t wp_x = (static_cast<uint8_t>(reg[REGISTER_BUFFER_OFFSET + 4]) << 8)
			                    |  static_cast<uint8_t>(reg[REGISTER_BUFFER_OFFSET + 5]);
			const uint16_t wp_y = (static_cast<uint8_t>(reg[REGISTER_BUFFER_OFFSET + 6]) << 8)
			                    |  static_cast<uint8_t>(reg[REGISTER_BUFFER_OFFSET + 7]);
			const uint16_t wp_u = (static_cast<uint8_t>(reg[REGISTER_BUFFER_OFFSET + 8]) << 8)
			                    |  static_cast<uint8_t>(reg[REGISTER_BUFFER_OFFSET + 9]);
			const uint16_t wp_pc = (static_cast<uint8_t>(reg[REGISTER_BUFFER_OFFSET + 10]) << 8)
			                     |  static_cast<uint8_t>(reg[REGISTER_BUFFER_OFFSET + 11]);
			if (wp_pc != 0u) {
				printf("  Watchpoint: PASS (caught write at PC=$%04X)\n", wp_pc);
				printf("  CC=%02X A=%02X B=%02X DP=%02X X=%04X Y=%04X U=%04X PC=%04X\n",
				       wp_cc, wp_a, wp_b, wp_dp, wp_x, wp_y, wp_u, wp_pc);
			} else {
				printf("  Watchpoint: FAIL (no break detected)\n");
				success = false;
			}
			// Reported here, so leave nothing for the console loop to
			// report a second time - and do not leave the guest's NMI
			// vector pointing at BREAK once the test is over.
			watchpoint_disarm();

			printf("Interrupt chunk test: %s\n", success ? "ALL PASS" : "FAIL");
			break;
		}

		if (final_stack_length) {
			bool success = true;
			for (uint16_t i = 0; i < final_stack_length; i++)
				if (reg[REGISTER_BUFFER_OFFSET + (16u - final_stack_length) + i] != final_stack[i]) {
					printf("mismatch %04X %02X %02X\n",
					       REGISTER_BUFFER_OFFSET + (16u - final_stack_length) + i,
					       static_cast<uint8_t>(registers::write(REGISTER_BUFFER_OFFSET + (16u - final_stack_length) + i)),
					       final_stack[i]);
					success = false;
				}
			if (timeout > 1000000u) {
				printf("Timeout reached\n");
				success = false;
			}
			printf("%s: Timeout counter: %u  Test result: %s\n",
			       test_name, timeout, success ? "PASS" : "FAIL");
#ifdef DEBUG
			static const char *irq_names[] = {
				"ILL", "SWI3", "SWI2", "FIRQ", "IRQ", "SWI", "NMI", "RESET", "NONE"
			};
			const uint32_t irq_ack_total = MC6809.get_irq_ack_total();
			const bool irq_ack_ok = (irq_ack_total == expected_irq_ack_count);
			if (!irq_ack_ok)
				success = false;
			printf("%s: IRQ ack: %s (expected %" PRIu32 ", got %" PRIu32 ") [",
			       test_name, irq_ack_ok ? "PASS" : "FAIL",
			       expected_irq_ack_count, irq_ack_total);
			for (int i = 0; i < NUM_INTERRUPTS; i++) {
				uint32_t c = MC6809.get_irq_ack_count(static_cast<interrupt>(i));
				if (c > 0u)
					printf(" %s=%" PRIu32, irq_names[i], c);
			}
			printf(" ]\n");
			const uint32_t rti_caught = MC6809.get_rti_count();
			const bool rti_ok = (rti_caught == expected_rti_count);
			if (!rti_ok)
				success = false;
			printf("%s: RTI capture: %s (expected %" PRIu32 ", got %" PRIu32 ")\n",
			       test_name, rti_ok ? "PASS" : "FAIL",
			       expected_rti_count, rti_caught);
#endif
		}
		break;
	}

	case CMD_TRACE:
	case CMD_TRACE_LEN: {
		// Decode the one-word-per-E-cycle capture via the bus_pins union.
		// A5-A0 is the low 6 bits of the bus address; for any $FFC0-page
		// access that is the full register offset.  Every pin is sampled at
		// E-falling, so address, data, R/W and status are all settled.
		const uint32_t have = MC6809.trace_count();
		if (have == 0u) {
			printf("trace: empty — arm with 'trace on', 'trace <irq>', "
			       "or a guest poke of $%04X\n\n", REGISTER_BASE + SYSTEM_TRACE);
			break;
		}
		// Row numbers stay absolute, so an offset dump still lines up with a
		// full one — the point of the offset is to reach deep into a capture
		// without pushing thousands of rows through the console.
		const uint32_t start = MIN(cmd.trace.start, have);
		const uint32_t avail = have - start;
		const uint32_t len = (cmd.tag == CMD_TRACE_LEN) ? MIN(cmd.trace.length, avail) : avail;
		// B L V are the pins as the capture state machine saw them at this
		// row's E-falling, so they are the raw in-advance signals: they
		// describe the NEXT bus cycle.  b l v are what the Q-rising handler
		// took from the DMA mirror for this row, which is the sample taken
		// at the END of the previous row - that one-cycle re-phasing is the
		// whole point of the mirror, and it is what makes LIC mean "opcode
		// fetch" and AVMA mean "this cycle is a valid access".
		//
		// So ! compares b l v against the PREVIOUS row's B L V, not this
		// row's.  A clean column means the mirror carries the pins faithfully
		// and lands them on the cycle they describe; a ! is a fault in the
		// DMA path or a lost sample, not something the 6809 did.
		const uint32_t mirrored = MC6809.trace_mirror_count();
		uint32_t mirror_differs = 0u;
		printf("    #  EQ R/W  A  D  CS  bus-state    B L V  b l v !  "
		       "H N F I  TSK Pg  FM\n");
		for (uint32_t i = start; i < start + len; i++) {
			bus_pins bp{}; bp.word = MC6809.trace_word(i);
			high_bus_pins hp{}; hp.word = MC6809.trace_hi_word(i);
			BLA bla{}; bla.byte = static_cast<uint8_t>(bp.bits.BUSY_LIC_AVMA);
			const bool have_mirror = i < mirrored;
			BLA mir{}; mir.byte = have_mirror ? MC6809.trace_mirror_byte(i) : 0u;
			// The pins from the row before: what this row's mirror value
			// should be carrying.  Row 0 has no predecessor to check against.
			BLA prev{};
			bus_pins pbp{}; pbp.word = (i > 0u) ? MC6809.trace_word(i - 1u) : 0u;
			prev.byte = static_cast<uint8_t>(pbp.bits.BUSY_LIC_AVMA);
			const bool differs = have_mirror && i > 0u && mir.byte != prev.byte;
			if (differs)
				mirror_differs++;
			printf("%5" PRIu32 "  %c%c  %c  %02X %02X  %c   %-11s  %c %c %c  "
			       "%c %c %c %c  %c %c %c %c  "
			       "%2u%c  %u  %c%c\n",
			       i,
			       bp.bits.E ? 'E' : '.',
			       bp.bits.Q ? 'Q' : '.',
			       bp.bits.RW ? 'R' : 'W',
			       static_cast<unsigned>(bp.bits.A),
			       static_cast<unsigned>(bp.bits.D),
			       bp.bits.CS ? '.' : '*',		// !CS: '*' = asserted (low)
			       bus_state_string[bp.bits.BS_BA & 0x03u],
			       bla.bit.BUSY ? 'B' : '.',
			       bla.bit.LIC  ? 'L' : '.',
			       bla.bit.AVMA ? 'V' : '.',
			       have_mirror ? (mir.bit.BUSY ? 'b' : '.') : '?',
			       have_mirror ? (mir.bit.LIC  ? 'l' : '.') : '?',
			       have_mirror ? (mir.bit.AVMA ? 'v' : '.') : '?',
			       differs ? '!' : '.',
			       bp.bits.HALT ? '.' : 'H',	// active-low outputs: letter = asserted
			       bp.bits.NMI  ? '.' : 'N',
			       bp.bits.FIRQ ? '.' : 'F',
			       bp.bits.IRQ  ? '.' : 'I',
			       static_cast<unsigned>(hp.bits.TASK),
			       hp.bits.TASK0 ? '0' : ' ',	// Root: task-zero flag to the DAT
			       static_cast<unsigned>(hp.bits.A_HI),	// A15-A13, the DAT page
			       hp.bits.FAULT ? 'F' : '.',
			       hp.bits.NMI_BLOCK ? 'M' : '.');
		}
		printf("%" PRIu32 " of %" PRIu32 " cycles%s\n", len, have,
		       MC6809.trace_is_full() ? " (buffer full)" : "");
		// The mirror column is written by the Q handler from trace_arm(),
		// while the DMA columns start when the trigger fires.  With 'trace
		// on' those are the same moment; with a vector trigger the handler
		// starts first, so expect a constant row offset and read the columns
		// for agreement of values rather than row-for-row.
		printf("mirror: %" PRIu32 " rows recorded, %" PRIu32
		       " of %" PRIu32 " shown differ from the pins\n\n",
		       mirrored, mirror_differs, len);
		break;
	}

	case CMD_TRACE_ARM:
		MC6809.trace_arm();
		printf("trace armed: %u-cycle one-shot, capturing from now\n\n", TRACE_SIZE);
		break;

	case CMD_TRACE_OFF:
		MC6809.trace_clear();
		printf("trace stopped and cleared\n\n");
		break;

	case CMD_TRACE_IRQ: {
		static constexpr const char *const inames[NUM_INTERRUPTS] = {
			"ILLEGAL", "SWI3", "SWI2", "FIRQ", "IRQ", "SWI", "NMI", "RESET", "NONE"
		};
		auto it = static_cast<interrupt>(cmd.trace.intr);
		MC6809.trace_set_irq_trigger(it);
		printf("trace will arm (one-shot) when the %s vector is taken\n\n", inames[it]);
		break;
	}

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
		if (!MC6809.assert_powered())
			break;
		MC6809.stop();
		break;

	case CMD_STOP_HALT:
		if (!MC6809.assert_powered())
			break;
		MC6809.halt();
		break;

	case CMD_STOP_RESTART:
		if (!MC6809.assert_powered())
			break;
		MC6809.release();
		break;

	case CMD_STOP_STOP:
		if (!MC6809.assert_powered())
			break;
		MC6809.stop();
		break;

	case CMD_BREAK: {
		if (!MC6809.assert_powered())
			break;
		if (break_active != INTERRUPT_NONE) {
			printf("Break already active (use 'break return' first)\n");
			break;
		}
		// NMI break: load the BREAK snippet into the snippet block,
		// point the NMI vector there, and trigger NMI.  The 6809 pushes its
		// full register set onto the stack and the snippet catches the 6809.
		static constexpr uint32_t BREAK_VECTOR_MS = 1000u;
		static constexpr uint32_t BREAK_SETTLE_MS = 50u;
		snippet_copy(BREAK);
		for (uint i = 0; i < 16; i++) {
			reg[REGISTER_BUFFER_OFFSET + i] = static_cast<uint8_t>(0x00);
			registers::write(REGISTER_BUFFER_OFFSET + i) = static_cast<uint8_t>(0x00);
		}
		break_saved_nmi_vector = static_cast<uint16_t>(reg[REGISTER_VECTOR_NMI_OFFSET]);
		reg[REGISTER_VECTOR_NMI_OFFSET] = static_cast<uint16_t>(REGISTER_SNIPPET);
		printf("NMI break -> %04X (was %04X)\n", REGISTER_SNIPPET, break_saved_nmi_vector);
		break_vector_fetched = false;
		console_set_interrupt(INTERRUPT_NMI);
		// Wait for the 6809 to take the NMI, then for the snippet's copy of
		// the stacked frame to settle.  Both waits are on evidence that says
		// what it means: the vector fetch the Q-rising observer reports as
		// BUS_IRQ - the bus event this wanted BS_IRQ for all along - and then
		// a frame that has arrived and stopped changing.
		//
		// Watching SHARED for the PC to go non-zero did both jobs badly.  It
		// returned mid-copy, printing a PC with only its high byte in, since
		// the snippet writes PC high before PC low and either byte satisfied
		// it.  And it could not tell a frame not yet copied from one whose PC
		// is legitimately $0000, so a break there reported "No NMI?" with the
		// frame sitting correctly in the buffer.
		uint32_t waited = 0u;
		while (!break_vector_fetched && ++waited <= BREAK_VECTOR_MS)
			sleep_ms(1u);
		console_set_interrupt(INTERRUPT_NONE);
		if (!break_vector_fetched) {
			// Put the guest's vector back.  A break that never happened
			// must leave nothing behind: see break_saved_nmi_vector.
			reg[REGISTER_VECTOR_NMI_OFFSET] = break_saved_nmi_vector;
			// Say WHY it might not have arrived.  The 6809 does not
			// recognise NMI after reset until the first program load
			// of S, so a guest running something that never touches
			// the stack pointer - LOOP_RW from the loop command, for
			// one - can never be broken into, and the NMI sits on the
			// pin ignored.  Without this note the message reads as a
			// failure of the break machinery, which is a different
			// thing entirely and wastes an afternoon.
			printf("Timeout - no NMI was taken.  NMI vector restored "
			       "to %04X\n", break_saved_nmi_vector);
			printf("  the 6809 ignores NMI until the guest's first LDS "
			       "after reset - has it loaded S?\n");
			printf("  (the loop command's snippet never does, so break "
			       "cannot interrupt it)\n");
			break;
		}
		uint8_t frame[BREAK_FRAME_LEN];
		const bool settled = break_frame_settled(frame, BREAK_SETTLE_MS);
		break_active = INTERRUPT_NMI;
		uint8_t cc = frame[0];
		printf("CC: ");
		for (uint i = 0; i < 8; i++) {
			printf("%c", cc & 0x80 ? "EFHINZVC"[i] : '-');
			cc <<= 1;
		}
		printf("\nA: %02X  B: %02X  DP: %02X\nX: %02X%02X  Y: %02X%02X\nU: %02X%02X  PC: %02X%02X\n",
			frame[1], frame[2], frame[3],
			frame[4], frame[5], frame[6], frame[7],
			frame[8], frame[9], frame[10], frame[11]);
		if (!settled)
			printf("(registers still changing after %" PRIu32 "ms - may be incomplete)\n",
			       BREAK_SETTLE_MS);
		break;
	}

	case CMD_BREAK_ADDR:
		// TODO: address-based breakpoint at $%04X
		printf("Address breakpoint at $%04X (not yet implemented)\n", cmd.brk.address);
		break;

	case CMD_BREAK_RETURN:
		if (!MC6809.assert_powered())
			break;
		if (break_active == INTERRUPT_NONE) {
			printf("No active break to return from\n");
			break;
		}
		printf("Returning from %s break\n",
			break_active == INTERRUPT_NMI ? "NMI" :
			break_active == INTERRUPT_SWI ? "SWI" : "???");
		reg[REGISTER_SNIPPET + 14] = static_cast<uint8_t>(0x00);
		// The guest gets its own NMI vector back with its execution.
		reg[REGISTER_VECTOR_NMI_OFFSET] = break_saved_nmi_vector;
		printf("NMI vector restored to %04X\n", break_saved_nmi_vector);
		break_active = INTERRUPT_NONE;
		break;

	case CMD_FREQ:
		if (!MC6809.assert_powered())
			break;
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

	case CMD_FLOW: {
		acia_status_t console{}, aux{};
		status_gather_flow(&console, 0u);
		status_gather_flow(&aux, 1u);
		printf("Flow control (the UART console has none and needs none):\n");
		status_print_flow("Console", &console);
		status_print_flow("Aux", &aux);
		break;
	}

	case CMD_FLOW_RTS: {
		const uint8_t port = static_cast<uint8_t>(cmd.flow.port - 1u);
		uart_port_get(port).honour_host_rts = cmd.flow.honour_host_rts;
		printf("Port %u: host RTS %s\n", cmd.flow.port,
		       cmd.flow.honour_host_rts
			       ? "honoured - output waits for a host that says it can take it"
			       : "ignored - output is written whatever RTS says");
		break;
	}

	case CMD_SRECORD: {
		if (!MC6809.assert_powered())
			break;
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

	// Only wait for Core 1 if there is a guest for it to be servicing.  When
	// the supply was missing at boot the switch was never closed and Core 1
	// was never launched, so waiting on it here would hang the console - and
	// the console is the one thing that has to survive a missing supply,
	// since it is where "power on" is typed once the supply arrives.
	// Bounded, and only entered when there is a guest for Core 1 to be
	// servicing.  A Core 1 that never comes up must not cost the console:
	// the console is where the problem gets diagnosed, and hanging here
	// leaves no way in at all.
	absolute_time_t core1_deadline = make_timeout_time_ms(CORE1_START_TIMEOUT_MS);
	while (POWER_IS_ASSERTED && !core1_initialised
	       && absolute_time_diff_us(get_absolute_time(), core1_deadline) > 0) {
		sleep_ms(1u);
		usb_poll();
	}

	print_time();
	if (!POWER_IS_ASSERTED)
		printf("Guest is unpowered; connect the supply, then \"power on\".\n");
	else if (!core1_initialised)
		printf("*** Core 1 never initialised (%" PRIu32 "ms); console only ***\n",
		       CORE1_START_TIMEOUT_MS);

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
	// A 32-bit microsecond deadline, read with one APB access from
	// timer_hw->timerawl, rather than an absolute_time_t: get_absolute_time()
	// takes two APB reads through the latching protocol and
	// absolute_time_diff_us() then does 64-bit arithmetic, and this is tested
	// on every pass of a loop whose rate governs the CDC pump and so
	// DriveWire's throughput.  A 32-bit microsecond counter wraps every 71
	// minutes, so the signed difference is good for any deadline within about
	// 35 minutes of now; this one is 10 ms away.
	uint32_t watchdog_next = timer_hw->timerawl + WATCHDOG_INTERVAL_MS * 1000u;
	bool watchdog_stalled = false;

	for (;;) {
		// The supply monitor has already opened the load switch; finish the
		// job here, where stopping Core 1, the PIOs and the DMA is safe.
		// The console itself is untouched, so the supply can be looked at
		// and "power on" tried again.
		if (rail_collapsed) {
			rail_collapsed = false;
			printf("\n*** external 5V collapsed to %.2fV - guest powered down ***\n",
			       adc_rail_volts(rail_trip_raw));
			printf("    (reads %.2fV now the load is off)\n",
			       adc_rail_volts(rail_raw));
			if (!MC6809.is_unpowered())
				reset_power();
			printf("Console still up; check the supply, then \"power on\".\n");
			prompted = false;
		}

		if (static_cast<int32_t>(timer_hw->timerawl - watchdog_next) >= 0) {
			const uint32_t cur_loop = core1_loop_count;
			const uint32_t cur_isr = core1_isr_count;
			if (cur_loop == watchdog_last_loop_count && core1_initialised) {
				watchdog_miss_count++;
				if (!watchdog_stalled && watchdog_miss_count >= WATCHDOG_THRESHOLD) {
					const bool isr_alive = (cur_isr != watchdog_last_isr_count);
					printf("\n*** Core 1 for(;;) loop stalled (%" PRIu32 "ms)!\n",
					       watchdog_miss_count * WATCHDOG_INTERVAL_MS);
					if (!isr_alive)
						printf("*** ISRs also stopped — probable hard fault or deadlock\n");
					else
						printf("*** ISRs still running — thread mode starved or hung\n");
					printf("  loop=%" PRIu32 "  isr=%" PRIu32 "\n",
					       cur_loop, cur_isr);
					printf("  last_write=%02X  last_read=%02X\n",
					       core1_last_write_loc, core1_last_read_loc);
					printf("  pending_interrupts=%02X  write_ring: head=%u tail=%u\n",
					       pending_interrupts, write_ring_head, write_ring_tail);
					printf("  ACIA: int_status=%02X  Aux: int_status=%02X\n",
					       fast_serial.interrupt_status(), aux_serial.interrupt_status());
					printf("  Tick: enabled=%d  irq_pending=%d\n",
					       static_cast<int>(system_tick.is_enabled()),
					       static_cast<int>(system_tick.is_irq_pending()));
					// Stack watermark check
					{
						volatile uint32_t *bottom = &__StackOneBottom;
						volatile uint32_t *top = &__StackOneTop;
						volatile uint32_t *p = bottom;
						while (p < top && *p == 0xDEADBEEFu)
							p++;
						uint32_t used = (top - p) * sizeof(uint32_t);
						uint32_t total = (top - bottom) * sizeof(uint32_t);
						printf("  stack: %" PRIu32 "/%" PRIu32 " bytes used", used, total);
						if (p == bottom)
							printf(" *** STACK OVERFLOW ***");
						printf("\n");
					}
					watchdog_stalled = true;
				}
			} else {
				if (watchdog_stalled) {
					printf("*** Core 1 for(;;) loop resumed after %" PRIu32 "ms (loop=%" PRIu32 ")\n",
					       watchdog_miss_count * WATCHDOG_INTERVAL_MS,
					       cur_loop);
				}
				watchdog_stalled = false;
				watchdog_miss_count = 0u;
			}
			watchdog_last_loop_count = cur_loop;
			watchdog_last_isr_count = cur_isr;
			watchdog_next = timer_hw->timerawl + WATCHDOG_INTERVAL_MS * 1000u;
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
				watchdog_next = timer_hw->timerawl + WATCHDOG_INTERVAL_MS * 1000u;
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
		mc6809::uart_task(idx > 0);

		// Check for a watchpoint hit.  Once the NMI has been raised the BREAK
		// snippet copies the stacked registers into SHARED, so report from a
		// settled copy rather than from whatever is there when we look.
		if (watchpoint_armed && watchpoint_fired) {
			static constexpr uint32_t WATCHPOINT_SETTLE_MS = 50u;
			uint8_t frame[BREAK_FRAME_LEN];
			const bool settled = break_frame_settled(frame, WATCHPOINT_SETTLE_MS);
			const uint16_t wp_pc = static_cast<uint16_t>((frame[10] << 8) | frame[11]);
			printf("\n*** WATCHPOINT HIT ***\n");
			printf("  CC=%02X A=%02X B=%02X DP=%02X X=%04X Y=%04X U=%04X PC=%04X\n",
			       frame[0], frame[1], frame[2], frame[3],
			       (frame[4] << 8) | frame[5], (frame[6] << 8) | frame[7],
			       (frame[8] << 8) | frame[9], wp_pc);
			if (!settled)
				printf("  (registers still changing after %" PRIu32 "ms - may be incomplete)\n",
				       WATCHPOINT_SETTLE_MS);
			printf("  Write to vector area at PC=$%04X\n", wp_pc);
			watchpoint_disarm();
			MC6809.stop();
			prompted = false;
		}
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
	printf("Core1 process starting\n");

	// Trace pins - ISR set.  Named individually rather than as a run: they
	// sit on 37-38 on version 1 but 38-39 on version 2, where 37 is TASK0.
	static constexpr uint8_t trace_pins[] = { GPIO_TRACE_C, GPIO_TRACE_D };
	for (const uint8_t p : trace_pins) {
		gpio_init(p);
		gpio_put(p, false);
		gpio_set_dir(p, GPIO_OUT);
	}

	fast_serial.reset();
	aux_serial.reset();
	printf("Core1 console initialised/reset\n");

	pio_sm_set_enabled(pio_clock.pio, pio_clock.sm, true);
	printf("Core1 E/Q clocks started\n");
	MC6809.set_e_frequency(GUEST_CLK_DEFAULT, pio_clock);
	printf("Core1 E/Q clock set to %.1f MHz\n", MC6809.get_e_frequency());

	// Added once per boot and never removed.  The SDK books a raw GPIO handler
	// against the core that adds it - raw_irq_mask[core] - and only that core
	// can take it off again.  Core 1 is killed with multicore_reset_core1(),
	// so it never gets the chance, and a removal from Core 0 clears Core 0's
	// mask instead; the relaunched Core 1 then trips "should not add multiple
	// handlers for the same event", which is a hard_assert on the second
	// "power on".  Leaving it in the chain is safe: only Core 1 ever enables
	// the bank 0 IRQ, and the enable below acknowledges any edge that latched
	// while the guest was powered off.
	static bool q_handler_added = false;
	if (!q_handler_added) {
		gpio_add_raw_irq_handler(GPIO_Q, gpio_q_irq_handler);
		q_handler_added = true;
	}
	gpio_set_irq_enabled(GPIO_Q, GPIO_IRQ_EDGE_RISE, true);
	irq_set_priority(pio_clock.irq, 1);
	irq_set_enabled(pio_clock.irq, true);
	printf("Core1 BA/BS and BUSY/LIC/AVMA bus observer started\n");

	irq_set_exclusive_handler(piodma_read.irq, dma_bus_read_irq_handler);
	irq_set_priority(piodma_read.irq, 0);
	irq_set_enabled(piodma_read.irq, true);
	dma_channel_set_irq0_enabled(piodma_read.data_channel, true);
	dma_channel_start(piodma_read.address_channel);
	pio_sm_put(piodma_read.pio, piodma_read.sm, registers::read_address() >> 6); // Six bits for A0-5
	printf("Core1 Main read DMA chain configured\n");

	irq_set_exclusive_handler(piodma_write.irq, dma_bus_write_irq_handler);
	irq_set_priority(piodma_write.irq, 0);
	irq_set_enabled(piodma_write.irq, true);
	dma_channel_set_irq1_enabled(piodma_write.data_channel, true);
	dma_channel_start(piodma_write.address_channel);
	pio_sm_put(piodma_write.pio, piodma_write.sm, registers::write_address() >> 6); // Six bits for A0-5
	printf("Core1 Main write DMA chain configured\n");

	pio_sm_set_enabled(piodma_read.pio, piodma_read.sm, true);
	pio_sm_set_enabled(piodma_write.pio, piodma_write.sm, true);
	printf("Core1 read pio%d/sm%d and write pio%d/sm%d started\n",
	       PIO_NUM(piodma_read.pio),
	       piodma_read.sm,
	       PIO_NUM(piodma_write.pio),
	       piodma_write.sm);

	pio_sm_set_enabled(task_output.pio, task_output.sm, true);
	printf("Core1 task pio%d/sm%d started\n", PIO_NUM(task_output.pio), task_output.sm);

	// After the clock program, which is what ticks it: this state machine
	// does nothing but wait on the clock's E-falling flag.  Its DMA was
	// started at configure time and is endless, so there is nothing else to
	// arm; with the state machine disabled the mirror simply stayed at zero,
	// which reads as BUSY/LIC/AVMA all low on every cycle.
	pio_sm_set_enabled(bus_status.pio, bus_status.sm, true);
	printf("Core1 bus status pio%d/sm%d started\n",
	       PIO_NUM(bus_status.pio), bus_status.sm);

	pio_sm_set_enabled(rti_detect.pio, rti_detect.sm, true);
	pio_set_irq0_source_enabled(rti_detect.pio, RTI_PIO_IRQ_SOURCE, true);
	irq_set_exclusive_handler(rti_detect.irq, pio_rti_irq_handler);
	irq_set_priority(rti_detect.irq, 1);
	irq_set_enabled(rti_detect.irq, true);
	printf("Core1 RTI detect pio%d/sm%d started\n", PIO_NUM(rti_detect.pio), rti_detect.sm);

	pio_sm_set_enabled(bus_fault.pio, bus_fault.sm, true);
	pio_set_irq0_source_enabled(bus_fault.pio, BUS_FAULT_PIO_IRQ_SOURCE, true);
	irq_set_exclusive_handler(bus_fault.irq, pio_bus_fault_irq_handler);
	// Above the Q-rising handler (priority 1), so a fault's NMI request is in
	// place before the pass that acts on it.  Safe only because this handler
	// touches no state that one read-modify-writes - see
	// dat_fault_nmi_request.  Level with the DMA ISRs, which it must not
	// delay.
	irq_set_priority(bus_fault.irq, 0);
	irq_set_enabled(bus_fault.irq, true);
	printf("Core1 bus fault detect pio%d/sm%d started\n", PIO_NUM(bus_fault.pio), bus_fault.sm);

	sysreq_initialise();

	// Start the 50 Hz tick timer (hardware alarm, runs independently of E clock).
	system_tick.start();
	printf("Core1 50 Hz tick timer started\n");

	printf("Core1 process started\n");

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
		printf("Core1 stack: %zu bytes total, %" PRIu32 " bytes free at start\n",
		       (&__StackOneTop - &__StackOneBottom) * sizeof(uint32_t),
		       stack_free);
	}

	core1_initialised = true;

	for (;;) {

		core1_loop_count++;
		// Interrupts need to be done before the falling edge of Q
		if (q_rising_edge) {
			q_rising_edge = false;
			core1_qrise_count++;

			// Ask each device what it wants and keep the three bits the
			// pins can carry.  This used to count into a
			// uint32_t[NUM_INTERRUPTS] array, zeroed and indexed on every
			// Q-rising, because the array was handed to
			// mc6809::apply_interrupts() - which no longer exists.  The
			// counts were never read as counts.
			//
			// Single volatile write at the end: the Q-rising ISR drives all
			// three pins from this byte.
			const uint8_t pend = pend_bit(console_has_interrupt())
					   | pend_bit(system_tick.has_interrupt())
					   | pend_bit(fast_serial.has_interrupt())
					   | pend_bit(aux_serial.has_interrupt());
			if (pend & PEND_IRQ)
				core1_irq_assert_count++;
			pending_interrupts = pend;
		}

		// Console writes into the register block, carried out as the guest's
		// would be: the byte into the write side, then the write ISR's own
		// follow-up, with interrupts off so the ISR cannot interleave.  The
		// ring entries that queues are dispatched just below, in this pass.
		// queue_try_remove() takes the queue's spinlock before it looks at
		// the level, so an empty queue - which is almost every pass - costs
		// a lock acquire, a release and their barriers.  The unsafe level
		// read is two loads and no lock; a stale "empty" only defers a
		// console write by one pass, and console writes are human-paced.
		if (queue_get_level_unsafe(&console_write_queue) != 0u) {
			write_entry w;
			while (queue_try_remove(&console_write_queue, &w)) {
				const uint32_t saved = save_and_disable_interrupts();
				registers::write(w.location) = w.value;
				bus_write_followup(w.location, w.value);
				restore_interrupts(saved);
				console_writes_done++;
			}
		}

		while (write_ring_tail != write_ring_head) {
			core1_write_ring_count++;
			const uint8_t t = write_ring_tail;
			const uint8_t written_location = write_ring[t].location;
			const uint8_t written_byte = write_ring[t].value;
			core1_last_write_loc = written_location;
			write_ring_tail = (t + 1u) & WRITE_RING_MASK;
			switch (written_location) {
			default:
				break;

			case SYSTEM_TASK: // Task reg for DAT/MMU
				// Nothing to do here, and nothing could be: the write is
				// applied in bus_write_followup() because the mapping has
				// to change before the guest's next instruction, so by the
				// time this ring entry is drained it has been live for many
				// cycles.  The write is vetted there - see
				// task_write_odd_count.
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
				case REQUEST_FLOAT: {
					// Operands and result are 4-byte big-endian floats in
					// SHARED; sysreq_float() is host-tested.
					const sysreq_float_result result =
						sysreq_float(registers::write(SYSTEM_SUBREQUEST), REGISTER_BUFFER_OFFSET);
					reg[SYSTEM_REQUEST_STATUS] = result.status;
					reg[SYSTEM_REQUEST_RESPONSE] = result.response;
					break;
				}
				}
				break;

			case SYSTEM_SUBREQUEST:
				// TODO: MarkM - implement finer-grained sysreqs
				break;

			case SYSTEM_WATCHPOINT:
				if (written_byte != 0u && !watchpoint_armed) {
					// Arm: save NMI vector, install BREAK snippet, set NMI vector
					watchpoint_saved_nmi_vector = static_cast<uint16_t>(reg[REGISTER_VECTOR_NMI_OFFSET]);
					snippet_copy(BREAK);
					reg[REGISTER_VECTOR_NMI_OFFSET] = REGISTER_SNIPPET;
					watchpoint_fired = false;
					watchpoint_armed = true;
				} else if (written_byte == 0u && watchpoint_armed) {
					// Disarm: restore original NMI vector
					reg[REGISTER_VECTOR_NMI_OFFSET] = watchpoint_saved_nmi_vector;
					watchpoint_armed = false;
				}
				break;

			case CONSOLE_CONTROL: { // ACIA command
#ifdef DEBUG
				mc6850_control cr = {written_byte};
				if (cr.divide_select == 0b11u) {
					ACIA_CONTROL_COUNT = 0u;
					ACIA_TX_DATA_COUNT = 0u;
					ACIA_RX_DATA_COUNT = 0u;
					CONSOLE_STATUS_READS = 0u;
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

			// Tick timer writes are handled directly in the DMA write ISR, and
			// SHARED, the snippet area and the vectors are plain storage that
			// needs nothing here, so they take the default.
			}
		} // while (write_ring not empty)

		// Drain the CONSOLE_RX_DATA counter before checking the shared slot.
		// Each ISR-detected RX data read must call guest_receive() exactly once.
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

		if (read_irq_received) {
			core1_read_irq_count++;
			read_irq_received = false;
			core1_last_read_loc = read_location;
			switch (read_location) {
			default:
				// The MC6809 got whatever byte was here. By default, do nothing else.
				break;
			case SYSTEM_TASK:
				// Will be set by writing to this address
				break;

			case SYSTEM_REQUEST_RESPONSE:
				// Will be set by the sysreq processor
				break;

			case CONSOLE_STATUS:
				// UNREACHABLE, and left here saying so: the read
				// ISR handles this address in a branch that never
				// sets read_irq_received, so nothing arrives for
				// it.  Counted in the ISR itself as
				// CONSOLE_STATUS_READS.
				break;

			case AUX_STATUS:
#ifdef DEBUG
				AUX_ACIA_STATUS_COUNT++;
#endif
				break;

			// CONSOLE_RX_DATA / AUX_RX_DATA handled via rx_read_count above.

			// Tick timer status read is handled directly in the DMA read ISR.

			}
		} // if (read_irq_received)
	} // for(;;)
}

static void
clock_pio_init()
{
	// SM0: E/Q clock
	pio_clock.offset = pio_add_program(pio_clock.pio, &mc6809_clock_program);
	if (pio_clock.offset < 0) {
		printf("Core0 insufficient space for E/Q program\n");
		panic("Unable to proceed");
	}
	pio_clock.sm = pio_claim_unused_sm(pio_clock.pio, false);
	if (pio_clock.sm < 0) {
		printf("Core0 unable to claim unused SM for E/Q program\n");
		panic("Unable to proceed");
	}
	mc6809_clock_program_init(pio_clock.pio, pio_clock.sm, pio_clock.offset);

	printf("Core0 E/Q clocks pio%d sm%d configured\n", PIO_NUM(pio_clock.pio), pio_clock.sm);
}

// RP2350 pads come out of reset with their pull-downs enabled, and nothing
// here turned them off.  The 6809 releases its address bus, data bus and
// R/!W while parked in SYNC or HALT, and a pull-down drags a released R/!W low
// - which, with E still running, is a write strobe: stray writes land wherever
// the draining address decodes.  So with the guest powered these pads pull up
// instead, R/!W holds at read, and the address lines seen here drift high
// rather than towards $0000.  !CS pulls up too, so an undriven select reads
// as not selected: without it, a missing or unprogrammed decoder left the
// Pico answering at every address, which looks like a working board.  At
// power-down every pull comes off, so none feeds the unpowered guest logic.
static void
bus_pin_pulls(const bool powered)
{
	static constexpr struct {
		uint8_t base;
		uint8_t count;
	} groups[] = {
		{ GPIO_RW, 1u },
		{ GPIO_CS, 1u },
		{ GPIO_A_BASE, 6u },		// A0-A5
		{ GPIO_D_BASE, 8u },		// D0-D7
		{ GPIO_A_HI_BASE, 3u },		// A13-A15
	};
	for (const auto &g : groups)
		for (uint8_t i = 0u; i < g.count; i++) {
			const uint8_t pin = static_cast<uint8_t>(g.base + i);
			if (powered)
				gpio_set_pulls(pin, true, false);
			else
				gpio_disable_pulls(pin);
		}
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

	// DAT fault input, active high. Sampled as a level by the Q-rising ISR
	// rather than raising an interrupt of its own.
	gpio_init(GPIO_FAULT);
	gpio_set_dir(GPIO_FAULT, GPIO_IN);
	gpio_set_pulls(GPIO_FAULT, false, true); // idle low until the decode logic drives it

	// A13-A15 inputs, latched with a fault into SYSTEM_DAT_FAULT_LOC to report
	// the page within the task that faulted.
	init_pins_range(GPIO_A_HI_BASE, 3);
	gpio_set_dir_in_masked64(0b111LLu << GPIO_A_HI_BASE);

	bus_pin_pulls(true);

	// Fault NMI mask output, active high, driven from SYSTEM_DAT_FAULT bit 7.
	// Starts low (faults reach the guest) to match the cleared register.
	gpio_init(GPIO_NMI_BLOCK);
	gpio_put(GPIO_NMI_BLOCK, false);
	gpio_set_dir(GPIO_NMI_BLOCK, GPIO_OUT);

	// DMA for the main MC6809 bus read loop
	if (dma_channel_is_claimed(piodma_read.address_channel)) {
		printf("Core0 DMA channel %d for bus_read address is already claimed\n",
		       piodma_read.address_channel);
		panic("Unable to proceed");
	}
	dma_channel_claim(piodma_read.address_channel);
	if (dma_channel_is_claimed(piodma_read.data_channel)) {
		printf("Core0 DMA channel %d for bus_read data is already claimed\n", piodma_read.data_channel);
		panic("Unable to proceed");
	}
	dma_channel_claim(piodma_read.data_channel);

	// SM: Bus read responder
	piodma_read.offset = pio_add_program(piodma_read.pio, &mc6809_bus_read_program);
	if (piodma_read.offset < 0) {
		printf("Core0 insufficient space for bus_read program\n");
		panic("Unable to proceed");
	}
	piodma_read.sm = pio_claim_unused_sm(piodma_read.pio, true);
	if (piodma_read.sm < 0) {
		printf("Core0 unable to claim unused SM for bus_read program\n");
		panic("Unable to proceed");
	}
	mc6809_bus_read_program_init(piodma_read.pio, piodma_read.sm, piodma_read.offset);
	printf("Core0 read pio%u sm%d configured\n", PIO_NUM(piodma_read.pio), piodma_read.sm);
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
	printf("Core0 DMA read_out channel configured\n");
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
	printf("Core0 DMA read_address channel configured\n");

	// SM: Bus write responder
	piodma_write.offset = pio_add_program(piodma_write.pio, &mc6809_bus_write_program);
	if (piodma_write.offset < 0) {
		printf("Core0 insufficient space for bus_write program\n");
		panic("Unable to proceed");
	}
	piodma_write.sm = pio_claim_unused_sm(piodma_write.pio, true);
	if (piodma_write.sm < 0) {
		printf("Core0 unable to claim unused SM for bus_write program\n");
		panic("Unable to proceed");
	}
	mc6809_bus_write_program_init(piodma_write.pio, piodma_write.sm, piodma_write.offset);
	printf("Core0 write pio%u sm%d configured\n", PIO_NUM(piodma_write.pio), piodma_write.sm);
	// DMA for the main MC6809 bus write loop
	if (dma_channel_is_claimed(piodma_write.address_channel)) {
		printf("Core0 DMA channel %d for bus_write address is already claimed\n",
		       piodma_write.address_channel);
		panic("Unable to proceed");
	}
	dma_channel_claim(piodma_write.address_channel);
	if (dma_channel_is_claimed(piodma_write.data_channel)) {
		printf("Core0 DMA channel %d for bus_write data is already claimed\n",
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
	printf("Core0 DMA write_in channel configured\n");
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
	printf("Core0 DMA write_address channel configured\n");

	// SM: Task pin output driver (pio2 with GPIO base 16 for access to GPIO 40-45)
	pio_set_gpio_base(task_output.pio, 16);
	task_output.offset = pio_add_program(task_output.pio, &task_output_program);
	if (task_output.offset < 0) {
		printf("Core0 insufficient space for task_output program\n");
		panic("Unable to proceed");
	}
	task_output.sm = pio_claim_unused_sm(task_output.pio, true);
	task_output_program_init(task_output.pio, task_output.sm, task_output.offset);
	printf("Core0 task output pio%u sm%d configured\n", PIO_NUM(task_output.pio), task_output.sm);

	// SM: RTI opcode detector (pio1, needs GPIO 0-21 visibility)
	rti_detect.offset = pio_add_program(rti_detect.pio, &rti_detect_program);
	if (rti_detect.offset < 0) {
		printf("Core0 insufficient space for rti_detect program\n");
		panic("Unable to proceed");
	}
	rti_detect.sm = pio_claim_unused_sm(rti_detect.pio, true);
	rti_detect_program_init(rti_detect.pio, rti_detect.sm, rti_detect.offset);
	printf("Core0 RTI detect pio%u sm%d configured\n", PIO_NUM(rti_detect.pio), rti_detect.sm);

	// SM pair: bus trace capture.  pio0 -> pio2 via "irq prev", which wraps
	// across the three blocks.  pio1 would be the tidier neighbour but has
	// no instruction memory left.
	trace_lo.offset = pio_add_program(trace_lo.pio, &bus_capture_lo_program);
	trace_hi.offset = pio_add_program(trace_hi.pio, &bus_capture_hi_program);
	if (trace_lo.offset < 0 || trace_hi.offset < 0) {
		printf("Core0 insufficient space for bus_capture programs\n");
		panic("Unable to proceed");
	}
	trace_lo.sm = pio_claim_unused_sm(trace_lo.pio, true);
	trace_hi.sm = pio_claim_unused_sm(trace_hi.pio, true);
	// Default trigger pin is !NMI; any pin 0-31 will do, since only the low
	// half ever sees it.  The high half follows via the IRQ handshake.
	bus_capture_lo_program_init(trace_lo.pio, trace_lo.sm,
				    static_cast<uint>(trace_lo.offset), GPIO_NMI);
	bus_capture_hi_program_init(trace_hi.pio, trace_hi.sm,
				    static_cast<uint>(trace_hi.offset));
	// DAT fault sampler, on pio2 beside the trace's high half.  The SM is
	// claimed rather than chosen: pio2 already runs task_output on sm0 and
	// bus_capture_hi on sm1, and initialising one of those by hand would
	// silently restart it with this program's configuration.
	bus_fault.offset = pio_add_program(bus_fault.pio, &bus_fault_program);
	if (bus_fault.offset < 0) {
		printf("Core0 insufficient space for bus_fault program\n");
		panic("Unable to proceed");
	}
	bus_fault.sm = pio_claim_unused_sm(bus_fault.pio, true);
	bus_fault_program_init(bus_fault.pio, bus_fault.sm,
			       static_cast<uint>(bus_fault.offset), GPIO_FAULT);

	// BUSY/LIC/AVMA mirror, on pio2's last state machine.  Ticked at
	// E-falling by the clock program's flag 1; the DMA pair below keeps the
	// newest sample in bus_status_mirror for ever.
	bus_status.offset = pio_add_program(bus_status.pio, &bus_status_program);
	if (bus_status.offset < 0) {
		printf("Core0 insufficient space for bus_status program\n");
		panic("Unable to proceed");
	}
	bus_status.sm = pio_claim_unused_sm(bus_status.pio, true);
	bus_status_program_init(bus_status.pio, static_cast<uint>(bus_status.sm),
				static_cast<uint>(bus_status.offset), GPIO_BUSY);

	// One channel in ENDLESS mode (RP2350 TRANS_COUNT MODE 0xF): the count
	// never decrements, so it drains the state machine's RX FIFO into the one
	// mirror word - no increment on either side - until an abort, raising no
	// interrupt and triggering nothing.
	//
	// This was a pair of channels chained to each other, which does NOT run
	// for ever: chaining fires ON completion, so after the first channel's
	// 0xFFFFFF transfers it hands to the second, and when that one is spent
	// it hands back to a channel whose count is now zero.  A channel
	// triggered with a zero count transfers nothing, so the mirror froze
	// after 2 x 16777215 E cycles - 33.5 s at 1 MHz, 11 s at 3 MHz - and
	// set_busy_lic_avma() fed stale bits from then on with nothing to say so.
	dma_channel_claim(static_cast<uint>(bus_status.data_channel));
	dma_channel_config cfg = dma_channel_get_default_config(static_cast<uint>(bus_status.data_channel));
	channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
	channel_config_set_read_increment(&cfg, false);
	channel_config_set_write_increment(&cfg, false);
	channel_config_set_dreq(&cfg, pio_get_dreq(bus_status.pio,
						   static_cast<uint>(bus_status.sm), false));
	dma_channel_configure(static_cast<uint>(bus_status.data_channel), &cfg,
			      const_cast<void *>(static_cast<volatile void *>(&bus_status_mirror)),
			      &bus_status.pio->rxf[bus_status.sm],
			      dma_encode_endless_transfer_count(), true);
	printf("Core0 bus status pio%u sm%d configured, DMA %d endless -> mirror\n",
	       PIO_NUM(bus_status.pio), bus_status.sm, bus_status.data_channel);

	dma_channel_claim(trace_lo.data_channel);
	dma_channel_claim(trace_hi.data_channel);
	dma_channel_config tlo = dma_channel_get_default_config(trace_lo.data_channel);
	channel_config_set_transfer_data_size(&tlo, DMA_SIZE_32);
	channel_config_set_read_increment(&tlo, false);
	channel_config_set_write_increment(&tlo, true);
	channel_config_set_dreq(&tlo, pio_get_dreq(trace_lo.pio, trace_lo.sm, false));
	dma_channel_configure(trace_lo.data_channel, &tlo, MC6809.trace_base(),
			      &trace_lo.pio->rxf[trace_lo.sm], TRACE_SIZE, false);
	// 16 bits: the high bank is only GPIO 32-47, so a half-word read of the
	// FIFO takes the whole sample and halves the buffer.
	dma_channel_config thi = dma_channel_get_default_config(trace_hi.data_channel);
	channel_config_set_transfer_data_size(&thi, DMA_SIZE_16);
	channel_config_set_read_increment(&thi, false);
	channel_config_set_write_increment(&thi, true);
	channel_config_set_dreq(&thi, pio_get_dreq(trace_hi.pio, trace_hi.sm, false));
	dma_channel_configure(trace_hi.data_channel, &thi, MC6809.trace_hi_base(),
			      &trace_hi.pio->rxf[trace_hi.sm], TRACE_SIZE, false);

	MC6809.set_trace_callbacks(trace_hw_arm, trace_hw_stop, trace_hw_count);
	printf("Core0 bus trace pio%u sm%d + pio%u sm%d, DMA %d/%d\n",
	       PIO_NUM(trace_lo.pio), trace_lo.sm, PIO_NUM(trace_hi.pio), trace_hi.sm,
	       trace_lo.data_channel, trace_hi.data_channel);
}

static void
pio_shutdown()
{
	// --- Disable IRQs first (handlers reference DMA/PIO resources) ---

	// GPIO Q edge IRQ (bus observer): nothing to do here.  Its enable, its
	// NVIC enable and its handler registration are all per-core, and all three
	// belong to Core 1, which has already been reset - so the IRQ cannot be
	// taken.  The handler stays registered for the next launch; see main_core1.

	// DMA read IRQ
	dma_channel_set_irq0_enabled(piodma_read.data_channel, false);
	irq_set_enabled(piodma_read.irq, false);
	irq_remove_handler(piodma_read.irq, dma_bus_read_irq_handler);

	// DMA write IRQ
	dma_channel_set_irq1_enabled(piodma_write.data_channel, false);
	irq_set_enabled(piodma_write.irq, false);
	irq_remove_handler(piodma_write.irq, dma_bus_write_irq_handler);

	// RTI detect IRQ (pio1)
	pio_set_irq0_source_enabled(rti_detect.pio, RTI_PIO_IRQ_SOURCE, false);
	irq_set_enabled(rti_detect.irq, false);
	irq_remove_handler(rti_detect.irq, pio_rti_irq_handler);

	// Bus fault detect IRQ (pio2)
	pio_set_irq0_source_enabled(bus_fault.pio, BUS_FAULT_PIO_IRQ_SOURCE, false);
	irq_set_enabled(bus_fault.irq, false);
	irq_remove_handler(bus_fault.irq, pio_bus_fault_irq_handler);

	// --- Bus trace pair (pio0 low half + pio2 high half) ---
	// Everything the trace takes has to go back, or a second power-on cannot
	// rebuild it: pio0 holds clock(4) + read(21) + bus_capture_lo(5) = 30 of
	// its 32 words, so leaving the capture program resident leaves no room to
	// add it again, and pio_add_program returns -1.  The state machines and
	// the two fixed DMA channels leak the same way behind it.
	MC6809.trace_stop();
	dma_channel_abort(trace_lo.data_channel);
	dma_channel_abort(trace_hi.data_channel);
	dma_channel_unclaim(trace_lo.data_channel);
	dma_channel_unclaim(trace_hi.data_channel);
	bus_capture_hi_program_deinit(trace_hi.pio, trace_hi.sm);
	bus_capture_lo_program_deinit(trace_lo.pio, trace_lo.sm);
	pio_remove_program(trace_hi.pio, &bus_capture_hi_program, static_cast<uint>(trace_hi.offset));
	pio_remove_program(trace_lo.pio, &bus_capture_lo_program, static_cast<uint>(trace_lo.offset));
	// The offsets are the "trace hardware exists" guard for trace_hw_arm(),
	// trace_hw_stop() and trace_hw_count(), so they go negative here.
	trace_lo.offset = trace_hi.offset = -1;
	trace_pending_start = false;
	MC6809.set_trace_callbacks(nullptr, nullptr, nullptr);
	bus_fault_program_deinit(bus_fault.pio, bus_fault.sm);
	if (bus_fault.offset >= 0) {
		pio_remove_program(bus_fault.pio, &bus_fault_program,
				   static_cast<uint>(bus_fault.offset));
		bus_fault.offset = -1;
	}
	bus_fault.sm = -1;

	// --- RTI detect (pio1) ---
	rti_detect_program_deinit(rti_detect.pio, rti_detect.sm);
	pio_remove_program(rti_detect.pio, &rti_detect_program, rti_detect.offset);

	// --- Task output (pio2) ---
	task_output_program_deinit(task_output.pio, task_output.sm);
	pio_remove_program(task_output.pio, &task_output_program, task_output.offset);

	// --- Bus write SM + DMA ---
	dma_channel_abort(piodma_write.address_channel);
	dma_channel_abort(piodma_write.data_channel);
	dma_channel_unclaim(piodma_write.address_channel);
	dma_channel_unclaim(piodma_write.data_channel);
	mc6809_bus_write_program_deinit(piodma_write.pio, piodma_write.sm);
	pio_remove_program(piodma_write.pio, &mc6809_bus_write_program, piodma_write.offset);

	// --- Bus read SM + DMA ---
	dma_channel_abort(piodma_read.address_channel);
	dma_channel_abort(piodma_read.data_channel);
	dma_channel_unclaim(piodma_read.address_channel);
	dma_channel_unclaim(piodma_read.data_channel);
	mc6809_bus_read_program_deinit(piodma_read.pio, piodma_read.sm);
	pio_remove_program(piodma_read.pio, &mc6809_bus_read_program, piodma_read.offset);

	// --- E/Q clock (stop last — other SMs synchronise to it) ---
	mc6809_clock_program_deinit(pio_clock.pio, pio_clock.sm);
	pio_remove_program(pio_clock.pio, &mc6809_clock_program, pio_clock.offset);

	// --- Trace pins to hi-Z (not a consecutive run on version 2) ---
	static constexpr uint8_t trace_pins[] = { GPIO_TRACE_C, GPIO_TRACE_D };
	for (const uint8_t p : trace_pins) {
		gpio_put(p, false);
		gpio_set_dir(p, GPIO_IN);
	}

	// --- Fault NMI mask to hi-Z, so it does not drive unpowered logic ---
	gpio_put(GPIO_NMI_BLOCK, false);
	gpio_set_dir(GPIO_NMI_BLOCK, GPIO_IN);

	// --- Bus pull-ups off, so they do not feed unpowered logic either ---
	bus_pin_pulls(false);
}

// Close the load switch, and give the rail time to settle before anything is
// driven into the guest logic.  On version 2 the external supply is measured
// first: ADC channel 6 reads it through the divider ahead of the switch, so
// it can be checked while the board itself is still unpowered.  No supply
// means the switch stays open - driving reset, halt and the interrupt lines
// into logic that has no rail is how latch-up happens.
static bool
power_rail_enable()
{
#if PICO_THING_BOARD_VERSION >= 2
	const float volts = adc_rail_volts(rail_raw);
	if (volts < POWER_RAIL_MIN_VOLTS) {
		printf("Core0 external 5V not present (%.2fV, need %.2fV) - power not switched on\n",
		       volts, POWER_RAIL_MIN_VOLTS);
		return false;
	}
	printf("Core0 external 5V present at %.2fV\n", volts);
#endif
	// Hold the monitor off until the rail has charged the board.  Closing
	// the switch pulls the supply down until the bulk capacitance is full,
	// and that dip is not a collapse.
	rail_collapsed = false;
	rail_low_count = 0u;
	rail_settle_until = time_us_64() + (RAIL_SETTLE_MS * 1000ull);
	POWER_ASSERT;
	sleep_ms(RAIL_SETTLE_MS);
	return true;
}

static void
power_rail_disable()
{
	POWER_DEASSERT;
}

static bool
set_power()
{
	if (!power_rail_enable())
		return false;

	// Setup !Reset, !Halt, and the interrupt pins.
	MC6809.init();
	MC6809.set_setup_callback([] {
		system_tick.reset();
		fast_serial.reset();
		aux_serial.reset();
		watchpoint_armed = false;
		watchpoint_nmi_pending = false;
		watchpoint_fired = false;
		// Lower the DAT fault mask.  A fault raises it and only a guest
		// write to SYSTEM_DAT_FAULT lowers it again, so a guest that
		// faulted and did not clear it - one that was killed by the fault,
		// or a test that parked instead of handling it - would hand the
		// mask to whatever ran next.  Fault detection would then be
		// silently dead: the fault line still asserts on every bad access
		// and the sampler ignores all of them.  Reset is the point at which
		// the guest stops being the one that faulted, so the mask goes down
		// here with the rest of the per-run state.
		dat_fault_block_countdown = 0u;
		dat_fault_nmi_request = false;
		nmi_assert_cycles = 0u;
		gpio_put(GPIO_NMI_BLOCK, false);
		reg[SYSTEM_DAT_FAULT] = static_cast<uint8_t>(0u);
		reg[SYSTEM_DAT_FAULT_LOC] = static_cast<uint8_t>(0u);
		// The sampler's interrupt goes back on with the mask, and its FIFO
		// is emptied of anything the previous run left behind.  A guest
		// that faulted and never cleared the mask would otherwise hand a
		// disabled fault interrupt to whatever ran next.
		if (bus_fault.sm >= 0) {
			pio_sm_clear_fifos(bus_fault.pio, static_cast<uint>(bus_fault.sm));
			pio_interrupt_clear(bus_fault.pio, BUS_FAULT_PIO_IRQ);
			pio_set_irq0_source_enabled(bus_fault.pio,
						    BUS_FAULT_PIO_IRQ_SOURCE, true);
		}
	});
	MC6809.set_task_pin_callback(task_pins_update);
	printf("Core0 guest powered on and initialised\n");

	clock_pio_init();
	bus_pio_dma_init();
	peripheral_clear(true);
	printf("Core0 emulated RAM and devices initialised\n");

	// A write queued for a Core 1 that has since been stopped must not act on
	// this power-up.
	write_entry stale;
	while (queue_try_remove(&console_write_queue, &stale))
		;

	multicore_launch_core1(main_core1);
	return true;
}

static void
reset_power()
{
	// Stop tick timer first (cancels SDK alarm so callback won't fire)
	system_tick.stop();
	printf("Core0 tick timer stopped\n");

	// Kill Core 1 before touching any resources it uses
	core1_initialised = false;
	multicore_reset_core1();
	printf("Core0 core1 stopped\n");

	// Now safe to tear down IRQs, DMA, PIO, and trace pins
	pio_shutdown();
	printf("Core0 PIO/DMA disabled\n");

	// Stop guest (RESET + HALT asserted) then release all pins
	MC6809.stop();
	printf("Core0 guest stopped\n");

	MC6809.deinit();
	printf("Core0 guest pins released\n");

	power_rail_disable();
	printf("Core0 5V rail off\n");
}

int
main()
{
	// VREG_VOLTAGE_MAX is 1.30V, and the regulator's own limit holds anything
	// above it there unless vreg_disable_voltage_limit() lifts it - which is
	// deliberately not called.  Asking for more than this only made the line
	// read as something it was not: 320 MHz has been reliable at 1.30V.
	vreg_set_voltage(VREG_VOLTAGE_1_30);
	sleep_ms(5);

	gpio_set_function(GPIO_UART_TX, UART_FUNCSEL_NUM(uart0, GPIO_UART_TX));
	gpio_set_function(GPIO_UART_RX, UART_FUNCSEL_NUM(uart0, GPIO_UART_RX));

	stdio_init_all();

	printf("\n\nPico2 MC6809E bus supervisor %s\n", DEVICE_VERSION);
	printf("Pico2 MC6809E built on %s %s\n", __DATE__, __TIME__);
	// Which pin map this image was built for.  The two revisions swap the
	// task pins and the DAT fault signals over, so an image flashed onto
	// the wrong board drives outputs into outputs - say so before anything
	// else has a chance to scroll off the console.
	printf("Pico2 MC6809E built for board revision %d\n", PICO_THING_BOARD_VERSION);
#ifdef DEBUG
	printf("Core0 build with DEBUG defined\n");
#endif

	printf("Core0 raising sys_clock to %d MHz ... ", SYS_CLK_DEFAULT/1000000);
	if (!set_sys_clock_hz(SYS_CLK_DEFAULT, false)) {
		printf("unsuccessful! Reverting to %d MHz ... ", SYS_CLK_HZ/1000000);
		set_sys_clock_hz(SYS_CLK_HZ, true);
	}
	stdio_init_all();
	uart_init(CONSOLE, CONSOLE_BAUDRATE);
	printf("successful!\n");
	printf("Core0 console baudrate set to %d\n", CONSOLE_BAUDRATE);

	adc_init();
#if PICO_THING_BOARD_VERSION >= 2
	adc_gpio_init(GPIO_POWER_ADC);   // 5V divider, version 2 only
#endif
	adc_set_temp_sensor_enabled(true);
	power_rail_monitor_start();
	printf("Core0 supply monitor sampling every %" PRIu32 "ms\n", RAIL_MONITOR_MS);
	usb_init();
	printf("Core0 initialised USB\n");

	queue_init(&console_write_queue, sizeof(write_entry), CONSOLE_WRITE_QUEUE_LEN);

	// Enable external electronics' power
	gpio_init(GPIO_POWER);
	gpio_set_dir(GPIO_POWER, GPIO_OUT);
	const bool powered = set_power();

	tm tm = build_time_tm();
	aon_timer_stop();
	aon_timer_start_calendar(&tm);

	printf("Core0 ready!\n");

	// Nothing below runs on a board whose 5V rail never came up: Core 1
	// was not launched, so the wait loop would never finish, and the
	// startup commands have no guest to talk to.  The same goes for a rail
	// that came up and then collapsed while the guest was starting - the
	// switch is already open by then.  Fall through to the console either
	// way, so the fault can be looked at and "power on" retried.
	if (powered && !rail_collapsed) {
		// Wait for Core 1's main loop to be running before executing
		// startup commands — core1_initialised is set before the loop
		// starts, but devices need a few loop iterations to settle.
		// Bounded: if Core 1 wedges on the way up, say so and carry on to
		// the console rather than hanging here with no way to look at it.
		absolute_time_t core1_deadline = make_timeout_time_ms(CORE1_START_TIMEOUT_MS);
		while (core1_loop_count < CORE1_START_LOOPS
		       && absolute_time_diff_us(get_absolute_time(), core1_deadline) > 0) {
			sleep_ms(1u);
			usb_poll();
		}
		// Sampled once, and both the decision and the report use that one
		// sample.  Reading the counter again for the message let it race
		// past the threshold between the two, which produced a complaint
		// that Core 1 had not reached its loop alongside a count saying it
		// had.
		const uint32_t core1_loops = core1_loop_count;
		if (core1_loops < CORE1_START_LOOPS) {
			printf("*** Core 1 reached only %" PRIu32 " of %" PRIu32
			       " loops in %" PRIu32 "ms ***\n",
			       core1_loops, CORE1_START_LOOPS, CORE1_START_TIMEOUT_MS);
			printf("    initialised=%s  isr=%" PRIu32 "\n",
			       core1_initialised ? "yes" : "no", core1_isr_count);
			printf("    skipping startup commands; the console is up\n");
		} else {
			// Startup commands — executed once before entering the console loop
			static const char *startup_commands[] = {
				"run 0",
				nullptr
			};
			for (const char **cmd = startup_commands; *cmd != nullptr; cmd++) {
				char cmdbuf[COMMAND_BUFFER_LEN];
				strncpy(cmdbuf, *cmd, sizeof(cmdbuf) - 1);
				cmdbuf[sizeof(cmdbuf) - 1] = '\0';
				printf("> %s\n", cmdbuf);
				process_command(cmdbuf);
			}
		}
	}

	poll_console();
}
