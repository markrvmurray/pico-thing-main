/**
* Copyright (c) 2025 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

//#include <sys/stat.h>

#include <cctype>
#include <cinttypes>
#include <cmath>
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
#include "hardware/structs/systick.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/regs/sysinfo.h"
#include "hardware/sync.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"
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

// THE THREE DIAGNOSTIC SWITCHES, DEFINED HERE BECAUSE A #if MUST FOLLOW ITS
// #define.  The reasoning behind each one is at its first heavy use further
// down - search ISR_COST_ON, DIAG_ON and TESTPOINTS_ON - and only the
// definitions are hoisted.
//
// THEY WERE NOT ALWAYS HERE, AND THAT WAS A BUILD BREAK WAITING TO HAPPEN.  An
// `#if DIAG_ON` guarding the /IRQ accounting counters sits at about line 350,
// which was ABOVE the old definition site: an undefined macro is 0 in #if, so
// with PICO_THING_DIAG=0 both the declarations and their uses vanished and
// everything matched.  Turn DIAG on and only the USES came back - six
// "use of undeclared identifier" errors, in the one configuration nobody had
// compiled since the switch went in.  Defining them before anything reads them
// is the fix; keep it that way.
#ifndef PICO_THING_ISR_COST
#define PICO_THING_ISR_COST 0	// match CMake's default; it is an instrument
#endif
#if defined(DEBUG) && PICO_THING_ISR_COST
#define ISR_COST_ON 1
#else
#define ISR_COST_ON 0
#endif

#ifndef PICO_THING_DIAG
#define PICO_THING_DIAG 0	// match CMake's default
#endif
#if defined(DEBUG) && PICO_THING_DIAG
#define DIAG_ON 1
#else
#define DIAG_ON 0
#endif

#ifndef PICO_THING_TRAP_TRIGGER
#define PICO_THING_TRAP_TRIGGER 0	// match CMake's default
#endif
// A scope trigger for the HD6309 trap: when the Q handler sees the $FFF0
// vector fetch it pulses GPIO_TRACE_D for ~50 ns.  The misread fetch that
// caused the trap is ~15 E cycles earlier (the 12-byte push comes first), so
// the scope wants ~10 us of pre-trigger at 2.5 MHz.  Costs nothing on any
// cycle that is not a trap vector fetch; off by default.
//
// The same pulse marks the SWI vector fetch, for RAM filled with SWI so that
// a misread opcode traps at once, and the guest write that freezes an armed
// trace ring, which DBGMON makes on its first character.  A single-shot
// scope keeps whichever comes first.
#define TRAP_TRIGGER_ON (PICO_THING_TRAP_TRIGGER != 0)
#if TRAP_TRIGGER_ON
static __force_inline void
trap_trigger_pulse()
{
	gpio_put(GPIO_TRACE_D, true);
	for (unsigned i = 0u; i < 16u; i++)
		__asm volatile("nop");
	gpio_put(GPIO_TRACE_D, false);
}
#endif
#ifndef PICO_THING_TESTPOINTS
#define PICO_THING_TESTPOINTS 0	// match CMake's default
#endif
#if defined(DEBUG) && PICO_THING_TESTPOINTS
#define TESTPOINTS_ON 1
#else
#define TESTPOINTS_ON 0
#endif

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
//
// Overridable from the build, because the ceiling on the guest's E clock is
// now set by how many clk_sys cycles the per-cycle work needs, not by the
// 6809, and that ceiling scales directly with this number.  The default comes
// from CMake, 360 MHz; another clock is a separate build tree:
//
//     cmake -D PICO_THING_SYS_CLK_HZ=320000000 ...
//
// The regulator is already at PICO_THING_VREG before this is applied, and
// set_sys_clock_hz() is called with panic=false so an unachievable target
// falls back rather than hanging.  What this does NOT touch is the QMI
// divider for XIP flash, so at a high enough clk_sys the flash clock goes out
// of spec - the symptom would be a boot that dies as soon as it executes
// anything not marked __time_critical_func or __no_inline_not_in_flash_func.
// The owner has had 360 MHz reliable and 400 MHz with unreliable starts on a
// different RP2350B module.
#ifndef SYS_CLK_DEFAULT
#define SYS_CLK_DEFAULT 360000000	// match CMake's default
#endif

// The ADC clock, in kHz.  48000 is what the SDK sets and what everything here
// was calibrated against; lower values lengthen each conversion and are a
// diagnostic for a drooping reference - see the note at the clock_configure().
#ifndef PICO_THING_ADC_CLK_KHZ
#define PICO_THING_ADC_CLK_KHZ 48000u
#endif

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
//
// WHY THE BLOCKS ARE DIVIDED THIS WAY.  pio2's GPIO base is 16, so only the
// three programs that need pins above 31 can live there and none of them can
// see E or Q.  pio0 and pio1 keep base 0.  That makes pio0 and pio1 the only
// homes for anything watching the bus, and both are nearly full: the read
// program, with its address filter and AVMA gate, is 30 of pio0's 32 words,
// and pio1 holds the clock (4) and the write program (22), plus the trace's
// low half (5) while a capture is armed - 31 of 32.
//
// The clock moved pio0 -> pio1 on 2026-10-07 to free 4 words for the address
// filter the per-cycle redesign needs, and the trace's low half followed it on
// 2026-10-11 to make room for the AVMA gate.  It costs no
// CPU wherever it sits - it is an autonomous state machine - so instruction
// space is the only thing that decides.  Moving it DID require changing its
// two "irq prev set" to "irq next set"; see the note on the program itself.
#define BUS_CLOCK_PIO	pio1		// E/Q quadrature clock (4 words)
#define BUS_READ_PIO	pio0		// guest reads of the register block
#define BUS_WRITE_PIO	pio1		// guest writes to the register block
#define TASK_OUTPUT_PIO	pio2		// task pins, GPIO base 16
#define TRACE_LO_PIO	pio1		// bus trace, GPIO 0-31 (moved from pio0 2026-10-11 to make room for the read SM's AVMA gate; pio1 = clock 4 + write 22 + this 5 = 31 words while armed)
#define TRACE_HI_PIO	pio2		// bus trace, GPIO 32-47
#define BUS_FAULT_PIO	pio2		// DAT fault sampler, needs GPIO_FAULT
#define INT_PINS_PIO	pio2		// !NMI/!FIRQ/!IRQ driver, GPIO 24-26 (41-43 on version 3)

//                                    pio             sm  off  addr data  irq
static pio_dma pio_clock     = { BUS_CLOCK_PIO,       -1, -1,  -1,  -1, IO_IRQ_BANK0 };
static pio_dma piodma_read   = { BUS_READ_PIO,        -1, -1,   0,   1, PIO0_IRQ_0 };
static pio_dma piodma_write  = { BUS_WRITE_PIO,       -1, -1,   4,   5, DMA_IRQ_1 };
static pio_dma task_output   = { TASK_OUTPUT_PIO,     -1, -1,  -1,  -1, -1 };
static pio_dma trace_lo      = { TRACE_LO_PIO,        -1, -1,  -1,   2, -1 };
static pio_dma trace_hi      = { TRACE_HI_PIO,        -1, -1,  -1,   3, -1 };
static pio_dma bus_fault     = { BUS_FAULT_PIO,       -1, -1,  -1,  -1, PIO2_IRQ_0 };
static pio_dma int_pins      = { INT_PINS_PIO,        -1, -1,  -1,  -1, -1 };


static inline void
task_pins_update(uint8_t task)
{
	pio_sm_put(task_output.pio, task_output.sm, task_output_word(task));
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

// NMI REQUESTED FROM AN ISR, COUNTED RATHER THAN FLAGGED.  The Q-rising handler
// used to OR PEND_NMI straight into pending_interrupts and consume it in the
// same pass, so it never had to survive anywhere.  Now the pin is driven from
// Core 1, which OVERWRITES pending_interrupts with the device-derived levels on
// every pass - so a bit left there by an ISR is destroyed, and since Core 1
// observes only about every second Q-rising it is usually destroyed unseen.
// That cost the E-cycle countdown its NMI entirely.
//
// A counter rather than a bool for the reason this file gives for rx_read_count:
// a flag set twice before the loop drains it is one event, and worse, a flag
// read-then-cleared can lose a request that lands between the two.  Requests
// collapsing into a single pulse is fine and always was; losing one is not.
static volatile uint8_t nmi_request_count = 0u;		// ISRs increment
static uint8_t nmi_request_done = 0u;			// Core 1 follows

// What the int_pins state machine was last sent.  Written by Core 1 in the
// ordinary way, and by the single-step gate directly - see there for why - so
// volatile and declared up here rather than beside interrupt_pins_update().
static volatile uint32_t int_pins_held = 0u;

// How many of Core 1's passes the NMI pin is held low for.  THIS USED TO COUNT
// Q-RISINGS, in the handler that drove the pins directly; the int_pins state
// machine holds whatever it was last given, so the hold is now counted where
// the value is produced.
//
// The width is therefore no longer exact.  Core 1 observes roughly every second
// Q-rising (the starvation ratio, see st), so two passes is about four E cycles
// against a datasheet minimum of one, and the figure drifts with load.  That is
// safe rather than sloppy: NMI is EDGE-triggered, so the edge is latched once
// and a longer low cannot re-trigger it - which is why the old comment here
// already said a wider pulse costs nothing.  What would be unsafe is a
// SHORTER one, and the floor is one whole pass.
static constexpr uint8_t NMI_HOLD_PASSES = 2u;
static volatile uint8_t nmi_hold_passes = 0u;

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
// The three interrupt pins are consecutive from GPIO_NMI, which is what lets
// int_pins drive all three with a single "out pindirs 3".  There is no mask
// constant any more: nothing on the CPU side writes these pins once the state
// machine owns them.
static_assert(GPIO_FIRQ == GPIO_NMI + 1 && GPIO_IRQ == GPIO_NMI + 2,
	      "int_pins drives !NMI/!FIRQ/!IRQ as three consecutive pins");

static volatile bool q_rising_edge = false;
// TWO MORE "IS THERE ANYTHING TO DO" BYTES FOR THE LOOP, kept by the producers
// so the pass with nothing to do tests one load each instead of walking the
// counters (2026-10-10 loop audit, findings 4 and 5).
//
// loop_nmi_work: non-zero when the masked update has NMI work - a request
// counted by the slow path, a watchpoint hit, a DAT fault, or hold passes
// still to run.  Set by those writers AFTER their source; re-derived from the
// sources at the end of the masked block, which runs with interrupts off, so
// no Core 1 writer can interleave with the re-derive and nothing on Core 0
// writes these sources (the console's NMI request travels in pend).
//
// loop_work: non-zero when an ISR has queued something for the drains - a
// write-ring entry or an RX data read.  console_write_pending is the same for
// the console's write queue, which Core 0 fills.  Each is cleared by the loop
// BEFORE it drains, and set by the producer AFTER it produces, so a producer
// landing on either side of the clear is drained this pass or flagged for the
// next; the counters underneath stay the exact handshake.
static volatile uint8_t loop_nmi_work = 0u;
static volatile uint8_t loop_work = 0u;
// `reset core1`: Core 0 asks, Core 1's loop performs (core1_reset_perform),
// because the state involved is Core 1's to write.  Checked only on a pass
// that has work (loop_work set by the requester), so the quiet pass pays
// nothing.
static volatile bool core1_reset_request = false;
static volatile uint8_t console_write_pending = 0u;
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

// WHICH DEVICES ARE ASSERTING /IRQ, so the read ISR can retire the console
// ACIA's contribution when the guest reads the data register instead of waiting
// for Core 1's next pass.
//
// THE PROBLEM IT ADDRESSES.  /IRQ comes from pending_interrupts, written
// wholesale by the loop, while the status byte the guest reads is updated
// promptly by sync_status() on the ISR path.  So after the guest's handler reads
// the data register - clearing RDRF - the pin stays low until the loop next
// runs.  If the handler's RTI completes inside that window the guest unmasks I
// with /IRQ still asserted and re-enters with nothing pending.  Measured as
// acia_torture test 8 failing from 2.4 MHz, "S_IRQ not set in handler", 234
// unexpected entries of which 258 found S_IRQ clear.
//
// TWO WORDS, ONE WRITER EACH.  irq_sources belongs to Core 1's loop and
// irq_retired to the read ISR, and the loop takes its update with interrupts
// masked so the ISR cannot interleave - both run on Core 1, so that suffices,
// and it is the idiom the console write path already uses.  An earlier version
// had the ISR read-modify-write a byte the loop stored wholesale, which lost a
// fresh assertion whenever a loop store landed mid-update.
//
// A RECOMPUTE IS NOT AN OPTION in the ISR: mc6850::has_interrupt() clears
// irq_dirty, refreshes the TX queue state and STAGES THE NEXT RX BYTE.  Only
// tick_timer::has_interrupt() is pure.  So the loop records what it found and
// the ISR only subtracts.
static constexpr uint8_t IRQ_SRC_CONSOLE_ACIA = 1u << 0;
static constexpr uint8_t IRQ_SRC_AUX_ACIA     = 1u << 1;
static constexpr uint8_t IRQ_SRC_TICK         = 1u << 2;
static constexpr uint8_t IRQ_SRC_CONSOLE_CMD  = 1u << 3;
static volatile uint8_t irq_sources = 0u;	// written by Core 1's loop only
static volatile uint8_t irq_retired = 0u;	// written by the read ISR only
// WHERE EACH /IRQ WENT UP AND CAME DOWN.  These three settled why
// acia_torture's test 8 failed from 2.4 MHz: the read-ISR retirement alone gave 462 vectors taken for 308 assertions, and a
// ratio above one is only possible if the guest re-enters on an assertion it
// has already taken.  Adding the vector-fetch release took it to 330 taken for
// 347 raised, with 330 of the retirements happening at the fetch - one per
// acknowledgement, which is what a device does.
//
// Kept because that ratio is the one number that distinguishes "the line is
// held too long" from "the device keeps asking", and they need opposite fixes.
// Behind DIAG_ON: three stores on paths a pure user gains nothing from.
#if DIAG_ON
static volatile uint32_t irq_isr_deassert = 0u;
static volatile uint32_t irq_loop_assert = 0u;
static volatile uint32_t irq_ack_deassert = 0u;	// dropped at the vector fetch
#endif

// Interrupt amplification.  acia_torture's handler is entered ~300 times for
// 24 real ACIA receive events, and the question is whether the 6809 really
// takes 300 interrupts (so /IRQ is being held asserted too long) or whether
// something else re-enters the handler.  irq_vectors counts the vector
// fetches the processor actually performs; pend_irq_cycles counts the E cycles
// /IRQ was asserted, so their ratio is the cycles per taken interrupt.
static volatile uint32_t irq_vectors = 0u;
static volatile uint32_t pend_irq_cycles = 0u;
#if DIAG_ON
static volatile uint32_t q_armed_missed = 0u;	// a live source with the byte clear for 2+ passes
static uint32_t q_armed_miss_run = 0u;		// consecutive passes of the above, loop-private
static volatile uint32_t nmi_work_missed = 0u;	// the same for loop_nmi_work
static uint32_t nmi_work_miss_run = 0u;
#endif

// Single-stepping.  Both halves of a step MUST happen in the same Q-rising
// pass: clearing the BREAK snippet's spin byte so the guest falls through
// to its RTI, and arming the countdown that will bring it straight back.
// Driven from Core 0 instead, the two writes are about a millisecond apart -
// a thousand E cycles, some hundreds of instructions of a tight loop - and
// what comes back is not a step at all.  Hence the request flag: Core 0 asks,
// the handler does it atomically.
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
// Two states.  The snippet's own RTI has to be stepped over first, and the
// RTI is found by the GAP BETWEEN LIC EDGES rather than by the RTI opcode
// detector, which is being removed to free its 13 words on pio1.
//
// Why a gap and not a count of edges: releasing the spin means clearing the
// BRA's offset byte, and if the 6809 has already read that byte the current
// BRA still loops.  So an unknown number of further spin iterations follow,
// each with its own LIC edge, and counting edges from the release is not
// reliable.  The INTERVALS are unambiguous though - while spinning on
// `BRA B@1` the edges are 3 E cycles apart, and the RTI with a full frame is
// 15 - so the first edge that ends a gap of RTI_GAP_MIN or more IS the RTI
// completing, however many times the spin went round.
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
enum : uint8_t { STEP_IDLE = 0u, STEP_RELEASED = 1u };
// A gap this long between LIC edges is the RTI rather than the spin: the spin
// is 3 E cycles and a full-frame RTI is 15, so anything in between separates
// them with room to spare.
static constexpr uint8_t RTI_GAP_MIN = 8u;
static volatile uint8_t step_gap = 0u;	// E cycles since the last LIC edge
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


// The pin bit each interrupt kind pends, by enum value, so that Core 1's loop
// turns a device's answer into its bit with one load.  The switch this
// replaces compiled to shifts of a packed constant, a clz and conditional
// ORs - about 32 instructions per pass for the four devices, measured in the
// 2026-10-10 loop audit - to produce three bits.
static constexpr uint8_t PEND_OF[NUM_INTERRUPTS] = {
	0u,		// INTERRUPT_ILLEGAL
	0u,		// INTERRUPT_SWI3
	0u,		// INTERRUPT_SWI2
	PEND_FIRQ,	// INTERRUPT_FIRQ
	PEND_IRQ,	// INTERRUPT_IRQ
	0u,		// INTERRUPT_SWI
	PEND_NMI,	// INTERRUPT_NMI
	0u,		// INTERRUPT_RESET
	0u,		// INTERRUPT_NONE
};
static_assert(PEND_OF[INTERRUPT_FIRQ] == PEND_FIRQ && PEND_OF[INTERRUPT_IRQ] == PEND_IRQ
	      && PEND_OF[INTERRUPT_NMI] == PEND_NMI && PEND_OF[INTERRUPT_NONE] == 0u,
	      "PEND_OF is indexed by enum interrupt; its order moved");

static inline uint8_t
pend_bit(const interrupt eirq)
{
	return PEND_OF[eirq];
}

// 50 Hz tick timer (IRQ) + E-cycle countdown (NMI).
// Constructed from getInstance() rather than reg: reg is a namespace-scope
// reference in another translation unit, and the order the two are initialised
// in is unspecified.  The function-local static behind getInstance() is built
// on first use, so there is nothing to get wrong.
static tick_timer system_tick(registers::getInstance(), SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);

#if DIAG_ON
// Read by st and nothing else, so they cost the hot loop nothing unless the
// diagnostics are built: a read-modify-write per pass and per IRQ pass, and
// two stores per ring entry (2026-10-10 loop audit).  st prints "off".
static volatile uint32_t core1_qrise_count = 0u;
static volatile uint32_t core1_irq_assert_count = 0u;
#endif
static volatile uint32_t core1_loop_count = 0u;

// Task register event log.  Every guest write to SYSTEM_TASK, every guest READ
// of it (the value the DMA served) and every firmware fault switch, in order,
// stamped with the Core 1 loop count.  'st' prints the tail.  Built 2026-10-11
// to settle whether a kernel read-back of $FFC0 can return a task the kernel
// had already replaced; the hangs behind that question turned out to be
// electrical, so it is a diagnostic now.  The events are not rare - NitrOS-9
// writes $FFC0 on every interrupt entry - and without DIAG task_log_add() is
// empty, so the calls cost nothing.
#if DIAG_ON
struct task_event { uint32_t when; uint8_t kind; uint8_t value; uint8_t in_force; uint8_t pad; };
static constexpr uint32_t TASK_LOG_LEN = 32u;
static volatile task_event task_log[TASK_LOG_LEN];
static volatile uint32_t task_log_count = 0u;
static inline void
task_log_add(const uint8_t kind, const uint8_t value, const uint8_t in_force)
{
	const uint32_t i = task_log_count;
	volatile task_event &e = task_log[i % TASK_LOG_LEN];
	e.when = core1_loop_count;
	e.kind = kind;
	e.value = value;
	e.in_force = in_force;
	task_log_count = i + 1u;
}
#else
static inline void
task_log_add(const uint8_t, const uint8_t, const uint8_t)
{
}
#endif
#if DIAG_ON
static volatile uint32_t core1_write_ring_count = 0u;
static volatile uint8_t core1_last_write_loc = 0u;
#endif
static volatile uint32_t core1_isr_count = 0u;	// incremented in GPIO ISR

// Bus trace capture hardware.  Two PIO state machines sample the bus once per
// E cycle and two DMA channels write straight into MC6809's buffers, so the
// CPU does no per-cycle work at all.  The low half needs a GPIO-base-0 PIO
// (it must see E on GPIO 0 and the whole bus) and the high half needs the
// base-16 one, so they hand off with the RP2350 cross-block IRQ: the low half
// lives on pio1, whose "next" is pio2.  Neither is resident while idle - see
// trace_hw_acquire() - because pio1 has only 6 words spare without it.
// PIO IRQ flags, which the instruction streams in pico_thing.pio raise and
// this file consumes.  They are named here because the two sides are edited
// independently and a silent mismatch is expensive: a renumbering on
// 2026-10-01 left the RTI detector raising one flag while this file enabled
// and cleared another, which killed RTI detection outright without a word.
// That detector is gone now, and pio1 raises nothing at all.
//
// Flags are per block.  pio1 and pio2 each have their own set of eight.
//
//   pio2 flag 0  mc6809_clock's "irq next set 0", once per E cycle at Q-fall
//                (next from pio1 is pio2), consumed by bus_fault's wait
//   pio2 flag 1  mc6809_clock's "irq next set 1", once per E cycle at E-fall,
//                CONSUMED BY NOTHING since the BUSY/LIC/AVMA mirror was
//                removed, so it simply stays set.  One waiter only if it is
//                ever used again: a satisfied wait clears the flag
//   pio2 flag 2  bus_capture_lo -> bus_capture_hi handshake, "irq next set 2"
//                from pio1
//   pio2 flag 3  bus_fault, the DAT fault signal to the CPU

// Raised in pio0 by the read program when the guest reads $FFC0-$FFCF, which
// is the only range either this ISR or Core 1's follow-up switch acts on.  It
// REPLACES the read DMA's completion interrupt: that fired on every served
// read, and the hottest reads in the window - vector fetches, and snippet
// instruction fetches at $FFE0 - need no CPU work at all.
static constexpr uint READ_DEVICE_PIO_IRQ = 4u;
static constexpr pio_interrupt_source READ_DEVICE_PIO_IRQ_SOURCE = pis_interrupt4;

// The clock program's Q-RISING tick, raised in pio2 by "irq next set 5", and
// consumed by the int_pins driver's wait.  Like flag 0 it should never be seen
// standing set; if it is, that driver has stopped and the guest's interrupt
// lines are frozen at whatever they last held.
static constexpr uint INT_PINS_TICK_PIO_IRQ = 5u;

// The clock program's Q-falling tick, raised in pio2 by "irq next set 0".
// Consumed by the bus_fault sampler's wait, so it should never be seen
// standing set - if "st" shows pio2 flag 0 set, that sampler has stalled.
static constexpr uint CLOCK_TICK_PIO_IRQ = 0u;
static constexpr uint BUS_FAULT_PIO_IRQ = 3u;
static constexpr pio_interrupt_source BUS_FAULT_PIO_IRQ_SOURCE = pis_interrupt3;

static constexpr uint TRACE_HANDSHAKE_PIO_IRQ = 2u;

// Set when the capture should wait for its trigger pin rather than start now.
static volatile bool trace_hw_triggered = false;
// Set when a capture is armed but the guest is not running yet.
static volatile bool trace_pending_start = false;
static void trace_hw_start();
static void trace_hw_freeze();

// THE TRACE TAKES ITS PIO RESOURCES ONLY WHILE A CAPTURE IS ARMED.
//
// WHY.  The two halves cost 2 state machines and 7 instruction words, and they
// used to hold them from boot to power-off for a facility used occasionally.
// That is what boxed in the whole PIO allocation: pio2 had all four of its
// state machines taken - task_output, int_pins, bus_fault and this trace's high
// half - so the E/Q clock could not move there, so pio1 stayed at 26 of 32
// words, so the write program had no room to grow.  See [[way2-mirror-blocked]]
// in the project notes for the census.
//
// The high half MUST live on pio2, because it samples GPIO 32-47 and only a
// base-16 block can reach them; the low half needs base 0.  So the resources
// cannot simply be moved elsewhere - they have to be given back when idle.
//
// WHAT "USABLE" VS "HELD" MEAN, because one flag used to do both jobs.
// trace_lo.offset went negative on power-off and was read as "there is no trace
// hardware".  Now it also goes negative whenever a capture is not armed, so the
// two conditions need separate flags - otherwise trace_hw_count() reports 0
// samples for a perfectly good completed capture.
static bool trace_hw_usable = false;	// the DMA channels exist; claiming may be tried
static bool trace_hw_held = false;	// the PIO programs and SMs are ours right now
// The sample count survives the release: the DMA transfer count is the
// authority while we hold the channels, and it is latched on the way out so a
// finished capture can still be dumped and counted afterwards.
static uint32_t trace_hw_samples = 0u;

// Ring mode runs the capture DMA for this many cycles with the write address
// wrapping inside the buffer: 2^28 - 1 is the widest transfer count the
// channel has, about 4.5 minutes at 1 MHz E.  A ring left longer than that
// simply stops with the last TRACE_SIZE cycles in place.
static constexpr uint32_t TRACE_RING_COUNT = 0x0FFFFFFFu;

// Program the two capture channels for a one-shot (write straight through
// the buffers, TRACE_SIZE transfers) or a ring (write address wrapping on
// the buffer's own size).  Called from arm, so a change of mode between
// captures takes effect without releasing the hardware.
static void
trace_hw_dma_setup(const bool ring)
{
	dma_channel_config tlo = dma_channel_get_default_config(trace_lo.data_channel);
	channel_config_set_transfer_data_size(&tlo, DMA_SIZE_32);
	channel_config_set_read_increment(&tlo, false);
	channel_config_set_write_increment(&tlo, true);
	channel_config_set_dreq(&tlo, pio_get_dreq(trace_lo.pio, trace_lo.sm, false));
	if (ring)
		channel_config_set_ring(&tlo, true, __builtin_ctz(TRACE_SIZE * sizeof(uint32_t)));
	dma_channel_configure(trace_lo.data_channel, &tlo, MC6809.trace_base(),
			      &trace_lo.pio->rxf[trace_lo.sm],
			      ring ? TRACE_RING_COUNT : TRACE_SIZE, false);
	dma_channel_config thi = dma_channel_get_default_config(trace_hi.data_channel);
	channel_config_set_transfer_data_size(&thi, DMA_SIZE_16);
	channel_config_set_read_increment(&thi, false);
	channel_config_set_write_increment(&thi, true);
	channel_config_set_dreq(&thi, pio_get_dreq(trace_hi.pio, trace_hi.sm, false));
	if (ring)
		channel_config_set_ring(&thi, true, __builtin_ctz(TRACE_SIZE * sizeof(uint16_t)));
	dma_channel_configure(trace_hi.data_channel, &thi, MC6809.trace_hi_base(),
			      &trace_hi.pio->rxf[trace_hi.sm],
			      ring ? TRACE_RING_COUNT : TRACE_SIZE, false);
}

static uint32_t
trace_hw_count()
{
	if (!trace_hw_usable)
		return 0u;
	if (!trace_hw_held)
		return trace_hw_samples;
	const uint32_t left = dma_channel_hw_addr(trace_lo.data_channel)->transfer_count;
	if (MC6809.trace_is_ring()) {
		// A ring counts down from TRACE_RING_COUNT, so what is left says
		// how many cycles have been written in all; the caller clamps.
		return (left > TRACE_RING_COUNT) ? 0u : (TRACE_RING_COUNT - left);
	}
	return (left > TRACE_SIZE) ? 0u : (TRACE_SIZE - left);
}

// Stop sampling WITHOUT giving the hardware back.  Used inside trace_hw_arm(),
// which has to quiesce the state machines before reprogramming them and must
// not release what it is about to configure.
static void
trace_hw_quiesce()
{
	if (!trace_hw_held)
		return;
	dma_channel_abort(trace_lo.data_channel);
	dma_channel_abort(trace_hi.data_channel);
	pio_sm_set_enabled(trace_lo.pio, trace_lo.sm, false);
	pio_sm_set_enabled(trace_hi.pio, trace_hi.sm, false);
}

// Claim both halves.  Can FAIL - another subsystem may hold the last state
// machine on pio2 - and failing is a refusal to arm, not a panic: the trace is
// a diagnostic, and nothing else should stop working because it cannot run.
static bool
trace_hw_acquire()
{
	if (trace_hw_held)
		return true;
	if (!trace_hw_usable)
		return false;
	const int lo_off = pio_add_program(trace_lo.pio, &bus_capture_lo_program);
	const int hi_off = pio_add_program(trace_hi.pio, &bus_capture_hi_program);
	const int lo_sm = (lo_off >= 0) ? pio_claim_unused_sm(trace_lo.pio, false) : -1;
	const int hi_sm = (hi_off >= 0) ? pio_claim_unused_sm(trace_hi.pio, false) : -1;
	if (lo_off < 0 || hi_off < 0 || lo_sm < 0 || hi_sm < 0) {
		printf("trace: no room on pio%u/pio%u right now"
		       " (words %d/%d, state machines %d/%d) - not armed\n",
		       PIO_NUM(trace_lo.pio), PIO_NUM(trace_hi.pio),
		       lo_off, hi_off, lo_sm, hi_sm);
		if (lo_sm >= 0)
			pio_sm_unclaim(trace_lo.pio, static_cast<uint>(lo_sm));
		if (hi_sm >= 0)
			pio_sm_unclaim(trace_hi.pio, static_cast<uint>(hi_sm));
		if (lo_off >= 0)
			pio_remove_program(trace_lo.pio, &bus_capture_lo_program,
					   static_cast<uint>(lo_off));
		if (hi_off >= 0)
			pio_remove_program(trace_hi.pio, &bus_capture_hi_program,
					   static_cast<uint>(hi_off));
		return false;
	}
	trace_lo.offset = lo_off;
	trace_hi.offset = hi_off;
	trace_lo.sm = lo_sm;
	trace_hi.sm = hi_sm;
	// Default trigger pin is !NMI; any pin 0-31 will do, since only the low
	// half ever sees it.  The high half follows via the IRQ handshake.
	// Version 3 moved !NMI to GPIO 41, outside the low half's window, so
	// it falls back to !CS.  Nothing sets trace_hw_triggered today, so the
	// trigger is never used - but a pin the PIO cannot see would wrap to
	// GPIO 9 (D0) without complaint.
	constexpr uint trace_trigger_pin = (GPIO_NMI < 32) ? GPIO_NMI : GPIO_CS;
	bus_capture_lo_program_init(trace_lo.pio, trace_lo.sm,
				    static_cast<uint>(trace_lo.offset), trace_trigger_pin);
	bus_capture_hi_program_init(trace_hi.pio, trace_hi.sm,
				    static_cast<uint>(trace_hi.offset));
	// THE DMA CONFIGURATION HAS TO BE REDONE HERE, not once at setup: both
	// the DREQ and the source address are per-STATE-MACHINE, and a claim on
	// demand can land on a different one than last time.  Configuring these
	// at boot against an SM number that later changes is a silent
	// misconfiguration - the channel would pace off another SM's FIFO.
	trace_hw_dma_setup(false);
	trace_hw_held = true;
	return true;
}

// Give both halves back.  The captured rows are in RAM, not in the PIO, so a
// completed capture survives this and can still be dumped - but the DMA
// transfer count does not, hence the latch.
static void
trace_hw_release()
{
	if (!trace_hw_held)
		return;
	trace_hw_samples = trace_hw_count();
	trace_hw_quiesce();
	bus_capture_hi_program_deinit(trace_hi.pio, trace_hi.sm);
	bus_capture_lo_program_deinit(trace_lo.pio, trace_lo.sm);
	pio_remove_program(trace_hi.pio, &bus_capture_hi_program,
			   static_cast<uint>(trace_hi.offset));
	pio_remove_program(trace_lo.pio, &bus_capture_lo_program,
			   static_cast<uint>(trace_lo.offset));
	trace_lo.offset = trace_hi.offset = -1;
	trace_lo.sm = trace_hi.sm = -1;
	trace_hw_held = false;
	trace_pending_start = false;
}

static void
trace_hw_stop()
{
	trace_hw_release();
}

static void
trace_hw_arm()
{
	if (!trace_hw_acquire())
		return;
	trace_hw_samples = 0u;
	trace_hw_quiesce();
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
	trace_hw_dma_setup(MC6809.trace_is_ring());
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
	// The deferred path reaches here from the reset-to-running transition in
	// the EQ ISR, by which time the capture may have been stopped and the
	// state machines handed back - in which case the SM numbers are -1.
	// trace_hw_release() clears trace_pending_start as well, so this is
	// belt and braces, but enabling state machine -1 is not a mistake worth
	// leaving reachable on a per-cycle path.
	if (!trace_hw_held)
		return;
	pio_sm_set_enabled(trace_hi.pio, trace_hi.sm, true);
	pio_sm_set_enabled(trace_lo.pio, trace_lo.sm, true);
}

// Freeze a ring where it is: stop the capture and work out which physical
// row is the oldest.  Runs from the Q-rising handler when the chosen vector
// is fetched, so it does nothing that cannot be done there: two DMA aborts
// and three register reads.  The hardware stays held for the dump.
static void
trace_hw_freeze()
{
	if (!trace_hw_held)
		return;
	const uint32_t written = trace_hw_count();
	trace_hw_quiesce();
	const uintptr_t base = reinterpret_cast<uintptr_t>(MC6809.trace_base());
	const uintptr_t next = dma_channel_hw_addr(trace_lo.data_channel)->write_addr;
	const uint32_t next_row = static_cast<uint32_t>((next - base) / sizeof(uint32_t)) % TRACE_SIZE;
	// Until the ring has wrapped, row 0 is the oldest; after that the row
	// the DMA would write next is.
	const uint32_t rows = written < TRACE_SIZE ? written : TRACE_SIZE;
	const uint32_t head = written < TRACE_SIZE ? 0u : next_row;
	MC6809.trace_ring_set(head, rows);
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
// The delay was needed while the GAL generated the fault NMI as a registered
// output, NMI.D = MMU_dat & Last_Block & !NMIBlock: raising the mask in the
// same pass that saw the fault removed the D term before the 6809 had sampled
// it.  U208 has since gone combinational (Fault = MMU_dat & Last_Block, NMI
// held at 0, Q201 removed), and the Pico raises the NMI itself through
// pending_interrupts, whose hold does not look at NMIBlock.  NMIBlock now
// only stops the sampler below from latching a later fault, and the countdown
// does that too while it runs, so the delay no longer has a hardware reason.
// It is kept because it costs nothing, and a wider window still makes the
// fault visible on a scope.  Set from CMake with
// -DPICO_THING_DAT_FAULT_BLOCK_DELAY=<cycles>.
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

// ---------------------------------------------------------------------------
// What the per-cycle ISRs actually cost, in clk_sys cycles.
//
// Why this exists: with the HD63C09E fitted the CPU stopped being the limit.
// 1.5 MHz runs, 2.0 MHz starves Core 1's for(;;) loop - 178 iterations in 28
// seconds, resuming the moment the clock drops back - so the budget is the
// thing to know:
//
//     E        E period     clk_sys 320 MHz cycles per E cycle
//     1.0 MHz   1000 ns      320
//     1.5 MHz    667 ns      213   works
//     2.0 MHz    500 ns      160   starves
//
// SysTick rather than DWT: it is per-core like the handler, always present
// without a debugger, and the SDK uses it for nothing (checked).  It counts
// DOWN from its reload, so a delta is (start - end) masked to 24 bits, which
// is correct across the wrap.  24 bits at 320 MHz is 52 ms, far longer than
// anything measured here.
//
// The instrument costs two SysTick reads and a few compares, so it INFLATES
// what it reports by perhaps 10-15 cycles.  Read the numbers as an upper
// bound, and mind that both ISRs land in the same E cycle whenever the guest
// touches this window: their SUM is what has to fit.
//
// Its own switch rather than plain DEBUG, because it costs ~21 cycles per E
// cycle - enough to move the ceiling by about 0.13 MHz, which is why 1.3 MHz
// started failing the moment it went in.  Build with
// -D PICO_THING_ISR_COST=0 to get those cycles back while keeping the rest of
// DEBUG.
// The switch itself is defined at the top of this file: a #if must follow
// its #define, and an `#if ISR_COST_ON` above that point reads as 0.

// PER-CYCLE DIAGNOSTIC COUNTING.  Separate from DEBUG, which is defined
// unconditionally (CMakeLists.txt) and therefore gates what st can REPORT
// rather than what anything COSTS.  This decides whether the Q-rising handler
// keeps counters at all: on the ungated path they are three volatile
// read-modify-writes, about 20-25 of its ~110 clk_sys cycles, every E cycle,
// for numbers only a human reading st ever looks at.
//
// COUNTERS THAT ARE MECHANISM ARE NEVER GATED BY THIS, however statistical
// their names sound.  rx_read_count/rx_processed_count,
// nmi_request_count/nmi_request_done and _count_irq_ack[] all carry work
// between an ISR and its consumer - the first two so a second event cannot be
// lost, the third because the interrupt tests hold a request asserted until
// its count moves, which its own declaration says in as many words.  Gating
// any of those loses interrupts rather than statistics.
// The switch itself is defined at the top of this file: a #if must follow
// its #define, and an `#if DIAG_ON` above that point reads as 0.

// GPIO SCOPE MARKERS.  Driven high on entry to an ISR and low on exit so a
// scope can see when each ran and for how long.  Two SIO writes per entry on
// paths that run every E cycle, for a signal that is worthless without probes
// on the board - hence its own switch, defaulting off.
//
// THE PIN ALLOCATION, WHICH IS NOT ARBITRARY.
//
// TRACE_C is the two DMA handlers, read and write.  They can share one pin
// because a bus cycle is either a read or a write, never both, so they are
// MUTUALLY EXCLUSIVE - and being the same IRQ priority they cannot nest within
// each other either.  The cost is only that the scope cannot tell which of the
// two it is looking at, and the address lines say that anyway.
//
// TRACE_D is the Q-rising handler, ON ITS OWN.  It must not share, because it
// runs at IRQ priority 1 while the DMA handlers run at 0, so they PREEMPT it.
// Until 2026-10-08 the read handler shared TRACE_C with it, and a read
// notification landing inside a Q pass drove the pin low on its own exit and
// truncated the width being measured - so the one measurement these pins exist
// for, a scope against Q for handler latency, was corrupted in exactly the case
// it is most wanted in.
// The switch itself is defined at the top of this file: a #if must follow
// its #define, and an `#if TESTPOINTS_ON` above that point reads as 0.

#if ISR_COST_ON
#define SYSTICK_MASK 0x00FFFFFFu
struct isr_cost {
	uint32_t last, min, max, n;
	uint64_t sum;
};
static volatile isr_cost q_cost = { 0u, 0xFFFFFFFFu, 0u, 0u, 0u };
static volatile isr_cost read_cost = { 0u, 0xFFFFFFFFu, 0u, 0u, 0u };
// THE WRITE ISR, which was absent from isr_demand entirely until now.  Every
// figure that line printed was therefore too LOW by whatever this path costs,
// while the instrument being in circuit for every measurement pushed it the
// other way: the headline cap was wrong in both directions at once.
//
// Worth having for its own sake now, not just completeness.  The write ISR is
// MEASURED as the first thing to give way: coalesced write completions go 0 at
// 2.4 MHz to 8610 at 2.5, so it is missing its deadline on thousands of writes
// at a clock where nothing else complains.  This says what it costs to do so.
//
// Its span is variable by design, because it drains however many ring entries
// are present - so max/mean apart is the interesting reading here, more than
// for the other two.
static volatile isr_cost write_cost = { 0u, 0xFFFFFFFFu, 0u, 0u, 0u };

static inline void
isr_cost_init()		// Core 1 only: SysTick is per-core
{
	systick_hw->csr = 0u;			// off while reconfiguring
	systick_hw->rvr = SYSTICK_MASK;		// free-run over the full 24 bits
	systick_hw->cvr = 0u;			// any write reloads it
	systick_hw->csr = 0x5u;			// ENABLE | CLKSOURCE=processor, no IRQ
}

static inline void
isr_cost_note(volatile isr_cost &c, const uint32_t d)
{
	c.last = d;
	if (d < c.min)
		c.min = d;
	if (d > c.max)
		c.max = d;
	c.sum += d;
	c.n++;
}

// Every cycle spent in either per-cycle ISR, so Core 1's loop can subtract
// what the ISRs took from its own wall time and report its OWN work.  32 bits
// deliberately: a single load is atomic on this core, where the 64-bit sums
// above could tear under an interrupt, and unsigned subtraction is correct
// across the wrap.
static volatile uint32_t isr_cycles_total = 0u;

static inline void
isr_cost_add(volatile isr_cost &c, const uint32_t start)
{
	const uint32_t d = (start - systick_hw->cvr) & SYSTICK_MASK;
	isr_cycles_total += d;
	isr_cost_note(c, d);
}

// Core 1's loop body, which the cliff position could only ever imply.  Timed
// from the top of one iteration to the top of the next, less the ISR cycles
// that landed in between - so this is the loop's OWN cost, not its wall time.
//
// Slightly overstated, and in a known direction: an ISR's measured span ends
// at its second SysTick read, so the bookkeeping after that read is not
// subtracted and lands here instead.
static volatile isr_cost loop_cost = { 0u, 0xFFFFFFFFu, 0u, 0u, 0u };

// THE GAP BETWEEN PASSES, which is a different quantity from loop_cost and the
// one Core 1's DEADLINE is actually about.  loop_cost holds the loop's own WORK
// - the instrument subtracts ISR time from it - whereas what matters for the
// write ring is the WALL-CLOCK interval between drains, because the write ISR
// keeps filling the ring throughout, ISR time included.  Reporting the own-work
// figure as "worst period" understated the gap; this bucket is the real thing.
static volatile isr_cost loop_period = { 0u, 0xFFFFFFFFu, 0u, 0u, 0u };
static volatile uint32_t loop_prev_stamp = 0u;
static volatile uint32_t loop_prev_isr = 0u;
static volatile bool loop_prev_valid = false;

static inline void
isr_cost_clear(volatile isr_cost &c)	// field by field: a volatile struct
{					// cannot be assigned as a whole
	c.last = 0u;
	c.min = 0xFFFFFFFFu;
	c.max = 0u;
	c.n = 0u;
	c.sum = 0u;
}

static void
isr_cost_reset()
{
	isr_cost_clear(q_cost);
	isr_cost_clear(read_cost);
	isr_cost_clear(write_cost);
	isr_cost_clear(loop_cost);
	isr_cost_clear(loop_period);
	loop_prev_valid = false;
}
#endif

// THE ARMED SOURCES, in one place: what the Q-rising handler's slow path is
// for beyond a bus-state change.  Read by the slow path to re-derive
// q_rising_armed, and by the DIAG audit in Core 1's loop.  HOW TO CHECK THIS
// LIST AFTER CHANGING THE SLOW PATH: list every file-scope volatile the slow
// path reads, and for each ask where it is written - inside BUS_IRQ or only
// consulted on a state change is already covered, because BUS_IRQ *is* a bus
// state change; written by Core 0 or another ISR and the handler has to
// react means it belongs here AND its writer must call q_rising_arm().
static inline bool
q_rising_sources()
{
	return MC6809.trace_is_capturing()
	    || MC6809.trace_ring_freeze_pending()
	    || step_state != STEP_IDLE
	    || step_request
	    || step_arm_cycles != 0u
	    || dat_fault_block_countdown != 0u
	    || system_tick.get_nmi_countdown() != 0u;
}

// THE SLOW HALF OF THE Q-RISING HANDLER: everything that reacts to a bus-state
// change or serves an armed feature.  Deliberately not inlined into the handler
// - see the call site for what that costs on the fast path.  It runs in the
// handler's context, at its priority, with its interrupts; only the register
// save is different.
// External linkage, not static: mc6809 names it as a friend, and a friend
// declaration gives a function external linkage, which a static definition
// would then contradict.
void __attribute__((noinline))
__time_critical_func(q_rising_slow_path)(const uint8_t bla, const uint8_t address_lsb,
					 const enum bus_state new_bs, const bool reset_now)
{

	// The count is read here anyway, so fullness is free to spot - and
	// spotting it is what stops this branch, and the gate above, from
	// running for ever after the one-shot completes.  trace_armed stays
	// set so the capture can still be dumped; only the capturing flag
	// is cleared.  A hardware-triggered capture sits at a count of zero
	// until its trigger arrives, which is correctly "still capturing".
	if (MC6809.trace_is_capturing()) {
		const uint32_t taken = trace_hw_count();
		if (taken >= TRACE_SIZE)
			MC6809.trace_capture_done();
		else
			MC6809.trace_bla_store(taken, bla);
	}
	if (reset_now) {
		if (MC6809.run_state != RS_RESET)
			MC6809.run_state = RS_RESET;
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
#if TRAP_TRIGGER_ON
			// Scope trigger on the trap and SWI vectors, see
			// PICO_THING_TRAP_TRIGGER.
			if (eirq == INTERRUPT_ILLEGAL || eirq == INTERRUPT_SWI)
				trap_trigger_pulse();
#endif
			// Both the break command and the watchpoint point the NMI
			// vector at the BREAK snippet, and the vector fetch is the
			// point at which the frame is on the stack and the copy is
			// about to run.  This is the bus telling the console the
			// break has been taken.
			if (eirq == INTERRUPT_IRQ) {
				irq_vectors++;
				// RELEASE /IRQ AT THE ACKNOWLEDGEMENT, which is
				// what a real device does: hold the request until
				// the processor fetches the vector, then let go.
				// The `irq` test command already works this way.
				//
				// WHY THE DATA READ IS NOT ENOUGH, measured: with
				// retirement only in the read ISR the guest took 462
				// vectors for 308 assertions.  A SPURIOUS entry finds
				// S_IRQ clear, so the handler never reads the data
				// register, so that retirement never runs - the line
				// stays low and the guest re-enters on every RTI until
				// Core 1's loop catches up.  Releasing here breaks that
				// regardless of what the handler does.
				//
				// It also cannot truncate an unsampled pulse, which is
				// the trap the read-ISR version fell into: the fetch IS
				// the proof the processor took it.  If a device still
				// wants service the loop reasserts on its next pass,
				// harmlessly, since the 6809 masks I on IRQ entry.
				//
				// Masked because the read DMA ISR is priority 0 against
				// this handler's 1 and so can preempt it; both touch
				// int_pins_held, and two lost updates on the same bit
				// would leave the shadow disagreeing with the PIO -
				// which would make the loop skip a NEEDED assertion.
				const uint32_t ack = save_and_disable_interrupts();
				irq_retired = 0xFFu;
				if (int_pins_held & PEND_IRQ) {
					const uint32_t oe = int_pins_held
						& ~static_cast<uint32_t>(PEND_IRQ);
					int_pins_held = oe;
					pio_sm_put(int_pins.pio,
						   static_cast<uint>(int_pins.sm),
						   oe);
#if DIAG_ON
					irq_ack_deassert++;
#endif
				}
				restore_interrupts(ack);
			}
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
			// leave it.  Nothing restores the task afterwards either:
			// the automatic restore after RTI went with the RTI
			// detector on 2026-10-06 (see the note in mc6809.cpp).
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
				task_log_add('F', 0u, MC6809.task_get());
				const uint8_t supervisor = MC6809.task_change(0u);
				reg[SYSTEM_TASK] = supervisor;
				task_pins_update(supervisor);
			}
			// Not DEBUG-only: the interrupt tests deassert on this
			// count, so it has to be maintained in every build.
			if (eirq <= INTERRUPT_RESET)
				MC6809.count_irq_ack(eirq);
			break;
		}
		case BUS_SYNC:  MC6809.run_state = RS_SYNCED; break;
		case BUS_HALT:  MC6809.run_state = RS_HALTED; break;
		case BUS_RUNNING:
			MC6809.run_state = RS_RUNNING;
			break;
		default:
			break;
		}
	}
	// Deferred NMIBlock assertion after a DAT fault; see the note by
	// DAT_FAULT_BLOCK_DELAY.
	if (dat_fault_block_countdown > 0u && --dat_fault_block_countdown == 0u) {
		gpio_put(GPIO_NMI_BLOCK, true);
		reg[SYSTEM_DAT_FAULT] |= DAT_FAULT_NMI_BLOCK;
	}
	// A ring triggered by a vector freezes a few cycles later, so the
	// vector fetch is in it; see TRACE_RING_FREEZE_PASSES.
	MC6809.trace_ring_freeze_tick();
	// LIC-gated step.  Release and gate are both here for the same
	// reason the cycle-count path is: from Core 0 the two would be a
	// millisecond apart, which is not a step.
	{
		// The pins, deliberately, and the same read the rest of this
		// handler uses: see the note on the states.
		const bool lic_now = (bla & 0x02u) != 0u;	// LIC is bit 1
		if (step_request) {
			// Zeroing the BRA's offset turns the snippet's spin into
			// "branch to the next byte", which is the RTI.
			reg[REGISTER_SNIPPET + 14] = static_cast<uint8_t>(0x00);
			step_state = STEP_RELEASED;
			step_gap = 0u;
			step_request = false;
			if (step_active) {	// restart the edge log
				step_pass = 0u;
				step_edges = 0u;
				step_armed_edge = 0u;
				step_logging = true;
			}
		}
		if (step_logging) {
			if (step_pass < 0xFFu)
				step_pass++;
			if (lic_now && !step_prev_lic
			    && step_edges < STEP_EDGE_MAX)
				step_edge[step_edges++] = step_pass;
		}
		if (step_state == STEP_RELEASED) {
			if (step_gap < 0xFFu)
				step_gap++;
			if (lic_now && !step_prev_lic) {
				if (step_gap >= RTI_GAP_MIN) {
					// That edge ended a long gap, so it
					// is the RTI finishing and the
					// guest's own instruction is next.
					// ASSERT IT HERE, NOT VIA CORE 1.
					// The edge has to land inside this
					// E cycle: the entire point of the
					// gate is that the guest executes
					// exactly ONE more instruction, and
					// Core 1 is a pass or two away -
					// which is one or more instructions.
					// Going through it broke stepping in
					// d954a2b: "irq 9" fell from ONE
					// INSTRUCTION PER STEP to 3/5.
					//
					// Core 1 deasserts on its next pass,
					// because its own computed mask has
					// no NMI in it.  That is fine: NMI is
					// edge-triggered, so what matters is
					// when the edge arrives, not how long
					// the line then stays low.
					// The hold matters as much as the
					// edge: without it Core 1's very
					// next pass recomputes a mask with
					// no NMI in it and lets the line go,
					// possibly inside one E cycle, which
					// is too short for the 6809 to latch
					// anything.  Asserting without this
					// gave "no NMI came back".
					nmi_hold_passes = NMI_HOLD_PASSES;
					loop_nmi_work = 1u;	// the loop releases after the hold
					int_pins_held |= PEND_NMI;
					pio_sm_put(int_pins.pio,
						   static_cast<uint>(int_pins.sm),
						   int_pins_held);
					step_armed_edge = step_edges;
					step_state = STEP_IDLE;
				} else {
					step_gap = 0u;	// still spinning
				}
			}
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
		// Restart the edge log here too, or `irq 9`'s count pass prints
		// whatever the previous run's LIC pass left in it.  The logging
		// above has already run for this pass, so count it as 1, as
		// the LIC path does for its release pass.
		if (step_active) {
			step_pass = 1u;
			step_edges = 0u;
			step_armed_edge = 0u;
			step_logging = true;
		}
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
	// through the one nmi_hold_passes pulse - two requests in
	// the same pass collapse into a single pulse, and only break has a
	// way (break_vector_fetched) to confirm its own was the one taken.
	if (system_tick.tick()) {
		nmi_request_count++;
		loop_nmi_work = 1u;
	}

	// THE PINS ARE NOT DRIVEN FROM HERE ANY MORE.  The int_pins state
	// machine on pio2 re-applies them at every Q rising by itself, and
	// Core 1 sends it a new word only when the pending set changes -
	// see interrupt_pins_update().  What used to happen here was a
	// three-bit mask being recomputed and rewritten a million times a
	// second, almost always to the value it already held.
	//
	// This counter stays because it is a per-E-cycle measure and there
	// is nowhere else to take it: the E cycles during which /IRQ was
	// asserted, which st divides by the vector count to report how long
	// each interrupt was held.

	// RE-DERIVE THE ARMED BYTE, last, after every countdown above has
	// moved.  Store 0 first, fence, then re-read the sources: a writer that
	// lands after the re-read stores its 1 after this 0, and one that lands
	// before it is seen (q_armed.h has the argument).  This is the only
	// place the byte is cleared.
	q_rising_armed = 0u;
	std::atomic_thread_fence(std::memory_order_seq_cst);
	if (q_rising_sources())
		q_rising_armed = 1u;
}

void __isr
__time_critical_func(gpio_q_irq_handler())
{
#if ISR_COST_ON
	const uint32_t q_cost_start = systick_hw->cvr;
#endif
#if TESTPOINTS_ON
	gpio_put(GPIO_TRACE_D, true);
#endif
	// Q-RISING ONLY.  The falling edge used to be enabled as well, purely to
	// poll GPIO_FAULT once per cycle - a million entries a second to read one
	// pin.  The bus_fault state machine on pio2 does that sampling now,
	// clocked by the clock program's "irq next set 0" at the same Q-falling
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
	// DIRECTLY, NOT THROUGH gpio_get_irq_event_mask(): that helper reads
	// SIO CPUID and multiplies to find THIS core's INTS register, on every
	// entry, to answer a question whose answer is fixed - this handler runs
	// on core 1 and nowhere else.  Same registers, same acknowledge, minus
	// the lookup.  GPIO_Q is 1, so its four event bits are 4-7 of word 0.
	static_assert(GPIO_Q < 8u, "GPIO_Q's event bits are assumed to be in "
				   "ints[0]/intr[0]; move the indexing with the pin");
	constexpr uint32_t Q_SHIFT = 4u * (GPIO_Q % 8u);
	const uint32_t q_events =
		(io_bank0_hw->proc1_irq_ctrl.ints[GPIO_Q / 8u] >> Q_SHIFT) & 0xFu;

	io_bank0_hw->intr[GPIO_Q / 8u] = (q_events & GPIO_IRQ_EDGE_RISE) << Q_SHIFT;
	if (q_events & GPIO_IRQ_EDGE_RISE) {
		// Counted here rather than on entry: one increment per E cycle
		// instead of two, and it then measures the same event as
		// core1_qrise, which the for(;;) loop counts when it observes the
		// rising edge.  The pair is the starvation ratio that diagnosed the
		// Core 1 livelock of 2026-09-15 - if this climbs while core1_qrise
		// and core1_loop stand still, the ISRs are eating the loop.
		// NOT GATED BY DIAG_ON, though it looks like a statistic.  Core 0's
		// watchdog compares it against core1_loop_count to tell "the loop
		// stalled but the ISRs are alive" - thread starvation - from "the
		// ISRs stopped too" - a hard fault or deadlock.  Gating it would
		// leave that watchdog permanently reporting the second, which is a
		// silent loss of the diagnosis that found the 2026-09-15 livelock.
		// One load and one store is cheap insurance.
		core1_isr_count++;
		q_rising_edge = true;
		bus_pins bus_pins{};
		bus_pins.word = gpio_get_all();
		// BUSY/LIC/AVMA straight out of the snapshot above, which is free -
		// one gpio_get_all() already read them.  This handler runs past
		// t_CD, measured, so these are the RAW IN-ADVANCE values: they
		// describe the NEXT cycle, not this one.  That is the phase the
		// step gate below needs, and it is one cycle fresher than the
		// DMA-maintained mirror this replaced - see union bus_pins.
		const auto bla = static_cast<uint8_t>(bus_pins.bits.BUSY_LIC_AVMA);
		MC6809.set_busy_lic_avma(bla);
		// Keep what we just read, against the row the capture DMA is about
		// to write, so the columns line up.  Only while armed: this reads a
		// DMA register, which is not something to do on every E cycle for
		// nothing.
		auto address_lsb = static_cast<uint8_t>(bus_pins.bits.A);
		const auto new_bs = static_cast<enum bus_state>(bus_pins.bits.BS_BA & 0x03u);
		// Out of the snapshot above rather than its own gpio_get().  The
		// union's RESET bit IS GPIO_RESET's bit - the fields are laid out
		// to match the pin map exactly, which is what lets .A, .BS_BA and
		// .BUSY_LIC_AVMA be taken from it too - so this is the same value
		// for one fewer SIO read on a path that runs every E cycle.  The
		// input register reflects the pad even for an output pin, so
		// reading our own !RESET back is sound.
#if PICO_THING_BOARD_VERSION >= 3
		// Version 3 moved !RESET to GPIO 39, out of the low word, so this
		// costs the one extra SIO load the comment above avoids.
		static_assert(GPIO_RESET == 39,
			      "union high_bus_pins' RESET bit is GPIO 39; if the pin "
			      "moves, the bitfield has to move with it");
		high_bus_pins reset_pins{};
		reset_pins.word = sio_hw->gpio_hi_in;
		const bool reset_now = !reset_pins.bits.RESET;
#else
		static_assert(GPIO_RESET == 29,
			      "union bus_pins' RESET bit is bit 29; if the pin "
			      "moves, the bitfield has to move with it");
		const bool reset_now = !bus_pins.bits.RESET;
#endif
#if DIAG_ON
		// Diagnostic only: st's LIC display and the stall warning.
		// start_with_timeout() verdicts on run_state == RS_SYNCED and
		// its lic_start only feeds a printf, so nothing depends on this
		// count.  Two volatile loads and a conditional store per E cycle.
		MC6809.count_lic();
#endif
		// THE GATE.  Everything below is either a reaction to the bus state
		// CHANGING, or work that only one of a handful of armed features
		// needs - and in steady running neither applies, so on almost every
		// E cycle this handler now ends here.  Measured: the body is 90 of
		// the handler's 131 clk_sys cycles, and the floor without it is 41,
		// which is what lifts the ISR-imposed E ceiling from 2.4 MHz to
		// comfortably past 3.
		//
		// WHAT IS *NOT* IN THE ARMED LIST, AND WHY THAT IS SAFE.  Vector
		// fetches, the break/watchpoint confirmation, the trace's interrupt
		// triggers, irq_vectors and the DAT fault's switch to task 0 all hang
		// off BUS_IRQ - and BUS_IRQ *is* a bus state change, so the first
		// condition catches every one of them structurally.  They cannot be
		// missed by someone forgetting to add a flag here, which is the
		// failure mode this arrangement is chosen to avoid.
		//
		// The armed list is therefore only the things that need EVERY cycle
		// rather than every transition: the trace's BUSY/LIC/AVMA column, the
		// single-step LIC gate, and the E-cycle countdown.
		const bool bus_changed = new_bs != MC6809.bus_state
				      || reset_now != (MC6809.run_state == RS_RESET);
		// HOW TO CHECK THIS LIST AFTER CHANGING THE BODY, because getting
		// it wrong fails silently and two conditions were missed on the
		// way in.  List every file-scope volatile the gated region READS,
		// and for each ask where it is WRITTEN:
		//
		//   written inside BUS_IRQ, or only consulted on a state change
		//     -> already covered, because BUS_IRQ *is* a bus state change
		//        and the first condition catches every transition;
		//   written by Core 0 or another ISR, and the handler has to react
		//     -> IT MUST BE NAMED HERE.
		//
		// The two that were missed, and what each looked like:
		//   dat_fault_block_countdown (set by the fault ISR) - the delayed
		//     NMI block never came on, so $FFCC read 03 instead of 83;
		//   step_request ("Core 0 asks; the handler acts") - the console's
		//     request was never seen, so "irq 9" reported "no NMI came
		//     back" and stalled after one step.
		//
		// acia_torture passed throughout BOTH of those, which is why it
		// alone cannot clear a change to this handler.  The DAT fault test
		// and "irq 9" are what found them.
		// ONE LOAD, NOT SIX: q_rising_armed is kept by the writers of the
		// sources listed in q_rising_sources() - see q_armed.h for the
		// rule and the ordering argument - and the slow path re-derives it
		// when it runs.  The six-way test it replaces cost ~10 clk_sys
		// cycles per E cycle, one of its loads needing a movw to reach an
		// object offset of 0x7039.
		const bool armed = q_rising_armed != 0u;
		// THE BODY IS A SEPARATE FUNCTION, and noinline, so that this fast
		// path - which is what runs on almost every E cycle - saves only
		// the registers IT needs.  With the 300 lines below inlined here
		// the prologue was stmdb {r4-r11, lr}: nine registers pushed and
		// popped per E cycle for a path that uses about four of them,
		// ~14 clk_sys cycles out of a 128-cycle budget at 2.5 MHz.  The
		// slow path pays a bl and its own prologue only when it runs,
		// which is on a bus-state change or while something is armed.
		if (bus_changed || armed)
			q_rising_slow_path(bla, address_lsb, new_bs, reset_now);
#if DIAG_ON
		if (pending_interrupts & PEND_IRQ)
			pend_irq_cycles++;
#endif
	}
#if TESTPOINTS_ON
	gpio_put(GPIO_TRACE_D, false);
#endif
#if ISR_COST_ON
	isr_cost_add(q_cost, q_cost_start);
#endif
}

// ISR for the DAT fault sampler PIO program.
//
// Fires when bus_fault finds GPIO_FAULT high at Q-falling, which means the
// guest has read or written an 8kB block the DAT does not map.  The flag is
// cleared first: it is a latch, and a handler that returns without clearing it
// re-enters for ever.
//
// Deliberately NOT a re-test of the FAULT pin.  The sampler established the
// fault at Q-falling, the point where the GAL's fault line has settled and the
// live bus pins both describe the same cycle; by the time this runs the pin may
// have gone low again, and a re-test would throw a real fault away.
//
// The task and page come from the sampler's own `in pins 32`, so they are the
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
	//
	// The sample is 32 pins from BUS_FAULT_SAMPLE_BASE.  Versions 1 and 2:
	// GPIO 32-47 in the low half, which is high_bus_pins as it stands.
	// Version 3: GPIO 16-47, so the top half is high_bus_pins and the bottom
	// half is the top of bus_pins, where the page and FAULT now live.
	// Unpacked into the pair a trace row uses, so row_a_hi() reads either.
	uint32_t word = 0u;
	bool sampled = false;
	while (!pio_sm_is_rx_fifo_empty(bus_fault.pio, static_cast<uint>(bus_fault.sm))) {
		const uint32_t w = pio_sm_get(bus_fault.pio, static_cast<uint>(bus_fault.sm));
		if (!sampled) {
			word = w;
			sampled = true;
		}
	}
	bus_pins low_pins{};
	high_bus_pins high_pins{};
	if (sampled) {
#if PICO_THING_BOARD_VERSION >= 3
		low_pins.word = word << 16;
		high_pins.word = word >> 16;
#else
		high_pins.word = word;
#endif
	} else {
		// Belt and braces: the flag cannot be raised without the push
		// that precedes it, but a live read beats recording nothing.
		low_pins.word = sio_hw->gpio_in;
		high_pins.word = sio_hw->gpio_hi_in;
	}

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
	if constexpr (DAT_FAULT_AUTO_BLOCK) {
		dat_fault_block_countdown = DAT_FAULT_BLOCK_DELAY;
		q_rising_arm();
	}
	// Tell the BUS_IRQ case that the NMI now on its way is a fault, so it
	// can drop the guest into task 0 for the handler.
	dat_fault_nmi_pending = true;
	// And ask for that NMI.  The GAL no longer raises one (U208's NMI is
	// held at 0): a registered NMI.D lasted only as long as the faulting
	// access was on the bus, which put the whole pulse inside the faulting
	// instruction's last cycle, and the 6809 samples !NMI on Q-falling of the
	// cycle BEFORE the last one.
	//
	// interrupt_pins_update() takes the request on Core 1's next pass and
	// holds !NMI for NMI_HOLD_PASSES, the path the watchpoint and the irq
	// tests already use.
	dat_fault_nmi_request = true;
	loop_nmi_work = 1u;
	// Nothing else writes bits 2-6.  The NMIBlock bit is not set here: it
	// mirrors the pin, and the pin is not up yet.
	reg[SYSTEM_DAT_FAULT] = static_cast<uint8_t>(
		DAT_FAULT_FAULT
		| (gpio_get(GPIO_RW) ? DAT_FAULT_RW : 0u));
	// The task pins are the ones the DAT hardware itself saw, not
	// MC6809.task.
	reg[SYSTEM_DAT_FAULT_LOC] = static_cast<uint8_t>(
		(high_pins.bits.TASK << DAT_FAULT_LOC_TASK_SHIFT)
		| row_a_hi(low_pins, high_pins));
}

// ISR for the R/!W bus read loop and DMA chain
// The PIO mediates the signalling
// There are 2 DMA transfers:
//   1) accept reg + A0-A5
//   2) return the byte at that address
//
// This reacts to a read after the fact. Consider breaking
// the DMA chain and deciding what to return on the fly.

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

// The status_seen[] / status_irq_rdrf / status_irq_nordrf instruments that used
// to live here are GONE (2026-10-08).  They existed to answer "why is the ACIA
// handler entered 300 times for 24 real events", which was answered in 27cf7d2
// - it is not, that was a cumulative counter read against a per-test one, and
// the real figures are 301 bytes / 301 interrupts / 301 entries with zero
// spurious - and the test bug behind it was fixed in 3b0473c.  They were six
// volatile accesses, including an array store, in the hottest branch of the
// read ISR, for a question that is closed.

void __isr
__time_critical_func(dma_bus_read_irq_handler())
{
#if ISR_COST_ON
	const uint32_t read_cost_start = systick_hw->cvr;
#endif
#if TESTPOINTS_ON
	gpio_put(GPIO_TRACE_C, true);
#endif
	pio_interrupt_clear(piodma_read.pio, READ_DEVICE_PIO_IRQ);
	// NO AVMA TEST ANY MORE.  AVMA is a forecast - asserted in cycle N it says
	// cycle N+1 is a valid bus cycle - so sampled here, after the read it was
	// meant to judge had completed, it described the FOLLOWING cycle, and the
	// measurement said as much: low on 1.1% of completions and ZERO rejections
	// in 610 ACIA data reads over a full acia_torture run.  A SIO read and a
	// branch on every device read, for nothing (2026-10-11).  The filter was
	// belt-and-braces anyway: this DMA only completes when the guest really
	// did read this window.  The run-state guard stays: a snippet's reads
	// must not take the device fast path.
	if (MC6809.run_state >= RS_RUNNING) {
		const uintptr_t read_addr = dma_channel_hw_addr(piodma_read.data_channel)->read_addr;
		const uint8_t loc = read_addr & registers::register_mask;
		if (loc == CONSOLE_RX_DATA) {
			// Use a dedicated wrapping counter so that a rapid CONSOLE_STATUS
			// ISR cannot overwrite this notification before Core 1 processes it.
			// A single read_irq_received flag would be silently lost if a second
			// ISR fired before Core 1 drained it, causing guest_receive() to be
			// skipped and RDRF to stay set with a stale byte.
			rx_read_count++;
			loop_work = 1u;
			// Fast-path: clear RDRF/IRQ immediately so the next 6809 status
			// read sees the change.  guest_receive() will stage the next byte
			// (if any) and re-set RDRF on the next for(;;) iteration.
			fast_serial.rx_read_fast_path();
			// DROP /IRQ HERE.  rx_read_fast_path() has just cleared RDRF, so
			// the console ACIA is no longer asking; holding the pin until the
			// loop's next pass is what let the guest's RTI complete with /IRQ
			// still low and re-enter with nothing pending.
			//
			// The comment here used to forbid this, on the grounds that
			// clearing would deassert even if another device still wanted the
			// line.  irq_retired answers that: the pin drops only when no
			// other source wants it.  The loop cannot interleave - it masks
			// for its update - so this read-modify-write is safe.
			irq_retired = static_cast<uint8_t>(irq_retired
							   | IRQ_SRC_CONSOLE_ACIA);
			if ((irq_sources & ~irq_retired) == 0u
			    && (int_pins_held & PEND_IRQ)) {
				pending_interrupts =
					static_cast<uint8_t>(pending_interrupts & ~PEND_IRQ);
				const uint32_t oe =
					int_pins_held & ~static_cast<uint32_t>(PEND_IRQ);
				int_pins_held = oe;
				pio_sm_put(int_pins.pio,
					   static_cast<uint>(int_pins.sm), oe);
#if DIAG_ON
				irq_isr_deassert++;
#endif
			}
		} else if (loc == CONSOLE_STATUS) {
#if DIAG_ON
			// Counted HERE because the generic notification path
			// cannot see it: this branch does not set
			// read_irq_received, so Core 1's switch never reaches
			// CONSOLE_STATUS.  That is what left st's console
			// "Status" column a permanent zero from 1527983 until
			// 2026-10-04 - and why a gated-off counter must print
			// "off" rather than 0.
			CONSOLE_STATUS_READS++;
#endif
		} else if (loc == AUX_RX_DATA) {
			aux_rx_read_count++;
			loop_work = 1u;
			aux_serial.rx_read_fast_path();
		} else if (loc == SYSTEM_TIMER_STATUS) {
			// Reading the timer status register acknowledges the IRQ.
			system_tick.acknowledge();
		} else if (loc == SYSTEM_TASK) {
			task_log_add('R', reg[SYSTEM_TASK], MC6809.task_get());
		} else if (loc == SYSTEM_DAT_FAULT) {
			// Reading the fault register clears the fault and the R/!W
			// latched with it — the DMA has already handed the pre-clear byte
			// to the PIO, so the guest sees the fault and the clear lands
			// before its next read.  NMIBlock (bit 7) is live state and
			// persists.  SYSTEM_DAT_FAULT_LOC is left alone: it only means
			// anything while a fault is latched, and the next fault
			// overwrites it, so the guest can read the two in either order.
			reg[SYSTEM_DAT_FAULT] &= static_cast<uint8_t>(~DAT_FAULT_LATCHED);
		}
		// No "else": a device register with nothing to do here used to be
		// handed to Core 1's loop through a single slot, whose consumer
		// was a switch in which every case was `break`.  Two volatile
		// stores per such read, for nothing.
	}
#if TESTPOINTS_ON
	gpio_put(GPIO_TRACE_C, false);
#endif
#if ISR_COST_ON
	isr_cost_add(read_cost, read_cost_start);
#endif
}

// ISR for the R/!W bus write loop and DMA chain
// The PIO mediates the signalling
// There are 2 DMA transfers:
//   1) accept write_registers + A0-A5
//   2) accept the byte to be written to that address

// EVERY GUEST WRITE'S ADDRESS, WRITTEN BY DMA RATHER THAN READ BACK BY THE ISR.
//
// THE BUG THIS EXISTS FOR.  The write chain hands the data channel its target
// by storing into that channel's write_addr register, and write_increment is
// false, so the register simply holds wherever the last write went.  The ISR
// used to read it back to learn where the guest had written:
//
//     written_addr = dma_channel_hw_addr(data_channel)->write_addr;
//
// THAT IS A READ-AFTER-OVERWRITE RACE, not a missed interrupt.  The interrupt
// is raised once per completion and the handler does run once per write - but
// the completion also CHAINS, re-arming the address channel, which loads the
// NEXT write's address into that same register the moment the next write
// starts.  If the handler has not run by then it reads the next write's
// address while it is supposed to be servicing this one: this write is skipped
// entirely, and the next is serviced twice.  For a mirror, doing one twice is
// harmless and losing one is fatal.
//
// MEASURED, AND BOTH MECHANISMS ARE REAL.  This went in believing the cause was
// two completions sharing one latched interrupt, and the drain counter first
// read worst 1, zero shared, which looked like a flat refutation.  A later
// 2.4 MHz run after a power cycle read WORST 2, ONE SHARED: coalescing does
// happen, just rarely - once in a 12-test suite against the hundreds of writes
// the register race was losing.
//
// So the race is the dominant mechanism and the one that broke test 8, because
// it needs only the next ADDRESS to arrive, which happens far sooner than a
// whole next completion.  Coalescing is the same loss by a rarer route.  The
// ring closes both, and the single-slot ISR would have dropped a write in
// either case - so do not take a worst of 1 in a short run as proof that
// coalescing cannot happen here.
//
// Why it shows above 2.3 MHz: an E cycle is 320 clk_sys cycles at 1 MHz and 133
// at 2.4, and a 6809 STD puts its two bytes one E cycle apart.  At 1 MHz the
// handler always wins that race; at 2.4, with the Q handler and the read ISR
// also contending, it does not.  Evidence at 2.4 MHz: the guest's handler did
// STD IRQ_COUNT to $FFD0/$FFD1 and the arrays came out write side 01 00, read
// side 00 00 - the high byte's mirror lost, the low byte's applied.  The guest
// had set its counter to 256, read back 0, and waited for ever.  At 2.3 MHz the
// two agreed.
//
// IT IS NOT ONLY THE MIRROR.  The same skipped write misses
// bus_write_followup()'s dispatch of SYSTEM_TASK, CONSOLE_CONTROL and
// CONSOLE_TX_DATA.  The note at the top of mc6850.cpp describes a related
// hazard on this chain from the other side: "two completions while masked are
// served as one, which misaligned DriveWire".
//
// HOW IT WORKS.  A third channel is chained in after the data byte lands and
// copies the data channel's write_addr - a fixed source, one 32-bit word - into
// this ring, whose write address wraps in hardware.  The address is captured AT
// THE MOMENT IT WAS USED rather than read back afterwards, so nothing can
// overwrite it first.  Nothing decides anything; it just records.  The handler
// then drains however many entries are present, which also makes real
// coalescing harmless if it ever does start.
//
// THE ADDRESS IS ENOUGH FOR THE MIRROR; THE VALUE IS NOT CAPTURED.  The drainer
// reads the byte back out of write_registers[], so two writes to the SAME
// location give two entries that both read the final value.  For the mirror
// that is exactly right - it is memory, and the last value written is the one
// that should be readable.
//
// FOR A DISPATCHED REGISTER IT LOSES THE INTERMEDIATE VALUE, and the handshakes
// do NOT rule that out.  An earlier version of this note claimed they did:
// CONSOLE_TX_DATA/AUX_TX_DATA drop TDRE until Core 1 takes the byte (see
// write_received()), and no kernel path stores twice to SYSTEM_TASK.  But TDRE
// only OFFERS that handshake - a guest is free to ignore it, and
// acia_torture's test 6 does exactly that on purpose:
//
//     T6Send  STA ACIATX / INCA / DECB / BNE T6Send     no TDRE poll
//
// so the protection is the drain keeping up, not the protocol.  Measured: 16
// bytes all arrive at 2.4 MHz and only 4 of 16 at 2.5, where the guest outruns
// Core 1's drain.  That is the loop's dispatch deadline, not this ring - the
// two $FFD0 dumps in st AGREE throughout, so nothing is being lost on the
// mirror path.  Capturing the value as well would fix it, and is the obvious
// next step if a real guest ever writes a device register without polling.
//
// Must be aligned to its own byte size: the hardware ring wraps address bits,
// so the buffer has to start on that boundary.
static constexpr uint32_t WRITE_ADDR_RING_SIZE = 64u;		// entries, power of 2
static constexpr uint32_t WRITE_ADDR_RING_MASK = WRITE_ADDR_RING_SIZE - 1u;
static constexpr uint32_t WRITE_ADDR_RING_BYTES = WRITE_ADDR_RING_SIZE * 4u;
static constexpr uint32_t WRITE_ADDR_RING_BITS = 8u;		// log2(256 bytes)
static_assert((1u << WRITE_ADDR_RING_BITS) == WRITE_ADDR_RING_BYTES,
	      "the DMA ring wrap is in address BITS, so the entry count and "
	      "the bit count have to agree");
alignas(WRITE_ADDR_RING_BYTES) static volatile uint32_t
	write_addr_ring[WRITE_ADDR_RING_SIZE];
// AND THE BYTE AS IT WAS WRITTEN, in a second ring with the same index.  The
// PIO sends the data twice - once for the DMA that stores it, once for here -
// so every write puts exactly one entry in each ring and one tail serves both.
//
// Captured rather than read back because reading back is only correct while no
// later write has landed on the same address.  A guest may write a device
// register twice in a row: TDRE offers a handshake but does not enforce one,
// and acia_torture's test 6 declines it deliberately, so at 2.5 MHz twelve of
// its sixteen bytes were read back as whatever arrived last.  One byte per
// entry, so the ring wrap is 6 bits rather than 8.
static constexpr uint32_t WRITE_DATA_RING_BITS = 6u;	// log2(64 bytes)
static_assert((1u << WRITE_DATA_RING_BITS) == WRITE_ADDR_RING_SIZE,
	      "the two rings share a tail, so they must hold the same number "
	      "of entries");
alignas(WRITE_ADDR_RING_SIZE) static volatile uint8_t
	write_data_ring[WRITE_ADDR_RING_SIZE];
static uint32_t write_addr_ring_tail = 0u;	// the ISR is its only reader
static constexpr int WRITE_ADDR_RING_CHANNEL = 6;
static constexpr int WRITE_DATA_RING_CHANNEL = 7;
#if DIAG_ON
// HOW MANY ENTRIES ONE INTERRUPT HAD TO DRAIN, worst case.  1 through several
// runs, then 2 with one shared at 2.4 MHz - so coalescing is real but rare, and
// a short run reading 1 proves nothing either way.  It is the dominant register
// race that this ring was built for (see above); this counter measures the rarer
// route to the same loss, and bounds how much runway the ring needs.
static volatile uint32_t write_addr_ring_worst = 0u;
static volatile uint32_t write_addr_ring_coalesced = 0u;	// entries beyond the first
// The producer lapping the consumer - 64 writes inside one interrupt latency,
// which should be unreachable.  Counted rather than assumed.
static volatile uint32_t write_addr_ring_overflow = 0u;
#endif

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
// WRITE RING OVERFLOW: a guest write whose dispatch was DROPPED because Core 1
// did not drain the ring in time.  That is real loss, not a statistic, which is
// why it is reported in st and why a non-zero value is called out rather than
// printed as just another column.  volatile: written in the write ISR.
//
// This is the only visible symptom of Core 1's loop missing ITS deadline.  The
// loop has one - it is just longer than an ISR's, set by how deep this ring is
// rather than by the E cycle - so "the loop has no deadline" is wrong.
static volatile uint32_t write_ring_overflow = 0u;
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
	// THE HOTTEST CASE FIRST, and it used to be LAST.
	//
	// $FFD0-$FFFF is plain memory to the guest - the shared buffer during a
	// block copy, the snippet area, the vectors - and it is far and away the
	// commonest write here, which is exactly why the dispatch ring already
	// filters it out (see the note further down).  Yet it was reached only
	// after NINE device-register comparisons it can never match, on a handler
	// measured at 124 clk_sys cycles mean against a budget of 133 at 2.4 MHz
	// and 128 at 2.5 - see write_cost and the project note write-isr-cost.
	//
	// One range test decides it now.  The device registers pay an extra
	// compare instead, and they are both rarer and already paced by their own
	// handshakes.
	//
	// Every dispatched register is at offset 0x00-0x0C, so none of them can be
	// shadowed by this early return; the static_assert on the ring filter
	// below says the same thing and would fail if one moved.
	if (wloc >= REGISTER_BUFFER_OFFSET) {
		static_assert(REGISTER_BUFFER_OFFSET + REGISTER_BUFFER_LEN * 3u
			      == registers::register_length,
			      "the three mirrored blocks are assumed to reach the end "
			      "of the register window, which is what lets a single "
			      ">= test stand in for the old range test");
		// The watchpoint deliberately does NOT copy: the byte stays in the
		// write side while the read side keeps its old value, and that is
		// what protects the BREAK handler and the vectors it guards.
		if (watchpoint_armed && wloc >= REGISTER_SNIPPET_OFFSET) {
			watchpoint_nmi_pending = true;
			loop_nmi_work = 1u;
		} else
			reg[wloc] = wval;
		return;
	}
	// Special-case any writes that need to be handled before the next guest
	// instruction picks up an incorrect read. I.e., these need to be _fast_.
	if (wloc == SYSTEM_TASK) {
		const uint8_t old_task = MC6809.task_get();
		task_log_add('W', wval, old_task);
		const uint8_t new_task = MC6809.task_change(wval);
		if (old_task != 0u && new_task != 0u && new_task != old_task) {
			task_write_odd_count++;
			task_write_odd_from = old_task;
			task_write_odd_to = new_task;
		}
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
	else if (wloc == SYSTEM_WATCHPOINT) {
		// Arm/disarm deferred to write ring handler (needs snippet_code[])
	}
	else if (wloc == SYSTEM_TRACE) {
		// An armed ring is evidence: any write freezes it, so that guest
		// code - DBGMON's PrintChar - can mark the moment things went
		// wrong.  Once frozen, later writes are ignored rather than
		// re-arming over the capture, because DBGMON goes on printing.
		// Otherwise the USim TraceCtl convention: non-zero arms a
		// one-shot bus trace, zero stops.
		if (MC6809.trace_is_ring() && MC6809.trace_is_armed()) {
#if TRAP_TRIGGER_ON
			if (!MC6809.trace_ring_is_frozen())
				trap_trigger_pulse();
#endif
			MC6809.trace_ring_freeze();
		}
		else if (wval)
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
	// QUEUE ONLY WHAT THE DISPATCH WILL ACT ON.  Core 1's switch handles
	// SYSTEM_TASK, SYSTEM_REQUEST, SYSTEM_SUBREQUEST, SYSTEM_WATCHPOINT,
	// CONSOLE_CONTROL, CONSOLE_TX_DATA, AUX_CONTROL and AUX_TX_DATA - every
	// one of them at offset 0x00 to 0x0A, so everything from
	// REGISTER_BUFFER_OFFSET (0x10) up hits its "default: break" and the
	// entry is pure waste.  And those are the HOTTEST writes there are: the
	// shared buffer during a copy, the snippet area, the vectors.
	//
	// WHICH MATTERS BECAUSE THE RING IS THE 2.3 MHz LIMITER, not ISR cost.
	// It holds 32 entries and the loop has to drain them before the write
	// ISR refills it; measured 2026-10-08, a guest storing to $FFD0 every 8
	// E cycles overflowed it between 2.2 and 2.3 MHz and dropped 331383
	// dispatches while still reporting RS_RUNNING.  Filtering here cuts the
	// arrival rate to device-register writes only.
	//
	// The mirror above is unaffected - it still runs for the whole
	// $FFD0-$FFFF range, which is what makes the shared buffer readable.
	// Only the DISPATCH queue is filtered.
	static_assert(SYSTEM_WATCHPOINT < REGISTER_BUFFER_OFFSET
		      && CONSOLE_CONTROL < REGISTER_BUFFER_OFFSET
		      && AUX_TX_DATA < REGISTER_BUFFER_OFFSET
		      && SYSTEM_SUBREQUEST < REGISTER_BUFFER_OFFSET,
		      "a dispatched register has moved to or above "
		      "REGISTER_BUFFER_OFFSET, so the ring filter below would "
		      "silently stop delivering it");
	if (wloc < REGISTER_BUFFER_OFFSET) {
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
		loop_work = 1u;
	}
}

// WHERE THE RING CHANNEL IS ABOUT TO WRITE, as an entry index.  Its write
// address wraps inside the ring in hardware, so the difference from the base is
// always in range and no masking is needed.
static inline uint32_t
__time_critical_func(write_addr_ring_head())
{
	const uintptr_t next = dma_channel_hw_addr(WRITE_ADDR_RING_CHANNEL)->write_addr;
	return static_cast<uint32_t>((next - reinterpret_cast<uintptr_t>(write_addr_ring)) >> 2);
}

void __isr
__time_critical_func(dma_bus_write_irq_handler())
{
#if ISR_COST_ON
	const uint32_t write_cost_start = systick_hw->cvr;
#endif
#if TESTPOINTS_ON
	gpio_put(GPIO_TRACE_C, true);
#endif
	dma_channel_acknowledge_irq1(WRITE_ADDR_RING_CHANNEL);
	// DRAIN EVERY ENTRY, not just one.  Two completions can share a single
	// latched interrupt, and before the ring existed the earlier one was lost
	// for good - see the note on write_addr_ring.  The head is read once:
	// anything that arrives during the drain keeps its entry and is taken by
	// the next interrupt, which the chain is guaranteed to raise.
	const uint32_t head = write_addr_ring_head();
#if DIAG_ON
	uint32_t drained = 0u;
#endif
	while (write_addr_ring_tail != head) {
		const uint32_t ring_slot = write_addr_ring_tail;
		const uintptr_t addr = write_addr_ring[ring_slot];
		write_addr_ring_tail = (ring_slot + 1u) & WRITE_ADDR_RING_MASK;
#if DIAG_ON
		drained++;
#endif
		// The tail advances either way, so a write made while the guest
		// was not running is DISCARDED rather than left to be applied
		// later out of order.  That is the behaviour the single-slot
		// version had, and the DAT fault path depends on it: a write
		// taken while held in reset must not reach the read side.
		if (MC6809.run_state >= RS_RUNNING) {
			const uint8_t wloc = addr & registers::register_mask;
			// The byte the PIO captured, NOT a read-back of
			// write_registers[]: a later write to the same address
			// would have replaced that one.  Applying them in order
			// also leaves the read side holding the final value, so
			// the mirror is right as well as the dispatch.
			const uint8_t wval = write_data_ring[ring_slot];
			bus_write_followup(wloc, wval);
		}
	}
#if DIAG_ON
	// Three volatile read-modify-writes on every write interrupt, for a
	// number only st ever reads: the "pure user" tier, so behind DIAG_ON.
	if (drained > write_addr_ring_worst)
		write_addr_ring_worst = drained;
	if (drained > 1u)
		write_addr_ring_coalesced += drained - 1u;
	// A full ring and an empty one look identical from the indices, so this
	// is the nearest honest test: draining the whole thing means the producer
	// may have lapped and an entry may already be gone.  64 writes inside one
	// interrupt latency should be unreachable - if this is ever non-zero the
	// ring is too small, not the drain too slow.
	if (drained >= WRITE_ADDR_RING_SIZE - 1u)
		write_addr_ring_overflow++;
#endif
#if TESTPOINTS_ON
	gpio_put(GPIO_TRACE_C, false);
#endif
	// Last, as in the read ISR, so the span covers the whole handler
	// including the DEBUG bookkeeping above - that work is this ISR's cost
	// and belongs in its bucket, not left to land in loop_cost.
#if ISR_COST_ON
	isr_cost_add(write_cost, write_cost_start);
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
	const interrupt eirq = console_interrupt;
	// NMI IS CONSUMED ON READ.  It is edge-triggered and unmaskable, so a
	// level request is wrong for it: Core 1 writes pending_interrupts on
	// every iteration of its loop, the Q-rising handler turns PEND_NMI into a
	// two-cycle pulse and then releases the pin, and the next iteration
	// asserts it again - a fresh falling edge every three E cycles or so, for
	// as long as the request stands.  Nothing stops the guest taking NMI
	// again INSIDE the handler it has just entered, because NMI masks neither
	// itself nor anything else.
	//
	// That is what made `break` print the BREAK snippet's own registers -
	// PC $FFE4, four bytes in, with X $FFD0, its copy pointer - instead of
	// the guest's: a nested NMI stacked the snippet mid-copy, and the frame
	// that reached SHARED was the snippet's.  break holds the request until
	// Core 0 SEES the vector fetch, which it polls in 1 ms steps, so there
	// was room for hundreds of extra entries.
	//
	// FIRQ and IRQ stay held, deliberately: they are level-sensitive, a short
	// pulse can expire unseen, and the guest's own masking on entry is what
	// stops them re-entering.
	if (eirq == INTERRUPT_NMI)
		console_interrupt = INTERRUPT_NONE;
	return eirq;
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
// because a gain or offset error in the ADC cancels in a difference but
// corrupts an absolute threshold; version 2 once read 0.5 to 0.7V high while
// serving the bus, before its ADC-bank pins were clamped.  Set with
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
// THE DIE BEFORE IT HAS WARMED ITSELF.  The sensor's 0.706V at 27C is a
// typical figure and real parts spread tens of mV around it, so a per-board
// correction needs the die at a KNOWN temperature - which, with no external
// probe on it, means ambient after a long rest unpowered.  Within seconds of
// running at full clock the die is warmer than the package, and calibrating
// against a surface reading then would hide exactly the self-heating the
// sensor is there to watch.  So this is taken once, as early as the ADC is
// up, and never again.
static uint16_t die_boot_raw = 0u;
static uint32_t die_boot_ms = 0u;	// when, after reset
// A SAMPLE AT FULL SCALE IS NOT A MEASUREMENT.  When the ADC reference sags
// under bus switching (2026-10-06: 3.24V down to ~2.03V) this channel clips,
// and a clipped 4095 reads as 6.49V - HIGH, so it happily seeds the baseline.
// Then the bus goes quiet, the reference recovers, the reading falls to a
// correct 5.00V, and the relative detector sees a 1.49V collapse that never
// happened.  That powered the board down twice on 2026-10-06, mid-test.
//
// So a clipped sample is excluded from the decision ENTIRELY: it cannot seed
// the baseline, cannot move it, and cannot trip.  The cutout reports itself
// disabled instead, which is the honest answer - while the channel is
// saturated a real sag is invisible until the rail falls over 20%.
static constexpr uint16_t RAIL_CLIP_COUNTS = 4090u;
static volatile bool rail_clipped = false;	// last sample was at full scale

static volatile uint32_t rail_low_count = 0u;	// consecutive samples below the cutout
static volatile float rail_baseline = 0.0f;	// slow average of a healthy +5VP
static volatile uint32_t rail_good_count = 0u;	// consecutive healthy samples seen
static volatile uint64_t rail_settle_until = 0u;	// us since boot; no judging before this
static volatile bool rail_sampled = false;	// at least one pass has run
static volatile bool rail_collapsed = false;	// set here, acted on by the console loop

// These readings hold in every run state.
//
// Until October 2026 they did not: whenever the Pico answered bus reads both
// channels read high - the supply by 350 to 420 counts, the die by about 125 -
// and later the reference itself collapsed to ~2.03V.  The cause was version
// 2 putting 5V signals (A13-A15, FAULT) on GPIO 41-45, which are in the ADC
// bank and the only pins on the RP2350B that are NOT 5V tolerant.  With 1.5k
// series resistors and Schottky clamps on those four lines, adcdump on
// 2026-10-19 put all six states - reset, Pico R/W, RAM R/W, faulting, halted,
// unpowered - within 3 counts on the supply and 2 on the die, every standard
// deviation under 1.4 counts.  A board without the clamps still has the fault.
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
	// A clipped sample tells us nothing at all: 4095 reads as 6.49V, which is
	// HIGH, so it would happily seed the baseline - and then the reference
	// recovers, the reading falls to a correct 5.00V and the relative
	// detector sees a collapse that never happened.  That powered the board
	// down twice on 2026-10-06.  The clamps should make this unreachable; it
	// stays because a board without them still clips.
	//
	// Every other sample is judged in full, whatever the guest is doing.
	rail_clipped = raw >= RAIL_CLIP_COUNTS;

	rail_judgement rj = { false, rail_low_count, rail_baseline,
			      rail_good_count };
	if (!rail_clipped) {
		rj = rail_judge(rs, rail_cfg);
		rail_low_count = rj.low_count;
		rail_baseline = rj.baseline;
		rail_good_count = rj.good_count;
	}
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
	       PRIu32 "   recomputes %" PRIu32 "\n",
	       name, w.staged, w.rx_read, w.rts_drop, w.reset, w.cts_change,
	       w.recomputes);
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
	s->temperature_boot = die_boot_raw;
	s->temperature_boot_ms = die_boot_ms;

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
	s->rail_clipped = rail_clipped;
	s->busy_lic_avma = MC6809.get_busy_lic_avma();
	// pio2 flag 1 is set by the clock at every E-falling and consumed by
	// nothing since the BUSY/LIC/AVMA mirror went, so SET is now the expected
	// reading and says only that the clock is running.  Flag 0 is the one
	// that matters: bus_fault's wait clears it, so it should read clear.
	s->pio2_irq = static_cast<uint8_t>(bus_fault.pio->irq & 0xFFu);
	// DID THE WRITE STATE MACHINE EVER STALL ON A FULL RX FIFO?  The question
	// is whether the data ring's second push is what costs 2.5 MHz: the SM
	// sends three words per write cycle into a 4-deep FIFO, and a blocking
	// push to a full one HALTS THE STATE MACHINE - which makes it miss bus
	// cycles outright, far worse than the stale dispatch byte it cured.
	//
	// RXSTALL is the hardware's own answer, latched per SM, so it needs no
	// sampling luck.  Cleared at Core 1 launch and by fq, so a reading always
	// belongs to the current clock.
	s->write_fdebug = piodma_write.pio->fdebug;
	s->clock_pc = (pio_clock.sm < 0) ? 0xFFu : static_cast<uint8_t>(
		pio_sm_get_pc(pio_clock.pio, static_cast<uint>(pio_clock.sm))
		- static_cast<uint>(pio_clock.offset));
	s->last_lic = last_LIC;
	s->lic_stalled = false;
	// Only meaningful when the count is actually being kept.  count_lic() is
	// behind DIAG_ON, so without it lic_count never leaves 0, "unchanged since
	// last time" is true on every call, and st printed WARNING: LIC stalled
	// every time the guest ran.  A false alarm that says the CPU has died is
	// worse than no alarm: it cost a diagnosis in this session, which is the
	// third time this counter has misled - see the "prints off, never 0" note.
#if DIAG_ON
	if (MC6809.is_running()) {
		if (last_LIC != s->lic_count)
			last_LIC = s->lic_count;
		else
			s->lic_stalled = true;
	}
#endif
#endif

	registers::copy_out_write(s->write_regs, 0, 64);
	registers::copy_out(s->read_regs, 0, 64);

	s->tick_enabled = system_tick.is_enabled();
	s->tick_irq_pending = system_tick.is_irq_pending();

	s->core1_loop = core1_loop_count;
#if DIAG_ON
	s->core1_qrise = core1_qrise_count;
	s->core1_irq_assert = core1_irq_assert_count;
	s->core1_write_ring = core1_write_ring_count;
#else
	s->core1_qrise = s->core1_irq_assert = s->core1_write_ring = 0u;
#endif
#ifdef DEBUG
	// Guarded to match the field, which lives in status.h's DEBUG section -
	// as do the increment and the report, so all three agree.
	s->write_ring_overflow = write_ring_overflow;
#if DIAG_ON
	s->write_addr_worst = write_addr_ring_worst;
	s->write_addr_coalesced = write_addr_ring_coalesced;
	s->write_addr_overflow = write_addr_ring_overflow;
#endif
#endif
#if DIAG_ON
	s->core1_last_write = core1_last_write_loc;
#else
	s->core1_last_write = 0u;
#endif
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
#if PICO_THING_BOARD_VERSION >= 2
	// The baseline as well as the sample: without it there is no way to see
	// whether the relative detector is armed, which is what the bench test
	// for the cutout has to watch while it sags the sense node at J107.
	if (s->rail_clipped)
		printf("  cutout DISABLED - supply channel at full scale, so the"
		       " rail cannot be seen\n");
	else if (s->rail_baseline > 0.0f)
		printf("  cutout baseline %.2fV (%" PRIu32 " good)%s\n",
		       (double) s->rail_baseline, s->rail_good,
		       s->rail_good >= 50u ? ", armed"
					   : ", still establishing");
	else
		printf("  cutout baseline not established - absolute floor only\n");
	printf("PICO SUPPLY (+5VP): %.2fV (%u counts)\n",
	       adc_rail_volts(s->voltage), s->voltage);
#endif
	printf("PICO TEMPERATURE: %.1fC (%u counts)\n",
	       adc_die_celsius(s->temperature), s->temperature);
	printf("  die at boot: %.1fC (%u counts, %" PRIu32 "ms after reset)\n",
	       adc_die_celsius(s->temperature_boot), s->temperature_boot,
	       s->temperature_boot_ms);
	// THE DIE SENSOR BOUNDS THE REFERENCE, which nothing else here can do.
	//
	// Every reading above is counts x ADC_VREF_VOLTS / 4096: a ratio against
	// the reference, so a reference that is not what the constant says makes
	// every voltage and every temperature wrong by that ratio, silently.  The
	// die sensor is the one input whose true voltage is KNOWN and which
	// nothing on the carrier board can touch - 0.706V at 27C, -1.721 mV/C -
	// so over the chip's whole legal -40C..+125C its voltage must lie in
	// 0.537V..0.821V.  Divide those by the counts and you have the reference,
	// no assumption about the board and none about the temperature:
	//
	//     V_sensor = counts x VREF / 4096   =>   VREF = V_sensor x 4096 / counts
	//
	// A temperature outside that range therefore does not mean the die is
	// impossibly hot or cold.  It means the REFERENCE IS WRONG, and this says
	// so rather than printing -213C and leaving it to be puzzled over.
	//
	// Measured on 2026-10-05 after fitting the HD63C09E: 1414 counts, which
	// puts the reference near 2.05V against the 3.2435V the constant claims -
	// and the +5VP channel pinned at 4095, where no sag can be seen at all.
	{
		const float t = adc_die_celsius(s->temperature);
		if (s->temperature != 0u && (t < -40.0f || t > 125.0f)) {
			const float lo = 0.5374f * 4096.0f
				       / (float) s->temperature;	// die at +125C
			const float hi = 0.8213f * 4096.0f
				       / (float) s->temperature;	// die at -40C
			const float nominal = 0.706f * 4096.0f
					    / (float) s->temperature;	// die at 27C
			printf("*** ADC REFERENCE IS WRONG: the die sensor cannot "
			       "read %.1fC ***\n", (double) t);
			printf("    it bounds the reference at %.2f-%.2fV "
			       "(%.2fV for a 27C die), not the %.4fV configured\n",
			       (double) lo, (double) hi, (double) nominal,
			       (double) ADC_VREF_VOLTS);
			printf("    so EVERY voltage above is wrong by that ratio, "
			       "and the supply cutout cannot be trusted\n");
		}
		if (s->voltage >= 4095u)
			printf("*** +5VP CHANNEL IS CLIPPED at full scale: the "
			       "reading is a floor, not a measurement ***\n");
	}
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
	// WHICH DIAGNOSTICS ARE COMPILED IN.  Printed because the alternative is
	// reading a zero and wondering: an absent count has been mistaken for a
	// fault three times on this firmware.  DEBUG is not listed - it is on in
	// every build by design and costs nothing on the per-cycle paths.
	printf("DIAG: counters %s  testpoints %s  ISR cost %s"
	       "  (-D PICO_THING_{DIAG,TESTPOINTS,ISR_COST}=1)\n",
	       DIAG_ON ? "ON" : "off",
	       TESTPOINTS_ON ? "ON" : "off",
	       ISR_COST_ON ? "ON" : "off");
	printf("  NMI: %-12s FIRQ: %-12s  IRQ: %s\n",
	       s->nmi ? "OUTPUT LOW" : "hi-z",
	       s->firq ? "OUTPUT LOW" : "hi-z",
	       s->irq ? "OUTPUT LOW" : "hi-z");
	printf("\n");

	// 2. Bus state, run state
	printf("Bus State: %-16s  Prev: %-16s\n",
	       bus_state_string[s->bus_state], bus_state_string[s->old_bus_state]);
	printf("Run State: %s\n", run_state_string[s->run_state]);
	// The interrupt-nesting depth and the task history that went with it are
	// gone with the RTI detector: without it there is nothing to unwind them,
	// so a count could only ever climb.
	printf("Task: %u  Odd task writes: %" PRIu32, s->task, s->task_write_odd_count);
	if (s->task_write_odd_count > 0u)
		printf("   <- last was task %u -> %u!",
		       s->task_write_odd_from, s->task_write_odd_to);
	printf("\n");
#if DIAG_ON
	{
		// Task register event log, copied without locking (diagnostic).
		const uint32_t n = task_log_count;
		const uint32_t shown = n < TASK_LOG_LEN ? n : TASK_LOG_LEN;
		printf("  Task log: %" PRIu32 " events; last %" PRIu32 " (W val<-in force: guest write; R served/in force: guest read-back; F: fault switch):\n", n, shown);
		for (uint32_t k = n - shown; k < n; ++k) {
			const volatile task_event &e = task_log[k % TASK_LOG_LEN];
			printf("    #%" PRIu32 " @%" PRIu32 " %c %02X/%02X\n", k, e.when, e.kind, e.value, e.in_force);
		}
	}
#else
	printf("  Task log: off (build with -D PICO_THING_DIAG=1)\n");
#endif

	// 3. LIC / RTI (DEBUG only)
#ifdef DEBUG
	// The live bits as well as the count: the count only moves while the guest
	// is running, so under HALT the bits are the only way to see BUSY/LIC/AVMA
	// at all - and LIC is HIGH when halted at an instruction boundary.
	BLA live{}; live.byte = s->busy_lic_avma;
#if DIAG_ON
	printf("LIC: %-10" PRIu32 "  pins: %c%c%c\n",
	       s->lic_count,
#else
	// NOT "0": a gated-off count that prints a number reads as a dead CPU.
	printf("LIC: %-10s  pins: %c%c%c\n", "off",
#endif
	       live.bit.BUSY ? 'B' : '.',
	       live.bit.LIC  ? 'L' : '.',
	       live.bit.AVMA ? 'V' : '.');
	printf("pio2 irq flags %02X (flag1 %s - unconsumed, expected)  clock pc +%u\n",
	       s->pio2_irq, (s->pio2_irq & 0x02u) ? "SET" : "clear", s->clock_pc);
	// THE WRITE SM'S STALL FLAGS.  RXSTALL set means a blocking push met a
	// full RX FIFO and HALTED THE STATE MACHINE, which loses bus cycles - the
	// thing to rule in or out before adding any more pushes to this program.
	// TXSTALL is expected to be set and means nothing here: the program pulls
	// its prefix once at startup and then never again, so the TX FIFO is
	// empty for ever after.
	{
		const uint sm = static_cast<uint>(piodma_write.sm);
		const bool rxstall = (s->write_fdebug >> (PIO_FDEBUG_RXSTALL_LSB + sm)) & 1u;
		const bool rxunder = (s->write_fdebug >> (PIO_FDEBUG_RXUNDER_LSB + sm)) & 1u;
		printf("write pio sm%u: RX FIFO push stalled %s   RX read when empty %s"
		       "   (fdebug %08" PRIX32 ")\n",
		       sm, rxstall ? "*** YES ***" : "no",
		       rxunder ? "YES" : "no", s->write_fdebug);
	}
	if (s->lic_stalled)
		printf("WARNING: LIC stalled at %" PRIu32 ", run state is %s\n",
		       s->lic_count, run_state_string[s->run_state]);
#if ISR_COST_ON
	// What the per-cycle ISRs cost against what an E cycle affords.  Both
	// land in the same E cycle whenever the guest touches this window, so
	// the two means added is the figure to compare with the budget.  The
	// instrument itself adds ~12 cycles to each, so these are upper bounds.
	{
		// Read the fields out one at a time, for the same reason.
		const uint32_t q_last = q_cost.last, q_min = q_cost.min,
			       q_max = q_cost.max, q_n = q_cost.n;
		const uint64_t q_sum = q_cost.sum;
		const uint32_t r_last = read_cost.last, r_min = read_cost.min,
			       r_max = read_cost.max, r_n = read_cost.n;
		const uint64_t r_sum = read_cost.sum;
		const uint32_t w_last = write_cost.last, w_min = write_cost.min,
			       w_max = write_cost.max, w_n = write_cost.n;
		const uint64_t w_sum = write_cost.sum;
		const float e_mhz = MC6809.get_e_frequency();	// MHz
		const uint32_t budget = (e_mhz > 0.0f)
			? (uint32_t) ((float) clock_get_hz(clk_sys)
				      / (e_mhz * 1.0e6f))
			: 0u;
		const uint32_t l_last = loop_cost.last, l_min = loop_cost.min,
			       l_max = loop_cost.max, l_n = loop_cost.n;
		const uint64_t l_sum = loop_cost.sum;
		const uint32_t qm = q_n ? (uint32_t) (q_sum / q_n) : 0u;
		const uint32_t rm = r_n ? (uint32_t) (r_sum / r_n) : 0u;
		const uint32_t wm = w_n ? (uint32_t) (w_sum / w_n) : 0u;
		const uint32_t lm = l_n ? (uint32_t) (l_sum / l_n) : 0u;
		// PER E CYCLE, NOT PER OCCURRENCE, and the two kinds of work are
		// kept APART, which is the whole point of this instrument.
		//
		// The ISRs have a DEADLINE: they must finish inside the E cycle
		// they belong to, so their total against the budget is what decides
		// how fast E can go.  Scale each by its own count against the
		// Q-rising count, which is exactly one per E cycle - the read ISR's
		// rate is no longer about 1.0, since the address filter stopped it
		// firing for anything but a device register.
		//
		// Core 1's loop has NO deadline and must not be added in.  It is a
		// SPINNER: it takes whatever the ISRs leave, so its share rises as
		// theirs falls and the total sits near 100% for ever.  Adding it is
		// what made this line read 99% across two changes that each removed
		// real work - 318 of 320 before and after - and look broken.  Its
		// passes went from 754k to 116M over the same two changes while
		// reporting the same utilisation, which is the giveaway.
		// THE WRITE ISR IS NOW IN THE SUM.  Scaled the same way as the read
		// ISR: one Q pass per E cycle, so n/q_n is
		// each handler's entries per E cycle.  Until this term existed the
		// figure was too low by exactly this much, and that is the half of
		// the "wrong in both directions" headline that made it understate.
		const uint32_t isr_demand = q_n
			? qm + (uint32_t) ((uint64_t) rm * r_n / q_n)
			   + (uint32_t) ((uint64_t) wm * w_n / q_n)
			: 0u;
		const uint32_t loop_share = q_n
			? (uint32_t) ((uint64_t) lm * l_n / q_n)
			: 0u;
		printf("Per-cycle cost in clk_sys cycles (instrument adds ~6 to each):\n");
		printf("  Q-rising  last %4" PRIu32 "  min %4" PRIu32 "  max %5" PRIu32
		       "  mean %4" PRIu32 "  n %" PRIu32 "\n",
		       q_last, q_n ? q_min : 0u, q_max, qm, q_n);
		printf("  read DMA  last %4" PRIu32 "  min %4" PRIu32 "  max %5" PRIu32
		       "  mean %4" PRIu32 "  n %" PRIu32 "\n",
		       r_last, r_n ? r_min : 0u, r_max, rm, r_n);
		// The write ISR's span VARIES BY DESIGN - it drains however many
		// ring entries are waiting - so the gap between mean and max is
		// the reading, not the mean alone.
		printf("  write DMA last %4" PRIu32 "  min %4" PRIu32 "  max %5" PRIu32
		       "  mean %4" PRIu32 "  n %" PRIu32 "\n",
		       w_last, w_n ? w_min : 0u, w_max, wm, w_n);
		printf("  Core1 loop last %4" PRIu32 "  min %4" PRIu32
		       "  max %5" PRIu32 "  mean %4" PRIu32 "  n %" PRIu32
		       "  (own work, ISRs subtracted)\n",
		       l_last, l_n ? l_min : 0u, l_max, lm, l_n);
		// WHERE THE READ SM SPENDS ITS TIME, by sampling its PC.  The DMA
		// round trip - push the address, wait for the byte to arrive in
		// the TX FIFO - is the one unmeasured number in the 3 MHz budget,
		// and it is invisible from the CPU because the DMA runs on its
		// own.  But the SM blocks on `pull block` while it waits, so the
		// fraction of samples found at that instruction IS the fraction
		// of an E cycle spent waiting for the DMA.
		//
		// The histogram prints raw offsets into mc6809_bus_read.  THEY MOVE
		// WHENEVER THAT PROGRAM IS EDITED: the DMA wait sat at +15 until
		// the address filter made the program 25 words, after which this
		// read "out null 4" and called it the round trip.  So the one
		// offset the arithmetic depends on is now taken from the assembler
		// as mc6809_bus_read_offset_dma_wait, declared "public" on the
		// instruction itself, and cannot drift again.
		//
		// Reading a PIO register is non-intrusive: the SM does not know.
		if (piodma_read.sm >= 0 && piodma_read.offset >= 0) {
			static constexpr uint PC_SAMPLES = 20000u;
			uint16_t hist[32] = {};
			const uint pc_base = (uint) piodma_read.offset;
			for (uint i = 0u; i < PC_SAMPLES; i++) {
				const uint pc = pio_sm_get_pc(piodma_read.pio,
							      (uint) piodma_read.sm);
				hist[(pc - pc_base) & 31u]++;
			}
			printf("  read SM PC, %u samples:", PC_SAMPLES);
			for (uint i = 0u; i < 32u; i++)
				if (hist[i] * 200u >= PC_SAMPLES)	// >= 0.5%
					printf("  +%u %.1f%%", i,
					       100.0 * hist[i] / PC_SAMPLES);
			printf("\n");
			// The SM blocks at dma_wait, so the share of samples found
			// there is the share of an E cycle spent waiting for the
			// DMA.  Scaled by the budget, that is the round trip.
			static constexpr uint DMA_WAIT_PC =
				mc6809_bus_read_offset_dma_wait;
			if (budget)
				printf("  DMA round trip ~%.0f clk_sys cycles "
				       "(%.1f%% of an E cycle at %.1f MHz)\n",
				       (double) hist[DMA_WAIT_PC] * budget / PC_SAMPLES,
				       100.0 * hist[DMA_WAIT_PC] / PC_SAMPLES,
				       (double) e_mhz);
		}
		printf("  ISR demand %" PRIu32 " of %" PRIu32 " cycles per E cycle"
		       " at %.1f MHz (%" PRIu32 "%%) - THE DEADLINE\n",
		       isr_demand, budget, (double) e_mhz,
		       budget ? isr_demand * 100u / budget : 0u);
		// What the ISRs alone allow, which is the number to watch as the
		// redesign proceeds: it falls as they get cheaper, where the old
		// combined figure could not.
		if (isr_demand)
			printf("    so the ISRs alone cap E at %.2f MHz\n",
			       (double) clock_get_hz(clk_sys)
			       / (double) isr_demand / 1.0e6);
		// CORE 1'S LOOP HAS A DEADLINE TOO - it is just longer than an
		// ISR's, and set by how deep the write ring is rather than by the
		// E cycle.  The ring holds WRITE_RING_SIZE entries and the write
		// ISR fills one per guest write to this window, so the loop has to
		// complete a drain within that many writes or the oldest dispatch
		// is dropped.  The label here used to read "NOT demand: it spins,
		// so it absorbs whatever is left", which was wrong: it IS demand,
		// against a longer deadline.
		//
		// The runway in E CYCLES depends on how fast the guest can store
		// into the window.  Extended addressing is five cycles and indexed
		// no quicker, so four is a conservative floor - it cannot sustain
		// one write per cycle.
		static constexpr uint32_t MIN_WRITE_SPACING_CYCLES = 4u;
		const uint32_t runway = (uint32_t) WRITE_RING_SIZE
				      * MIN_WRITE_SPACING_CYCLES;
		// THE GAP BETWEEN PASSES, from loop_period - the wall-clock
		// interval, ISR time included, because the write ISR fills the
		// ring throughout it.  loop_cost would have been the wrong bucket:
		// it is the loop's own work with ISR time subtracted, so it
		// understates the gap.
		//
		// Printed with a decimal because at 1 MHz a real 1.8 E cycles
		// truncated to "1" and both columns read the same.
		const uint32_t p_n = loop_period.n, p_max = loop_period.max;
		const uint64_t p_sum = loop_period.sum;
		const double e_per = budget ? (double) budget : 1.0;
		const double gap_mean = p_n
			? (double) p_sum / (double) p_n / e_per : 0.0;
		const double gap_max = (double) p_max / e_per;
		printf("  Core1 loop gap: mean %.1f, worst %.1f E cycles;"
		       " own work %" PRIu32 " cycles per E cycle\n",
		       gap_mean, gap_max, loop_share);
		printf("    its DEADLINE is %u ring entries, about %" PRIu32
		       " E cycles%s\n", WRITE_RING_SIZE, runway,
		       (gap_max > (double) runway)
			       ? "  *** WORST GAP EXCEEDS THE RUNWAY ***" : "");
	}
#endif
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
	// Interrupt bookkeeping that is Core 1's, not any one device's: it sat
	// under the console ACIA only because the first of these counters was
	// added there.
	printf("Interrupts:\n");
	// A GATED-OFF COUNTER PRINTS "off", NEVER 0.  Three times in one session
	// an absent or zeroed count was read as a fault - this console "Status"
	// column, st's "Read IRQ" after the address filter, and the LIC count -
	// so a zero here has to mean zero.
#if DIAG_ON
	printf("  IRQ taken %" PRIu32 " times; /IRQ asserted %" PRIu32
	       " E cycles (%.1f cycles per taken interrupt)\n",
	       irq_vectors, pend_irq_cycles,
	       irq_vectors ? (double) pend_irq_cycles / (double) irq_vectors
			   : 0.0);
#else
	printf("  IRQ taken %" PRIu32 " times; /IRQ asserted: off"
	       " (build with -D PICO_THING_DIAG=1)\n", irq_vectors);
#endif
#if DIAG_ON
	printf("  /IRQ: loop raised %" PRIu32 ", dropped %" PRIu32 " at the"
	       " data read, %" PRIu32 " at the vector fetch\n",
	       irq_loop_assert, irq_isr_deassert, irq_ack_deassert);
#else
	printf("  /IRQ: where it was raised and dropped is off"
	       " (build with -D PICO_THING_DIAG=1)\n");
#endif
#if DIAG_ON
	printf("  Armed byte: %" PRIu32 " time%s a live source found it clear for two passes%s\n",
	       q_armed_missed, q_armed_missed == 1u ? "" : "s",
	       q_armed_missed ? "  *** A WRITER IS MISSING q_rising_arm() ***" : "");
	printf("  NMI-work byte: %" PRIu32 " time%s a live source found it clear for two passes%s\n",
	       nmi_work_missed, nmi_work_missed == 1u ? "" : "s",
	       nmi_work_missed ? "  *** A WRITER IS MISSING loop_nmi_work = 1 ***" : "");
#else
	printf("  Armed byte audit: off (build with -D PICO_THING_DIAG=1)\n");
	printf("  NMI-work byte audit: off (build with -D PICO_THING_DIAG=1)\n");
#endif
	printf("Core 1:\n");
	// READ IRQ AND "Last Read" ONLY SEE $FFC0-$FFCF NOW, and a zero in that
	// column is normal rather than broken.  The read program's address filter
	// notifies the CPU for device registers only, so reads of the shared
	// buffer, the snippet area and the vectors - which are most of the traffic
	// - never reach the ISR at all.  On top of that the ISR handles the five
	// hottest device registers in branches of their own that do not set
	// read_irq_received, so this counts only the remainder.  Said here because
	// a column of zeros was read as a bug once before, in the console ACIA's
	// "Status" count, and went unexplained for weeks.
	//
	// Loop against ISR is the pair worth watching: it is the starvation ratio.
	// 12.7:1 before the filter, 1.95:1 after it.
	// No "Read IRQ" column any more: the single-slot path it counted was
	// deleted, its consumer having been a switch of nothing but `break`.
	printf("       Loop     Q Rise   IRQ Asrt   Wrt Ring        ISR\n");
#if DIAG_ON
	printf("  %9" PRIu32 "  %9" PRIu32 "  %9" PRIu32
	       "  %9" PRIu32 "  %9" PRIu32 "\n",
	       s->core1_loop, s->core1_qrise, s->core1_irq_assert,
	       s->core1_write_ring, s->core1_isr);
#else
	// A GATED-OFF COUNTER PRINTS "off", NEVER 0 (see the ACIA status
	// column): these three live in the loop and are read by nothing but
	// this table, so the production build does not keep them.
	printf("  %9" PRIu32 "        off        off        off  %9" PRIu32
	       "  (build with -D PICO_THING_DIAG=1)\n",
	       s->core1_loop, s->core1_isr);
#endif
#ifdef DEBUG
	// THE RING OVERFLOW, called out rather than tabulated.  Each one is a
	// guest write whose dispatch was dropped because the loop did not drain
	// the ring within its 32 entries - the loop's own deadline.  It was
	// counted and never reported until 2026-10-08, so the loss was silent.
	if (s->write_ring_overflow)
		printf("  *** WRITE RING OVERFLOW x%" PRIu32 " - guest write"
		       " dispatches were DROPPED; Core 1's loop missed its"
		       " deadline ***\n", s->write_ring_overflow);
	else
		printf("  Write ring overflow: none (%u entries of runway)\n",
		       WRITE_RING_SIZE);
	// COALESCED WRITE COMPLETIONS.  "worst 1" means every DMA completion got its
	// own interrupt; above 1 means some shared one, and each of those is a write
	// the old single-slot ISR would have dropped.  Measured 2 with one shared at
	// 2.4 MHz, so it is rare - the bulk of the loss was the read-after-overwrite
	// race on the data channel's write_addr, which this does not count.
#if DIAG_ON
	printf("  Write completions: worst %" PRIu32 " per interrupt, %" PRIu32
	       " shared one (%u entries of ring)\n",
	       s->write_addr_worst, s->write_addr_coalesced,
	       WRITE_ADDR_RING_SIZE);
	if (s->write_addr_overflow)
		printf("  *** WRITE ADDRESS RING LAPPED x%" PRIu32 " - make it"
		       " bigger; a write address may have been lost ***\n",
		       s->write_addr_overflow);
#else
	printf("  Write completions: off (build with -D PICO_THING_DIAG=1)\n");
#endif
#endif
#if DIAG_ON
	printf("  Last Write: %02X\n", s->core1_last_write);
#else
	printf("  Last Write: off (build with -D PICO_THING_DIAG=1)\n");
#endif
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
#if !DIAG_ON
	printf("    (Status column is OFF, not zero - build with"
	       " -D PICO_THING_DIAG=1)\n");
#endif
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
	printf(R"json(,"task":%u)json", s->task);
	printf(R"json(,"task_write_odd":{"count":%)json" PRIu32 R"json(,"from":%u,"to":%u})json",
	       s->task_write_odd_count, s->task_write_odd_from, s->task_write_odd_to);
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
	printf(R"json(,"temperature_boot_raw":%u,"temperature_boot_ms":%)json" PRIu32,
	       s->temperature_boot, s->temperature_boot_ms);
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
	printf(",\"lic_count\":%" PRIu32 ",\"lic_stalled\":%s",
	       s->lic_count, s->lic_stalled ? "true" : "false");
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
	printf(R"(,"core1_write_ring":%)" PRIu32, s->core1_write_ring);
	printf(R"(,"core1_last_write":%u,"core1_isr":%)" PRIu32,
	       s->core1_last_write, s->core1_isr);
	// The three loop counters and last_write above are 0 on a build that
	// does not keep them; this says which kind of 0 that is.
	printf(R"(,"core1_counters":%s)", DIAG_ON ? "true" : "false");
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

// `reset statistics`: every counter st prints goes to zero, nothing
// operational changes.  Core 0 stores zeros under Core 1's increments; a
// count lost in that race is a diagnostic, not state, so no masking.
static void
statistics_clear()
{
#if ISR_COST_ON
	isr_cost_reset();
#endif
	irq_vectors = 0u;
	pend_irq_cycles = 0u;
	write_ring_overflow = 0u;
	core1_isr_count = 0u;
#if DIAG_ON
	irq_loop_assert = 0u;
	irq_isr_deassert = 0u;
	irq_ack_deassert = 0u;
	q_armed_missed = 0u;
	nmi_work_missed = 0u;
	write_addr_ring_worst = 0u;
	write_addr_ring_coalesced = 0u;
	write_addr_ring_overflow = 0u;
	core1_qrise_count = 0u;
	core1_irq_assert_count = 0u;
	core1_write_ring_count = 0u;
	core1_last_write_loc = 0u;
#endif
	copy_buffer_disagreements = 0u;
	task_write_odd_count = 0u;
	task_write_odd_from = 0u;
	task_write_odd_to = 0u;
#if DIAG_ON
	task_log_count = 0u;
#endif
	fast_serial.stats_clear();
	aux_serial.stats_clear();
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
}

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
		"  reset [level]      (rs)  reset firmware state; levels weakest first,\n"
		"                           each including the ones before it:\n"
		"                             statistics  st's counters only (stats)\n"
		"                             peripherals registers, devices, tick timer\n"
		"                                         (the default)\n"
		"                             vectors     + every vector at the SYNC snippet\n"
		"                             run0        + task 0 and the identity DAT\n"
		"                                         (run also works; CPU stopped)\n"
		"                             core1       + Core 1's rings, interrupt state,\n"
		"                                         step gate, DAT-fault latch, read\n"
		"                                         SM and ACIA queues (CPU stopped)\n"
		"                             reboot      watchdog-reset the Pico\n"
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
		"  trace <vector>     (tr)  arm on swi|swi2|swi3|nmi|firq|irq|illegal\n"
		"                           (illegal = the HD6309 trap through $FFF0, trap also works)\n"
		"  trace ring [vector] (tr) keep the last %u cycles, frozen when the\n"
		"                           vector is taken, or by trace stop\n"
		"  trace stop         (tr)  freeze a running ring\n"
		"  break [addr|r]     (br)  breakpoint; r resumes\n"
		"  irq <0-9>          (ir)  interrupt tests; 7 is NMIBlock latency,\n"
		"                           8 is the DAT fault path, 9 single-steps\n"
		"  loop <type> <addr> [run]\n"
		"                     (lo)  hammer addr for scope work, interrupts\n"
		"                           masked.  The loop runs at $FFE0, or is\n"
		"                           copied to run and runs there.  The type is\n"
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
		"  nitros9 [ide|drivewire] [auto|6809|6309]  (ni)\n"
		"                           boot NitrOS-9; drivewire is also dw; 6809\n"
		"                           keeps an HD6309 in 6809 mode for a 6809 image\n"
		"  help               (h)   this text\n"
		"\nAn S-record pasted at the prompt is loaded directly.\n\n",
		static_cast<double>(GUEST_CLK_MIN), static_cast<double>(GUEST_CLK_MAX),
		TRACE_SIZE, TRACE_SIZE);
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

	case CMD_RESET_STATS:
		statistics_clear();
		printf("reset: statistics - every st counter zeroed, nothing else touched\n");
		break;

	case CMD_RESET:
		if (!MC6809.assert_powered())
			break;
		statistics_clear();
		peripheral_clear(false);
		printf("reset: peripherals - statistics, registers, devices, tick timer\n");
		break;

	case CMD_RESET_VECTORS:
		if (!MC6809.assert_powered())
			break;
		statistics_clear();
		peripheral_clear(true);
		printf("reset: vectors - peripherals, and every vector at the SYNC snippet\n");
		break;

	case CMD_RESET_RUN0:
		if (!MC6809.assert_stopped())
			break;
		statistics_clear();
		peripheral_clear(false);
		task_pins_update(MC6809.task_change(0u));
		snippet_copy(DAT_INIT);
		if (!MC6809.start_with_timeout(1000u)) {
			printf("reset: run0 - DAT_INIT failed, CPU did not respond\n");
			break;
		}
		printf("reset: run0 - peripherals, task 0, identity DAT\n");
		break;

	case CMD_RESET_CORE1: {
		if (!MC6809.assert_stopped())
			break;
		statistics_clear();
		peripheral_clear(false);
		// The trace is Core 0's to release (trace off does this already).
		MC6809.trace_clear();
		// Everything else is Core 1's: ask, and wait for the loop to act.
		core1_reset_request = true;
		loop_work = 1u;
		uint32_t waited = 0u;
		while (core1_reset_request && ++waited <= 100u)
			sleep_ms(1u);
		if (core1_reset_request) {
			printf("reset: core1 - Core 1 did not act within 100 ms; is it running?\n");
			break;
		}
		task_pins_update(MC6809.task_change(0u));
		snippet_copy(DAT_INIT);
		if (!MC6809.start_with_timeout(1000u)) {
			printf("reset: core1 - DAT_INIT failed, CPU did not respond\n");
			break;
		}
		printf("reset: core1 - run0, trace released, write path drained, interrupts"
		       " cleared, step and fault state idle, read chain re-primed, ACIA queues flushed\n");
		break;
	}

	case CMD_RESET_REBOOT:
		printf("reset: reboot - watchdog reset of the Pico now\n");
		sleep_ms(20u);			// let the line out of the UART
		watchdog_reboot(0u, 0u, 0u);
		for (;;)
			tight_loop_contents();

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
				console_write_pending = 1u;	// after the add: the loop clears before it drains
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

	case CMD_LOOP: {
		if (!MC6809.assert_powered())
			break;
		if (!cmd.loop.in_ram) {
			// LOOP_RW masks interrupts with ORCC, then repeats the
			// access at offset 2: opcode, then address.
			snippet_copy(LOOP_RW);
			reg[REGISTER_SNIPPET_OFFSET + 2u] = cmd.loop.opcode;
			reg[REGISTER_SNIPPET_OFFSET + 3u] = cmd.loop.address;
			MC6809.start();
			break;
		}
		// The same loop, laid down in guest memory and run from there, so
		// that its fetches go wherever that address maps rather than to
		// the Pico's window.
		const uint16_t run = cmd.loop.run_address;
		const uint8_t code[LOOP_CODE_LEN] = {
			0x1Au, 0x50u,				// ORCC #$50
			cmd.loop.opcode,			// the access...
			static_cast<uint8_t>(cmd.loop.address >> 8),
			static_cast<uint8_t>(cmd.loop.address),	// ...and its address
			0x20u, 0xFBu				// BRA back to the access
		};
		// copy() runs a snippet of its own, so it goes first and the
		// reset vector is pointed at the loop after it.
		if (!copy(OUTWARDS, LOOP_CODE_LEN, run, code)) {
			printf("Could not write the loop at %04X\n", run);
			break;
		}
		MC6809.stop();
		sleep_ms(1u);
		reg[REGISTER_VECTOR_RESET_OFFSET] = run;
		MC6809.start();
		break;
	}

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
		// LOOP_RW is ORCC, then five bytes of LDA extended and BRA, and it runs at
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
			0x1Au, 0x50u,		// ORCC #$50, as LOOP_RW does
			0xB6u, 0x20u, 0xD0u,	// LDA $20D0
			0x20u, 0xFBu		// BRA -5, back to $2002
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
					reg[REGISTER_SNIPPET_OFFSET + 2u] = static_cast<uint8_t>(0xB6u);	// LDA extended
					reg[REGISTER_SNIPPET_OFFSET + 3u] =
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
			// Single step, composed from parts that already work: the
			// countdown NMI re-enabled in f1b36b7, the BREAK snippet's
			// copy of the stacked frame, break_vector_fetched for "the
			// NMI was taken", and break_frame_settled() for "the copy
			// has finished".
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
				q_rising_arm();
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
			// Force the gate idle.  A pass that bailed out early - "no
			// NMI came back" - leaves step_state at RELEASED or
			// AFTER_RTI, and AFTER_RTI arms on the very next LIC edge,
			// so the guest collects a spurious NMI long after the test
			// has finished and parks in the BREAK snippet.  The symptom
			// is a later `break` reporting a PC inside $FFE0 rather than
			// the guest's own.
			step_state = STEP_IDLE;
			step_request = false;
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
			// And put back the DAT entries the cases marked unavailable,
			// as ramtest and synctest do.  Leaving them was how `irq 9`
			// came to read all-$FF frames: its stack at $DFFx is task 0
			// page 6, which this test leaves unmapped.
			snippet_copy(DAT_INIT);
			if (!MC6809.start_with_timeout(4u))
				printf("  restoring the identity DAT failed\n");
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
			printf(" ]\n");
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
#endif
		}
		break;
	}

	case CMD_TRACE:
	case CMD_TRACE_LEN: {
		// Decode the one-word-per-E-cycle capture via the bus_pins union.
		// A0-A5 is the low 6 bits of the bus address; for any $FFC0-page
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
		// row's E-falling.  b l v are what the Q-rising handler read for the
		// same row.  Both land after t_CD, so both carry the raw in-advance
		// values and the two columns should be IDENTICAL: measured 0 rows
		// differing in 512, against 446 differing from the previous row.
		//
		// ! is therefore not a phase check but a t_CD check - the only thing
		// that can separate the columns is the handler reading before the
		// 6809 has updated the pins, which would make b l v a cycle older.
		// Everything that acts on these bits assumes it does not (the step
		// gate, the read ISR's AVMA guard), so a ! column is the warning that
		// the assumption has broken - worth re-reading after any change to
		// this handler's cost, or at a higher E where t_CD is tighter.
		const bool ring = MC6809.trace_is_ring();
		if (ring && !MC6809.trace_ring_is_frozen()) {
			printf("trace: ring still running (%" PRIu32 " cycles so far) - "
			       "'trace stop' or the trigger vector freezes it\n\n", have);
			break;
		}
		// A ring has no mirror column: nothing ran per cycle to record it.
		const uint32_t blas = ring ? 0u : MC6809.trace_bla_count();
		uint32_t bla_differs = 0u, bla_differs_prev = 0u;
		printf("    #  EQ R/W  A  D  CS  bus-state    B L V  b l v !  "
		       "H N F I  TSK Pg  FM\n");
		for (uint32_t i = start; i < start + len; i++) {
			// Rows are numbered oldest first; a frozen ring maps them
			// onto the buffer from wherever its oldest cycle landed.
			const uint32_t r = MC6809.trace_row(i);
			bus_pins bp{}; bp.word = MC6809.trace_word(r);
			high_bus_pins hp{}; hp.word = MC6809.trace_hi_word(r);
			BLA bla{}; bla.byte = static_cast<uint8_t>(bp.bits.BUSY_LIC_AVMA);
			const bool have_bla = i < blas;
			BLA mir{}; mir.byte = have_bla ? MC6809.trace_bla_byte(i) : 0u;
			// The pins from the row before: what this row's handler read
			// should match.  Row 0 has no predecessor to check against.
			BLA prev{};
			bus_pins pbp{}; pbp.word = (i > 0u) ? MC6809.trace_word(MC6809.trace_row(i - 1u)) : 0u;
			prev.byte = static_cast<uint8_t>(pbp.bits.BUSY_LIC_AVMA);
			// Counted BOTH ways, because which row the handler's read
			// belongs to is a measurement, not an argument: against this
			// row's capture (same instant, if the read lands after t_CD)
			// and against the previous row's (one cycle older, if it
			// lands before).  The footer prints both.
			const bool differs = have_bla && mir.byte != bla.byte;
			if (differs)
				bla_differs++;
			if (have_bla && i > 0u && mir.byte != prev.byte)
				bla_differs_prev++;
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
			       have_bla ? (mir.bit.BUSY ? 'b' : '.') : '?',
			       have_bla ? (mir.bit.LIC  ? 'l' : '.') : '?',
			       have_bla ? (mir.bit.AVMA ? 'v' : '.') : '?',
			       differs ? '!' : '.',
			       row_halt(bp, hp) ? '.' : 'H',	// active-low outputs: letter = asserted
			       row_nmi(bp, hp)  ? '.' : 'N',
			       row_firq(bp, hp) ? '.' : 'F',
			       row_irq(bp, hp)  ? '.' : 'I',
			       static_cast<unsigned>(hp.bits.TASK),
			       row_task0(bp, hp) ? '0' : ' ',	// Root: task-zero flag to the DAT
			       row_a_hi(bp, hp),		// A15-A13, the DAT page
			       row_fault(bp, hp) ? 'F' : '.',
			       hp.bits.NMI_BLOCK ? 'M' : '.');
		}
		if (ring)
			printf("%" PRIu32 " of %" PRIu32 " cycles, oldest first; ring frozen%s\n",
			       len, have, have >= TRACE_SIZE ? " after wrapping" : " before wrapping");
		else
			printf("%" PRIu32 " of %" PRIu32 " cycles%s\n", len, have,
			       MC6809.trace_is_full() ? " (buffer full)" : "");
		// The mirror column is written by the Q handler from trace_arm(),
		// while the DMA columns start when the trigger fires.  With 'trace
		// on' those are the same moment; with a vector trigger the handler
		// starts first, so expect a constant row offset and read the columns
		// for agreement of values rather than row-for-row.
		printf("handler reads: %" PRIu32 " rows recorded; of %" PRIu32
		       " shown, %" PRIu32 " differ from THIS row's capture and %"
		       PRIu32 " from the PREVIOUS row's\n\n",
		       blas, len, bla_differs, bla_differs_prev);
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

	case CMD_TRACE_RING: {
		static constexpr const char *const rnames[NUM_INTERRUPTS] = {
			"ILLEGAL", "SWI3", "SWI2", "FIRQ", "IRQ", "SWI", "NMI", "RESET", "NONE"
		};
		const auto it = static_cast<interrupt>(cmd.trace.intr);
		const uint8_t mask = (it < INTERRUPT_NONE) ? static_cast<uint8_t>(1u << it) : 0u;
		MC6809.trace_ring_arm(mask);
		if (mask)
			printf("trace ring armed: keeping the last %u cycles, frozen when the %s vector is taken (or by 'trace stop')\n\n",
			       TRACE_SIZE, rnames[it]);
		else
			printf("trace ring armed: keeping the last %u cycles until 'trace stop'\n\n", TRACE_SIZE);
		break;
	}

	case CMD_TRACE_FREEZE:
		if (!MC6809.trace_is_ring() || !MC6809.trace_is_armed()) {
			printf("trace: no ring is running - arm one with 'trace ring [<irq>]'\n\n");
			break;
		}
		if (MC6809.trace_ring_is_frozen()) {
			printf("trace: ring already frozen\n\n");
			break;
		}
		MC6809.trace_ring_freeze();
		printf("trace ring frozen: %" PRIu32 " cycles, oldest first\n\n", MC6809.trace_count());
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

	case CMD_FREQ_QUERY: {
		// What the clock program is actually dividing by: the PIO divider
		// keeps 8 fractional bits, so the delivered E differs slightly from
		// the requested one at most settings.
		const float requested = MC6809.get_e_frequency();
		const float sys_hz = static_cast<float>(clock_get_hz(clk_sys));
		const float div = sys_hz / (4.0f * requested * 1.0e6f);
		const float div_pio = roundf(div * 256.0f) / 256.0f;
		const float actual = sys_hz / (4.0f * div_pio) / 1.0e6f;
		printf("E/Q clock: %.1f MHz requested, %.4f MHz delivered"
		       " (clk_sys %u kHz / 4 / %.4f)\n",
		       requested, actual, clock_get_hz(clk_sys) / 1000u, div_pio);
		break;
	}

	case CMD_FREQ:
		if (!MC6809.assert_powered())
			break;
		MC6809.set_e_frequency(cmd.freq.mhz, pio_clock);
		printf("Set E to %.1f MHz\n", MC6809.get_e_frequency());
#if ISR_COST_ON
		// The budget just changed, so the old samples describe a
		// different question.
		isr_cost_reset();
		// Start-of-measurement: the stall flags are sticky and latch per
		// SM, so clearing them here means an RXSTALL seen afterwards was
		// earned at THIS clock and not inherited from a faster one.
		piodma_write.pio->fdebug = PIO_FDEBUG_RXSTALL_BITS
					 | PIO_FDEBUG_TXSTALL_BITS
					 | PIO_FDEBUG_RXUNDER_BITS
					 | PIO_FDEBUG_TXOVER_BITS;
#endif
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
		/* Set CPU-mode byte at $0136 (0 = auto, 3 = 6309 native, 8 = 6809) */
		uint8_t cpu_mode = cmd.nitros9.cpu_mode;
		if (!copy(OUTWARDS, 1u, 0x0136u, &cpu_mode))
			break;
		reg[REGISTER_VECTOR_RESET_OFFSET] = CHUNK_ADDRESS;
		printf("Booting NitrOS-9 (%s, %s)\n", bdev ? "DriveWire" : "IDE",
		       cpu_mode == 8u ? "6809 mode" : cpu_mode == 3u ? "6309 native" : "CPU auto");
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
#if DIAG_ON
					printf("  last_write=%02X\n", core1_last_write_loc);
#endif
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

// Hand the int_pins state machine a new mask, but only when it differs from the
// one it already holds.  That "only" is the whole point of moving the pins into
// the PIO: the machine re-applies its last value at every Q rising by itself, so
// a pending set that has not changed - which is almost every pass, since
// devices change on human and 50 Hz timescales - costs nothing at all.
//
// PEND_NMI, PEND_FIRQ and PEND_IRQ are bits 0, 1 and 2, and the program's
// "out pindirs 3" maps bit 0 to !NMI, bit 1 to !FIRQ and bit 2 to !IRQ.  So the
// level lines need no translation whatsoever - the byte IS the word - and only
// NMI has to be substituted, because it is a one-shot held for a few passes
// rather than a level.
static inline void
interrupt_pins_update()
{
	const uint8_t pend = pending_interrupts;
	// A request from any of the three sources starts the hold.  Taken here
	// rather than in the Q handler, which no longer looks at them; the cost is
	// that a request waits for Core 1's next pass instead of the next
	// Q-rising, a couple of E cycles later.
	const bool isr_nmi = nmi_request_count != nmi_request_done;
	if (isr_nmi || (pend & PEND_NMI) || watchpoint_nmi_pending
	    || dat_fault_nmi_request) {
		nmi_request_done = nmi_request_count;
		if (watchpoint_nmi_pending) {
			reg[REGISTER_BUFFER_OFFSET] = static_cast<uint8_t>(0u);
			reg[REGISTER_BUFFER_OFFSET + 10] = static_cast<uint8_t>(0u);
			reg[REGISTER_BUFFER_OFFSET + 11] = static_cast<uint8_t>(0u);
			watchpoint_fired = true;
		}
		nmi_hold_passes = NMI_HOLD_PASSES;
		pending_interrupts = pend & ~PEND_NMI;
		watchpoint_nmi_pending = false;
		dat_fault_nmi_request = false;
	}
	uint32_t oe = pend & (PEND_FIRQ | PEND_IRQ);
	if (nmi_hold_passes > 0u) {
		oe |= PEND_NMI;
		nmi_hold_passes--;
	}
	if (oe != int_pins_held) {
		int_pins_held = oe;
		pio_sm_put(int_pins.pio, static_cast<uint>(int_pins.sm), oe);
	}
}

// `reset core1`, performed ON CORE 1 at the loop's request point: as much of
// Core 1's state as can be cleared without a reboot, with the guest stopped
// (the handler insists) so no bus cycle is in flight.  Each item mirrors
// what power-off/power-on or the start-up code does to the same thing; what
// is NOT here needs a reboot (`reset reboot`).
static void
core1_reset_perform()
{
	const uint32_t saved = save_and_disable_interrupts();

	// The write path: drop every queued follow-up and resync the drain
	// pointers to the DMA heads, so nothing stale is applied later.
	write_ring_tail = write_ring_head;
	write_addr_ring_tail = write_addr_ring_head();

	// Interrupt state: nothing pending, nothing retired, pins released.
	// pending_interrupts is cleared BEFORE the pin update so the update
	// sees a change against int_pins_held and drives the deassert.
	pending_interrupts = 0u;
	irq_sources = 0u;
	irq_retired = 0u;
	nmi_request_done = nmi_request_count;
	nmi_hold_passes = 0u;
	loop_nmi_work = 0u;
	watchpoint_nmi_pending = false;
	dat_fault_nmi_request = false;
	interrupt_pins_update();

	// The step machinery, as the step test's own bail-out leaves it.
	step_state = STEP_IDLE;
	step_request = false;
	step_active = false;
	step_logging = false;
	step_arm_cycles = 0u;

	// DAT fault: the mask down, the registers clear, the sampler re-armed
	// (the same sequence as the per-run setup callback).
	dat_fault_block_countdown = 0u;
	dat_fault_nmi_pending = false;
	gpio_put(GPIO_NMI_BLOCK, false);
	reg[SYSTEM_DAT_FAULT] = static_cast<uint8_t>(0u);
	reg[SYSTEM_DAT_FAULT_LOC] = static_cast<uint8_t>(0u);
	if (bus_fault.sm >= 0) {
		pio_sm_clear_fifos(bus_fault.pio, static_cast<uint>(bus_fault.sm));
		pio_interrupt_clear(bus_fault.pio, BUS_FAULT_PIO_IRQ);
		pio_set_irq0_source_enabled(bus_fault.pio, BUS_FAULT_PIO_IRQ_SOURCE, true);
	}

	// The sticky PIO FIFO flags on both bus state machines.
	piodma_read.pio->fdebug = PIO_FDEBUG_RXSTALL_BITS | PIO_FDEBUG_TXSTALL_BITS
				| PIO_FDEBUG_RXUNDER_BITS | PIO_FDEBUG_TXOVER_BITS;
	piodma_write.pio->fdebug = PIO_FDEBUG_RXSTALL_BITS | PIO_FDEBUG_TXSTALL_BITS
				 | PIO_FDEBUG_RXUNDER_BITS | PIO_FDEBUG_TXOVER_BITS;

	// The read chain, re-primed from scratch: the SM back at its first
	// instruction (the pull of the register-array base), both DMA channels
	// aborted and the address channel re-armed, exactly as at start-up.  A
	// chain left half way - an address pushed and never answered - would
	// otherwise serve the next read with a stale byte.
	dma_channel_abort(piodma_read.address_channel);
	dma_channel_abort(piodma_read.data_channel);
	pio_sm_set_enabled(piodma_read.pio, piodma_read.sm, false);
	pio_sm_clear_fifos(piodma_read.pio, piodma_read.sm);
	pio_sm_restart(piodma_read.pio, piodma_read.sm);
	pio_sm_exec(piodma_read.pio, piodma_read.sm,
		    pio_encode_jmp(static_cast<uint>(piodma_read.offset)));
	pio_interrupt_clear(piodma_read.pio, READ_DEVICE_PIO_IRQ);
	dma_channel_start(piodma_read.address_channel);
	pio_sm_put(piodma_read.pio, piodma_read.sm, registers::read_address() >> 6);
	pio_sm_set_enabled(piodma_read.pio, piodma_read.sm, true);

	// The ACIA queues: unread input and untaken output dropped, which
	// only Core 1 may do (see mc6850::reset()).
	fast_serial.flush_queues_from_core1();
	aux_serial.flush_queues_from_core1();

	core1_reset_request = false;
	restore_interrupts(saved);
}

static void
__no_inline_not_in_flash_func(main_core1)()
{
	printf("Core1 process starting\n");

	// Trace pins - ISR set.  Named individually rather than as a run: they
	// sit on 37-38 on version 1 but 38-39 on version 2, where 37 is TASK0.
#if TESTPOINTS_ON || TRAP_TRIGGER_ON
	static constexpr uint8_t trace_pins[] = { GPIO_TRACE_C, GPIO_TRACE_D };
	for (const uint8_t p : trace_pins) {
		gpio_init(p);
		gpio_put(p, false);
		gpio_set_dir(p, GPIO_OUT);
	}
#endif

#if ISR_COST_ON
	isr_cost_init();		// SysTick is per-core, so it must be here
#endif
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
		// EXCLUSIVE, NOT SHARED.  gpio_add_raw_irq_handler() installs a
		// shared handler, which the SDK dispatches through its
		// irq_handler_chain trampoline: slot -> first_slot (push, load,
		// bx) -> handler -> slot's pop {pc}.  About a dozen cycles on
		// EVERY Q-rising, to arbitrate between handlers of which there is
		// exactly one - nothing else on core 1 uses IO_IRQ_BANK0, and no
		// gpio callback is installed.  An exclusive handler goes straight
		// into the RAM vector table.  gpio_set_irq_enabled() below is
		// independent of how the handler was installed.
		irq_set_exclusive_handler(IO_IRQ_BANK0, gpio_q_irq_handler);
		q_handler_added = true;
	}
	gpio_set_irq_enabled(GPIO_Q, GPIO_IRQ_EDGE_RISE, true);
	irq_set_priority(pio_clock.irq, 1);
	irq_set_enabled(pio_clock.irq, true);
	printf("Core1 BA/BS and BUSY/LIC/AVMA bus observer started\n");

	irq_set_exclusive_handler(piodma_read.irq, dma_bus_read_irq_handler);
	irq_set_priority(piodma_read.irq, 0);
	irq_set_enabled(piodma_read.irq, true);
	// The read program's address filter is the interrupt source now, NOT the
	// DMA completing - see READ_DEVICE_PIO_IRQ.  Cleared before enabling for
	// the same reason the fault sampler's tick is: the state machine may
	// already have raised it.
	pio_interrupt_clear(piodma_read.pio, READ_DEVICE_PIO_IRQ);
	pio_set_irq0_source_enabled(piodma_read.pio, READ_DEVICE_PIO_IRQ_SOURCE, true);
	dma_channel_start(piodma_read.address_channel);
	pio_sm_put(piodma_read.pio, piodma_read.sm, registers::read_address() >> 6); // Six bits for A0-5
	printf("Core1 Main read DMA chain configured\n");

	irq_set_exclusive_handler(piodma_write.irq, dma_bus_write_irq_handler);
	irq_set_priority(piodma_write.irq, 0);
	irq_set_enabled(piodma_write.irq, true);
	// RESYNCHRONISE THE TAIL TO THE HARDWARE before enabling the interrupt.
	// Core 1 is reset and relaunched on every power-on, but the ring channel
	// is configured once by Core 0 and write_addr_ring_tail is a file-scope
	// static that survives - so after a relaunch the two can disagree and the
	// first interrupt would drain up to 63 stale entries as though they were
	// new writes.  Taking the head as the starting tail discards whatever the
	// previous run left, which is what is wanted.  This is the same trap that
	// put a bogus 8.7-million-cycle maximum in loop_cost.max.
	write_addr_ring_tail = write_addr_ring_head();
	// Same reason as the tail above: these latch and survive a relaunch, so a
	// stall from a previous run would be read as one from this one.
	piodma_write.pio->fdebug = PIO_FDEBUG_RXSTALL_BITS
				 | PIO_FDEBUG_TXSTALL_BITS
				 | PIO_FDEBUG_RXUNDER_BITS
				 | PIO_FDEBUG_TXOVER_BITS;
	// THE INTERRUPT COMES FROM THE RING CHANNEL, the last link in the chain,
	// so by the time the handler runs the entry for this write is already in
	// the ring.  On the data channel it could run first and find nothing.
	dma_channel_set_irq1_enabled(WRITE_ADDR_RING_CHANNEL, true);
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

	// Clear the Q-rising tick before starting, for the same reason the fault
	// sampler's is cleared: the clock has been free-running since Core 1 came
	// up, so flag 5 is already standing set and a wait satisfied by that stale
	// flag would apply the pins half a cycle out of phase.
	pio_interrupt_clear(int_pins.pio, INT_PINS_TICK_PIO_IRQ);
	pio_sm_set_enabled(int_pins.pio, int_pins.sm, true);
	printf("Core1 interrupt pins pio%d/sm%d started\n",
	       PIO_NUM(int_pins.pio), int_pins.sm);


	// CLEAR THE CLOCK'S TICK BEFORE STARTING, so the first wait blocks until a
	// REAL Q-falling.  The clock SM was enabled further up and has been
	// free-running ever since, so flag 0 is already standing set; a wait
	// satisfied by that stale flag samples GPIO_FAULT at an arbitrary phase
	// instead of in the settled window Q-falling gives, and can read a fault
	// that is not there.  Measured 2026-10-07: one spurious fault latched into
	// $FFCC on every power-on, with the mask then raised against a fault that
	// never happened.  The identical hazard is documented for flag 1 on the
	// clock program itself - "the first wait is satisfied immediately".
	//
	// The FIFO goes too: a spurious sample will have pushed an address into it,
	// and nothing drains it until a real fault is reported.
	pio_interrupt_clear(bus_fault.pio, CLOCK_TICK_PIO_IRQ);
	pio_sm_clear_fifos(bus_fault.pio, static_cast<uint>(bus_fault.sm));
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

#if ISR_COST_ON
	// INVALIDATE THE LOOP TIMER'S PREVIOUS STAMP.  Core 1 is reset and
	// relaunched on every "power on", and loop_prev_valid is a file-scope
	// static that survives that - so the first pass afterwards measured
	// against a stamp from the previous run and planted one enormous bogus
	// delta in loop_cost.max, which never leaves because max is monotonic.
	// That is where the 8.7-million-cycle "worst case" came from, and it
	// hid the number Core 1's own deadline is judged against.
	// isr_cost_reset() clears this too, but it is only called from CMD_FREQ.
	loop_prev_valid = false;
#endif
	core1_initialised = true;

	for (;;) {
#if ISR_COST_ON
		{
			const uint32_t now = systick_hw->cvr;
			const uint32_t isr_now = isr_cycles_total;
			if (loop_prev_valid) {
				const uint32_t wall =
					(loop_prev_stamp - now) & SYSTICK_MASK;
				const uint32_t isr = isr_now - loop_prev_isr;
				isr_cost_note(loop_cost,
					      (wall > isr) ? wall - isr : 0u);
				isr_cost_note(loop_period, wall);
			}
			loop_prev_stamp = now;
			loop_prev_isr = isr_now;
			loop_prev_valid = true;
		}
#endif
		core1_loop_count++;
		// Interrupts need to be done before the falling edge of Q
		if (q_rising_edge) {
			q_rising_edge = false;
#if DIAG_ON
			core1_qrise_count++;
#endif

			// Ask each device what it wants and keep the three bits the
			// pins can carry.  This used to count into a
			// uint32_t[NUM_INTERRUPTS] array, zeroed and indexed on every
			// Q-rising, because the array was handed to
			// mc6809::apply_interrupts() - which no longer exists.  The
			// counts were never read as counts.
			//
			// Single volatile write at the end: the Q-rising ISR drives all
			// three pins from this byte.
			// Four separate results so the per-source mask can be
			// recorded.  Each still called EXACTLY ONCE per pass:
			// console_has_interrupt() consumes an NMI request on read and
			// the two ACIA ones stage the next byte.  Outside the masked
			// section below, because they are the expensive part.
			const interrupt con = console_has_interrupt();
			const interrupt tck = system_tick.has_interrupt();
			const interrupt fst = fast_serial.has_interrupt();
			const interrupt axs = aux_serial.has_interrupt();
			const uint8_t pend = pend_bit(con) | pend_bit(tck)
					   | pend_bit(fst) | pend_bit(axs);
			const uint8_t src = static_cast<uint8_t>(
				  (con == INTERRUPT_IRQ ? IRQ_SRC_CONSOLE_CMD  : 0u)
				| (tck == INTERRUPT_IRQ ? IRQ_SRC_TICK         : 0u)
				| (fst == INTERRUPT_IRQ ? IRQ_SRC_CONSOLE_ACIA : 0u)
				| (axs == INTERRUPT_IRQ ? IRQ_SRC_AUX_ACIA     : 0u));
#if DIAG_ON
			if (pend & PEND_IRQ)
				core1_irq_assert_count++;
#endif
			// ONLY WHEN SOMETHING WOULD CHANGE.  The masked block below
			// ran on every pass - about 45 clk_sys cycles with interrupts
			// off, every pass, which at 2.5 MHz is a third of an E cycle
			// during which no DMA completion can be serviced, for an
			// update that almost never changed anything.  Measured as part
			// of the loop's 405-cycle pass in the Core 1 audit.
			//
			// THE WRONG GATE, and why: "pend unchanged since last pass".
			// The vector-fetch release in the Q handler drops /IRQ while
			// the ACIA still wants service, and it is THIS block on the
			// next pass that puts it back up - pend is unchanged, the pin
			// is not.  So the comparison is against the PIN, int_pins_held,
			// not against last time.  The NMI inputs are listed because
			// interrupt_pins_update() consumes them, and nmi_hold_passes
			// because it is a countdown that block decrements - skipping
			// it would hold /NMI low for ever.  int_pins_held and the NMI
			// flags are read unmasked: a race means the block runs one
			// pass late or one pass extra, both harmless.
			//
			// The four polls above stay OUTSIDE this gate on purpose:
			// console_has_interrupt() consumes a request on read, and the
			// two ACIA polls stage the next byte.
			//
			// NMI IS IN THE PIN COMPARISON, AND IT HAS TO BE.  The first
			// cut compared only FIRQ and IRQ, reasoning that the NMI
			// inputs were all named in nmi_work.  They were - but the
			// RELEASE is not an input.  interrupt_pins_update() holds the
			// pin for nmi_hold_passes passes and lets go on the pass
			// AFTER the count reaches zero, when it computes an oe
			// without NMI and finds it differs from the pin.  On that
			// pass nothing in nmi_work is true any more, so the block was
			// skipped and /NMI stayed low for ever.  irq 2 and irq 5
			// passed, because they test the EDGE; irq 8 failed, because
			// a second DAT fault needs a second edge on a line that was
			// never released.  With NMI in the mask, held-but-not-wanted
			// is a difference, the block runs once more, and releases.
			//
			// ONE STRAIGHT-LINE OR, NOT A SHORT-CIRCUIT CHAIN.  On the pass
			// that has nothing to do - almost all of them - every term is
			// false, so || evaluated every load anyway; what it added was
			// a branch per term, and the compiler, short of registers in
			// those arms, re-materialised four base addresses with
			// movw/movt in each of them: ~20 instructions a pass for no
			// load saved (2026-10-10 loop audit).  The loads are the same;
			// the arms are gone.
			const uint8_t held = static_cast<uint8_t>(int_pins_held);
			const uint8_t pins_differ = static_cast<uint8_t>(
				(pend ^ held) & (PEND_NMI | PEND_FIRQ | PEND_IRQ));
			// The NMI work is one byte now (loop_nmi_work, see its
			// declaration); a console NMI request arrives in pend and is
			// caught by pins_differ, whose mask includes PEND_NMI.
			if ((pins_differ | loop_nmi_work | static_cast<uint8_t>(src ^ irq_sources)) != 0u) {
				// Masked for the update only.  irq_retired is cleared
				// because the has_interrupt() calls above have just read
				// the devices, so their answer supersedes any earlier
				// retirement; one that lands straight after the clear
				// applies to the next pass.
				const uint32_t irq_saved = save_and_disable_interrupts();
				const bool was_irq = (int_pins_held & PEND_IRQ) != 0u;
				pending_interrupts = pend;
				irq_sources = src;
				irq_retired = 0u;
				interrupt_pins_update();
				// Re-derive the NMI-work byte from its sources, with
				// interrupts still off so no writer can interleave: this
				// is where it is cleared, and where it stays set while
				// hold passes remain.
				loop_nmi_work = static_cast<uint8_t>(
					  static_cast<uint8_t>(nmi_request_count != nmi_request_done)
					| static_cast<uint8_t>(watchpoint_nmi_pending)
					| static_cast<uint8_t>(dat_fault_nmi_request)
					| nmi_hold_passes);
#if DIAG_ON
				if (!was_irq && (int_pins_held & PEND_IRQ))
					irq_loop_assert++;
#endif
				restore_interrupts(irq_saved);
			}
#if DIAG_ON
			// The same audit for the NMI-work byte as for the armed one:
			// a live source with the byte clear for two consecutive
			// passes is a writer that forgot to set it.
			if (loop_nmi_work == 0u
			    && (nmi_request_count != nmi_request_done || watchpoint_nmi_pending
				|| dat_fault_nmi_request || nmi_hold_passes != 0u)) {
				if (++nmi_work_miss_run == 2u)
					nmi_work_missed++;
			} else
				nmi_work_miss_run = 0u;
#endif
#if DIAG_ON
			// AUDIT OF THE ARMED BYTE against its sources: a live source
			// with the byte clear is a writer that forgot q_rising_arm(),
			// which otherwise fails silently - until the next bus-state
			// change re-derives the byte, so it is a delay, not a loss,
			// but still a bug.  A writer IN FLIGHT looks the same for one
			// pass (source stored, byte not yet - "irq 9" showed exactly
			// one), so only a miss that persists across two consecutive
			// passes is counted.  Counted, not repaired, so the count
			// means something.
			if (q_rising_armed == 0u && q_rising_sources()) {
				if (++q_armed_miss_run == 2u)
					q_armed_missed++;
			} else
				q_armed_miss_run = 0u;
#endif
		}

		// Console writes into the register block, carried out as the guest's
		// would be: the byte into the write side, then the write ISR's own
		// follow-up, with interrupts off so the ISR cannot interleave.  The
		// ring entries that queues are dispatched just below, in this pass.
		// queue_try_remove() takes the queue's spinlock before it looks at
		// the level, so an empty queue - which is almost every pass - costs
		// a lock acquire, a release and their barriers.  So the pass tests
		// console_write_pending instead, one load: Core 0 sets it after a
		// successful add, and this clears it before draining, so an add on
		// either side of the clear is drained now or flagged for the next
		// pass.  It replaced an unsafe level read of two loads and the
		// arithmetic behind it (2026-10-10 loop audit).
		if (console_write_pending != 0u) {
			console_write_pending = 0u;	// BEFORE the drain
			write_entry w;
			while (queue_try_remove(&console_write_queue, &w)) {
				const uint32_t saved = save_and_disable_interrupts();
				registers::write(w.location) = w.value;
				bus_write_followup(w.location, w.value);
				restore_interrupts(saved);
				console_writes_done++;
			}
		}

		if (loop_work == 0u)
			continue;
		loop_work = 0u;				// BEFORE the drains
		if (core1_reset_request)
			core1_reset_perform();
		while (write_ring_tail != write_ring_head) {
			const uint8_t t = write_ring_tail;
			const uint8_t written_location = write_ring[t].location;
			const uint8_t written_byte = write_ring[t].value;
#if DIAG_ON
			core1_write_ring_count++;
			core1_last_write_loc = written_location;
#endif
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
				// Nothing to do: the request handler reads it (REQUEST_FLOAT)
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
	// Chains to the ADDRESS RING, not straight back to the address channel:
	// the ring entry has to be written after the byte has landed, so that a
	// drainer reading write_registers[] for that address sees the new value.
	// The ring then chains back to the address channel and the cycle closes.
	channel_config_set_chain_to(&write_in_dma, WRITE_ADDR_RING_CHANNEL);
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
	// DMA record every write's address into a wrapping ring, so a coalesced
	// completion cannot lose one.  See the long note on write_addr_ring.
	//
	// No DREQ: it is triggered by the chain from the data channel and has
	// nothing to wait for, so it runs unpaced and is done in a few cycles.
	// Fixed source - the data channel's write_addr register - and an
	// incrementing destination with a hardware wrap, which is what makes the
	// channel self-reloading and never in need of reconfiguration.
	if (dma_channel_is_claimed(WRITE_ADDR_RING_CHANNEL)) {
		printf("Core0 DMA channel %d for the write address ring is already claimed\n",
		       WRITE_ADDR_RING_CHANNEL);
		panic("Unable to proceed");
	}
	dma_channel_claim(WRITE_ADDR_RING_CHANNEL);
	dma_channel_config write_ring_dma = dma_channel_get_default_config(WRITE_ADDR_RING_CHANNEL);
	channel_config_set_high_priority(&write_ring_dma, true);
	channel_config_set_read_increment(&write_ring_dma, false);
	channel_config_set_write_increment(&write_ring_dma, true);
	channel_config_set_ring(&write_ring_dma, true, WRITE_ADDR_RING_BITS);
	channel_config_set_transfer_data_size(&write_ring_dma, DMA_SIZE_32);
	channel_config_set_chain_to(&write_ring_dma, WRITE_DATA_RING_CHANNEL);
	dma_channel_configure(
		WRITE_ADDR_RING_CHANNEL,
		&write_ring_dma,
		write_addr_ring,						// dst - wraps
		&dma_channel_hw_addr(piodma_write.data_channel)->write_addr,	// src - fixed
		dma_encode_transfer_count(1),
		false);
	printf("Core0 DMA write address ring channel configured\n");
	// And the data ring: the PIO's SECOND push of the same data byte, taken
	// straight from the FIFO so it cannot have been overwritten by a later
	// write.  DREQ-paced like the other two FIFO readers, because it has to
	// wait for that push; the address ring above needs no DREQ, being a
	// register-to-memory copy.  Last link in the chain, so it closes the cycle
	// back to the address channel.
	if (dma_channel_is_claimed(WRITE_DATA_RING_CHANNEL)) {
		printf("Core0 DMA channel %d for the write data ring is already claimed\n",
		       WRITE_DATA_RING_CHANNEL);
		panic("Unable to proceed");
	}
	dma_channel_claim(WRITE_DATA_RING_CHANNEL);
	dma_channel_config write_data_dma = dma_channel_get_default_config(WRITE_DATA_RING_CHANNEL);
	channel_config_set_high_priority(&write_data_dma, true);
	channel_config_set_dreq(&write_data_dma, pio_get_dreq(piodma_write.pio, piodma_write.sm, false));
	channel_config_set_read_increment(&write_data_dma, false);
	channel_config_set_write_increment(&write_data_dma, true);
	channel_config_set_ring(&write_data_dma, true, WRITE_DATA_RING_BITS);
	channel_config_set_transfer_data_size(&write_data_dma, DMA_SIZE_8);
	channel_config_set_chain_to(&write_data_dma, piodma_write.address_channel);
	dma_channel_configure(
		WRITE_DATA_RING_CHANNEL,
		&write_data_dma,
		write_data_ring,					// dst - wraps
		&piodma_write.pio->rxf[piodma_write.sm],	// src - the second push
		dma_encode_transfer_count(1),
		false);
	printf("Core0 DMA write data ring channel configured\n");

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

	// SM: interrupt pin driver, on the same base-16 block so it can reach
	// GPIO 24-26.  MUST come after MC6809.init(), which does gpio_init() on
	// these pins and would take them back from the PIO - set_power() calls
	// them in that order.
	int_pins.offset = pio_add_program(int_pins.pio, &int_pins_program);
	if (int_pins.offset < 0) {
		printf("Core0 insufficient space for int_pins program\n");
		panic("Unable to proceed");
	}
	int_pins.sm = pio_claim_unused_sm(int_pins.pio, true);
	int_pins_program_init(int_pins.pio, int_pins.sm, int_pins.offset, GPIO_NMI);
	// Nothing asserted, and say so, so the first real change is always sent.
	int_pins_held = 0u;
	nmi_hold_passes = 0u;
	printf("Core0 interrupt pin driver pio%u sm%d configured\n",
	       PIO_NUM(int_pins.pio), int_pins.sm);

	// SM pair: bus trace capture.  pio1 -> pio2 via "irq next set 2".
	//
	// NOTHING IS CLAIMED HERE ANY MORE - see trace_hw_acquire().  The programs
	// and state machines are taken when a capture is armed and given back when
	// it stops, so an idle board leaves a state machine and 5 instruction
	// words free on pio1, and a state machine and 2 words on pio2.  Only the two DMA channels are permanent, because they are
	// not the scarce resource and because the sample count lives in one of
	// them.
	//
	// bus_fault below STILL claims its pio2 state machine at boot, and that
	// ordering is deliberate: it always wins, and the trace takes what is left
	// when someone asks for a capture.  A diagnostic that cannot arm is an
	// inconvenience; a DAT fault sampler that cannot start is not.
	// DAT fault sampler, on pio2 beside the trace's high half.  The SM is
	// claimed rather than chosen: pio2 already runs task_output and int_pins,
	// and initialising one of those by hand would silently restart it with
	// this program's configuration.
	bus_fault.offset = pio_add_program(bus_fault.pio, &bus_fault_program);
	if (bus_fault.offset < 0) {
		printf("Core0 insufficient space for bus_fault program\n");
		panic("Unable to proceed");
	}
	bus_fault.sm = pio_claim_unused_sm(bus_fault.pio, true);
	bus_fault_program_init(bus_fault.pio, bus_fault.sm,
			       static_cast<uint>(bus_fault.offset), GPIO_FAULT);

	// The channels are permanent; their CONFIGURATION is not, because the DREQ
	// and the source address both depend on which state machine the trace
	// happened to claim.  trace_hw_acquire() configures them each time.
	dma_channel_claim(trace_lo.data_channel);
	dma_channel_claim(trace_hi.data_channel);

	trace_hw_usable = true;
	trace_hw_held = false;
	trace_hw_samples = 0u;
	MC6809.set_trace_callbacks(trace_hw_arm, trace_hw_stop, trace_hw_count, trace_hw_freeze);
	printf("Core0 bus trace DMA %d/%d; pio%u+pio%u programs and state"
	       " machines claimed only while a capture is armed\n",
	       trace_lo.data_channel, trace_hi.data_channel,
	       PIO_NUM(trace_lo.pio), PIO_NUM(trace_hi.pio));
}

static void
pio_shutdown()
{
	// --- Disable IRQs first (handlers reference DMA/PIO resources) ---

	// GPIO Q edge IRQ (bus observer): nothing to do here.  Its enable, its
	// NVIC enable and its handler registration are all per-core, and all three
	// belong to Core 1, which has already been reset - so the IRQ cannot be
	// taken.  The handler stays registered for the next launch; see main_core1.

	// Read notification IRQ.  The source is the read program's address filter
	// on pio0, not the DMA completing - so this disables the PIO source, and
	// clears any flag the state machine raised on its way down, which would
	// otherwise be waiting to fire the instant the handler is re-registered.
	pio_set_irq0_source_enabled(piodma_read.pio, READ_DEVICE_PIO_IRQ_SOURCE, false);
	pio_interrupt_clear(piodma_read.pio, READ_DEVICE_PIO_IRQ);
	irq_set_enabled(piodma_read.irq, false);
	irq_remove_handler(piodma_read.irq, dma_bus_read_irq_handler);

	// DMA write IRQ.  Disabled on the RING channel, which is where it is now
	// raised - leaving it enabled on the data channel would arm an interrupt
	// the handler is no longer registered for.
	dma_channel_set_irq1_enabled(WRITE_ADDR_RING_CHANNEL, false);
	irq_set_enabled(piodma_write.irq, false);
	irq_remove_handler(piodma_write.irq, dma_bus_write_irq_handler);

	// Interrupt pin driver (pio2).  Stopped and the pins handed back to SIO as
	// inputs, which is deasserted for an open collector line; mc6809::deinit()
	// releases them properly just afterwards.
	if (int_pins.sm >= 0) {
		int_pins_program_deinit(int_pins.pio, static_cast<uint>(int_pins.sm),
					GPIO_NMI);
		pio_remove_program(int_pins.pio, &int_pins_program, int_pins.offset);
		int_pins.sm = -1;
		int_pins.offset = -1;
	}
	int_pins_held = 0u;

	// Bus fault detect IRQ (pio2)
	pio_set_irq0_source_enabled(bus_fault.pio, BUS_FAULT_PIO_IRQ_SOURCE, false);
	irq_set_enabled(bus_fault.irq, false);
	irq_remove_handler(bus_fault.irq, pio_bus_fault_irq_handler);

	// --- Bus trace pair (pio1 low half + pio2 high half) ---
	// Everything the trace takes has to go back, or a second power-on cannot
	// rebuild it: pio1 holds clock(4) + write(22) + bus_capture_lo(5) = 31 of
	// its 32 words, so leaving the capture program resident leaves no room to
	// add it again, and pio_add_program returns -1.  The state machines and
	// the two fixed DMA channels leak the same way behind it.
	// MC6809.trace_stop() runs trace_hw_release(), which hands back the
	// programs and state machines if a capture was armed - so the explicit
	// deinit and pio_remove_program calls that used to be here have moved
	// there, and doing them again would unclaim what is no longer ours.
	MC6809.trace_stop();
	dma_channel_abort(trace_lo.data_channel);
	dma_channel_abort(trace_hi.data_channel);
	dma_channel_unclaim(trace_lo.data_channel);
	dma_channel_unclaim(trace_hi.data_channel);
	trace_hw_usable = false;
	trace_hw_samples = 0u;
	trace_pending_start = false;
	MC6809.set_trace_callbacks(nullptr, nullptr, nullptr);
	bus_fault_program_deinit(bus_fault.pio, bus_fault.sm);
	if (bus_fault.offset >= 0) {
		pio_remove_program(bus_fault.pio, &bus_fault_program,
				   static_cast<uint>(bus_fault.offset));
		bus_fault.offset = -1;
	}
	bus_fault.sm = -1;

	// --- Task output (pio2) ---
	task_output_program_deinit(task_output.pio, task_output.sm);
	pio_remove_program(task_output.pio, &task_output_program, task_output.offset);

	// --- Bus write SM + DMA ---
	// The ring channel is part of this chain and goes down with it.  Aborted
	// first: it is what the other two chain into, so stopping it stops the
	// cycle rather than leaving a link that re-triggers what was just halted.
	dma_channel_abort(WRITE_DATA_RING_CHANNEL);
	dma_channel_abort(WRITE_ADDR_RING_CHANNEL);
	dma_channel_abort(piodma_write.address_channel);
	dma_channel_abort(piodma_write.data_channel);
	dma_channel_unclaim(WRITE_DATA_RING_CHANNEL);
	dma_channel_unclaim(WRITE_ADDR_RING_CHANNEL);
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
	// The BASELINE has to go too, not just the counter.  It is a slow average
	// of how the rail looked under the last load, and the load is about to
	// change; worse, if the reference was faulty when it established, it holds
	// an inflated number that the next healthy reading falls away from.  A
	// stale 6.49V baseline survived a trip and a power cycle on 2026-10-06 and
	// re-killed the board on the following quiet sample.
	rail_baseline = 0.0f;
	rail_good_count = 0u;
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
		nmi_hold_passes = 0u;
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
	// above it there unless vreg_disable_voltage_limit() lifts it.  Both come
	// from CMake (PICO_THING_VREG, PICO_THING_VREG_UNLIMITED) so an experiment
	// above 1.30V is a separate build tree, not an edit: 320 MHz has been
	// reliable at 1.30V, and nothing watches the die temperature above it.
#if PICO_THING_VREG_UNLIMITED
	vreg_disable_voltage_limit();
#endif
	vreg_set_voltage(PICO_THING_VREG);
	sleep_ms(5);

	gpio_set_function(GPIO_UART_TX, UART_FUNCSEL_NUM(uart0, GPIO_UART_TX));
	gpio_set_function(GPIO_UART_RX, UART_FUNCSEL_NUM(uart0, GPIO_UART_RX));

	stdio_init_all();

	printf("\n\nPico2 MC6809E bus supervisor %s\n", DEVICE_VERSION);
	printf("Pico2 MC6809E built on %s %s\n", __DATE__, __TIME__);
	printf("Core at %s%s, clk_sys %u Hz\n", PICO_THING_VREG_NAME,
	       PICO_THING_VREG_UNLIMITED ? " (limit disabled)" : "", SYS_CLK_DEFAULT);
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

	// A CONVERSION TAKES 96 ADC CLOCKS, so this sets how long the SAR
	// comparator spends drawing charge from ADC_VREF: 2 us at 48 MHz, 8 us
	// at 12 MHz.  It is a diagnostic lever, not a tuning knob.
	//
	// Why it exists (2026-10-05): both ADC channels read as though the
	// reference were 2.03V while a meter on A101 pin 53 says 3.2470V and the
	// 3.3V rail says 3.2855V.  If the module's local decoupling on that pin
	// has failed, the reference droops during the conversion while a DC meter
	// still sees the unloaded value - and a slower ADC clock gives it more
	// time to recover between bit trials, so the gain error should SHRINK.
	// If the error does not move with this, the droop theory is dead and the
	// fault is the module's internal ADC_AVDD (not brought out on a PGA2350)
	// or the converter itself.
	if (PICO_THING_ADC_CLK_KHZ != 48000u) {
		clock_configure(clk_adc, 0,
				CLOCKS_CLK_ADC_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
				48 * MHZ, PICO_THING_ADC_CLK_KHZ * 1000u);
		printf("Core0 clk_adc set to %u kHz (conversion %.1f us)\n",
		       PICO_THING_ADC_CLK_KHZ,
		       96.0 / (double) PICO_THING_ADC_CLK_KHZ * 1000.0);
	}
	adc_init();
#if PICO_THING_BOARD_VERSION >= 2
	adc_gpio_init(GPIO_POWER_ADC);   // 5V divider, version 2 only
#endif
	adc_set_temp_sensor_enabled(true);
	// Discard the converter's first conversions, as adcdump does: just
	// after adc_init() with the sensor freshly enabled, the first boot
	// sample read 852 counts against 860 settled - 3.7C hotter than the die
	// a few minutes later, which cannot be right for a die still warming.
	static constexpr uint32_t DIE_BOOT_WARMUP = 64u;
	adc_select_input(NUM_ADC_CHANNELS - 1);
	for (uint32_t n = 0u; n < DIE_BOOT_WARMUP; n++)
		(void) adc_read();
	die_boot_raw = adc_read();
	die_boot_ms = to_ms_since_boot(get_absolute_time());
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
