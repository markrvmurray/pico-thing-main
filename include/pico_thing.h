/**
* Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#define DEVICE_VERSION "v0.2.0"

extern volatile bool verbose;

#include <cstdint>
#include <cstdio>

#include "hardware/pio.h"

// Board revision. All revisions carry the same signals, on different pins:
// version 1 puts the task pins and the power enable at the bottom of the GPIO
// range and the DAT fault signals above them, version 2 the other way round.
// Version 3 moves every 5 V input into GPIO 0-39, the RP2350B's 5 V-tolerant
// pads: versions 1 and 2 put A13-A15 and FAULT on pins that are not (on
// version 2 the ADC pins 40-47), and the clamp current wrecked the ADC
// reference and, probably, the guest.  So on version 3 A13-A15 and FAULT
// sit in the low word beside the bus, the interrupt outputs - pulled up to
// 3.3 V, never 5 V - move up to the ADC pins, and the UART to 44/45.
// Set from CMake with -DPICO_THING_BOARD_VERSION=N; anything that includes
// this header without the build system (the unit tests) gets version 2.
#ifndef PICO_THING_BOARD_VERSION
#define PICO_THING_BOARD_VERSION 2
#endif

#if PICO_THING_BOARD_VERSION < 1 || PICO_THING_BOARD_VERSION > 3
#error "PICO_THING_BOARD_VERSION must be 1, 2 or 3"
#endif

// === Pin map ===
enum {
	// Fast/timing-critical - GPIO Bank0, done by PIO
	//	GPIO_E = 0,		// [Y] [o]  E clock
	//	GPIO_Q = 1,		// [Y] [o]  Q clock
	//	GPIO_RW = 2,		// [Y] [i]  Bus R/!W
	//	GPIO_A_BASE = 3,	// [Y] [i]  Address bus, 6 pins
	//	GPIO_D_BASE = 9,	// [Y] [io] Data bus, 8 pins
	//	GPIO_CS = 17,		// [Y] [i]  Device !CS

	// Fast/cycle-critical - GPIO Bank0, done by ISR
	// BS/BA and BUSY/LIC/AVMA must be on consecutive pin runs in this order
	GPIO_BS = 18,   // [Y] [i]  BS - this and BA reach the LED indicator swapped on the V2 PCB; patched on the replacement board
	GPIO_BA = 19,   // [Y] [i]  BA
	GPIO_BUSY = 20, // [Y] [i]  BUSY
	GPIO_LIC = 21,  // [Y] [i]  LIC
	GPIO_AVMA = 22, // [Y] [i]  AVMA

#if PICO_THING_BOARD_VERSION >= 3
	// Medium - per-instruction timing.  5 V inputs, so they must stay on
	// the 5 V-tolerant pads; they land in the low word with the bus.
	GPIO_A_HI_BASE = 23,       // [Y] [i]  Address bus A13-A15, the DAT page of a fault
	GPIO_FAULT = 26,           // [Y] [i]  Attempt to read or write unassigned memory

	// Fast/timing-critical - GPIO Bank0 - will be removed when debugging is done
	//	GPIO_TRACE_RD = 27,	// [Y] [o]  PIO side-set temporary debugging output pins for the scope
	//	GPIO_TRACE_WR = 28,	// [Y] [o]  PIO side-set temporary debugging output pins for the scope

	// Cycle-speed, non-critical temporary output, set by hand inside ISRs - will be removed when debugging is done
	GPIO_TRACE_C = 29,         // [Y] [o]  Temporary debugging output pins for the scope
	GPIO_TRACE_D = 30,         // [Y] [o]  Temporary debugging output pins for the scope

	// Slow - guest instruction-level timing.  TASK0 (Root) sits BELOW
	// TASK[4:0] on this board, so the task word is laid out the other way
	// round - see task_output_word().
	GPIO_TASK_0 = 31,          // [Y] [o]  TASK0, set when the task number is zero
	GPIO_TASK_BASE = 32,       // [Y] [o]  TASK[4:0], five pins

	// Slow - set at a leisurely pace at human-user speeds
	GPIO_POWER = 37,           // [Y] [o]  External power enable

	// Medium - per-instruction timing
	GPIO_NMI_BLOCK = 38,       // [Y] [o]  Prevent DAT Fault NMI

	// Slow - set at a leisurely pace at human-user speeds
	GPIO_RESET = 39, // [Y] [o]  !RESET

	// Asynchronous, at cycle speed.  On the ADC pins, which are not 5 V
	// tolerant: safe only because the pull-ups (R102-R105) go to 3.3 V.
	GPIO_HALT = 40, // [Y] [o]  !HALT
	GPIO_NMI = 41,  // [Y] [o*] !NMI
	GPIO_FIRQ = 42, // [Y] [o*] !FIRQ
	GPIO_IRQ = 43,  // [Y] [o*] !IRQ

	// UART0 on its plain function (F2) here, not the AUX one 30/31 need
	GPIO_UART_TX = 44, // [Y] [o]  RP2xxx UART Tx
	GPIO_UART_RX = 45, // [Y] [i]  RP2xxx UART Rx

	GPIO_POWER_ADC = 46,       // [Y] [i]  ADC on the 5V line, divided by 2
	// GPIO 47 is the PSRAM !CS on the PGA2350
#else
	// Asynchronous, at cycle speed
	GPIO_HALT = 23, // [Y] [o]  !HALT
	GPIO_NMI = 24,  // [Y] [o*] !NMI
	GPIO_FIRQ = 25, // [Y] [o*] !FIRQ
	GPIO_IRQ = 26,  // [Y] [o*] !IRQ

	// Fast/timing-critical - GPIO Bank0 - will be removed when debugging is done
	//	GPIO_TRACE_RD = 27,	// [Y] [o]  PIO side-set temporary debugging output pins for the scope
	//	GPIO_TRACE_WR = 28,	// [Y] [o]  PIO side-set temporary debugging output pins for the scope

	// Slow - set at a leisurely pace at human-user speeds
	GPIO_RESET = 29, // [Y] [o]  !RESET

	// Default assignment
	GPIO_UART_TX = 30, // [Y] [o]  RP2xxx UART Tx
	GPIO_UART_RX = 31, // [Y] [i]  RP2xxx UART Rx

#if PICO_THING_BOARD_VERSION == 2
	// Slow - guest instruction-level timing
	GPIO_TASK_BASE = 32,       // [Y] [o]  TASK[4:0], five pins
	GPIO_TASK_0 = 37,          // [Y] [o]  TASK0, set when the task number is zero

	// Cycle-speed, non-critical temporary output, set by hand inside ISRs - will be removed when debugging is done
	// NOTE: version 1 has these on 37-38, but 37 is TASK0/Root here, so
	// they shift up one to 38-39.  Provisional - see the pin map.
	GPIO_TRACE_C = 38,         // [Y] [o]  Temporary debugging output pins for the scope
	GPIO_TRACE_D = 39,         // [Y] [o]  Temporary debugging output pins for the scope

	// Slow - set at a leisurely pace at human-user speeds
	GPIO_POWER = 40,           // [Y] [o]  External power enable

	// Medium - per-instruction timing
	GPIO_FAULT = 41,           // [Y] [i]  Attempt to read or write unassigned memory
	GPIO_NMI_BLOCK = 42,       // [Y] [o]  Prevent DAT Fault NMI
	GPIO_A_HI_BASE = 43,       // [Y] [i]  Address bus A13-A15, the DAT page of a fault
	GPIO_POWER_ADC = 46,       // [Y] [i]  ADC on the 5V line, divided by 2.0001 measured
#else
	// Version 1: the task pins and the power enable sit at the bottom of the
	// range, so the DAT fault signals go in the free run above them.

	// Medium - per-instruction timing
	GPIO_FAULT = 32,           // [Y] [i]  Attempt to read or write unassigned memory
	GPIO_NMI_BLOCK = 33,       // [Y] [o]  Prevent DAT Fault NMI
	GPIO_A_HI_BASE = 34,       // [Y] [i]  Address bus A13-A15, the DAT page of a fault

	// Cycle-speed, non-critical temporary output, set by hand inside ISRs - will be removed when debugging is done
	GPIO_TRACE_C = 37,         // [Y] [o]  Temporary debugging output pins for the scope
	GPIO_TRACE_D = 38,         // [Y] [o]  Temporary debugging output pins for the scope

	// Slow - guest instruction-level timing
	GPIO_TASK_BASE = 40,       // [Y] [o]  TASK[4:0], five pins
	GPIO_TASK_0 = 45,          // [Y] [o]  TASK0, set when the task number is zero

	// Slow - set at a leisurely pace at human-user speeds
	GPIO_POWER = 46,           // [Y] [o]  External power enable

	// Used by the PSRAM !CS on the PGA2350. Bummer.
	// GPIO_POWER_ADC = 47,    // [N] [i]  ADC connected to the 5V line, divided by 2
#endif
#endif
};


void init_pins_range(uint8_t base, uint8_t count);

// One PIO program and everything claimed on its behalf.  Every program in
// this firmware has one of these, so that a block, a state machine, an
// instruction offset, a DMA channel or an interrupt line is never named by a
// loose global or a bare constant: a mismatch between any two of them is
// silent on hardware and expensive to find.
//
// -1 means "not claimed" or "the program has none", so a teardown can tell an
// unclaimed resource from resource zero, and a premature use shows up as a
// bad index rather than quietly touching state machine 0 or DMA channel 0.
struct pio_dma {
	PIO pio;		// the block the program runs on
	int sm;			// state machine, claimed at power-on
	int offset;		// instruction offset from pio_add_program()
	int address_channel;	// DMA channels feeding the program
	int data_channel;
	int irq;		// NVIC line this program's IRQ drives
};

union bus_pins {
	struct {
		uint32_t E : 1;
		uint32_t Q : 1;
		uint32_t RW : 1;
		uint32_t A : 6;
		uint32_t D : 8;
		uint32_t CS : 1;
		uint32_t BS_BA : 2;
		uint32_t BUSY_LIC_AVMA : 3;
#if PICO_THING_BOARD_VERSION >= 3
		uint32_t A_HI : 3;	// 23-25, A13-A15
		uint32_t FAULT : 1;	// 26
		uint32_t TRACE_RW : 2;	// 27-28
		uint32_t TRACE_CD : 2;	// 29-30
		uint32_t TASK0 : 1;	// 31
#else
		uint32_t HALT : 1;
		uint32_t NMI : 1;
		uint32_t FIRQ : 1;
		uint32_t IRQ : 1;
		uint32_t TRACE_RW : 2;
		uint32_t RESET : 1;
		uint32_t UART : 2;
#endif
	} bits;
	uint32_t word;
};

// BUSY, LIC and AVMA are "in advance" signals: they describe the NEXT bus
// cycle, and the datasheet makes them valid only t_CD after Q-rising (300 ns at
// 1 MHz, 250 at 1.5, 200 at 2).
//
// MEASURED, not assumed (2026-10-04): the Q-rising handler's gpio_get_all()
// lands AFTER t_CD, so what it reads is already the next cycle's description -
// byte-identical to what the trace's capture state machine records at
// E-falling of the same cycle, 0 rows differing out of 512, where 446 of those
// rows differ from the previous cycle's capture.  So a reader here gets raw
// in-advance semantics: LIC means "the NEXT cycle is an opcode fetch", AVMA
// "the NEXT cycle is a valid bus cycle".
//
// That is exactly what something acting a cycle ahead wants, and it is why the
// single-stepper gates on these bits: arming NMI on a LIC edge leaves it
// pending through the whole of the instruction about to be fetched, so the 6809
// completes that one instruction and takes it (c53ac5f).
//
// A PIO sampler at E-falling feeding a DMA-maintained mirror word used to
// re-phase them to "THIS cycle" instead (6b77bb3).  It worked for a display,
// and it was wrong for acting: the mirror was written a quarter cycle before
// the handler read it with nothing ordering the two, so what the handler got
// was a cycle behind even that - two instructions per step, and a read ISR
// whose AVMA guard rejected accesses preceded by a dead cycle (9106b91).  The
// mirror is gone and the pins are the source; the trace's `!` column is what
// watches the t_CD assumption above.
//
// Bit order matches union BLA, so the field feeds set_busy_lic_avma() unchanged.
//
// READ THE WORD ONCE.  Every bitfield read of a volatile is its own load, so
// testing two fields in one expression can mix samples from adjacent E cycles.
// Copy `word` into a local and decode that.

union high_bus_pins {
	struct {
#if PICO_THING_BOARD_VERSION >= 3
		uint32_t TASK : 5;	// 32-36
		uint32_t POWER : 1;	// 37
		uint32_t NMI_BLOCK : 1;	// 38
		uint32_t RESET : 1;	// 39
		uint32_t HALT : 1;	// 40
		uint32_t NMI : 1;	// 41
		uint32_t FIRQ : 1;	// 42
		uint32_t IRQ : 1;	// 43
		uint32_t UART : 2;	// 44-45
		uint32_t POWER_ADC : 1;	// 46
#elif PICO_THING_BOARD_VERSION == 2
		uint32_t TASK : 5;	// 32-36
		uint32_t TASK0 : 1;	// 37
		uint32_t TRACE_C : 1;	// 38
		uint32_t TRACE_D : 1;	// 39
		uint32_t POWER : 1;	// 40
		uint32_t FAULT : 1;	// 41
		uint32_t NMI_BLOCK : 1;	// 42
		uint32_t A_HI : 3;	// 43-45, A13-A15
		uint32_t POWER_ADC : 1;	// 46
#else
		uint32_t FAULT : 1;	// 32
		uint32_t NMI_BLOCK : 1;	// 33
		uint32_t A_HI : 3;	// 34-36, A13-A15
		uint32_t TRACE_C : 1;	// 37
		uint32_t TRACE_D : 1;	// 38
		uint32_t : 1;		// 39, unused
		uint32_t TASK : 5;	// 40-44
		uint32_t TASK0 : 1;	// 45
		uint32_t POWER : 1;	// 46
#endif
	} bits;
	uint32_t word;
};

// The signals that change halves between board revisions.  A trace row is a
// (bus_pins, high_bus_pins) pair; these read a signal from whichever half it
// is in on the board being built for, so a decoder never needs its own #if.
// Interrupt and !HALT bits are the raw active-low pin levels.
#if PICO_THING_BOARD_VERSION >= 3
static inline bool row_halt(bus_pins, high_bus_pins hp) { return hp.bits.HALT; }
static inline bool row_nmi(bus_pins, high_bus_pins hp) { return hp.bits.NMI; }
static inline bool row_firq(bus_pins, high_bus_pins hp) { return hp.bits.FIRQ; }
static inline bool row_irq(bus_pins, high_bus_pins hp) { return hp.bits.IRQ; }
static inline bool row_task0(bus_pins bp, high_bus_pins) { return bp.bits.TASK0; }
static inline unsigned row_a_hi(bus_pins bp, high_bus_pins) { return bp.bits.A_HI; }
static inline bool row_fault(bus_pins bp, high_bus_pins) { return bp.bits.FAULT; }
#else
static inline bool row_halt(bus_pins bp, high_bus_pins) { return bp.bits.HALT; }
static inline bool row_nmi(bus_pins bp, high_bus_pins) { return bp.bits.NMI; }
static inline bool row_firq(bus_pins bp, high_bus_pins) { return bp.bits.FIRQ; }
static inline bool row_irq(bus_pins bp, high_bus_pins) { return bp.bits.IRQ; }
static inline bool row_task0(bus_pins, high_bus_pins hp) { return hp.bits.TASK0; }
static inline unsigned row_a_hi(bus_pins, high_bus_pins hp) { return hp.bits.A_HI; }
static inline bool row_fault(bus_pins, high_bus_pins hp) { return hp.bits.FAULT; }
#endif

// Chunks are like snippets, except they are too big for snippet space, so they go into RAM
#define	CHUNK_ADDRESS	uint16_t(0x0130u)
// === Device addresses ===
#define REGISTER_BASE	uint16_t(0xFFC0u)

#define ADDRESS(index)	uint16_t(REGISTER_BASE + index)

#define REGISTER_DEVICES_OFFSET uint16_t(0)
#define REGISTER_DEVICES uint16_t(REGISTER_BASE + REGISTER_DEVICES_OFFSET)
#define REGISTER_BUFFER_OFFSET uint16_t(16)
#define REGISTER_BUFFER uint16_t(REGISTER_BASE + REGISTER_BUFFER_OFFSET)
#define REGISTER_BUFFER_LEN uint16_t(16)
#define REGISTER_SNIPPET_OFFSET uint16_t(32)
#define REGISTER_SNIPPET_LEN uint16_t(16)
#define REGISTER_SNIPPET uint16_t(REGISTER_BASE + REGISTER_SNIPPET_OFFSET)
#define REGISTER_VECTORS_OFFSET uint16_t(48)
#define REGISTER_VECTORS uint16_t(REGISTER_BASE + REGISTER_VECTORS_OFFSET)

#define REGISTER_VECTOR_RESERVED uint16_t(REGISTER_VECTORS + 0)
#define REGISTER_VECTOR_RESERVED_OFFSET uint16_t(REGISTER_VECTORS_OFFSET + 0)
#define REGISTER_VECTOR_SWI3 uint16_t(REGISTER_VECTORS + 2)
#define REGISTER_VECTOR_SWI3_OFFSET uint16_t(REGISTER_VECTORS_OFFSET + 2)
#define REGISTER_VECTOR_SWI2 uint16_t(REGISTER_VECTORS + 4)
#define REGISTER_VECTOR_SWI2_OFFSET uint16_t(REGISTER_VECTORS_OFFSET + 4)
#define REGISTER_VECTOR_FIRQ uint16_t(REGISTER_VECTORS + 6)
#define REGISTER_VECTOR_FIRQ_OFFSET uint16_t(REGISTER_VECTORS_OFFSET + 6)
#define REGISTER_VECTOR_IRQ uint16_t(REGISTER_VECTORS + 8)
#define REGISTER_VECTOR_IRQ_OFFSET uint16_t(REGISTER_VECTORS_OFFSET + 8)
#define REGISTER_VECTOR_SWI uint16_t(REGISTER_VECTORS + 10)
#define REGISTER_VECTOR_SWI_OFFSET uint16_t(REGISTER_VECTORS_OFFSET + 10)
#define REGISTER_VECTOR_NMI uint16_t(REGISTER_VECTORS + 12)
#define REGISTER_VECTOR_NMI_OFFSET uint16_t(REGISTER_VECTORS_OFFSET + 12)
#define REGISTER_VECTOR_RESET uint16_t(REGISTER_VECTORS + 14)
#define REGISTER_VECTOR_RESET_OFFSET uint16_t(REGISTER_VECTORS_OFFSET + 14)

// Task register bits
#define NUM_TASK_PINS		(5u)
#define	TASK_PINS_MASK		((1u << NUM_TASK_PINS) - 1u)

// The six task pins are consecutive, so task_output drives them with one
// "out pins 6" - but TASK0 (Root, high when the task is zero) is the top pin
// on versions 1 and 2 and the BOTTOM one on version 3.
#if PICO_THING_BOARD_VERSION >= 3
#define	TASK_OUTPUT_BASE	GPIO_TASK_0
#define	BUS_FAULT_SAMPLE_BASE	16u	// GPIO 16-47: the page and FAULT are at 23-26
static inline uint32_t
task_output_word(uint8_t task)
{
	return (static_cast<uint32_t>(task & TASK_PINS_MASK) << 1) | ((task == 0u) ? 1u : 0u);
}
#else
#define	TASK_OUTPUT_BASE	GPIO_TASK_BASE
#define	BUS_FAULT_SAMPLE_BASE	32u	// GPIO 32-47, then 16-31 as the window wraps
static inline uint32_t
task_output_word(uint8_t task)
{
	return task | ((task == 0u) ? (1u << NUM_TASK_PINS) : 0u);
}
#endif

// System registers relative to REGISTER_DEVICES
#define SYSTEM_TASK		uint16_t(0x00u)

#define SYSTEM_REQUEST		uint16_t(0x02u) // Write-only
#define SYSTEM_REQUEST_STATUS	uint16_t(0x02u) // Read-only
#define SYSTEM_SUBREQUEST	uint16_t(0x03u) // Write-only
#define SYSTEM_REQUEST_RESPONSE	uint16_t(0x03u) // Read-only

// Emulated UARTs — register offsets relative to REGISTER_DEVICES
// Primary UART (console)
#define CONSOLE_CONTROL		uint16_t(0x04u)
#define CONSOLE_STATUS		uint16_t(0x04u)
#define CONSOLE_TX_DATA		uint16_t(0x05u)
#define CONSOLE_RX_DATA		uint16_t(0x05u)
// Auxiliary UART
#define AUX_CONTROL		uint16_t(0x06u)
#define AUX_STATUS		uint16_t(0x06u)
#define AUX_TX_DATA		uint16_t(0x07u)
#define AUX_RX_DATA		uint16_t(0x07u)

// Simple 50 Hz tick timer.
// $FFC8 write: bit 0 = enable/disable.
// $FFC9 read:  bit 7 = IRQ pending; reading clears it.
#define SYSTEM_TIMER_CONTROL_13	uint16_t(0x08u) // Write-only (enable/disable)
#define SYSTEM_TIMER_STATUS	uint16_t(0x09u) // Read-only (IRQ flag, cleared on read)

// Vector watchpoint: guest writes non-zero to arm, zero to disarm.
// When armed, writes to $FFF0-$FFFF are blocked and trigger NMI.
#define SYSTEM_WATCHPOINT	uint16_t(0x0Au) // Write-only

// Bus trace control (USim TraceCtl convention): guest writes non-zero to arm a
// one-shot capture, zero to stop. One immediate store per poke.
#define SYSTEM_TRACE		uint16_t(0x0Bu) // Write-only

// DAT RAM fault - attempt to read or write a block of unassigned DAT-addressed RAM
// This register will identify a fault as the source of an NMI, and can be used to mask
// fault NMIs in the hardware.
// Bit 0 - Fault (read - clears, write - no effect)
// Bit 1 - R/!W of the faulting access (read-only, write - no effect)
// Bit 7 - NMIBlock (read - returns set/clear status, write set/clear status)
//
// First fault wins: latching a fault also asserts NMIBlock, and while NMIBlock
// is set no further fault touches either register.  A storm of faults
// therefore leaves the record of the one that started it, rather than the last
// one.  The guest re-arms by writing zero here, which is also the only way to
// clear NMIBlock.
//
// The resulting NMI takes its vector with the guest switched to task 0, so the
// handler can reach the DAT RAM.  The faulting instruction cannot be retried —
// the 6809 takes NMI at an instruction boundary and the access has already
// completed on garbage — so a fault is fatal to the process that caused it and
// these registers are post-mortem evidence, not inputs to a repair.  The
// faulting task is on the task stack by then, and apply_rti() restores it
// unconditionally ~15 E-cycles after any RTI; an OS that terminates the
// process has to sequence its task switch around that.
#define SYSTEM_DAT_FAULT	uint16_t(0x0Cu) // Read/Write ($FFCC)

// Where the fault happened, latched with it: the task that was selected and
// the 8kB page inside it, laid out as the DAT RAM index of the unmapped block.
// Bits 0-2 - A15-A13 of the faulting access, the page within the task
// Bits 3-7 - TASK[4:0] as driven on the task pins
//
// The page is always 0-6.  Block 7 is the fixed page: the GAL translates only
// $0000-$DFFF (MMU_dat), so a block 7 access cannot assert the fault line, and
// a page of 7 here is therefore impossible in correct operation.  If one is
// ever observed it means the fault flag and the address pins have come from
// different bus cycles — see the sampling note in the Q-falling ISR.
#define SYSTEM_DAT_FAULT_LOC	uint16_t(0x0Du) // Read ($FFCD)

// SYSTEM_DAT_FAULT bits
#define DAT_FAULT_FAULT		uint8_t(1u << 0) // set when GPIO_FAULT is sampled high
#define DAT_FAULT_RW		uint8_t(1u << 1) // R/!W latched with the fault, read-only
#define DAT_FAULT_NMI_BLOCK	uint8_t(1u << 7) // mirrors the GPIO_NMI_BLOCK output pin

// Everything the fault sampler latches into SYSTEM_DAT_FAULT, and the read clears.
#define DAT_FAULT_LATCHED	uint8_t(DAT_FAULT_FAULT | DAT_FAULT_RW)

// SYSTEM_DAT_FAULT_LOC fields
#define DAT_FAULT_LOC_PAGE_MASK	 uint8_t(0x07u) // A15-A13, already in bus order
#define DAT_FAULT_LOC_TASK_SHIFT uint8_t(3)
#define DAT_FAULT_LOC_TASK_MASK	 uint8_t(TASK_PINS_MASK << DAT_FAULT_LOC_TASK_SHIFT)

// System requests, their responses, and floating-point requests
#include "sysreq.h"
