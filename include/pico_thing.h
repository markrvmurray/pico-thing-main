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

// Board revision. Both revisions carry the same signals, on different pins:
// version 1 puts the task pins and the power enable at the bottom of the GPIO
// range and the DAT fault signals above them, version 2 the other way round.
// Set from CMake with -DPICO_THING_BOARD_VERSION=1 to build for the older
// board; anything that includes this header without the build system (the
// unit tests) gets version 2.
#ifndef PICO_THING_BOARD_VERSION
#define PICO_THING_BOARD_VERSION 2
#endif

#if PICO_THING_BOARD_VERSION != 1 && PICO_THING_BOARD_VERSION != 2
#error "PICO_THING_BOARD_VERSION must be 1 or 2"
#endif

// === Pin map ===
enum {
	// Fast/timing-critical - GPIO Bank0, done by PIO
	//	GPIO_E = 0,		// [ ] [o]  E clock
	//	GPIO_Q = 1,		// [ ] [o]  Q clock
	//	GPIO_RW = 2,		// [ ] [i]  Bus R/!W
	//	GPIO_A_BASE = 3,	// [ ] [i]  Address bus, 6 pins
	//	GPIO_D_BASE = 9,	// [ ] [io] Data bus, 8 pins
	//	GPIO_CS = 17,		// [ ] [i]  Device !CS

	// Fast/cycle-critical - GPIO Bank0, done by ISR
	// BS/BA and BUSY/LIC/AVMA must be on consecutive pin runs in this order
	GPIO_BS = 18,   // [ ] [i]  BS - this and BA are swapped on the LED indicator on V2
	GPIO_BA = 19,   // [ ] [i]  BA
	GPIO_BUSY = 20, // [ ] [i]  BUSY
	GPIO_LIC = 21,  // [ ] [i]  LIC
	GPIO_AVMA = 22, // [ ] [i]  AVMA

	// Asynchronous, at cycle speed
	GPIO_HALT = 23, // [ ] [o]  !HALT
	GPIO_NMI = 24,  // [ ] [o*] !NMI
	GPIO_FIRQ = 25, // [ ] [o*] !FIRQ
	GPIO_IRQ = 26,  // [ ] [o*] !IRQ

	// Fast/timing-critical - GPIO Bank0 - will be removed when debugging is done
	//	GPIO_TRACE_RD = 27,	// [ ] [o]  PIO side-set temporary debugging output pins for the scope
	//	GPIO_TRACE_WR = 28,	// [ ] [o]  PIO side-set temporary debugging output pins for the scope

	// Slow - set at a leisurely pace at human-user speeds
	GPIO_RESET = 29, // [ ] [o]  !RESET

	// Default assignment
	GPIO_UART_TX = 30, // [ ] [o]  RP2xxx UART Tx
	GPIO_UART_RX = 31, // [ ] [i]  RP2xxx UART Rx

#if PICO_THING_BOARD_VERSION >= 2
	// Slow - guest instruction-level timing
	GPIO_TASK_BASE = 32,       // [ ] [o]  TASK[4:0], five pins
	GPIO_TASK_0 = 37,          // [ ] [o]  TASK0, set when the task number is zero

	// Cycle-speed, non-critical temporary output, set by hand inside ISRs - will be removed when debugging is done
	// NOTE: version 1 has these on 37-38, but 37 is TASK0/Root here, so
	// they shift up one to 38-39.  Provisional - see the pin map.
	GPIO_TRACE_C = 38,         // [ ] [o]  Temporary debugging output pins for the scope
	GPIO_TRACE_D = 39,         // [ ] [o]  Temporary debugging output pins for the scope

	// Slow - set at a leisurely pace at human-user speeds
	GPIO_POWER = 40,           // [ ] [o]  External power enable

	// Medium - per-instruction timing
	GPIO_FAULT = 41,           // [ ] [i]  Attempt to read or write unassigned memory
	GPIO_NMI_BLOCK = 42,       // [ ] [o]  Prevent DAT Fault NMI
	GPIO_A_HI_BASE = 43,       // [ ] [i]  Address bus A13-A15, the DAT page of a fault
	GPIO_POWER_ADC = 46,       // [ ] [i]  ADC on the 5V line, divided by 1.9407 measured
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
};


void init_pins_range(uint8_t base, uint8_t count);

struct pio_dma {
	PIO pio;
	int sm;
	int address_channel;
	int data_channel;
	uint32_t irq;
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
		uint32_t HALT : 1;
		uint32_t NMI : 1;
		uint32_t FIRQ : 1;
		uint32_t IRQ : 1;
		uint32_t TRACE_RW : 2;
		uint32_t RESET : 1;
		uint32_t UART : 2;
	} bits;
	uint32_t word;
};

// GPIO 32 upwards, as read from the SIO high input register.
union high_bus_pins {
	struct {
#if PICO_THING_BOARD_VERSION >= 2
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

// System registers relative to REGISTER_DEVICES
#define SYSTEM_TASK		uint16_t(0x00u)
#define SYSTEM_RTI_SIGNAL	uint16_t(0x01u) // Write-only: guest signals RTI

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

// System request opcodes
#define	REQUEST_NULL		uint8_t(0x00u)
#define REQUEST_TIME		uint8_t('T')
#define REQUEST_SET_TIME	uint8_t('t')
#define REQUEST_MOUNT		uint8_t('M')
#define REQUEST_UNMOUNT		uint8_t('m')
#define REQUEST_FLOAT		uint8_t('R')

// System status values
#define	RESPONSE_NULL		uint8_t(0x00u)
#define	RESPONSE_DONE		uint8_t(0x01u)
#define	RESPONSE_BUSY		uint8_t(0x7Fu)
#define RESPONSE_UNKNOWN	uint8_t(0xFFu)
#define	RESPONSE_UNAVAILABLE	uint8_t(0xFDu)

// Floating-point requests
#define FLOAT_ADD		uint8_t('+')
#define FLOAT_SUBTRACT		uint8_t('-')
#define FLOAT_MULTIPLY		uint8_t('*')
#define FLOAT_DIVIDE		uint8_t('/')
