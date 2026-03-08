/**
* Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#define DEVICE_VERSION "vX.X.x"

extern volatile bool verbose;

#include <cstdint>
#include <cstdio>

#include "hardware/pio.h"

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
	GPIO_BS = 18,   // [Y] [i]  BS
	GPIO_BA = 19,   // [Y] [i]  BA
	GPIO_BUSY = 20, // [Y] [i]  BUSY
	GPIO_LIC = 21,  // [Y] [i]  LIC
	GPIO_AVMA = 22, // [Y] [i]  AVMA

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

	// Fast communication with the video Pico
	// GPIO_PIX_BASE = 32, // [ ] Four pins for fast communication

	// Cycle-speed, non-critical temporary output, set by hand inside ISRs - will be removed when debugging is done
	GPIO_TRACE_CORE1 = 36,     // [Y] [o]  Temporary debugging output pins for the scope
	GPIO_TRACE_EQ_IRQ = 37,    // [Y] [o]  Temporary debugging output pins for the scope
	GPIO_TRACE_READ_IRQ = 38,  // [Y] [o]  Temporary debugging output pins for the scope
	GPIO_TRACE_WRITE_IRQ = 39, // [Y] [o]  Temporary debugging output pins for the scope

	// Slow - guest instruction-level timing
	GPIO_TASK_BASE = 40,
	GPIO_TASK_0 = 45,

	// Slow - set at a leisurely pace at human-user speeds
	GPIO_POWER = 46, // [Y] [o]  External power enable

	// Used by the PSRAM !CS on the PGA2350. Bummer.
	// GPIO_POWER_ADC = 47, // [Y] [i]  ADC connected to the 5v line, divided by 2
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

union high_bus_pins {
	struct {
		uint16_t PIX : 4;
		uint16_t TRACE_CORE1 : 1;
		uint16_t TRACE_EQ : 1;
		uint16_t TRACE_RD : 1;
		uint16_t TRACE_RW : 1;
		uint16_t TASK : 5;
		uint16_t TASK0 : 1;
		uint16_t POWER : 1;
		uint16_t POWER_ADC : 1;
	} bits;
	uint16_t hword;
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

#define SYSTEM_REQUEST		uint16_t(0x01u) // Write-only
#define SYSTEM_REQUEST_STATUS	uint16_t(0x01u) // Read-only
#define SYSTEM_SUBREQUEST	uint16_t(0x02u) // Write-only
#define SYSTEM_REQUEST_RESPONSE	uint16_t(0x02u) // Read-only

// Emulated UART
// UART registers relative to REGISTER_DEVICES
#define CONSOLE_CONTROL		uint16_t(0x03u)
#define CONSOLE_STATUS		uint16_t(0x03u)
#define CONSOLE_TX_DATA		uint16_t(0x04u)
#define CONSOLE_RX_DATA		uint16_t(0x04u)

// Simple 50 Hz tick timer.
// $FFC8 write: bit 0 = enable/disable.
// $FFC9 read:  bit 7 = IRQ pending; reading clears it.
#define SYSTEM_TIMER_CONTROL_13	uint16_t(0x08u) // Write-only (enable/disable)
#define SYSTEM_TIMER_STATUS	uint16_t(0x09u) // Read-only (IRQ flag, cleared on read)

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
