/**
* Copyright (c) 2025 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef PICO_MC6809_H
#define PICO_MC6809_H

#include <cstdint>

#define DEVICE_VERSION "vX.X.x"

#define DEBUG

#define stringify(a) x_stringify(a)
#define x_stringify(a) #a

#define enum_list(e) e,
#define enum_list_strings(s) #s,

#define reverse_2_bytes(x) __builtin_bswap16(x)
#define reverse_4_bytes(x) __builtin_bswap32(x)

// === Pin map ===
enum {
	// Default assignment
	GPIO_UART_TX = 0, // [Y] [o]  RP2xxx UART Tx
	GPIO_UART_RX = 1, // [Y] [i]  RP2xxx UART Rx

	// Fast/timing-critical - GPIO Bank0, done by PIO
	//	GPIO_E = 2,		// [Y] [o]  E clock
	//	GPIO_Q = 3,		// [Y] [o]  Q clock
	//	GPIO_RW = 4,		// [Y] [i]  Bus R/!W
	//	GPIO_A_BASE = 5,	// [Y] [i]  Address bus, 6 pins
	//	GPIO_D_BASE = 11,	// [Y] [io] Data bus, 8 pins
	//	GPIO_CS = 19,		// [Y] [i]  Device !CS

	// Fast/cycle-critical - GPIO Bank0, done by ISR
	// BS/BA and BUSY/LIC/AVMA must be on consecutive pin runs in this order
	GPIO_BS = 20,   // [Y] [i]  BS
	GPIO_BA = 21,   // [Y] [i]  BA
	GPIO_BUSY = 22, // [Y] [i]  BUSY
	GPIO_LIC = 23,  // [Y] [i]  LIC
	GPIO_AVMA = 24, // [Y] [i]  AVMA

	// Asynchronous, at cycle speed
	GPIO_HALT = 25, // [Y] [o]  !HALT
	GPIO_NMI = 26,  // [Y] [o*] !NMI
	GPIO_IRQ = 27,  // [Y] [o*] !IRQ
	GPIO_FIRQ = 28, // [Y] [o*] !FIRQ

	// Fast/timing-critical - GPIO Bank0 - will be removed when debugging is done
	//	GPIO_TRACE_RD = 29,	// [Y] [o]  PIO side-set temporary debugging output pins for the scope
	//	GPIO_TRACE_WR = 30,	// [Y] [o]  PIO side-set temporary debugging output pins for the scope

	// Slow - guest instruction-level timing
	GPIO_TASK_BASE = 31,
	GPIO_TASK_0 = 36,

	// Slow - set at a leisurely pace at human-user speeds
	GPIO_POWER = 40, // [Y] [o]  External power enable
	GPIO_RESET = 41, // [Y] [o]  !RESET

	// Cycle-speed, non-critical temporary output, set by hand inside ISRs - will be removed when debugging is done
	GPIO_TRACE_G = 43, // [Y] [o]  Temporary debugging output pins for the scope
	GPIO_TRACE_C = 44, // [Y] [o]  Temporary debugging output pins for the scope
	GPIO_TRACE_D = 45, // [Y] [o]  Temporary debugging output pins for the scope
	GPIO_TRACE_E = 46, // [Y] [o]  Temporary debugging output pins for the scope
	GPIO_TRACE_F = 47, // [Y] [o]  Temporary debugging output pins for the scope
};

enum interrupt {
	INTERRUPT_ILLEGAL,
	INTERRUPT_SWI3,
	INTERRUPT_SWI2,
	INTERRUPT_FIRQ,
	INTERRUPT_IRQ,
	INTERRUPT_SWI,
	INTERRUPT_NMI,
	INTERRUPT_RESET,
	INTERRUPT_NONE,
	NUM_INTERRUPTS
};
#define foreach_busstate(busstate) \
busstate(BS_RUNNING)       \
busstate(BS_IRQ)           \
busstate(BS_SYNC)          \
busstate(BS_HALT)          \
busstate(BS_RUNNING_RESET) \
busstate(BS_IRQ_RESET)     \
busstate(BS_SYNC_RESET)    \
busstate(BS_HALT_RESET)

enum ba_bs {
	foreach_busstate(enum_list)
	BUS_STATE_COUNT
};

union ba_bs_u {
	uint8_t byte;
	struct {
		uint8_t BS: 1;
		uint8_t BA: 1;
		uint8_t RESET: 1;
		uint8_t : 5;
	} bit;
	enum ba_bs state;
};

#define foreach_runstate(runstate) \
runstate(RS_STARTED) \
runstate(RS_INTERRUPTED) \
runstate(RS_HALTED) \
runstate(RS_SYNCED) \
runstate(RS_STOPPED) \
runstate(RS_HEADLESS_STARTED) \
runstate(RS_HEADLESS_INTERRUPTED) \
runstate(RS_HEADLESS_HALTED) \
runstate(RS_HEADLESS_SYNCED) \
runstate(RS_HEADLESS_STOPPED)

enum run_state {
	foreach_runstate(enum_list)
	RUN_STATE_COUNT
};

union BLA {
	struct {
		uint8_t BUSY: 1;
		uint8_t LIC: 1;
		uint8_t AVMA: 1;
		uint8_t : 5;
	} bit;
	uint8_t byte;
};

class processor {
private:
	volatile ba_bs_u old_bus_state{};
	volatile ba_bs_u bus_state{};
	volatile enum run_state run_state;
	volatile BLA busy_lic_avma{};
	volatile uint8_t task;
	volatile uint32_t task_stack_ptr;
	volatile bool busy, lic, vma;
#ifdef DEBUG
	volatile uint32_t _count_lic;
#endif
	float e_freq;
	void task_initialise();
	void setup(enum run_state rs);
public:
	void set_busy_lic_avma(uint8_t value) { busy_lic_avma.byte = value; }
	[[nodiscard]] uint8_t get_busy_lic_avma() const { return busy_lic_avma.byte; }
	void unpack_busy_lic_avma() { busy = busy_lic_avma.bit.BUSY; lic = busy_lic_avma.bit.LIC; vma = busy_lic_avma.bit.AVMA; }
	[[nodiscard]] bool get_busy() const { return busy; }
	[[nodiscard]] bool get_lic() const { return lic; }
	[[nodiscard]] bool get_vma() const { return vma; }
	void clear_lic_count() { _count_lic = 0u; }
	[[nodiscard]] uint32_t get_lic_count() const { return _count_lic; }
	void count_lic() { if (lic) _count_lic++; }
	processor();
	void init();
	uint8_t task_change(uint8_t new_task);

	[[nodiscard]] bool assert_stopped() const;
	[[nodiscard]] uint8_t task_get() const;
	void start();
	void start_headless();
	void start_with_timeout(uint32_t timeout_ms);
	void stop();
	void halt();
	void release();
	bool try_loop();
	void set_e_frequency(float target_mhz);
	[[nodiscard]] float get_e_frequency() const;
	friend void gpio_clock_eq_irq_handler();
	friend void dma_bus_read_irq_handler();
	friend void dma_bus_write_irq_handler();
	friend void process_command(char *buf);
};

void init_pins_range(uint8_t base, uint8_t count);

// === Device addresses ===
#define REGISTER_BASE 0xFFC0u

#define REGISTER_INDEX(addr) ((addr) & 0x3Fu)

#define REGISTER_DEVICES_OFFSET 0
#define REGISTER_DEVICES (REGISTER_BASE + REGISTER_DEVICES_OFFSET)
#define REGISTER_BUFFER_OFFSET 16
#define REGISTER_BUFFER (REGISTER_BASE + REGISTER_BUFFER_OFFSET)
#define REGISTER_SNIPPET_OFFSET 32
#define REGISTER_SNIPPET (REGISTER_BASE + REGISTER_SNIPPET_OFFSET)
#define REGISTER_VECTORS_OFFSET 48
#define REGISTER_VECTORS (REGISTER_BASE + REGISTER_VECTORS_OFFSET)

#define REGISTER_VECTOR_RESERVED (REGISTER_VECTORS + 0)
#define REGISTER_VECTOR_RESERVED_OFFSET (REGISTER_VECTORS_OFFSET + 0)
#define REGISTER_VECTOR_SWI3 (REGISTER_VECTORS + 2)
#define REGISTER_VECTOR_SWI3_OFFSET (REGISTER_VECTORS_OFFSET + 2)
#define REGISTER_VECTOR_SWI2 (REGISTER_VECTORS + 4)
#define REGISTER_VECTOR_SWI2_OFFSET (REGISTER_VECTORS_OFFSET + 4)
#define REGISTER_VECTOR_FIRQ (REGISTER_VECTORS + 6)
#define REGISTER_VECTOR_FIRQ_OFFSET (REGISTER_VECTORS_OFFSET + 6)
#define REGISTER_VECTOR_IRQ (REGISTER_VECTORS + 8)
#define REGISTER_VECTOR_IRQ_OFFSET (REGISTER_VECTORS_OFFSET + 8)
#define REGISTER_VECTOR_SWI (REGISTER_VECTORS + 10)
#define REGISTER_VECTOR_SWI_OFFSET (REGISTER_VECTORS_OFFSET + 10)
#define REGISTER_VECTOR_NMI (REGISTER_VECTORS + 12)
#define REGISTER_VECTOR_NMI_OFFSET (REGISTER_VECTORS_OFFSET + 12)
#define REGISTER_VECTOR_RESET (REGISTER_VECTORS + 14)
#define REGISTER_VECTOR_RESET_OFFSET (REGISTER_VECTORS_OFFSET + 14)

// System registers relative to REGISTER_DEVICES
#define SYSTEM_TASK		0x00u

#define SYSTEM_REQUEST		0x01u // Write-only
#define SYSTEM_REQUEST_STATUS	0x01u // Read-only
#define SYSTEM_SUBREQUEST	0x02u // Write-only
#define SYSTEM_REQUEST_RESPONSE	0x02u // Read-only

#define SYSTEM_TIMER_CONTROL	0x03u // Write-only
#define SYSTEM_TIMER_STATUS	0x03u // Read-only
#define SYSTEM_TIMER_COUNTERS	0x04u // 4/5 and 6/7 are used

// Task register bits
#define NUM_TASK_PINS		5
#define	TASK_PINS_MASK		0b00011111u

// System request opcodes
#define	REQUEST_NULL		0x00u
#define REQUEST_TIME		'T'
#define REQUEST_SETTIME		't'
#define REQUEST_MOUNT		'M'
#define REQUEST_UNMOUNT		'm'
#define REQUEST_FLOAT		'R'

// System status values
#define	RESPONSE_NULL		0x00u
#define	RESPONSE_DONE		0x01u
#define	RESPONSE_BUSY		0x7Fu
#define RESPONSE_UNKNOWN	0xFFu
#define	RESPONSE_UNAVAILABLE	0xFDu

// Floating-point requests
#define FLOAT_ADD		'+'
#define FLOAT_SUBTRACT		'-'
#define FLOAT_MULTIPLY		'*'
#define FLOAT_DIVIDE		'/'

// System timer control/status bits
#define TIMER_IRQ		0b00000001u
#define TIMER_ENABLED		0b00000010u
#define TIMER_REPEAT		0b00000100u

// Emulated UART
// UART registers relative to REGISTER_DEVICES
#define CONSOLE_CONTROL		0x08u
#define CONSOLE_STATUS		0x08u
#define CONSOLE_TX_DATA		0x09u
#define CONSOLE_RX_DATA		0x09u
// UART register bits
#define CONSOLE_NONE		0b00000000u
#define CONSOLE_TX_MASK		0b11110000u
#define CONSOLE_TX_AVAIL	0b00010000u
#define CONSOLE_TX_ERROR	0b00100000u
#define CONSOLE_TX_IRQ		0b10000000u
#define CONSOLE_RX_MASK		0b00001111u
#define CONSOLE_RX_AVAIL	0b00000001u
#define CONSOLE_RX_ERROR	0b00000010u
#define CONSOLE_RX_IRQ		0b00001000u
//UART miscellaneous constants
#define CONSOLE_QUEUE_LEN	256

#endif
