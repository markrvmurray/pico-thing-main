/**
* Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef PICO_THING_H
#define PICO_THING_H

#define DEVICE_VERSION "vX.X.x"

extern volatile bool verbose;

#include <cstdio>

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
	GPIO_TRACE_CORE1 = 43, // [Y] [o]  Temporary debugging output pins for the scope
	GPIO_TRACE_EQ_IRQ = 44, // [Y] [o]  Temporary debugging output pins for the scope
	GPIO_TRACE_READ_IRQ = 45, // [Y] [o]  Temporary debugging output pins for the scope
	GPIO_TRACE_WRITE_IRQ = 46, // [Y] [o]  Temporary debugging output pins for the scope
	GPIO_POWER_ADC = 47, // [Y] [i]  ADC connected to the 5v line, divided by 2
};


void init_pins_range(uint8_t base, uint8_t count);

struct pio_dma {
	PIO pio;
	int sm;
	int address_channel;
	int data_channel;
	uint32_t irq;
};

class registers {
	static constexpr uint16_t register_length = 64u;
	__scratch_y("")
	__attribute__((aligned(register_length)))
	inline static volatile uint8_t read_registers[register_length];
	__scratch_y("")
	__attribute__((aligned(register_length)))
	inline static volatile uint8_t write_registers[register_length];
	// This is a singleton class
	registers() = default; // no public constructor
	~registers() = default; // no public destructor
	inline static registers* instance = nullptr;
	struct ReadProxy {
		size_t i;
		operator uint8_t() const { return read_registers[i]; }
		explicit operator uint16_t() const { return __builtin_bswap16(*reinterpret_cast<volatile uint16_t*>(&read_registers[i])); }
		explicit operator float() const { uint32_t u = __builtin_bswap32(*reinterpret_cast<volatile uint32_t*>(&read_registers[i])); float f; std::memcpy(&f,&u,sizeof f); return f; }		ReadProxy& operator=(volatile uint8_t v){ read_registers[i] = v; return *this; }
		ReadProxy& operator=(uint16_t v) { *reinterpret_cast<volatile uint16_t*>(&read_registers[i]) = __builtin_bswap16(v); return *this; }
		ReadProxy& operator=(float v) { uint32_t u; std::memcpy(&u, &v, sizeof u); *reinterpret_cast<volatile uint32_t*>(&read_registers[i]) = __builtin_bswap32(u); return *this; }
		bool operator==(uint8_t v) const { return read_registers[i] == v; }
		bool operator==(uint16_t v) const { return *reinterpret_cast<volatile uint16_t*>(&read_registers[i]) == __builtin_bswap16(v); }
		bool operator==(float v) const { uint32_t u = __builtin_bswap32(*reinterpret_cast<volatile uint32_t*>(&read_registers[i])); float f; std::memcpy(&f,&u,sizeof f); return f == v; }
		ReadProxy& operator&=(uint8_t v) { read_registers[i] &= v; return *this; }
		ReadProxy& operator&=(uint16_t v) { auto* p = reinterpret_cast<volatile uint16_t*>(&read_registers[i]); *p &= __builtin_bswap16(v); return *this; }
		ReadProxy& operator|=(uint8_t v) { read_registers[i] |= v; return *this; }
		ReadProxy& operator|=(uint16_t v) { auto* p = reinterpret_cast<volatile uint16_t*>(&read_registers[i]); *p |= __builtin_bswap16(v); return *this; }
		// volatile uint8_t *operator&() const { return &read_registers[i]; }
		// uint8_t operator&(uint8_t v) const { return static_cast<uint8_t>(*this) & v; }
	};
	struct WriteProxy {
		uint16_t i;
		volatile uint8_t* wbuf;
		operator uint8_t() const { return wbuf[i]; }
		explicit operator uint16_t() const { return __builtin_bswap16(*reinterpret_cast<volatile uint16_t*>(&wbuf[i])); }
		WriteProxy& operator=(uint8_t v) { wbuf[i] = v; return *this; }
		WriteProxy& operator=(uint16_t v) { *reinterpret_cast<volatile uint16_t*>(&wbuf[i]) = __builtin_bswap16(v); return *this; }
		[[nodiscard]] volatile uint8_t* address(uint16_t index) const { return wbuf + index; }
		WriteProxy &operator&=(uint8_t v) { wbuf[i] &= v; return *this; }
	};

public:
	static registers& getInstance() { if (!instance) { instance = new registers(); } return *instance; }
	registers(const registers&) = delete;
	registers& operator=(const registers&) = delete;
	ReadProxy operator[](uint16_t i) const { return ReadProxy{static_cast<uint16_t>(i&(register_length - 1))}; }
	static WriteProxy write(uint16_t i) { return WriteProxy{static_cast<uint16_t>(i&(register_length - 1)), write_registers}; }
	static void clear() { for (uint16_t i = 0; i < register_length; i++) read_registers[i] = write_registers[i] = 0u; }
	static void copy_in(uint16_t addr, const uint8_t *src, uint16_t len);
	static void copy_out(uint8_t *dst, uint16_t addr, uint16_t len);
	static void copy_out_write(uint8_t *dst, uint16_t addr, uint16_t len);
	static uintptr_t read_address() { return reinterpret_cast<uintptr_t>(read_registers); }
	static uintptr_t write_address() { return reinterpret_cast<uintptr_t>(write_registers); }
};

extern registers &reg;

// Chunks are like snippets, except they are too big for snippet space, so they go into RAM
#define	CHUNK_ADDRESS	uint16_t(0x0130u)
// === Device addresses ===
#define REGISTER_BASE	uint16_t(0xFFC0u)

#define ADDRESS(index)	uint16_t(REGISTER_BASE + index)

#define REGISTER_DEVICES_OFFSET uint16_t(0)
#define REGISTER_DEVICES uint16_t(REGISTER_BASE + REGISTER_DEVICES_OFFSET)
#define REGISTER_BUFFER_OFFSET uint16_t(16)
#define REGISTER_BUFFER uint16_t(REGISTER_BASE + REGISTER_BUFFER_OFFSET)
#define REGISTER_SNIPPET_OFFSET uint16_t(32)
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

// System registers relative to REGISTER_DEVICES
#define SYSTEM_TASK		uint16_t(0x00u)

#define SYSTEM_REQUEST		uint16_t(0x01u) // Write-only
#define SYSTEM_REQUEST_STATUS	uint16_t(0x01u) // Read-only
#define SYSTEM_SUBREQUEST	uint16_t(0x02u) // Write-only
#define SYSTEM_REQUEST_RESPONSE	uint16_t(0x02u) // Read-only

// Try to emulate an MC6840 PTM, within reason. The output of
// Timer 1 is connected to NMI.
#define SYSTEM_TIMER_BASE	uint16_t(0x08u)
#define SYSTEM_TIMER_CONTROL_13	uint16_t(0x08u) // Write-only
#define SYSTEM_TIMER_CONTROL_2	uint16_t(0x09u) // Write-only
#define SYSTEM_TIMER_STATUS	uint16_t(0x09u) // Reade-only
#define SYSTEM_TIMER_1_MSB	uint16_t(0x0Au) // Read/Write
#define SYSTEM_TIMER_1_LSB	uint16_t(0x0Bu) // Read/Write
#define SYSTEM_TIMER_2_MSB	uint16_t(0x0Cu) // Read/Write
#define SYSTEM_TIMER_2_LSB	uint16_t(0x0Du) // Read/Write
#define SYSTEM_TIMER_3_MSB	uint16_t(0x0Eu) // Read/Write
#define SYSTEM_TIMER_3_LSB	uint16_t(0x0Fu) // Read/Write

// Task register bits
#define NUM_TASK_PINS		5
#define	TASK_PINS_MASK		0b00011111u

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

// System timer control/status bits
//#define TIMER_IRQ		uint8_t(0b00000001u)
//#define TIMER_ENABLED		uint8_t(0b00000010u)
//#define TIMER_REPEAT		uint8_t(0b00000100u)

// Emulated UART
// UART registers relative to REGISTER_DEVICES
#define CONSOLE_CONTROL		uint16_t(0x03u)
#define CONSOLE_STATUS		uint16_t(0x03u)
#define CONSOLE_TX_DATA		uint16_t(0x04u)
#define CONSOLE_RX_DATA		uint16_t(0x04u)

//UART miscellaneous constants

#endif // PICO_THING_H
