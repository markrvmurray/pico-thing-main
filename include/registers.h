/**
 * Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "pico.h"

// 64-byte memory-mapped registers
// There are 2 blocks, one for the Read DMA and the other for the Write DMA.
// This allows for software enforcement of read-only areas and for device
// registers to share status and control at the same address.
// The alignment is there to allow the base address to have the lower 6 bits
// all zeroes, so that the rest can be used as DMA prefixes without faffing
// around with pointer arithmetic.

class registers {
public:
	static constexpr uint16_t register_address_bits = 6u;
	static constexpr uint16_t register_length = 1u << register_address_bits;
	static constexpr uint16_t register_mask = register_length - 1u;
private:
	__scratch_y("")
	__attribute__((aligned(register_length)))
	inline static volatile uint8_t read_registers[register_length];
	__scratch_y("")
	__attribute__((aligned(register_length)))
	inline static volatile uint8_t write_registers[register_length];
	// This is a singleton class
	registers() = default; // no public constructor
	~registers() = default; // no public destructor
	struct ReadProxy {
		size_t i;
		operator uint8_t() const { return read_registers[i]; }
		explicit operator uint16_t() const { return __builtin_bswap16(*reinterpret_cast<volatile uint16_t*>(&read_registers[i])); }
		explicit operator float() const { uint32_t u = __builtin_bswap32(*reinterpret_cast<volatile uint32_t*>(&read_registers[i])); float f; std::memcpy(&f,&u,sizeof f); return f; }		ReadProxy& operator=(volatile uint8_t v){ read_registers[i] = v; return *this; }
		ReadProxy& operator=(uint16_t v) { *reinterpret_cast<volatile uint16_t*>(&read_registers[i]) = __builtin_bswap16(v); return *this; }
		ReadProxy& operator=(float v) { uint32_t u; std::memcpy(&u, &v, sizeof u); *reinterpret_cast<volatile uint32_t*>(&read_registers[i]) = __builtin_bswap32(u); return *this; }
		bool operator==(uint8_t v) const { return read_registers[i] == v; }
		bool operator!=(uint8_t v) const { return read_registers[i] != v; }
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
	static registers& getInstance() { static registers instance; return instance; }
	registers(const registers&) = delete;
	registers& operator=(const registers&) = delete;
	ReadProxy operator[](uint16_t i) const { return ReadProxy{static_cast<uint16_t>(i&(register_length - 1))}; }
	static WriteProxy write(uint16_t i) { return WriteProxy{static_cast<uint16_t>(i&(register_length - 1)), write_registers}; }
	static void clear() { for (uint16_t i = 0; i < register_length; i++) read_registers[i] = write_registers[i] = 0u; }
	static void copy_in(uint16_t addr, const uint8_t *src, uint16_t len);
	static void copy_out(uint8_t *dst, uint16_t addr, uint16_t len);
	static void copy_out_write(uint8_t *dst, uint16_t addr, uint16_t len);
	static void dump();
	static uintptr_t read_address() { return reinterpret_cast<uintptr_t>(read_registers); }
	static uintptr_t write_address() { return reinterpret_cast<uintptr_t>(write_registers); }
};

extern registers &reg;
