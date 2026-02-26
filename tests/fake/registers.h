#pragma once
// Host-testable drop-in for the real registers.h.
// Removes Pico-specific attributes; otherwise identical interface.

#include <cstddef>
#include <cstdint>
#include <cstring>

class registers {
public:
	static constexpr uint16_t register_address_bits = 6u;
	static constexpr uint16_t register_length = 1u << register_address_bits;
	static constexpr uint16_t register_mask = register_length - 1u;

private:
	inline static uint8_t read_registers[register_length] = {};
	inline static uint8_t write_registers[register_length] = {};

	registers() = default;
	~registers() = default;

	struct ReadProxy {
		size_t i;
		operator uint8_t() const { return read_registers[i]; }
		explicit operator uint16_t() const {
			return static_cast<uint16_t>(read_registers[i] << 8) | read_registers[i + 1];
		}
		ReadProxy& operator=(uint8_t v) { read_registers[i] = v; return *this; }
		ReadProxy& operator=(uint16_t v) {
			// big-endian, matching the real ReadProxy behaviour
			read_registers[i]     = static_cast<uint8_t>(v >> 8);
			read_registers[i + 1] = static_cast<uint8_t>(v);
			return *this;
		}
		bool operator==(uint8_t v) const { return read_registers[i] == v; }
		bool operator!=(uint8_t v) const { return read_registers[i] != v; }
		ReadProxy& operator|=(uint8_t v) { read_registers[i] |= v; return *this; }
		ReadProxy& operator&=(uint8_t v) { read_registers[i] &= v; return *this; }
	};

	struct WriteProxy {
		uint16_t i;
		uint8_t* wbuf;
		operator uint8_t() const { return wbuf[i]; }
		WriteProxy& operator=(uint8_t v) { wbuf[i] = v; return *this; }
		WriteProxy& operator=(uint16_t v) {
			wbuf[i]     = static_cast<uint8_t>(v >> 8);
			wbuf[i + 1] = static_cast<uint8_t>(v);
			return *this;
		}
		WriteProxy& operator&=(uint8_t v) { wbuf[i] &= v; return *this; }
	};

public:
	static registers& getInstance() { static registers instance; return instance; }
	registers(const registers&) = delete;
	registers& operator=(const registers&) = delete;

	ReadProxy operator[](uint16_t i) const {
		return ReadProxy{static_cast<size_t>(i & register_mask)};
	}
	static WriteProxy write(uint16_t i) {
		return WriteProxy{static_cast<uint16_t>(i & register_mask), write_registers};
	}

	// Test helpers
	static void clear() {
		memset(read_registers, 0, sizeof(read_registers));
		memset(write_registers, 0, sizeof(write_registers));
	}
	static uint8_t peek(uint16_t i) { return read_registers[i & register_mask]; }

	// Stubs for methods referenced elsewhere in the real firmware
	static void copy_in(uint16_t, const uint8_t*, uint16_t) {}
	static void copy_out(uint8_t*, uint16_t, uint16_t) {}
	static void copy_out_write(uint8_t*, uint16_t, uint16_t) {}
	static void dump() {}
	static uintptr_t read_address() { return reinterpret_cast<uintptr_t>(read_registers); }
	static uintptr_t write_address() { return reinterpret_cast<uintptr_t>(write_registers); }
};

extern registers &reg;
