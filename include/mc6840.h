/**
* Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#define UPPER 0
#define LOWER 1

union latch {
	uint8_t	half[2];
	uint16_t whole;
};

class mc6840 {
public:
	enum control_flags : uint8_t {
		DIV8	= 0x01, // CR3
		CR1	= 0x01, // CR2
		RESET	= 0x01, // CR1
		E_CLK	= 0x02,
		BITS8	= 0x04,
		MODE1	= 0x08,
		MODE2	= 0x10,
		MODE3	= 0x20,
		MODE_MASK = 0x38,
		IRQ_EN	= 0x40,
		OUT_EN	= 0x80,
	};

	enum mode : uint8_t {
		CONT_0 = 0b00000000,
		FREQ_COMP_0 = 0b00001000,
		CONT_1 = 0b00010000,
		PW_COMP_0 = 0b00011000,
		SINGLE_0 = 0b00100000,
		FREQ_COMP_1 = 0b00101000,
		SINGLE_1 = 0b00110000,
		PW_COMP_1 = 0b00111000
	};

	enum status_flags : uint8_t {
		IRQ1	= 0x01,
		IRQ2	= 0x02,
		IRQ3	= 0x04,
		IRQ_ANY	= 0x80
	};

private:
	registers &reg;

	volatile bool busy = true; // the constructor will clear this

	uint8_t control[3], status;
	latch counter[3], ctr_latch[3];
	bool running[3];

	uint16_t interval;	// how often to poll
	uint16_t cycles;	// cycles since last poll

	bool nmi_pending{};

	void hard_reset();
	void soft_reset();
	void initialise(uint ctr);

public:
	mc6840(registers &reg, uint16_t interval);
	~mc6840() = default;

	// uint8_t read(uint16_t offset);
	void write(uint16_t offset, uint8_t val);
	[[nodiscard]] interrupt has_interrupt();
	void tick(uint8_t);
};
