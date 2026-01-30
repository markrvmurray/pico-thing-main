/**
* Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#define UPPER 0
#define LOWER 1

union mc6840_counter {
	uint16_t word;
	uint8_t	byte[2];
};

union mc6840_control {
	uint8_t byte;
	struct {
		uint8_t bit0 : 1;
		uint8_t e_clk : 1;
		uint8_t bits_8 : 1;
		uint8_t mode : 3;
		uint8_t irq_en : 1;
		uint8_t out_en : 1;
	};
};

union mc6840_status {
	uint8_t byte;
	struct {
		union {
			struct {
				uint8_t irq1 : 1;
				uint8_t irq2 : 1;
				uint8_t irq3 : 1;
			};
			uint8_t irqs : 3;
		};
		uint8_t : 4;
		uint8_t irq : 1;
	};
};

enum {CTR1, CTR2, CTR3};
enum {CONT_0, FREQ_COMP_0, CONT_1, PW_COMP_0, SINGLE_0, FREQ_COMP_1, SINGLE_1, PW_COMP_1};

class mc6840 {
	registers &reg;

	mc6840_control control[3] = {0x00u, 0x00u, 0x00u};
	mc6840_status status = {0x00u};
	mc6840_counter counter[3] = {0x0000u, 0x0000u, 0x0000u};
	mc6840_counter ctr_latch[3] = {0xFFFFu, 0xFFFFu, 0xFFFFu};
	bool running[3] = {false, false, false};

	uint16_t interval;	// how often to poll
	uint16_t cycles = 0u;	// cycles since last poll

	bool nmi_pending = false;

	void hard_reset();
	void soft_reset();
	void initialise(uint ctr);

public:
	mc6840(registers &reg, uint16_t interval);
	~mc6840() = default;

	// TODO: MarkM: Implement read() as it is used to clear interrupt bits. See datasheet, page 7
	// uint8_t read(uint16_t offset);
	void write(uint16_t offset, uint8_t val);
	[[nodiscard]] interrupt has_interrupt();
	void tick(uint8_t);
};
