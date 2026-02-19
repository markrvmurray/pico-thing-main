/**
* Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <cstring>

#include "pico/aon_timer.h"
#include "pico/assert.h"
#include "pico/multicore.h"

#include "hardware/pio.h"

#include "registers.h"
#include "pico_thing.h"
#include "mc6809.h"
#include "mc6840.h"

mc6840::mc6840(registers &reg, uint16_t interval)
	: reg(reg), interval(interval)
{
	hard_reset();
}

void
mc6840::hard_reset()
{
	for (uint i = CTR1; i <= CTR3; i++) {
		control[i].byte = 0x00;
		counter[i].word = 0x0000;
		ctr_latch[i].word = 0xFFFF;
		running[i] = false;
	}
	status.byte = 0x00;
	cycles = 0;
	// I'm simulating the output of the first timer being
	// wired to !NMI through an inverter.
	nmi_pending = false;
}

void
mc6840::soft_reset()
{
	status.byte = 0x00;
	for (uint i = CTR1; i <= CTR3; i++)
		initialise(i);
	cycles = 0;
}

void
mc6840::initialise(uint ctr)
{
	assert(ctr < 3 && "Invalid MC6840 counter");

	if (control[ctr].e_clk) {
		counter[ctr].word = ctr_latch[ctr].word;
		reg[SYSTEM_TIMER_1_MSB + ctr*2] = ctr_latch[ctr].word;
		running[ctr] = true;
		// I'm simulating the output of the first timer being
		// wired to !NMI through an inverter.
		if (ctr == 0)
			nmi_pending = false;
	}
}

void
mc6840::tick(uint8_t ticks)
{
	cycles += ticks;
	if (cycles < interval)
		return;
	if (control[CTR3].bit0) // RESET
		return;
	for (uint i = CTR1; i <= CTR3; i++) {
		if (running[i]) {
			if (control[i].mode == SINGLE_0) {
				if (control[i].bits_8) {
					if (counter[i].byte[LOWER] != 0x00u)
						counter[i].byte[LOWER]--;
					else {
						// I'm simulating the output of the first timer being
						// wired to !NMI through an inverter.
						if (i == CTR1)
							nmi_pending = true;
						if (counter[i].byte[UPPER] != 0x00u)
							counter[i].byte[UPPER]--;
						else {
							// I'm simulating the output of the first timer being
							// wired to !NMI through an inverter.
							if (i == CTR1)
								nmi_pending = false;
							running[i] = false;
							if (control[i].irq_en)
								status.irqs |= (0b00000001u << i);
						}
					}
					reg[SYSTEM_TIMER_1_MSB + i*2] = counter[i].word;
				}
			}
		}
	}
	cycles = 0;
	status.irq = static_cast<bool>(status.irqs);
	reg[SYSTEM_TIMER_STATUS] = status.byte;
}

void
mc6840::write(uint16_t offset, uint8_t val)
{
	assert(offset < 8 && "Invalid MC6840 register write offset");

	switch (offset) {
	default:
		break;
	case SYSTEM_TIMER_CONTROL_13:
		if (control[CTR2].bit0) { // CR1
			control[CTR1].byte = val;
			if (control[CTR1].bit0) // RESET
				soft_reset();
		} else
			control[CTR3].byte = val;
		break;
	case SYSTEM_TIMER_CONTROL_2:
		control[CTR2].byte = val;
		break;
	case SYSTEM_TIMER_1_MSB:
		ctr_latch[CTR1].byte[UPPER] = val;
		break;
	case SYSTEM_TIMER_1_LSB:
		ctr_latch[CTR1].byte[LOWER] = val;
		initialise(CTR1);
		break;
	case SYSTEM_TIMER_2_MSB:
		ctr_latch[CTR2].byte[UPPER] = val;
		break;
	case SYSTEM_TIMER_2_LSB:
		ctr_latch[CTR2].byte[LOWER] = val;
		initialise(CTR2);
		break;
	case SYSTEM_TIMER_3_MSB:
		ctr_latch[CTR3].byte[UPPER] = val;
		break;
	case SYSTEM_TIMER_3_LSB:
		ctr_latch[CTR3].byte[LOWER] = val;
		initialise(CTR3);
		break;
	}
}

interrupt
mc6840::has_interrupt()
{
	// I'm simulating the output of the first timer being
	// wired to !NMI through an inverter.
	if (nmi_pending) {
		nmi_pending = false;
		return INTERRUPT_NMI;
	}
	return status.irq ? INTERRUPT_IRQ : INTERRUPT_NONE;
}
