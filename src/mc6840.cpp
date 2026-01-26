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

#include "pico_thing.h"
#include "processor.h"
#include "mc6840.h"

mc6840::mc6840(registers &reg, uint16_t interval)
	: reg(reg), interval(interval)
{
	hard_reset();
}

void
mc6840::hard_reset()
{
	busy = true;
	for (uint i = 0; i < 3; i++) {
		control[i] = 0x00;
		counter[i].whole = 0x0000;
		ctr_latch[i].whole = 0xFFFF;
		running[i] = false;
	}
	status = 0x00;
	cycles = 0;
	// I'm simulating the output of the first timer being
	// wired to !NMI through an inverter.
	nmi_pending = false;
	busy = false;
}

void
mc6840::soft_reset()
{
	status = 0x00;
	for (uint i = 0; i < 3; i++)
		initialise(i);
	cycles = 0;
}

void
mc6840::initialise(uint ctr)
{
	assert(ctr < 3 && "Invalid MC6840 counter");

	counter[ctr].whole = ctr_latch[ctr].whole;
	reg[SYSTEM_TIMER_1_MSB + ctr*2] = ctr_latch[ctr].whole;
	running[ctr] = true;
	// I'm simulating the output of the first timer being
	// wired to !NMI through an inverter.
	if (ctr == 0)
		nmi_pending = false;
}

void
mc6840::tick(uint8_t ticks)
{
	cycles += ticks;
	if (cycles < interval)
		return;
	if (control[2] & RESET)
		return;
	for (uint i = 0; i < 3; i++) {
		if (running[i]) {
			if ((control[i] & MODE_MASK) == SINGLE_0 && (control[i] & E_CLK)) {
				if (control[i] & BITS8) {
					if (counter[i].half[LOWER] != 0x00u)
						counter[i].half[LOWER]--;
					else {
						if (counter[i].half[UPPER] != 0x00u)
							counter[i].half[UPPER]--;
						else {
							if ((control[i] & IRQ_EN) == IRQ_EN)
								status |= (0b00000001u << i);
							running[i] = false;
							// I'm simulating the output of the first timer being
							// wired to !NMI through an inverter.
							if (i == 0)
								nmi_pending = true;
						}
					}
					reg[SYSTEM_TIMER_1_MSB + i*2] = ctr_latch[i].whole;
				}
			}
		}
	}
	cycles = 0;
	if (status & 0b00000111u)
		status |= IRQ_ANY;
	else
		status = 0x00u;
}

void
mc6840::write(uint16_t offset, uint8_t val)
{
	assert(offset < 8 && "Invalid MC6840 register write offset");

	switch (offset) {
	default:
		break;
	case SYSTEM_TIMER_CONTROL_13:
		if (control[1] & CR1) {
			control[0] = val;
			if (control[0] & RESET)
				soft_reset();
		} else
			control[2] = val;
		break;
	case SYSTEM_TIMER_CONTROL_2:
		control[1] = val;
		break;
	case SYSTEM_TIMER_1_MSB:
		ctr_latch[0].half[UPPER] = val;
		break;
	case SYSTEM_TIMER_1_LSB:
		ctr_latch[0].half[LOWER] = val;
		initialise(0);
		break;
	case SYSTEM_TIMER_2_MSB:
		ctr_latch[1].half[UPPER] = val;
		break;
	case SYSTEM_TIMER_2_LSB:
		ctr_latch[1].half[LOWER] = val;
		initialise(1);
		break;
	case SYSTEM_TIMER_3_MSB:
		ctr_latch[2].half[UPPER] = val;
		break;
	case SYSTEM_TIMER_3_LSB:
		ctr_latch[2].half[LOWER] = val;
		initialise(2);
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
	return status & IRQ_ANY ? INTERRUPT_IRQ : INTERRUPT_NONE;
}