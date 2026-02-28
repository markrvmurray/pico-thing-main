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
		latch_active[i] = false;
	}
	status.irqs = 0;
	status.irq = 0;
	cycles = 0;
	// I'm simulating the output of the first timer being
	// wired to !NMI through an inverter.
	nmi_pending = false;
}

void
mc6840::soft_reset()
{
	status.irqs = 0;
	status.irq = 0;
	for (uint i = CTR1; i <= CTR3; i++)
		initialise(i);
	cycles = 0;
}

void
mc6840::initialise(uint ctr)
{
	assert(ctr < 3 && "Invalid MC6840 counter");

	if (control[ctr].e_clk) {
		latch_active[ctr] = false;
		counter[ctr].word = ctr_latch[ctr].word;
		reg[SYSTEM_TIMER_1_MSB + ctr*2] = static_cast<uint16_t>((ctr_latch[ctr].byte[UPPER] << 8) | ctr_latch[ctr].byte[LOWER]);
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
					// Dual 8-bit single-shot (MC6840 datasheet mode 4):
					// Phase 1 — UPPER counts the delay before NMI asserts.
					// Phase 2 — LOWER counts the NMI pulse width.
					// ASSIST09 programs $0701: 7 cycles delay, 1 cycle NMI.
					if (counter[i].byte[UPPER] != 0x00u) {
						// Phase 1: delay counting (NMI not yet asserted).
						counter[i].byte[UPPER]--;
						if (counter[i].byte[UPPER] == 0x00u && i == CTR1)
							nmi_pending = true;  // NMI asserts as UPPER hits 0
					} else {
						// Phase 2: NMI pulse counting (UPPER already at 0).
						// I'm simulating the output of the first timer being
						// wired to !NMI through an inverter.
						if (i == CTR1)
							nmi_pending = true;
						if (counter[i].byte[LOWER] != 0x00u)
							counter[i].byte[LOWER]--;
						else {
							if (i == CTR1)
								nmi_pending = false;
							running[i] = false;
							if (control[i].irq_en)
								status.irqs |= (0b00000001u << i);
						}
					}
					reg[SYSTEM_TIMER_1_MSB + i*2] = counter[i].byte[UPPER];
					if (!latch_active[i])
						reg[SYSTEM_TIMER_1_MSB + i*2 + 1] = counter[i].byte[LOWER];
				}
			} else if (control[i].mode == CONT_0 && !control[i].bits_8) {
				// 16-bit continuous: reload and fire on each underflow.
				// Bytes are big-endian (UPPER=MSB at [0], LOWER=LSB at [1]);
				// do NOT use word-- (LE arithmetic gives wrong result).
				if (counter[i].byte[LOWER]-- == 0)
					counter[i].byte[UPPER]--;
				if (counter[i].word == 0u) {
					counter[i].word = ctr_latch[i].word;
					if (control[i].irq_en)
						status.irqs |= (0b00000001u << i);
				}
				reg[SYSTEM_TIMER_1_MSB + i*2] = counter[i].byte[UPPER];
				if (!latch_active[i])
					reg[SYSTEM_TIMER_1_MSB + i*2 + 1] = counter[i].byte[LOWER];
			}
		}
	}
	cycles = 0;
	status.irq = static_cast<bool>(status.irqs);
	// Write status explicitly — avoid relying on union byte-alias of bitfields
	// which may not pack identically across all compilers.
	reg[SYSTEM_TIMER_STATUS] = static_cast<uint8_t>(
	    (status.irqs & 0x07u) | (status.irq ? 0x80u : 0x00u));
}

void
mc6840::write(uint16_t offset, uint8_t val)
{
	assert(offset < 8 && "Invalid MC6840 register write offset");

	switch (offset) {
	default:
		break;
	case SYSTEM_TIMER_CONTROL_13 - SYSTEM_TIMER_BASE:	// 0
		if (control[CTR2].bit0) { // CR1
			control[CTR1].byte = val;
			if (control[CTR1].bit0) // RESET
				soft_reset();
		} else
			control[CTR3].byte = val;
		break;
	case SYSTEM_TIMER_CONTROL_2 - SYSTEM_TIMER_BASE:	// 1
		control[CTR2].byte = val;
		break;
	case SYSTEM_TIMER_1_MSB - SYSTEM_TIMER_BASE:		// 2
		ctr_latch[CTR1].byte[UPPER] = val;
		break;
	case SYSTEM_TIMER_1_LSB - SYSTEM_TIMER_BASE:		// 3
		ctr_latch[CTR1].byte[LOWER] = val;
		initialise(CTR1);
		break;
	case SYSTEM_TIMER_2_MSB - SYSTEM_TIMER_BASE:		// 4
		ctr_latch[CTR2].byte[UPPER] = val;
		break;
	case SYSTEM_TIMER_2_LSB - SYSTEM_TIMER_BASE:		// 5
		ctr_latch[CTR2].byte[LOWER] = val;
		initialise(CTR2);
		break;
	case SYSTEM_TIMER_3_MSB - SYSTEM_TIMER_BASE:		// 6
		ctr_latch[CTR3].byte[UPPER] = val;
		break;
	case SYSTEM_TIMER_3_LSB - SYSTEM_TIMER_BASE:		// 7
		ctr_latch[CTR3].byte[LOWER] = val;
		initialise(CTR3);
		break;
	}
}

// The 6809 has already read the register RAM value (via DMA) before this
// is called.  read() can only update state for future read cycles.
//
// MSB read: latch the 16-bit counter snapshot and write the latched LSB to
//   the LSB register so the next-cycle LSB read returns a coherent pair.
// LSB read: clear that timer's IRQ flag and update the status register.
void
mc6840::read(uint16_t offset)
{
	assert(offset < 8 && "Invalid MC6840 register read offset");

	switch (offset) {
	default:
		break;
	case SYSTEM_TIMER_1_MSB - SYSTEM_TIMER_BASE:	// 2: latch counter 1 MSB
		read_latch[CTR1].word = counter[CTR1].word;
		latch_active[CTR1] = true;
		reg[SYSTEM_TIMER_1_LSB] = read_latch[CTR1].byte[LOWER];
		break;
	case SYSTEM_TIMER_1_LSB - SYSTEM_TIMER_BASE:	// 3: clear timer 1 IRQ
		latch_active[CTR1] = false;
		status.irq1 = 0;
		status.irq = static_cast<bool>(status.irqs);
		reg[SYSTEM_TIMER_STATUS] = static_cast<uint8_t>(
		    (status.irqs & 0x07u) | (status.irq ? 0x80u : 0x00u));
		break;
	case SYSTEM_TIMER_2_MSB - SYSTEM_TIMER_BASE:	// 4: latch counter 2 MSB
		read_latch[CTR2].word = counter[CTR2].word;
		latch_active[CTR2] = true;
		reg[SYSTEM_TIMER_2_LSB] = read_latch[CTR2].byte[LOWER];
		break;
	case SYSTEM_TIMER_2_LSB - SYSTEM_TIMER_BASE:	// 5: clear timer 2 IRQ
		latch_active[CTR2] = false;
		status.irq2 = 0;
		status.irq = static_cast<bool>(status.irqs);
		reg[SYSTEM_TIMER_STATUS] = static_cast<uint8_t>(
		    (status.irqs & 0x07u) | (status.irq ? 0x80u : 0x00u));
		break;
	case SYSTEM_TIMER_3_MSB - SYSTEM_TIMER_BASE:	// 6: latch counter 3 MSB
		read_latch[CTR3].word = counter[CTR3].word;
		latch_active[CTR3] = true;
		reg[SYSTEM_TIMER_3_LSB] = read_latch[CTR3].byte[LOWER];
		break;
	case SYSTEM_TIMER_3_LSB - SYSTEM_TIMER_BASE:	// 7: clear timer 3 IRQ
		latch_active[CTR3] = false;
		status.irq3 = 0;
		status.irq = static_cast<bool>(status.irqs);
		reg[SYSTEM_TIMER_STATUS] = static_cast<uint8_t>(
		    (status.irqs & 0x07u) | (status.irq ? 0x80u : 0x00u));
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
