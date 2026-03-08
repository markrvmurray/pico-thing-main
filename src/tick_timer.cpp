/**
 * Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "pico_thing.h"
#include "mc6809.h"
#include "registers.h"
#include "tick_timer.h"

tick_timer::tick_timer(registers &r, volatile uint8_t &pending, uint8_t pend_bit,
                       uint16_t ctrl_off, uint16_t stat_off)
	: reg(r)
	, pending_irq_flags(pending)
	, pend_irq_bit(pend_bit)
	, control_offset(ctrl_off)
	, status_offset(stat_off)
{
}

bool
tick_timer::callback(repeating_timer *rt)
{
	auto *self = static_cast<tick_timer *>(rt->user_data);
	if (self->enabled) {
		self->irq_pending = true;
		self->reg[self->status_offset] = static_cast<uint8_t>(0x80u);
	}
	return true;
}

void
tick_timer::start()
{
	add_repeating_timer_us(TICK_PERIOD_US, callback, this, &hw_timer);
}

void
tick_timer::control(uint8_t val)
{
	enabled = (val & 0x01u);
	if (!enabled) {
		irq_pending = false;
		reg[status_offset] = static_cast<uint8_t>(0x00u);
	}
}

void
tick_timer::acknowledge()
{
	irq_pending = false;
	reg[status_offset] = static_cast<uint8_t>(0x00u);
	pending_irq_flags &= ~pend_irq_bit;
}

void
tick_timer::reset()
{
	enabled = false;
	irq_pending = false;
	reg[status_offset] = static_cast<uint8_t>(0x00u);
}

interrupt
tick_timer::has_interrupt() const
{
	return irq_pending ? INTERRUPT_IRQ : INTERRUPT_NONE;
}
