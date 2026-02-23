/**
* Copyright (c) 2025 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <cstdint>
#include <cstring>
#include <utility>

#include "pico/stdlib.h"

#include "hardware/pio.h"

#include "registers.h"
#include "pico_thing.h"
#include "mc6809.h"
#include "mc6850.h"

mc6850::mc6850() :
	tx(sizeof(uint8_t), CONSOLE_QUEUE_LEN),
	rx(sizeof(uint8_t), CONSOLE_QUEUE_LEN),
	reg(registers::getInstance())
{
	registers::write(CONSOLE_CONTROL) = CONSOLE_NONE;
	reg[CONSOLE_STATUS] = reset_status.byte;
	registers::write(CONSOLE_TX_DATA) = static_cast<uint8_t>(0x00);
	reg[CONSOLE_RX_DATA] = static_cast<uint8_t>(0x00);
}

void
mc6850::reset()
{
	registers::write(CONSOLE_CONTROL) = CONSOLE_NONE;
	reg[CONSOLE_STATUS] = CONSOLE_NONE;
	registers::write(CONSOLE_TX_DATA) = static_cast<uint8_t>(0x00);
	reg[CONSOLE_RX_DATA] = static_cast<uint8_t>(0x00);
	transmit_buffer_empty = true;
	receive_buffer_full = false;
	transmit_irq = false;
	receive_irq = false;
	assert_transmit_irq = false;
	assert_receive_irq = false;
	cts_deasserted = false;
	//tx.reset();
	//rx.reset();
	reg[CONSOLE_STATUS] = reset_status.byte;
}

// Sync TDRE/CTS status bits from current TX queue state.
inline void
mc6850::sync_transmit()
{
	transmit_buffer_empty = tx.has_space();
	cts_deasserted = tx.get_level() > CTS_THRESHOLD;
	mc6850_status sr = {reg[CONSOLE_STATUS]};
	sr.cts  = cts_deasserted ? 1u : 0u;
	sr.tdre = (transmit_buffer_empty && !cts_deasserted) ? 1u : 0u;
	reg[CONSOLE_STATUS] = sr.byte;
}

// Called periodically from the Core 1 timer. Keeps TDRE/CTS current for
// the 6809's TX path. Receive loading and interrupt state are handled by
// has_interrupt() on every Q-rising edge.
void
mc6850::update_task()
{
	sync_transmit();
}

void
mc6850::guest_control()
{
	mc6850_control cr = {registers::write(CONSOLE_CONTROL)};
	if (cr.divide_select == 0b11u) {
		reset();
	} else {
		transmit_irq = cr.tx_irq == 0b01u;
		receive_irq = cr.rx_irq;
		sync_transmit();
	}
}

void
mc6850::guest_transmit()
{
	if (tx.has_space())
		tx.put(registers::write(CONSOLE_TX_DATA));
	sync_transmit();
}

void
mc6850::guest_receive()
{
	receive_buffer_full = rx.has_bytes();
	if (receive_buffer_full)
		reg[CONSOLE_RX_DATA] = rx.get();
	mc6850_status sr = {reg[CONSOLE_STATUS]};
	sr.rdrf = receive_buffer_full;
	reg[CONSOLE_STATUS] = sr.byte;
}

interrupt
mc6850::has_interrupt()
{
	// Stage the next byte if the slot is empty. Called every Q-rising edge,
	// so RDRF is current before apply_interrupts() drives /IRQ. There is an
	// inherent one-cycle lag (the DMA already served the previous value for
	// this cycle), but that is harmless: the 6809 sees the updated RDRF on
	// the very next UARTS read.
	if (!receive_buffer_full && rx.has_bytes()) {
		receive_buffer_full = true;
		reg[CONSOLE_RX_DATA] = rx.get();
		mc6850_status sr = {reg[CONSOLE_STATUS]};
		sr.rdrf = 1;
		reg[CONSOLE_STATUS] = sr.byte;
	}
	assert_receive_irq = receive_irq && receive_buffer_full;
	assert_transmit_irq = transmit_irq && transmit_buffer_empty;
	mc6850_status sr = {reg[CONSOLE_STATUS]};
	sr.irq = assert_receive_irq || assert_transmit_irq;
	reg[CONSOLE_STATUS] = sr.byte;
	if (assert_transmit_irq || assert_receive_irq)
		return INTERRUPT_IRQ;
	return INTERRUPT_NONE;
}
