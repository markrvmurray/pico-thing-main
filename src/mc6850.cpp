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

#include "pico_thing.h"
#include "processor.h"
#include "mc6850.h"

mc6850::mc6850() :
	tx(sizeof(uint8_t), CONSOLE_QUEUE_LEN),
	rx(sizeof(uint8_t), CONSOLE_QUEUE_LEN),
	reg(registers::getInstance())
{
	registers::write(CONSOLE_CONTROL) = CONSOLE_NONE;
	reg[CONSOLE_STATUS] = reset_status.byte;
}

void
mc6850::reset()
{
	if (busy)
		return;
	busy = true;
	registers::write(CONSOLE_CONTROL) = CONSOLE_NONE;
	reg[CONSOLE_STATUS] = CONSOLE_NONE;
	transmit_buffer_empty = false;
	receive_buffer_full = false;
	transmit_irq = false;
	receive_irq = false;
	tx.reset();
	rx.reset();
	reg[CONSOLE_STATUS] = reset_status.byte;
	busy = false;
}

inline void
mc6850::update_transmit(bool do_transmit)
{
	if (busy)
		return;
	// The byte just arrived, so deal with it quickly
	if (do_transmit) {
		if (tx.has_space())
			tx.put(registers::write(CONSOLE_TX_DATA));
	}
	transmit_buffer_empty = tx.has_space();
	Status sr = {reg[CONSOLE_STATUS]};
	sr.tdre = transmit_buffer_empty;
	reg[CONSOLE_STATUS] = sr.byte;
}

inline void
mc6850::update_receive(bool do_receive)
{
	if (busy)
		return;
	if (do_receive) {
		// in this case, the guest just read the byte, so we can replace it
		receive_buffer_full = rx.has_bytes();
		if (receive_buffer_full)
			reg[CONSOLE_RX_DATA] = rx.get();
	} else {
		// Otherwise, we are polling for incoming bytes from the Rx queue,
		// but only if there isn't already a byte waiting for the guest
		if (!receive_buffer_full) {
			receive_buffer_full = rx.has_bytes();
			if (receive_buffer_full)
				reg[CONSOLE_RX_DATA] = rx.get();
		}
	}
	Status sr = {reg[CONSOLE_STATUS]};
	sr.rdrf = receive_buffer_full;
	reg[CONSOLE_STATUS] = sr.byte;
}

inline void
mc6850::update_interrupt()
{
	if (busy)
		return;
	Control cr = {registers::write(CONSOLE_CONTROL)};
	transmit_irq = cr.tx_irq == 0b01u;
	receive_irq = cr.rx_irq;
	assert_receive_irq = receive_irq && receive_buffer_full;
	assert_transmit_irq = transmit_irq && transmit_buffer_empty;
	Status sr = {reg[CONSOLE_STATUS] };
	sr.irq = assert_receive_irq || assert_transmit_irq;
	reg[CONSOLE_STATUS] = sr.byte;
}

void
mc6850::guest_status()
{
	update_transmit(false);
	update_receive(false);
	update_interrupt();
}

void
mc6850::task()
{
	update_transmit(false);
	update_receive(false);
	update_interrupt();
}

void
mc6850::guest_control()
{
	Control cr = {registers::write(CONSOLE_CONTROL)};
	printf("G in  %02X %02X   T: %d %d %d     R: %d %d %d\n", uint8_t(registers::write(CONSOLE_CONTROL)), uint8_t(reg[CONSOLE_STATUS]), assert_transmit_irq, transmit_irq, transmit_buffer_empty, assert_receive_irq, receive_irq, receive_buffer_full);
	if (cr.divide_select == 0b11u)
		reset();
	else {
		update_transmit(false);
		update_receive(false);
		update_interrupt();
	}
	printf("G out %02X %02X   T: %d %d %d     R: %d %d %d\n", uint8_t(registers::write(CONSOLE_CONTROL)), uint8_t(reg[CONSOLE_STATUS]), assert_transmit_irq, transmit_irq, transmit_buffer_empty, assert_receive_irq, receive_irq, receive_buffer_full);
}

void
mc6850::guest_transmit()
{
	update_transmit();
	update_interrupt();
}

void
mc6850::guest_receive()
{
	update_receive();
	update_interrupt();
}

uint
mc6850::host_transmit_level_avail()
{
	return CONSOLE_QUEUE_LEN - rx.get_level();
}

uint
mc6850::transmit_level()
{
	return rx.get_level();
}

uint
mc6850::host_receive_level_avail()
{
	return tx.get_level();
}

uint
mc6850::receive_level()
{
	return tx.get_level();
}

bool
mc6850::host_transmit_avail()
{
	return !rx.is_full();
}

bool
mc6850::host_receive_avail()
{
	return !tx.is_empty();
}

uint8_t
mc6850::host_receive()
{
	return tx.get();
}

void
mc6850::host_transmit(const uint8_t ch)
{
	rx.put(ch);
}

interrupt
mc6850::has_interrupt() const
{
	if (busy)
		return INTERRUPT_NONE;
	if (assert_transmit_irq || assert_receive_irq)
		return INTERRUPT_IRQ;
	return INTERRUPT_NONE;
}
