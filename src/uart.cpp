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

#include "mc6809.h"
#include "processor.h"
#include "uart.h"

emulated_uart::emulated_uart() :
	tx(sizeof(uint8_t), CONSOLE_QUEUE_LEN),
	rx(sizeof(uint8_t), CONSOLE_QUEUE_LEN),
	reg(registers::getInstance())
{
	registers::write(CONSOLE_CONTROL) = CONSOLE_NONE;
	reg[CONSOLE_STATUS] = CONSOLE_S_TX_EMPTY;
}

void
emulated_uart::reset()
{
	transmit_buffer_empty = true;
	receive_buffer_full = false;
	transmit_irq = false;
	receive_irq = false;
	tx.reset();
	rx.reset();
	registers::write(CONSOLE_CONTROL) = CONSOLE_NONE;
	reg[CONSOLE_STATUS] = CONSOLE_S_TX_EMPTY;
}

inline void
emulated_uart::update_transmit(bool do_transmit)
{
	// The byte just arrived, so deal with it quickly
	if (do_transmit) {
		if (tx.has_space())
			tx.put(registers::write(CONSOLE_TX_DATA));
	}
	transmit_buffer_empty = tx.has_space();
	if (transmit_buffer_empty)
		assert_transmit_irq = transmit_irq;
	else
		assert_transmit_irq = false;
	reg[CONSOLE_STATUS] = static_cast<uint8_t>((reg[CONSOLE_STATUS] & static_cast<uint8_t>(~(CONSOLE_S_TX_IRQ | CONSOLE_S_TX_EMPTY))) |
		(transmit_buffer_empty ? CONSOLE_S_TX_EMPTY : CONSOLE_NONE) |
		(assert_transmit_irq ? CONSOLE_S_TX_IRQ : CONSOLE_NONE));
}

inline void
emulated_uart::update_receive(bool do_receive)
{
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
	assert_receive_irq = receive_buffer_full ? receive_irq : false;
	reg[CONSOLE_STATUS] = static_cast<uint8_t>((reg[CONSOLE_STATUS] & static_cast<uint8_t>(~(CONSOLE_S_RX_IRQ | CONSOLE_S_RX_FULL))) |
		(receive_buffer_full ? CONSOLE_S_RX_FULL : CONSOLE_NONE) |
		(assert_receive_irq ? CONSOLE_S_RX_IRQ : CONSOLE_NONE));
}

void
emulated_uart::guest_status()
{
	if (reg[CONSOLE_STATUS] & CONSOLE_S_BUSY)
		return;
	update_transmit(false);
	update_receive(false);
}

void
emulated_uart::task()
{
	if (registers::write(CONSOLE_CONTROL) & CONSOLE_C_RESET) {
		registers::write(CONSOLE_CONTROL) = CONSOLE_NONE;
		reg[CONSOLE_STATUS] = CONSOLE_S_BUSY;
	} else if (reg[CONSOLE_STATUS] & CONSOLE_S_BUSY)
		reset();
	else
		guest_status();
}

void
emulated_uart::guest_control()
{
	if (registers::write(CONSOLE_CONTROL) & CONSOLE_C_RESET) {
		//registers::write(CONSOLE_CONTROL) = CONSOLE_NONE;
		reg[CONSOLE_STATUS] = CONSOLE_S_BUSY;
	} else {
		uint8_t cont = registers::write(CONSOLE_CONTROL);
		transmit_irq = cont & CONSOLE_C_TX_IRQ;
		receive_irq = cont & CONSOLE_C_RX_IRQ;
	}
}

void
emulated_uart::guest_transmit()
{
	if (reg[CONSOLE_STATUS] & CONSOLE_S_BUSY)
		return;
	update_transmit();
}

void
emulated_uart::guest_receive()
{
	if (reg[CONSOLE_STATUS] & CONSOLE_S_BUSY)
		return;
	update_receive();
}

uint
emulated_uart::host_transmit_level_avail()
{
	return CONSOLE_QUEUE_LEN - rx.get_level();
}

uint
emulated_uart::transmit_level()
{
	return rx.get_level();
}

uint
emulated_uart::host_receive_level_avail()
{
	return tx.get_level();
}

uint
emulated_uart::receive_level()
{
	return tx.get_level();
}

bool
emulated_uart::host_transmit_avail()
{
	return !rx.is_full();
}

bool
emulated_uart::host_receive_avail()
{
	return !tx.is_empty();
}

uint8_t
emulated_uart::host_receive()
{
	return tx.get();
}

void
emulated_uart::host_transmit(const uint8_t ch)
{
	rx.put(ch);
}

interrupt
emulated_uart::has_interrupt() const
{
	if (assert_transmit_irq || assert_receive_irq)
		return INTERRUPT_IRQ;
	return INTERRUPT_NONE;
}
