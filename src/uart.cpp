/**
* Copyright (c) 2025 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "mc6809.h"
#include "uart.h"

emulated_uart::emulated_uart()
	: tx(Queue(sizeof(uint8_t), CONSOLE_QUEUE_LEN)),
	  rx(Queue(sizeof(uint8_t), CONSOLE_QUEUE_LEN))
{
	needs_tx_reset = true;
	needs_rx_reset = true;
	tx_has_irq = false;
	rx_has_irq = false;
}

void
emulated_uart::reset(volatile uint8_t *status)
{
	needs_tx_reset = true;
	needs_rx_reset = true;
	tx_has_irq = false;
	rx_has_irq = false;
	*status = 0x00u;
}

void
emulated_uart::task(volatile uint8_t *config, volatile uint8_t *status)
{
	uint8_t conf = *config;
	uint8_t stat = *status;
	if (needs_tx_reset) {
		uint8_t ch;
		while (queue_try_remove(&tx.queue, &ch))
			continue;
		conf &= ~CONSOLE_TX_MASK;
		stat &= ~CONSOLE_TX_MASK;
	}
	if (needs_rx_reset) {
		uint8_t ch;
		while (queue_try_remove(&rx.queue, &ch))
			continue;
		conf &= ~CONSOLE_RX_MASK;
		stat &= ~CONSOLE_RX_MASK;
	}
	if (!(stat & CONSOLE_TX_ERROR))
		stat = (stat & ~CONSOLE_TX_AVAIL) | (queue_is_full(&tx.queue) ? CONSOLE_NONE : CONSOLE_TX_AVAIL);
	if (!(stat & CONSOLE_RX_ERROR))
		stat = (stat & ~CONSOLE_RX_AVAIL) | (queue_is_empty(&rx.queue) ? CONSOLE_NONE : CONSOLE_RX_AVAIL);
	tx_has_irq = (stat & (CONSOLE_TX_AVAIL | CONSOLE_TX_IRQ)) == (CONSOLE_TX_AVAIL | CONSOLE_TX_IRQ);
	rx_has_irq = (stat & (CONSOLE_RX_AVAIL | CONSOLE_RX_IRQ)) == (CONSOLE_RX_AVAIL | CONSOLE_RX_IRQ);
	*config = conf;
	*status = stat;
}

uint
emulated_uart::send_level()
{
	return queue_get_level(&rx.queue);
}

uint
emulated_uart::recv_level()
{
	return queue_get_level(&tx.queue);
}

bool
emulated_uart::host_send_avail()
{
	return !queue_is_full(&rx.queue);
}

bool
emulated_uart::host_receive_avail()
{
	return !queue_is_empty(&tx.queue);
}

uint8_t
emulated_uart::host_receive()
{
	uint8_t ch;
	queue_try_remove(&tx.queue, &ch);
	return ch;
}

void
emulated_uart::host_send(const uint8_t ch)
{
	queue_try_add(&rx.queue, &ch);
}

uint8_t
emulated_uart::guest_receive(volatile uint8_t *status)
{
	uint8_t stat = *status, ch = 0x00u;
	if (!(stat & CONSOLE_RX_AVAIL) || (stat & CONSOLE_RX_ERROR))
		stat |= CONSOLE_RX_ERROR;
	else
		stat = (stat & ~CONSOLE_RX_AVAIL) | (queue_try_remove(&rx.queue, &ch) ? CONSOLE_RX_AVAIL : CONSOLE_NONE);
	*status = stat;
	return ch;
}

void
emulated_uart::guest_send(const uint8_t ch, volatile uint8_t *status)
{
	uint8_t stat = *status;
	if (!(stat & CONSOLE_TX_AVAIL) || (stat & CONSOLE_TX_ERROR))
		stat |= CONSOLE_TX_ERROR;
	else {
		if (queue_is_full(&tx.queue))
			stat |= CONSOLE_TX_ERROR;
		else {
			if (queue_try_add(&tx.queue, &ch))
				stat &= ~(CONSOLE_TX_IRQ | CONSOLE_TX_AVAIL);
		}
	}
	*status = stat;
}

uint8_t
emulated_uart::guest_control(uint8_t written, volatile uint8_t *status)
{
	// Bits in this written byte are
	// TX_IRQ_ENABLE, 0, TX_ERROR_RESET, 0, RX_IRQ_ENABLE, 0, RX_ERROR_RESET, 0
	needs_tx_reset = static_cast<bool>(written & CONSOLE_TX_ERROR);
	if (needs_tx_reset)
		written &= ~CONSOLE_TX_IRQ;
	needs_rx_reset = static_cast<bool>(written & CONSOLE_RX_ERROR);
	if (needs_rx_reset)
		written &= ~CONSOLE_RX_IRQ;
	written &= CONSOLE_TX_IRQ | CONSOLE_RX_IRQ;
	*status = written;
	return written;
}

interrupt
emulated_uart::has_interrupt() const
{
	if (tx_has_irq || rx_has_irq)
		return INTERRUPT_IRQ;
	return INTERRUPT_NONE;
}
