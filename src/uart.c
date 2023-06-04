/**
* Copyright (c) 2025 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include "pico/util/queue.h"

#include "uart.h"
#include "mc6809.h"

struct emulated_uart {
	queue_t tx;
	queue_t rx;
	bool needs_tx_reset;
	bool needs_rx_reset;
	bool tx_has_irq;
	bool rx_has_irq;
};

static struct emulated_uart console_instance;

// Done once at start of run
struct emulated_uart *
uart_initialise(volatile uint8_t *status)
{
	struct emulated_uart *console_device = &console_instance;
	queue_init(&console_device->tx, sizeof(uint8_t), CONSOLE_QUEUE_LEN);
	queue_init(&console_device->rx, sizeof(uint8_t), CONSOLE_QUEUE_LEN);
	console_device->needs_tx_reset = true;
	console_device->needs_rx_reset = true;
	console_device->tx_has_irq = false;
	console_device->rx_has_irq = false;
	*status = 0x00u;
	return console_device;
}

// Done regularly
void
uart_task(struct emulated_uart *console_device, volatile uint8_t *config, volatile uint8_t *status)
{
	uint8_t conf = *config;
	uint8_t stat = *status;
	if (console_device->needs_tx_reset) {
		uint8_t ch;
		while (queue_try_remove(&console_device->tx, &ch))
			continue;
		conf &= ~CONSOLE_TX_MASK;
		stat &= ~CONSOLE_TX_MASK;
	}
	if (console_device->needs_rx_reset) {
		uint8_t ch;
		while (queue_try_remove(&console_device->rx, &ch))
			continue;
		conf &= ~CONSOLE_RX_MASK;
		stat &= ~CONSOLE_RX_MASK;
	}
	if (!(stat & CONSOLE_TX_ERROR))
		stat = (stat & ~CONSOLE_TX_AVAIL) | (queue_is_full(&console_device->tx) ? CONSOLE_NONE : CONSOLE_TX_AVAIL);
	if (!(stat & CONSOLE_RX_ERROR))
		stat = (stat & ~CONSOLE_RX_AVAIL) | (queue_is_empty(&console_device->rx) ? CONSOLE_NONE : CONSOLE_RX_AVAIL);
	console_device->tx_has_irq = (stat & (CONSOLE_TX_AVAIL | CONSOLE_TX_IRQ)) == (CONSOLE_TX_AVAIL | CONSOLE_TX_IRQ);
	console_device->rx_has_irq = (stat & (CONSOLE_RX_AVAIL | CONSOLE_RX_IRQ)) == (CONSOLE_RX_AVAIL | CONSOLE_RX_IRQ);
	*config = conf;
	*status = stat;
}

uint
uart_send_level(struct emulated_uart *console_device)
{
	return queue_get_level(&console_device->rx);
}

uint
uart_recv_level(struct emulated_uart *console_device)
{
	return queue_get_level(&console_device->tx);
}

bool
uart_host_send_avail(struct emulated_uart *console_device)
{
	return !queue_is_full(&console_device->rx);
}

bool
uart_host_receive_avail(struct emulated_uart *console_device)
{
	return !queue_is_empty(&console_device->tx);
}

uint8_t
uart_host_receive(struct emulated_uart *console_device)
{
	uint8_t ch;
	queue_try_remove(&console_device->tx, &ch);
	return ch;
}

void
uart_host_send(struct emulated_uart *console_device, const uint8_t ch)
{
	queue_try_add(&console_device->rx, &ch);
}

uint8_t
uart_guest_receive(struct emulated_uart *console_device, volatile uint8_t *status)
{
	uint8_t stat = *status, ch = 0x00u;
	if (!(stat & CONSOLE_RX_AVAIL) || (stat & CONSOLE_RX_ERROR))
		stat |= CONSOLE_RX_ERROR;
	else
		stat = (stat & ~CONSOLE_RX_AVAIL) | (queue_try_remove(&console_device->rx, &ch) ? CONSOLE_RX_AVAIL : CONSOLE_NONE);
	*status = stat;
	return ch;
}

void
uart_guest_send(struct emulated_uart *console_device, const uint8_t ch, volatile uint8_t *status)
{
	uint8_t stat = *status;
	if (!(stat & CONSOLE_TX_AVAIL) || (stat & CONSOLE_TX_ERROR))
		stat |= CONSOLE_TX_ERROR;
	else {
		if (queue_is_full(&console_device->tx))
			stat |= CONSOLE_TX_ERROR;
		else {
			if (queue_try_add(&console_device->tx, &ch))
				stat &= ~(CONSOLE_TX_IRQ | CONSOLE_TX_AVAIL);
		}
	}
	*status = stat;
}

uint8_t
uart_guest_control(struct emulated_uart *console_device, uint8_t written, volatile uint8_t *status)
{
	// Bits in this written byte are
	// TX_IRQ_ENABLE, 0, TX_ERROR_RESET, 0, RX_IRQ_ENABLE, 0, RX_ERROR_RESET, 0
	console_device->needs_tx_reset = (bool)(written & CONSOLE_TX_ERROR);
	if (console_device->needs_tx_reset)
		written &= ~CONSOLE_TX_IRQ;
	console_device->needs_rx_reset = (bool)(written & CONSOLE_RX_ERROR);
	if (console_device->needs_rx_reset)
		written &= ~CONSOLE_RX_IRQ;
	written &= CONSOLE_TX_IRQ | CONSOLE_RX_IRQ;
	*status = written;
	return written;
}

enum interrupt
uart_has_interrupt(const struct emulated_uart *console_device)
{
	if (console_device->tx_has_irq || console_device->rx_has_irq)
		return INTERRUPT_IRQ;
	return INTERRUPT_NONE;
}