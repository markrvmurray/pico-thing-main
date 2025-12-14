/**
* Copyright (c) 2025 Mark R V Murray.
	*
	* SPDX-License-Identifier: BSD-2-Clause
	*/

#ifndef PICO_EXAMPLES_PIO_MC6809_UART_H
#define PICO_EXAMPLES_PIO_MC6809_UART_H

#include "pico/util/queue.h"

// Wrapper for C-only queue_t type.
struct Queue {
	Queue(size_t elt_size, size_t len)
	{
		queue_init(&queue, elt_size, len);
	}
	queue_t queue;
};

class emulated_uart {
	Queue tx;
	Queue rx;
	bool needs_tx_reset;
	bool needs_rx_reset;
	bool tx_has_irq;
	bool rx_has_irq;
public:
	emulated_uart();
	void reset(volatile uint8_t *status);
	void task(volatile uint8_t *config, volatile uint8_t *status);
	uint send_level();
	uint recv_level();
	bool host_send_avail();
	bool host_receive_avail();
	uint8_t host_receive();
	void host_send(const uint8_t ch);
	uint8_t guest_receive(volatile uint8_t *status);
	void guest_send(const uint8_t ch, volatile uint8_t *status);
	uint8_t guest_control(uint8_t written, volatile uint8_t *status);
	interrupt has_interrupt() const;
};

#endif // PICO_EXAMPLES_PIO_MC6809_UART_H
