/**
* Copyright (c) 2025 Mark R V Murray.
	*
	* SPDX-License-Identifier: BSD-2-Clause
	*/

#ifndef PICO_EXAMPLES_PIO_MC6809_UART_H
#define PICO_EXAMPLES_PIO_MC6809_UART_H

#include "pico/util/queue.h"

#define CONSOLE_QUEUE_LEN	256

// Wrapper for C-only queue_t type.
class Queue {
	queue_t queue{};
	size_t element_size;
	size_t length;
public:
	explicit Queue(size_t elt_size = sizeof(uint8_t), size_t len = CONSOLE_QUEUE_LEN)
	{
		element_size = elt_size;
		length = len;
		queue_init(&queue, element_size, length);
	}
	void reset()
	{
		uint8_t dummy;
		while (!queue_is_empty(&queue))
			queue_try_remove(&queue, &dummy);
	}
	bool is_empty() { return queue_is_empty(&queue); }
	bool has_bytes() { return !queue_is_empty(&queue); }
	bool is_full() { return queue_is_full(&queue); }
	bool has_space() { return !queue_is_full(&queue); }
	uint16_t get_level() { return queue_get_level(&queue); }
	uint8_t get() { uint8_t ch; queue_try_remove(&queue, &ch); return ch; }
	void put(const uint8_t ch) { queue_try_add(&queue, &ch); }
};

class emulated_uart {
	Queue tx;
	Queue rx;
	registers &reg;
	bool transmit_buffer_empty = true;
	bool receive_buffer_full = false;
	bool transmit_irq = false;
	bool receive_irq = false;
	bool assert_transmit_irq = false;
	bool assert_receive_irq = false;
	void update_receive(bool do_receive = true);
	void update_transmit(bool do_transmit = true);
public:
	emulated_uart();
	void reset();
	void task();
	uint transmit_level();
	uint host_transmit_level_avail();
	uint host_receive_level_avail();
	uint receive_level();
	bool host_transmit_avail();
	bool host_receive_avail();
	uint8_t host_receive();
	void host_transmit(uint8_t ch);
	[[nodiscard]] uint8_t interrupt_status() const
	{
		return	(transmit_buffer_empty ? 0b00000001 : 0b00000000) |
			(transmit_irq ? 0b00000010 : 0b00000000) |
			(assert_transmit_irq ? 0b00000100 : 0b00000000) |
			(receive_buffer_full ? 0b00010000 : 0b00000000) |
			(receive_irq ? 0b00100000 : 0b00000000) |
			(assert_receive_irq ? 0b01000000 : 0b00000000);
	}
	void guest_receive();
	void guest_status();
	void guest_transmit();
	void guest_control();
	[[nodiscard]] interrupt has_interrupt() const;
};

extern emulated_uart fast_serial;

#endif // PICO_EXAMPLES_PIO_MC6809_UART_H
