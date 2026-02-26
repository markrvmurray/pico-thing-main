/**
* Copyright (c) 2025 Mark R V Murray.
	*
	* SPDX-License-Identifier: BSD-2-Clause
	*/

#pragma once

#include "pico/util/queue.h"

#define CONSOLE_QUEUE_LEN	256

// UART register bits
#define CONSOLE_NONE		uint8_t(0b00000000u)

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
#ifdef NOT_NOW
		uint8_t dummy;
		while (!queue_is_empty(&queue))
			queue_try_remove(&queue, &dummy);
#endif
	}
	bool is_empty() { return queue_is_empty(&queue); }
	bool has_bytes() { return !queue_is_empty(&queue); }
	bool is_full() { return queue_is_full(&queue); }
	bool has_space() { return !queue_is_full(&queue); }
	uint16_t get_level() { return queue_get_level(&queue); }
	uint8_t get() { uint8_t ch; queue_try_remove(&queue, &ch); return ch; }
	void put(const uint8_t ch) { queue_try_add(&queue, &ch); }
};

union mc6850_control {
	uint8_t byte;
	struct {
		uint8_t divide_select : 2;
		uint8_t word_select : 3;
		uint8_t tx_irq : 2;
		uint8_t rx_irq : 1;
	};
};

union mc6850_status {
	uint8_t byte;
	struct {
		uint8_t rdrf : 1;
		uint8_t tdre : 1;
		uint8_t dcd : 1;
		uint8_t cts : 1;
		uint8_t fe : 1;
		uint8_t ovrn : 1;
		uint8_t pe : 1;
		uint8_t irq : 1;
	};
};

class mc6850 {
	Queue tx;
	Queue rx;
	registers &reg;
	static constexpr uint CTS_THRESHOLD = 200u;  // out of CONSOLE_QUEUE_LEN=256
	const mc6850_status reset_status = { .tdre = 1 };
	bool transmit_buffer_empty = true;
	bool receive_buffer_full = false;
	bool transmit_irq = false;
	bool receive_irq = false;
	bool assert_transmit_irq = false;
	bool assert_receive_irq = false;
	bool cts_deasserted = false;
	bool rts_deasserted = false;
	volatile bool tx_write_pending = false;	// set by write ISR, cleared by guest_transmit()
	void sync_transmit();
public:
	mc6850();
	void reset();
	void write_received();		// called from Core 1 write ISR: clears TDRE until guest_transmit() runs
	void update_task();
	uint host_transmit_level_avail() { return CONSOLE_QUEUE_LEN - rx.get_level(); }
	uint transmit_level() { return rx.get_level(); }
	uint host_receive_level_avail() { return tx.get_level(); }
	uint receive_level() { return tx.get_level(); }
	bool host_transmit_avail() { return !rx.is_full(); }
	bool host_receive_avail() { return !tx.is_empty(); }
	uint8_t host_receive() { return tx.get(); }
	void host_transmit(const uint8_t ch) { rx.put(ch); }

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
	void guest_transmit();
	void guest_control();
	[[nodiscard]] interrupt has_interrupt();
};

extern mc6850 fast_serial;
