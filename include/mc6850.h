/**
* Copyright (c) 2025 Mark R V Murray.
	*
	* SPDX-License-Identifier: BSD-2-Clause
	*/

#ifndef PICO_MC6809_MC6850_H
#define PICO_MC6809_MC6850_H

#include "pico/util/queue.h"

#define CONSOLE_QUEUE_LEN	256

// UART register bits
#define CONSOLE_NONE		uint8_t(0b00000000u)
// Control (write)
#define CONSOLE_C_DIVIDE_0	uint8_t(0b00000001u)
#define CONSOLE_C_DIVIDE_1	uint8_t(0b00000010u)
#define CONSOLE_C_WORD_0	uint8_t(0b00000100u)
#define CONSOLE_C_WORD_1	uint8_t(0b00001000u)
#define CONSOLE_C_WORD_2	uint8_t(0b00010000u)
#define CONSOLE_C_TX_0		uint8_t(0b00100000u)
#define CONSOLE_C_TX_1		uint8_t(0b01000000u)
#define CONSOLE_C_RX		uint8_t(0b10000000u)

// Status (read)
#define CONSOLE_S_RDRF		uint8_t(0b00000001u) // Rx Data Reg Full
#define CONSOLE_S_TDRE		uint8_t(0b00000010u) // Tx Data Reg Empty
#define CONSOLE_S_DCD		uint8_t(0b00000100u) // DCD
#define CONSOLE_S_CTS		uint8_t(0b00001000u) // CTS
#define CONSOLE_S_FE		uint8_t(0b00010000u) // FE
#define CONSOLE_S_OVRN		uint8_t(0b00100000u) // OVRN
#define CONSOLE_S_PE		uint8_t(0b01000000u) // PE
#define CONSOLE_S_IRQ		uint8_t(0b10000000u) // IRQ

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

union Control {
	uint8_t byte;
	struct {
		uint8_t divide_select : 2;
		uint8_t word_select : 3;
		uint8_t tx_irq : 2;
		uint8_t rx_irq : 1;
	};
};

union Status {
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
	volatile bool busy = true; // Constructor will reset this when done
	const Status reset_status = { .tdre = 1 };
	bool transmit_buffer_empty = true;
	bool receive_buffer_full = false;
	bool transmit_irq = false;
	bool receive_irq = false;
	bool assert_transmit_irq = false;
	bool assert_receive_irq = false;
	void update_receive(bool do_receive = true);
	void update_transmit(bool do_transmit = true);
	void update_interrupt();
public:
	mc6850();
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

extern mc6850 fast_serial;

#endif // PICO_MC6809_MC6850_H
