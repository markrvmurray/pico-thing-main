/**
* Copyright (c) 2025 Mark R V Murray.
	*
	* SPDX-License-Identifier: BSD-2-Clause
	*/

#pragma once

#include <atomic>
#include <cstdint>

#define CONSOLE_QUEUE_LEN	256

// UART register bits
#define CONSOLE_NONE		uint8_t(0b00000000u)

// Lock-free SPSC ring buffer.  Power-of-2 capacity, acquire/release ordering.
// TX queue: Core 1 produces (guest_transmit), Core 0 consumes (host_receive).
// RX queue: Core 0 produces (host_transmit), Core 1 consumes (has_interrupt/guest_receive).
class SpscQueue {
	static constexpr uint16_t CAPACITY = CONSOLE_QUEUE_LEN;
	static constexpr uint16_t MASK = CAPACITY - 1;
	static_assert((CAPACITY & MASK) == 0, "CAPACITY must be a power of 2");
	uint8_t buf[CAPACITY]{};
	std::atomic<uint16_t> head{0};	// write position (producer)
	std::atomic<uint16_t> tail{0};	// read position (consumer)
public:
	bool is_empty() const { return head.load(std::memory_order_acquire) == tail.load(std::memory_order_acquire); }
	bool has_bytes() const { return !is_empty(); }
	bool is_full() const { return ((head.load(std::memory_order_acquire) + 1) & MASK) == tail.load(std::memory_order_acquire); }
	bool has_space() const { return !is_full(); }
	uint16_t get_level() const
	{
		return (head.load(std::memory_order_acquire) - tail.load(std::memory_order_acquire)) & MASK;
	}
	uint8_t get()
	{
		auto t = tail.load(std::memory_order_relaxed);
		uint8_t ch = buf[t];
		tail.store((t + 1) & MASK, std::memory_order_release);
		return ch;
	}
	void put(const uint8_t ch)
	{
		auto h = head.load(std::memory_order_relaxed);
		buf[h] = ch;
		head.store((h + 1) & MASK, std::memory_order_release);
	}
	void reset()
	{
		head.store(0, std::memory_order_release);
		tail.store(0, std::memory_order_release);
	}
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
	const uint16_t ctl_offset;	// control (write) / status (read) register offset
	const uint16_t data_offset;	// TX data (write) / RX data (read) register offset
	SpscQueue tx;
	SpscQueue rx;
	registers &reg;
	static constexpr uint CTS_THRESHOLD = 200u;  // out of CONSOLE_QUEUE_LEN=256
	const mc6850_status reset_status = { .tdre = 1 };
	uint8_t cached_status_byte = 0;
	interrupt cached_irq_result = INTERRUPT_NONE;
	bool transmit_buffer_empty = true;
	bool receive_buffer_full = false;
	bool transmit_irq = false;
	bool receive_irq = false;
	bool assert_transmit_irq = false;
	bool assert_receive_irq = false;
	bool cts_deasserted = false;
	bool rts_deasserted = false;
	bool loopback_close = false;	// tx_irq==0b11 (BREAK): TX→RX register directly
	bool loopback_queue = false;	// divide_select==0b10 (÷64): TX→RX queue
	volatile bool tx_write_pending = false;	// set by write ISR, cleared by guest_transmit()
	volatile bool irq_dirty = true;		// set by any state change, cleared by has_interrupt()
	volatile bool host_cts_deasserted = false;  // set by Core 0 when USB host can't accept data
	void sync_status();
	void sync_transmit();
public:
	mc6850(uint16_t ctl_off, uint16_t data_off);
	void reset();
	void write_received();		// called from Core 1 write ISR: clears TDRE until guest_transmit() runs
	uint host_transmit_level_avail() { return (CONSOLE_QUEUE_LEN - 1) - rx.get_level(); }
	uint transmit_level() { return rx.get_level(); }
	uint host_receive_level_avail() { return tx.get_level(); }
	uint receive_level() { return tx.get_level(); }
	bool host_transmit_avail() { return !tx.is_empty(); }
	bool host_receive_avail() { return !tx.is_empty(); }
	uint8_t host_receive() { return tx.get(); }
	void host_transmit(const uint8_t ch) { rx.put(ch); irq_dirty = true; }

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
	void guest_transmit(uint8_t data);
	void guest_control(uint8_t val);
	void rx_read_fast_path();	// called from read ISR: clears RDRF immediately
	[[nodiscard]] interrupt has_interrupt();
	[[nodiscard]] bool is_rts_deasserted() const { return rts_deasserted; }
	void set_host_cts(bool deasserted);
	[[nodiscard]] uint16_t get_ctl_offset() const { return ctl_offset; }
	[[nodiscard]] uint16_t get_data_offset() const { return data_offset; }
};

extern mc6850 fast_serial;
extern mc6850 aux_serial;
