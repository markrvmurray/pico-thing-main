/**
* Copyright (c) 2025 Mark R V Murray.
	*
	* SPDX-License-Identifier: BSD-2-Clause
	*/

#pragma once

#include <atomic>
#include <cstdint>

#define CONSOLE_QUEUE_LEN	1024

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
	// The level and whether there is room, from one read of the indices.
	// is_full() is exactly level == MASK - if head + 1 == tail then
	// head - tail == MASK, both modulo CAPACITY - so a caller that wants
	// both, as the ACIA's status recompute does on every dirty Q-rising,
	// need not load head and tail twice.
	struct level_space {
		uint16_t level;
		bool space;
	};
	[[nodiscard]] level_space get_level_space() const
	{
		const uint16_t level = (head.load(std::memory_order_acquire)
					- tail.load(std::memory_order_acquire)) & MASK;
		return { level, level != MASK };
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
	// NEVER STORE BOTH INDICES.  head belongs to the producer and tail to the
	// consumer, and the reset() that used to live here stored both: between its
	// two stores the queue read head == 0 with tail still at its old value, so
	// head != tail said NOT EMPTY and the consumer's drain loop walked tail the
	// whole way round the ring - emitting whatever was still in it, then the
	// never-written zeros - until tail wrapped to meet head.  That was the
	// console corruption above 2.3 MHz: the guest's C_RESET is applied on
	// Core 1 while Core 0 sits inside that loop, which under load it nearly
	// always does, so the window gets hit instead of being skipped over.
	//
	// AND THE QUEUE DOES NOT HAVE TO BE HOLDING ANYTHING.  An EMPTY queue with
	// both indices at V reads, after the first store alone, as head == 0
	// against tail == V - a level of CAPACITY - V.  Measured on the board: 922
	// NUL bytes in one capture, and 1024 - 922 = 102, the index at the time.
	// An instrument that sampled get_level() at reset therefore read 0 and
	// said nothing was wrong; the hazard is the index VALUE, not the level.
	//
	// Each of these stores ONE index - its own caller's - and sets the two
	// equal, so no interleaving can make the queue read anything but EMPTY.
	// Even racing the other side's get()/put(), the only reachable outcomes are
	// "empty" and "one element moved", both of which are valid states; the
	// ring-walk state is unreachable by construction.
	void discard_unread()		// consumer side: drop what was never read
	{
		tail.store(head.load(std::memory_order_acquire),
			   std::memory_order_release);
	}
	void abandon_unsent()		// producer side: drop what was never consumed
	{
		head.store(tail.load(std::memory_order_acquire),
			   std::memory_order_release);
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
public:
	static constexpr uint CTS_THRESHOLD = 800u;  // out of CONSOLE_QUEUE_LEN=1024

	// How sync_status() stores the status register.  open() returns the
	// register's current value and starts an exclusive access; commit() stores
	// a new value and returns false if anything intervened since open(), and
	// abandon() ends an access with nothing to store.  native_store uses
	// plain reads and stores that never fail on the host.  store is what
	// the HOST's sync_status() uses, and a test may replace it; on the
	// device neither is defined - sync_status() holds interrupts off
	// inline, so a status store pays no indirect calls (Core 1 audit,
	// 2026-10-09).
	struct status_store {
		uint8_t (*open)(volatile uint8_t *status);
		bool (*commit)(volatile uint8_t *status, uint8_t value);
		void (*abandon)();
	};
	static const status_store native_store;
	static status_store store;
private:
	const uint16_t ctl_offset;	// control (write) / status (read) register offset
	const uint16_t data_offset;	// TX data (write) / RX data (read) register offset
	SpscQueue tx;
	SpscQueue rx;
	registers &reg;
	const mc6850_status reset_status = { .tdre = 1 };
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
	[[nodiscard]] interrupt has_interrupt_recompute();
	volatile bool host_cts_deasserted = false;  // set by Core 0 when USB host can't accept data
	// Why RDRF went away.  Three places clear receive_buffer_full and the
	// difference between them is the whole question when a staged byte
	// vanishes - see acia_torture test 3, which fails only while a host is
	// attached to the CDC port.  Counters, not flags: the interesting
	// quantity is how OFTEN, against a run where the port is closed.
	volatile uint32_t n_reset = 0u;		// reset() - clears everything
	volatile uint32_t n_rx_read = 0u;	// rx_read_fast_path() - guest read RX
	volatile uint32_t n_rts_drop = 0u;	// guest_receive() with RTS deasserted
	volatile uint32_t n_staged = 0u;	// a byte moved queue -> RX register
	volatile uint32_t n_cts_change = 0u;	// set_host_cts() actually changed it
	[[nodiscard]] uint8_t status_byte() const;
	void sync_status();
	void sync_transmit();
public:
	mc6850(uint16_t ctl_off, uint16_t data_off);
	void reset();
	void write_received();		// called from Core 1 write ISR: clears TDRE until guest_transmit() runs
	// Called from the Core 1 write ISR on every guest write to a control
	// register, so it is here rather than out of line.  RTS must take effect
	// before the for(;;) loop processes the write: otherwise has_interrupt()
	// can stage another byte in the window after the 6809 deasserted RTS.
	void control_written_isr(const uint8_t val)
	{
		const mc6850_control cr = {val};
		if (cr.divide_select != 0b11u)		// not a reset
			rts_deasserted = (cr.tx_irq == 0b10u);
		irq_dirty = true;
	}
	uint host_transmit_level_avail() { return (CONSOLE_QUEUE_LEN - 1) - rx.get_level(); }
	uint transmit_level() { return rx.get_level(); }
	uint host_receive_level_avail() { return tx.get_level(); }
	uint receive_level() { return tx.get_level(); }
	bool host_transmit_avail() { return !tx.is_empty(); }
	bool host_receive_avail() { return !tx.is_empty(); }
	// Draining can bring the queue back under CTS_THRESHOLD, so it marks the
	// state dirty: has_interrupt() has usually cached CTS already.
	uint8_t host_receive() { const uint8_t ch = tx.get(); irq_dirty = true; return ch; }
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
	// Called once per Q-rising per ACIA from Core 1's loop - two million
	// calls a second between the two of them at 1 MHz E - and the answer is
	// usually the cached one.  That test is here so the common case is a
	// load, a branch and a return; the recompute, which is an order of
	// magnitude bigger, stays out of line.
	[[nodiscard]] interrupt has_interrupt()
	{
		if (!irq_dirty)
			return cached_irq_result;
		return has_interrupt_recompute();
	}
	[[nodiscard]] bool is_rts_deasserted() const { return rts_deasserted; }
	struct rdrf_why {
		uint32_t reset, rx_read, rts_drop, staged, cts_change;
	};
	// Diagnostics only: the RDRF accounting that st prints.  Core 0 writes
	// zeros under Core 1's increments; a lost count is harmless here.
	void stats_clear() { n_reset = n_rx_read = n_rts_drop = n_staged = n_cts_change = 0u; }
	// Core 1 ONLY (see the note in reset()): drop the input the guest never
	// read and the output the host never took.  Both are Core 1's stores.
	void flush_queues_from_core1() { rx.discard_unread(); tx.abandon_unsent(); }
	[[nodiscard]] rdrf_why why_rdrf() const
		{ return { n_reset, n_rx_read, n_rts_drop, n_staged, n_cts_change }; }
	void set_host_cts(bool deasserted);
	[[nodiscard]] uint16_t get_ctl_offset() const { return ctl_offset; }
	[[nodiscard]] uint16_t get_data_offset() const { return data_offset; }
};

extern mc6850 fast_serial;
extern mc6850 aux_serial;
