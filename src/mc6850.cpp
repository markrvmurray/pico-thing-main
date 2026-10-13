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
#include "hardware/sync.h"

#include "registers.h"
#include "pico_thing.h"
#include "mc6809.h"
#include "mc6850.h"

// The status register is built from flags that the bus ISRs' fast paths -
// write_received(), control_written_isr() and rx_read_fast_path() - change and
// then store themselves.  Core 1's loop builds and stores it too, so a fast
// path landing between the loop's build and store would be overwritten with a
// stale byte: RDRF set again for a byte the guest has already read.
//
// Two earlier attempts at keeping them apart failed on this bus.  Masking the
// DMA interrupts around the loop's ACIA work lost bus reads, because two
// completions while masked are served as one, which misaligned DriveWire.  An
// exclusive store (LDREXB/STREXB, retried when an exception cancelled it) lost
// no reads but could not finish under load: with a Q-edge interrupt twice per E
// cycle and a DMA interrupt for every guest access, an interrupt landed between
// the load and the store every time, and Core 1's thread mode starved.
//
// So the store simply holds interrupts off for the compare and the store - a
// few tens of nanoseconds, against an E cycle of a microsecond.  Nothing can
// come between; no interrupt is lost, only delayed well inside one bus cycle;
// and there is nothing to retry, so it cannot spin.  Only Core 1 gets here (its
// loop and its own ISRs), so one saved interrupt state is enough.
#if !PICO_ON_DEVICE
// THE SEAM EXISTS FOR THE HOST TESTS ONLY, which replace it to drive a fast
// path into the window between building and storing.  On the device it is not
// compiled at all: sync_status() holds interrupts off inline instead.  Paying
// for the seam there meant two indirect calls (blx through a function-pointer
// table) with their prologues on every status store - on the read ISR's data
// path and in has_interrupt_recompute() - measured at 20-30 clk_sys cycles
// per store in the 2026-10-09 Core 1 audit.  On the host nothing can come
// between, so a plain store never fails.
static uint8_t
plain_open(volatile uint8_t *status)
{
	return *status;
}

static bool
plain_commit(volatile uint8_t *status, const uint8_t value)
{
	*status = value;
	return true;
}

static void
plain_abandon()
{
}
const mc6850::status_store mc6850::native_store = { plain_open, plain_commit, plain_abandon };
mc6850::status_store mc6850::store = mc6850::native_store;
#endif

mc6850::mc6850(uint16_t ctl_off, uint16_t data_off) :
	ctl_offset(ctl_off),
	data_offset(data_off),
	reg(registers::getInstance())
{
	registers::write(ctl_offset) = CONSOLE_NONE;
	reg[ctl_offset] = reset_status.byte;
	registers::write(data_offset) = static_cast<uint8_t>(0x00);
	reg[data_offset] = static_cast<uint8_t>(0x00);
}

void
mc6850::reset()
{
	registers::write(ctl_offset) = CONSOLE_NONE;
	reg[ctl_offset] = CONSOLE_NONE;
	registers::write(data_offset) = static_cast<uint8_t>(0x00);
	reg[data_offset] = static_cast<uint8_t>(0x00);
	n_reset++;
	transmit_buffer_empty = true;
	receive_buffer_full = false;
	transmit_irq = false;
	receive_irq = false;
	assert_transmit_irq = false;
	assert_receive_irq = false;
	cts_deasserted = false;
	rts_deasserted = false;
	loopback_close = false;
	loopback_queue = false;
	host_cts_deasserted = false;
	tx_write_pending = false;
	irq_dirty = true;
	// THE TX QUEUE IS LEFT ALONE, and that is the whole fix for the console
	// corruption.  A real MC6850 has no transmit FIFO for a master reset to
	// clear; this queue models the host link, not the part.  Not storing is
	// also the only way to be sure of closing the window, since any store to
	// head here reopens it.
	//
	// The receive side is different: a master reset does clear the receiver, so
	// dropping unread input is faithful.  Core 1 consumes rx, so Core 1 owns
	// tail and discard_unread() is its store to make.  The Core 0 call sites
	// (init, CMD_GO, set_power) are starting the guest, where at worst a couple
	// of stale input bytes are gained or lost; no interleaving can produce the
	// ring walk, which is the property that matters.
	rx.discard_unread();
	reg[ctl_offset] = reset_status.byte;
}

// Derive the full status register from internal state variables and store it,
// through the exclusive store described above.  The byte is compared with the
// register itself, not a remembered copy: an ISR can change the register after
// this stores it, and a copy would then let a later identical byte go unstored.
// The status byte as the internal flags currently describe it.  Pulled out so
// that the cheap pre-check below and the guarded rebuild inside the loop
// cannot drift apart.
inline uint8_t
mc6850::status_byte() const
{
	mc6850_status sr = {};
	sr.rdrf = receive_buffer_full ? 1u : 0u;
	sr.tdre = (transmit_buffer_empty && !cts_deasserted && !tx_write_pending) ? 1u : 0u;
	sr.cts  = cts_deasserted ? 1u : 0u;
	// The receive half is derived LIVE, not read back from
	// assert_receive_irq.  That cached flag belongs to the pin driver and is
	// only recomputed by has_interrupt_recompute(); several callers move
	// receive_buffer_full and then store the status without it - guest_receive()
	// stages a byte, rx_read_fast_path() clears one - and a cached copy then
	// described the previous state.  The guest saw RDRF set with S_IRQ clear,
	// which is a state the part cannot be in, and the torture suite's test 8
	// counted 137 of them in 256 bytes at 2.4 MHz.  On real silicon this bit
	// IS the live condition, so compute it that way and the class of bug goes.
	// guest_transmit()'s close-loopback arm had to assign the flag by hand for
	// this very reason; it no longer has to.
	sr.irq  = ((receive_irq && receive_buffer_full) || assert_transmit_irq) ? 1u : 0u;
	return sr.byte;
}

inline void
mc6850::sync_status()
{
	const auto status = reinterpret_cast<volatile uint8_t *>(registers::read_address() + ctl_offset);

	// Look before holding interrupts off.  Under load the usual answer is
	// that nothing moved - irq_dirty is set by queue activity that often
	// does not touch TDRE, RDRF, CTS or IRQ - and then there is nothing to
	// protect, so the CPSID/CPSIE pair is pure cost.  This is called twice
	// per E cycle per ACIA from Core 1's loop.
	//
	// Safe because this path NEVER STORES.  Both races are benign:
	//   - a flag changes right after the compare: the ISR that changed it
	//     sets irq_dirty, so the next pass recomputes and stores;
	//   - the register changes right after the read: our value was stale,
	//     but we are not writing, so there is nothing to clobber.
	// What it must not do is skip a store that was needed, and an equal
	// comparison means no store was needed.
	if (status_byte() == *status)
		return;

#if PICO_ON_DEVICE
	// Interrupts off for the rebuild, the compare and the store, inline: see
	// the note at the top of this file for why this and not an exclusive
	// store, and the seam comment for why the host's function pointers are
	// not paid for here.  Holding interrupts off cannot be interrupted, so
	// there is nothing to retry.
	const uint32_t saved = save_and_disable_interrupts();
	// Rebuilt inside the critical section, not reused from above: a fast
	// path may have moved the flags since, and its value is the newer one.
	// This is the race that took three attempts in September.
	const uint8_t want = status_byte();
	if (want != *status)
		*status = want;
	restore_interrupts(saved);
#else
	for (;;) {
		const uint8_t current = store.open(status);
		// Rebuilt inside the critical section, not reused from above: a
		// fast path may have moved the flags since, and its value is the
		// newer one.  This is the race that took three attempts in
		// September - see the note at the top of this file.
		const uint8_t want = status_byte();
		if (want == current) {
			store.abandon();
			return;
		}
		if (store.commit(status, want))
			return;
	}
#endif
}

// Sync TDRE/CTS status bits from current TX queue state.
inline void
mc6850::sync_transmit()
{
	const SpscQueue::level_space tx_state = tx.get_level_space();
	transmit_buffer_empty = tx_state.space;
	cts_deasserted = (tx_state.level > CTS_THRESHOLD) || host_cts_deasserted;
	sync_status();
}

// Called from Core 1 write ISR when CONSOLE_TX_DATA is written.
// Immediately clears TDRE so the 6809's OUTCH poll-loop stalls until
// guest_transmit() runs on the next for(;;) iteration.  This prevents
// back-to-back writes (e.g. CR then LF from PUTCHAR) from overwriting
// write_registers[CONSOLE_TX_DATA] before the for(;;) loop reads it.
void
mc6850::write_received()
{
	tx_write_pending = true;
	irq_dirty = true;
	// CLEAR ONE BIT, do not rebuild the byte.  sync_status() builds the whole
	// status twice - once for its cheap pre-check and once inside the critical
	// section, about sixteen volatile reads between them - and then stores all
	// eight bits.  But tx_write_pending can only ever affect TDRE, and it has
	// just been SET, so TDRE is unconditionally false: there is exactly one bit
	// to change and its new value is known without reading anything.
	//
	// This is the hot write on this bus.  A guest sending a byte stores to
	// CONSOLE_TX_DATA/AUX_TX_DATA and that lands here, which is why the
	// device-register path measured 126 clk_sys cycles mean against the high
	// range's 82 - and 126 does not fit a 2.5 MHz E cycle (128) while 82 does.
	// See the project note write-isr-cost.
	//
	// SAFER than the store it replaces, not just cheaper: masking and then
	// ANDing one bit leaves every other bit as whoever owns it last wrote it,
	// where commit() writes all eight from a snapshot and can therefore undo a
	// change another path made in between.  The mask is still needed, because a
	// read-modify-write is not atomic and both Core 1's loop and its ISRs store
	// this byte.
	// bit 1 of mc6850_status; the status_byte() above is the other place that
	// has to agree with the union's layout.
	static constexpr uint8_t TDRE_MASK = 0b00000010u;
	const auto status = reinterpret_cast<volatile uint8_t *>(
		registers::read_address() + ctl_offset);
#if PICO_ON_DEVICE
	const uint32_t saved = save_and_disable_interrupts();
	*status = static_cast<uint8_t>(*status & static_cast<uint8_t>(~TDRE_MASK));
	restore_interrupts(saved);
#else
	// On the host nothing can come between, which is the same assumption
	// plain_open()/plain_commit() make above.
	*status = static_cast<uint8_t>(*status & static_cast<uint8_t>(~TDRE_MASK));
#endif
}

void
mc6850::set_host_cts(bool deasserted)
{
	if (host_cts_deasserted != deasserted) {
		n_cts_change++;
		host_cts_deasserted = deasserted;
		irq_dirty = true;
	}
}

void
mc6850::guest_control(uint8_t val)
{
	mc6850_control cr = {val};
	if (cr.divide_select == 0b11u) {
		reset();
	} else {
		// tx_irq==0b11 (BREAK): close loopback (TX→RX register, no queues)
		// divide_select==0b10 (÷64): queue loopback (TX→RX queue)
		loopback_close = (cr.tx_irq == 0b11u);
		loopback_queue = !loopback_close && (cr.divide_select == 0b10u);
		transmit_irq = cr.tx_irq == 0b01u;
		receive_irq = cr.rx_irq;
		// rts_deasserted is NOT set here — control_written_isr() already
		// updated it immediately in the write ISR.  Setting it here from
		// the (deferred) write_ring would clobber the current state with
		// a stale value when write_ring entries pile up.
		irq_dirty = true;
		sync_transmit();
	}
}

void
mc6850::guest_transmit(uint8_t data)
{
	tx_write_pending = false;		// acknowledge: for(;;) loop has taken ownership
	if (loopback_close) {
		// Close loopback: TX byte goes directly into the RX register,
		// bypassing both queues.  RDRF and S_IRQ are set immediately
		// so the 6809 sees correct status without waiting for has_interrupt().
		reg[data_offset] = data;
		receive_buffer_full = true;
		assert_receive_irq = receive_irq;
	} else if (loopback_queue) {
		// Queue loopback: TX byte enters the RX queue, delivered via
		// the normal guest_receive()/has_interrupt() staging path.
		if (rx.has_space())
			rx.put(data);
	} else {
		// Normal: byte enters the TX queue for host consumption.
		if (tx.has_space())
			tx.put(data);
	}
	irq_dirty = true;
	sync_transmit();			// restores TDRE=1 (tx_write_pending is now false)
}

// Called from Core 1 read ISR when the 6809 reads CONSOLE_RX_DATA.
// Immediately clears RDRF and receive_buffer_full so the next status read
// sees the correct state, matching real MC6850 behavior (RDRF clears on
// data register read).  guest_receive() will stage the next byte if available.
void
mc6850::rx_read_fast_path()
{
	n_rx_read++;
	receive_buffer_full = false;
	assert_receive_irq = false;
	irq_dirty = true;
	// CLEAR THE BITS A DATA READ CAN CHANGE, do not rebuild the byte: the
	// write_received() reasoning in reverse.  Reading the data register can
	// only take RDRF away, and with it the receive half of IRQ; the transmit
	// half is whatever assert_transmit_irq already says, one load.  Every
	// other bit is left as its owner last stored it, and the recompute that
	// irq_dirty requests rebuilds the whole byte from the flags afterwards.
	// This runs on the read ISR's data path, where sync_status() cost two
	// status_byte() builds and the critical section per guest read.
	// bits 0 and 7 of mc6850_status; status_byte() is the other place that
	// has to agree with the union's layout.
	static constexpr uint8_t RDRF_MASK = 0b00000001u;
	static constexpr uint8_t IRQ_MASK  = 0b10000000u;
	const uint8_t clear = static_cast<uint8_t>(
		RDRF_MASK | (assert_transmit_irq ? 0u : IRQ_MASK));
	const auto status = reinterpret_cast<volatile uint8_t *>(
		registers::read_address() + ctl_offset);
#if PICO_ON_DEVICE
	const uint32_t saved = save_and_disable_interrupts();
	*status = static_cast<uint8_t>(*status & static_cast<uint8_t>(~clear));
	restore_interrupts(saved);
#else
	*status = static_cast<uint8_t>(*status & static_cast<uint8_t>(~clear));
#endif
}

void
mc6850::guest_receive()
{
	// If RTS is deasserted (6809 ring buffer full) do not stage the next byte;
	// leave it in the rx queue so it is delivered once RTS is re-asserted.
	if (rts_deasserted) {
		n_rts_drop++;
		receive_buffer_full = false;
		irq_dirty = true;
		sync_status();
		return;
	}
	// If has_interrupt() already staged a byte (receive_buffer_full=true),
	// don't re-stage — that would double-consume from the rx queue,
	// overwriting the byte has_interrupt() staged before the 6809 reads it.
	if (!receive_buffer_full) {
		receive_buffer_full = rx.has_bytes();
		if (receive_buffer_full)
			reg[data_offset] = rx.get();
	}
	irq_dirty = true;
	sync_status();
}

interrupt
mc6850::has_interrupt_recompute()
{
	// The caller has already found irq_dirty set; see the inline half in
	// mc6850.h.
	irq_dirty = false;
	n_recompute++;

	// Absorb TX queue state refresh (formerly update_task/sync_transmit).
	// One read of the queue's indices, not two: see get_level_space().
	const SpscQueue::level_space tx_state = tx.get_level_space();
	transmit_buffer_empty = tx_state.space;
	cts_deasserted = (tx_state.level > CTS_THRESHOLD) || host_cts_deasserted;

	// Stage the next byte if the slot is empty. Called every Q-rising edge,
	// so RDRF is current before apply_interrupts() drives /IRQ. There is an
	// inherent one-cycle lag (the DMA already served the previous value for
	// this cycle), but that is harmless: the 6809 sees the updated RDRF on
	// the very next UARTS read.
	if (!receive_buffer_full && rx.has_bytes() && !rts_deasserted) {
		n_staged++;
		receive_buffer_full = true;
		reg[data_offset] = rx.get();
	}
	assert_receive_irq = receive_irq && receive_buffer_full;
	assert_transmit_irq = transmit_irq && transmit_buffer_empty && !tx_write_pending && !cts_deasserted;
	sync_status();
	cached_irq_result = (assert_transmit_irq || assert_receive_irq)
		? INTERRUPT_IRQ : INTERRUPT_NONE;
	return cached_irq_result;
}
