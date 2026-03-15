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

#include "registers.h"
#include "pico_thing.h"
#include "mc6809.h"
#include "mc6850.h"

mc6850::mc6850(uint16_t ctl_off, uint16_t data_off) :
	ctl_offset(ctl_off),
	data_offset(data_off),
	reg(registers::getInstance())
{
	registers::write(ctl_offset) = CONSOLE_NONE;
	reg[ctl_offset] = reset_status.byte;
	cached_status_byte = reset_status.byte;
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
	tx.reset();
	rx.reset();
	reg[ctl_offset] = reset_status.byte;
	cached_status_byte = reset_status.byte;
}

// Derive the full status register from internal state variables.
// Must NOT read-modify-write reg[ctl_offset] — the read ISR
// (rx_read_fast_path) can preempt and clear RDRF between our read
// and write, causing us to clobber the ISR's update.
// Skips the register write if the byte hasn't changed.
inline void
mc6850::sync_status()
{
	mc6850_status sr = {};
	sr.rdrf = receive_buffer_full ? 1u : 0u;
	sr.tdre = (transmit_buffer_empty && !cts_deasserted && !tx_write_pending) ? 1u : 0u;
	sr.cts  = cts_deasserted ? 1u : 0u;
	sr.irq  = (assert_receive_irq || assert_transmit_irq) ? 1u : 0u;
	if (sr.byte != cached_status_byte) {
		cached_status_byte = sr.byte;
		reg[ctl_offset] = sr.byte;
	}
}

// Sync TDRE/CTS status bits from current TX queue state.
inline void
mc6850::sync_transmit()
{
	transmit_buffer_empty = tx.has_space();
	cts_deasserted = (tx.get_level() > CTS_THRESHOLD) || host_cts_deasserted;
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
	sync_status();
}

void
mc6850::set_host_cts(bool deasserted)
{
	if (host_cts_deasserted != deasserted) {
		host_cts_deasserted = deasserted;
		irq_dirty = true;
	}
}

// Called from Core 1 write ISR to update RTS immediately, before the
// write_ring delivers the full guest_control() call.  This closes the
// window where has_interrupt() could stage another byte after the 6809
// deasserted RTS but before the for(;;) loop processed the control write.
void
mc6850::control_written_isr(uint8_t val)
{
	mc6850_control cr = {val};
	if (cr.divide_select != 0b11u)		// not a reset
		rts_deasserted = (cr.tx_irq == 0b10u);
	irq_dirty = true;
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
	receive_buffer_full = false;
	assert_receive_irq = false;
	irq_dirty = true;
	sync_status();
}

void
mc6850::guest_receive()
{
	// If RTS is deasserted (6809 ring buffer full) do not stage the next byte;
	// leave it in the rx queue so it is delivered once RTS is re-asserted.
	if (rts_deasserted) {
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
mc6850::has_interrupt()
{
	// Fast path: if no state has changed since the last call, return
	// the cached result without recomputing anything.
	if (!irq_dirty)
		return cached_irq_result;
	irq_dirty = false;

	// Absorb TX queue state refresh (formerly update_task/sync_transmit).
	transmit_buffer_empty = tx.has_space();
	cts_deasserted = (tx.get_level() > CTS_THRESHOLD) || host_cts_deasserted;

	// Stage the next byte if the slot is empty. Called every Q-rising edge,
	// so RDRF is current before apply_interrupts() drives /IRQ. There is an
	// inherent one-cycle lag (the DMA already served the previous value for
	// this cycle), but that is harmless: the 6809 sees the updated RDRF on
	// the very next UARTS read.
	if (!receive_buffer_full && rx.has_bytes() && !rts_deasserted) {
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
