/**
 * Host-side unit tests for mc6850.cpp.
 *
 * Build: cmake -S .. -B build && cmake --build build (from tests/)
 *
 * The fake/ stubs replace:
 *   pico/stdlib.h        — defines uint
 *   (SpscQueue is now inline in mc6850.h — no pico/util/queue.h needed)
 *   hardware/pio.h       — empty stub
 *   registers.h          — simplified read/write proxy
 *   pico_thing.h         — CONSOLE_* register offsets
 *   mc6809.h             — interrupt enum
 *
 * MC6850 register layout (offsets from $FFC0, i.e. register_base):
 *   0x03  CONSOLE_CONTROL  (write) / CONSOLE_STATUS (read)
 *   0x04  CONSOLE_TX_DATA  (write) / CONSOLE_RX_DATA (read)
 *
 * mc6850_control bit layout:
 *   bits 0-1  divide_select  (0b11 = master reset)
 *   bits 2-4  word_select
 *   bits 5-6  tx_irq         (0b01 = tx IRQ on, 0b10 = RTS off)
 *   bit  7    rx_irq
 *
 * mc6850_status bit layout:
 *   bit 0  rdrf   (Receive Data Register Full)
 *   bit 1  tdre   (Transmit Data Register Empty)
 *   bit 2  dcd
 *   bit 3  cts
 *   bit 4  fe
 *   bit 5  ovrn
 *   bit 6  pe
 *   bit 7  irq
 */

#include <sys/types.h>
#include <cstdint>
#include <catch2/catch_test_macros.hpp>

#include "registers.h"
#include "pico_thing.h"
#include "mc6809.h"
#include "mc6850.h"

// Required by 'extern registers &reg' in registers.h.
registers &reg = registers::getInstance();

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

static mc6850_status status()
{
	return mc6850_status{registers::peek(CONSOLE_STATUS)};
}

// Simulate a 6809 write to UARTTX: load write_registers then call the
// two steps that pico_thing.cpp's write ISR + for(;;) loop perform.
static void uart_write(mc6850 &uart, uint8_t byte)
{
	registers::write(CONSOLE_TX_DATA) = byte;
	uart.write_received();        // ISR step: clears TDRE
	uart.guest_transmit(byte);    // for(;;) step: queues byte, restores TDRE
}

// Simulate a 6809 write to UARTC (control register).
// Mirrors the real path: write ISR calls control_written_isr() (immediate),
// then the for(;;) loop calls guest_control() (deferred via write_ring).
static void uart_control(mc6850 &uart, uint8_t val)
{
	registers::write(CONSOLE_CONTROL) = val;
	uart.control_written_isr(val);
	uart.guest_control(val);
}

// -----------------------------------------------------------------------
// Construction
// -----------------------------------------------------------------------

TEST_CASE("mc6850 construction: TDRE=1, RDRF=0") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	CHECK(status().tdre == 1);
	CHECK(status().rdrf == 0);
	CHECK(status().irq  == 0);
}

// -----------------------------------------------------------------------
// Master reset
// -----------------------------------------------------------------------

TEST_CASE("mc6850 master reset restores TDRE=1") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// Prime some pending state
	registers::write(CONSOLE_TX_DATA) = static_cast<uint8_t>('A');
	uart.write_received();
	REQUIRE(status().tdre == 0);

	// divide_select=0b11 triggers reset()
	uart_control(uart, 0x03u);

	CHECK(status().tdre == 1);
	CHECK(status().rdrf == 0);

	// has_interrupt() after reset must leave TDRE=1
	(void)uart.has_interrupt();
	CHECK(status().tdre == 1);
}

// -----------------------------------------------------------------------
// write_received / guest_transmit handshake (the TX drop fix)
// -----------------------------------------------------------------------

TEST_CASE("write_received clears TDRE") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	registers::write(CONSOLE_TX_DATA) = static_cast<uint8_t>('H');
	uart.write_received();

	CHECK(status().tdre == 0);
}

TEST_CASE("guest_transmit restores TDRE and queues byte") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	registers::write(CONSOLE_TX_DATA) = static_cast<uint8_t>('H');
	uart.write_received();
	uart.guest_transmit('H');

	CHECK(status().tdre == 1);
	REQUIRE(uart.host_receive_avail());
	CHECK(uart.host_receive() == 'H');
}

TEST_CASE("has_interrupt while write_pending does not restore TDRE") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	registers::write(CONSOLE_TX_DATA) = static_cast<uint8_t>('X');
	uart.write_received();
	CHECK(status().tdre == 0);

	(void)uart.has_interrupt();   // must NOT restore TDRE while tx_write_pending
	CHECK(status().tdre == 0);

	uart.guest_transmit('X');
	CHECK(status().tdre == 1);
}

TEST_CASE("back-to-back writes: both bytes queued, neither dropped") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// PUTCHAR(LF) issues OUTCH(CR) then OUTCH(LF) — the original drop scenario
	uart_write(uart, '\r');
	uart_write(uart, '\n');

	CHECK(uart.receive_level() == 2);
	CHECK(uart.host_receive() == '\r');
	CHECK(uart.host_receive() == '\n');
}

TEST_CASE("sequential single-byte writes all arrive") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	const char msg[] = "Hello";
	for (char c : msg) {
		if (c == '\0') break;
		uart_write(uart, static_cast<uint8_t>(c));
	}

	CHECK(uart.receive_level() == 5);
	CHECK(uart.host_receive() == 'H');
	CHECK(uart.host_receive() == 'e');
	CHECK(uart.host_receive() == 'l');
	CHECK(uart.host_receive() == 'l');
	CHECK(uart.host_receive() == 'o');
}

// -----------------------------------------------------------------------
// RX path
// -----------------------------------------------------------------------

TEST_CASE("guest_receive stages byte: RDRF=1, byte in read registers") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	uart.host_transmit('Z');
	uart.guest_receive();

	CHECK(status().rdrf == 1);
	CHECK(registers::peek(CONSOLE_RX_DATA) == 'Z');
}

TEST_CASE("guest_receive with empty queue: RDRF=0") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	uart.guest_receive();

	CHECK(status().rdrf == 0);
}

TEST_CASE("guest_receive with rts_deasserted skips staging") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// tx_irq=0b10 (bits 5-6) → rts_deasserted=true
	uart_control(uart, 0b01000000);

	uart.host_transmit('Q');
	uart.guest_receive();

	CHECK(status().rdrf == 0);
}

TEST_CASE("has_interrupt: stages byte and returns IRQ when rx_irq enabled") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// rx_irq = bit 7
	uart_control(uart, 0b10000000);

	uart.host_transmit('A');
	CHECK(uart.has_interrupt() == INTERRUPT_IRQ);
	CHECK(status().rdrf == 1);
}

TEST_CASE("has_interrupt: returns NONE when no IRQs enabled") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	uart.host_transmit('B');
	CHECK(uart.has_interrupt() == INTERRUPT_NONE);
}

TEST_CASE("has_interrupt: tx irq fires when tx_irq enabled and buffer empty") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// tx_irq=0b01 (bits 5-6)
	uart_control(uart, 0b00100000);

	CHECK(uart.has_interrupt() == INTERRUPT_IRQ);
}

// -----------------------------------------------------------------------
// CTS / flow control
// -----------------------------------------------------------------------

TEST_CASE("TDRE remains 1 below CTS threshold") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// Fill to exactly the threshold
	for (uint i = 0; i < mc6850::CTS_THRESHOLD; i++)
		uart_write(uart, static_cast<uint8_t>(i));

	CHECK(status().tdre == 1);
	CHECK(status().cts  == 0);
}

TEST_CASE("CTS deasserts TDRE at threshold+1") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	for (uint i = 0; i < mc6850::CTS_THRESHOLD + 1; i++)
		uart_write(uart, static_cast<uint8_t>(i));

	CHECK(status().tdre == 0);
	CHECK(status().cts  == 1);
}

TEST_CASE("TDRE recovers after draining below CTS threshold") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	for (uint i = 0; i < mc6850::CTS_THRESHOLD + 1; i++)
		uart_write(uart, static_cast<uint8_t>(i));
	REQUIRE(status().tdre == 0);

	// Drain two bytes to bring level below threshold
	uart.host_receive();
	uart.host_receive();
	(void)uart.has_interrupt();

	CHECK(status().tdre == 1);
	CHECK(status().cts  == 0);
}

// In the firmware has_interrupt() runs on every Q-rising edge, so CTS is
// cached and irq_dirty clear by the time the host drains the queue.  Draining
// has to mark the state dirty, or TDRE stays down for good.
TEST_CASE("TDRE recovers after draining, even once has_interrupt() has run") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	for (uint i = 0; i < mc6850::CTS_THRESHOLD + 1; i++)
		uart_write(uart, static_cast<uint8_t>(i));
	(void)uart.has_interrupt();
	REQUIRE(status().tdre == 0);

	uart.host_receive();
	uart.host_receive();
	(void)uart.has_interrupt();

	CHECK(status().tdre == 1);
	CHECK(status().cts  == 0);
}

// -----------------------------------------------------------------------
// Close loopback (tx_irq=0b11, BREAK mode repurposed)
// -----------------------------------------------------------------------

TEST_CASE("close loopback: TX byte appears in RX register immediately") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// tx_irq=0b11 (bits 5-6) = close loopback
	uart_control(uart, 0b01100000);

	uart_write(uart, 'L');

	CHECK(status().rdrf == 1);
	CHECK(registers::peek(CONSOLE_RX_DATA) == 'L');
	// Byte should NOT appear in host TX queue
	CHECK_FALSE(uart.host_receive_avail());
}

TEST_CASE("close loopback: TDRE restored after write") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	uart_control(uart, 0b01100000);

	uart_write(uart, 'M');

	CHECK(status().tdre == 1);
}

TEST_CASE("close loopback: second TX overwrites RX if not read") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	uart_control(uart, 0b01100000);

	uart_write(uart, 'A');
	uart_write(uart, 'B');

	// Second byte overwrites the first
	CHECK(registers::peek(CONSOLE_RX_DATA) == 'B');
	CHECK(status().rdrf == 1);
}

TEST_CASE("close loopback: RX IRQ fires on looped byte") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// tx_irq=0b11 (close loopback) + rx_irq=1 (bit 7)
	uart_control(uart, 0b11100000);

	uart_write(uart, 'I');

	CHECK(uart.has_interrupt() == INTERRUPT_IRQ);
	CHECK(status().irq  == 1);
	CHECK(status().rdrf == 1);
}

TEST_CASE("close loopback: S_IRQ set immediately by guest_transmit (no has_interrupt needed)") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// tx_irq=0b11 (close loopback) + rx_irq=1 (bit 7)
	uart_control(uart, 0b11100000);

	// Write a byte — guest_transmit must set both RDRF and S_IRQ
	// WITHOUT requiring has_interrupt() to run first.
	uart_write(uart, 'Q');

	CHECK(status().rdrf == 1);
	CHECK(status().irq  == 1);
	CHECK(registers::peek(CONSOLE_RX_DATA) == 'Q');
}

TEST_CASE("close loopback: S_IRQ not set when rx_irq disabled") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// tx_irq=0b11 (close loopback) but rx_irq=0 (bit 7 clear)
	uart_control(uart, 0b01100000);

	uart_write(uart, 'R');

	CHECK(status().rdrf == 1);
	CHECK(status().irq  == 0);
}

TEST_CASE("close loopback: rx_read_fast_path clears RDRF immediately") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// tx_irq=0b11 (close loopback) + rx_irq=1
	uart_control(uart, 0b11100000);

	uart_write(uart, 'S');
	CHECK(status().rdrf == 1);
	CHECK(status().irq  == 1);

	// Simulate 6809 reading UARTRX — read ISR calls rx_read_fast_path
	uart.rx_read_fast_path();

	CHECK(status().rdrf == 0);
	CHECK(status().irq  == 0);
}

TEST_CASE("close loopback: host TX queue is not affected") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	uart_control(uart, 0b01100000);

	for (int i = 0; i < 10; i++)
		uart_write(uart, static_cast<uint8_t>('0' + i));

	// Nothing in the host TX queue
	CHECK(uart.receive_level() == 0);
}

// -----------------------------------------------------------------------
// Queue loopback (divide_select=0b10, div64 mode repurposed)
// -----------------------------------------------------------------------

TEST_CASE("queue loopback: TX byte arrives via RX queue") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// divide_select=0b10 (bits 0-1) = queue loopback
	uart_control(uart, 0b00000010);

	uart_write(uart, 'Q');

	// Byte is in the RX queue, not yet staged (needs guest_receive or has_interrupt)
	CHECK(status().rdrf == 0);
	CHECK_FALSE(uart.host_receive_avail());  // not in host TX queue

	// Stage it via guest_receive
	uart.guest_receive();
	CHECK(status().rdrf == 1);
	CHECK(registers::peek(CONSOLE_RX_DATA) == 'Q');
}

TEST_CASE("queue loopback: multiple bytes queue up") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	uart_control(uart, 0b00000010);

	uart_write(uart, 'X');
	uart_write(uart, 'Y');
	uart_write(uart, 'Z');

	// Stage and check each byte in order (simulate 6809 reading between stages)
	uart.guest_receive();
	CHECK(registers::peek(CONSOLE_RX_DATA) == 'X');

	uart.rx_read_fast_path();   // simulate 6809 reading UARTRX
	uart.guest_receive();
	CHECK(registers::peek(CONSOLE_RX_DATA) == 'Y');

	uart.rx_read_fast_path();
	uart.guest_receive();
	CHECK(registers::peek(CONSOLE_RX_DATA) == 'Z');
}

TEST_CASE("queue loopback: has_interrupt stages byte with rx_irq") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// divide_select=0b10 (queue loopback) + rx_irq=1 (bit 7)
	uart_control(uart, 0b10000010);

	uart_write(uart, 'R');

	CHECK(uart.has_interrupt() == INTERRUPT_IRQ);
	CHECK(status().rdrf == 1);
	CHECK(registers::peek(CONSOLE_RX_DATA) == 'R');
}

TEST_CASE("queue loopback: host TX queue is not affected") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	uart_control(uart, 0b00000010);

	for (int i = 0; i < 10; i++)
		uart_write(uart, static_cast<uint8_t>('a' + i));

	CHECK(uart.receive_level() == 0);  // host TX queue empty
}

TEST_CASE("queue loopback: TDRE stays 1 (bytes go to RX queue, not TX)") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	uart_control(uart, 0b00000010);

	uart_write(uart, 'T');
	CHECK(status().tdre == 1);
	CHECK(status().cts  == 0);
}

// -----------------------------------------------------------------------
// Loopback mode transitions
// -----------------------------------------------------------------------

TEST_CASE("master reset clears loopback state") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// Enable close loopback
	uart_control(uart, 0b01100000);
	uart_write(uart, 'L');
	REQUIRE(registers::peek(CONSOLE_RX_DATA) == 'L');

	// Master reset
	uart_control(uart, 0x03);

	// Normal mode: TX should go to host queue
	uart_write(uart, 'N');
	CHECK(uart.host_receive_avail());
	CHECK(uart.host_receive() == 'N');
}

TEST_CASE("switching from close to queue loopback") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// Close loopback
	uart_control(uart, 0b01100000);
	uart_write(uart, 'C');
	CHECK(registers::peek(CONSOLE_RX_DATA) == 'C');

	// Switch to queue loopback
	uart_control(uart, 0b00000010);
	uart_write(uart, 'Q');

	// Byte now in RX queue, needs staging; clear old close-loopback byte first
	uart.rx_read_fast_path();   // simulate 6809 reading the close-loopback byte
	uart.guest_receive();
	CHECK(registers::peek(CONSOLE_RX_DATA) == 'Q');
}

// -----------------------------------------------------------------------
// Host CTS (USB backpressure) flow control
// -----------------------------------------------------------------------

TEST_CASE("set_host_cts(true) masks TDRE even when queue is empty") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	REQUIRE(status().tdre == 1);

	uart.set_host_cts(true);
	(void)uart.has_interrupt();

	CHECK(status().tdre == 0);
	CHECK(status().cts  == 1);
}

TEST_CASE("set_host_cts(false) restores TDRE") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	uart.set_host_cts(true);
	(void)uart.has_interrupt();
	REQUIRE(status().tdre == 0);

	uart.set_host_cts(false);
	(void)uart.has_interrupt();

	CHECK(status().tdre == 1);
	CHECK(status().cts  == 0);
}

TEST_CASE("host CTS + queue CTS: both must clear for TDRE=1") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// Fill queue past threshold
	for (uint i = 0; i < mc6850::CTS_THRESHOLD + 1; i++)
		uart_write(uart, static_cast<uint8_t>(i));
	REQUIRE(status().tdre == 0);

	// Also set host CTS
	uart.set_host_cts(true);

	// Drain queue below threshold — host CTS still blocks
	for (uint i = 0; i < 3; i++)
		uart.host_receive();
	(void)uart.has_interrupt();
	CHECK(status().tdre == 0);
	CHECK(status().cts  == 1);

	// Clear host CTS — now TDRE should restore
	uart.set_host_cts(false);
	(void)uart.has_interrupt();
	CHECK(status().tdre == 1);
	CHECK(status().cts  == 0);
}

TEST_CASE("reset() clears host CTS") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	uart.set_host_cts(true);
	(void)uart.has_interrupt();
	REQUIRE(status().cts == 1);

	// Master reset (divide_select=0b11)
	uart_control(uart, 0x03u);

	CHECK(status().tdre == 1);
	CHECK(status().cts  == 0);
}

TEST_CASE("is_rts_deasserted() returns correct state") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// Default: RTS asserted
	CHECK_FALSE(uart.is_rts_deasserted());

	// tx_irq=0b10 (bits 5-6) → RTS deasserted
	uart_control(uart, 0b01000000);
	CHECK(uart.is_rts_deasserted());

	// tx_irq=0b01 (bits 5-6) → RTS asserted
	uart_control(uart, 0b00100000);
	CHECK_FALSE(uart.is_rts_deasserted());

	// tx_irq=0b11 (bits 5-6) → close loopback, RTS NOT deasserted
	uart_control(uart, 0b01100000);
	CHECK_FALSE(uart.is_rts_deasserted());
}

TEST_CASE("control_written_isr updates RTS immediately") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// Default: RTS asserted
	CHECK_FALSE(uart.is_rts_deasserted());

	// tx_irq=0b10 (bits 5-6) → RTS deasserted
	uart.control_written_isr(0b01000000);
	CHECK(uart.is_rts_deasserted());

	// tx_irq=0b01 → RTS asserted again
	uart.control_written_isr(0b00100000);
	CHECK_FALSE(uart.is_rts_deasserted());

	// divide_select=0b11 (reset) → RTS not changed by ISR path
	uart.control_written_isr(0b01000000);	// deassert first
	CHECK(uart.is_rts_deasserted());
	uart.control_written_isr(0b00000011);	// reset — should NOT touch RTS
	CHECK(uart.is_rts_deasserted());		// still deasserted

	// has_interrupt() respects ISR-set RTS: no staging while deasserted
	uart.control_written_isr(0b01000000);	// deassert RTS
	uart.host_transmit('Z');
	CHECK(uart.has_interrupt() == INTERRUPT_NONE);	// byte not staged
}

TEST_CASE("host CTS does not affect TX IRQ assertion") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// Enable TX IRQ (tx_irq=0b01)
	uart_control(uart, 0b00100000);
	REQUIRE(uart.has_interrupt() == INTERRUPT_IRQ);

	// Set host CTS — TDRE masked, TX IRQ should NOT fire
	uart.set_host_cts(true);
	CHECK(uart.has_interrupt() == INTERRUPT_NONE);

	// Clear host CTS — TX IRQ fires again
	uart.set_host_cts(false);
	CHECK(uart.has_interrupt() == INTERRUPT_IRQ);
}

// -----------------------------------------------------------------------
// Loopback mode transitions
// -----------------------------------------------------------------------

TEST_CASE("SpscQueue empties from either side with one store") {
	// The consumer's flush.  tail moves up to head, so nothing unread is left.
	SpscQueue q;
	for (int i = 0; i < 100; i++)
		q.put(static_cast<uint8_t>(i));
	CHECK(q.get_level() == 100);
	q.discard_unread();
	CHECK(q.is_empty());
	CHECK(q.get_level() == 0);
	CHECK(q.has_space());

	// The producer's flush.  head comes back down to tail.
	SpscQueue p;
	for (int i = 0; i < 100; i++)
		p.put(static_cast<uint8_t>(i));
	p.abandon_unsent();
	CHECK(p.is_empty());
	CHECK(p.get_level() == 0);

	// AND IT EMPTIES FROM A WRAPPED POSITION, which is the case that matters
	// and the one the old reset() got wrong.  That version stored 0 into BOTH
	// indices, so after the first store the queue read head == 0 against an
	// unchanged tail == V: not empty, but a level of CAPACITY - V, and the
	// consumer walked that many slots round the ring.
	//
	// NO BACKLOG WAS NEEDED.  The queue below is EMPTY at the flush - head and
	// tail both at V - which is why an instrument measuring get_level() at
	// reset read 0 and said there was nothing wrong.  The hazard is in the
	// index VALUE, not the level.  On the board the burst length matched
	// CAPACITY - V exactly: 922 NULs for V == 102.
	SpscQueue w;
	constexpr int V = 102;
	for (int i = 0; i < V; i++) {		// walk both indices up to V
		w.put(static_cast<uint8_t>(i));
		(void) w.get();
	}
	CHECK(w.is_empty());
	CHECK(w.get_level() == 0);
	// What the old code exposed from here: 1024 - 102 = 922 phantom bytes.
	CHECK((CONSOLE_QUEUE_LEN - V) == 922);
	// What this code exposes: nothing.  Still empty, from either side.
	w.discard_unread();
	CHECK(w.is_empty());
	CHECK(w.get_level() == 0);
	w.abandon_unsent();
	CHECK(w.is_empty());
	CHECK(w.get_level() == 0);
}

// DELIBERATE BEHAVIOUR CHANGE.  This case used to assert the opposite - that a
// master reset drained the TX queue.  A real MC6850 has no transmit FIFO for a
// reset to clear; this queue models the host link, not the part, and draining it
// discards output the guest has already handed over and believes it has sent.
// Not storing to head at all is also what closes the corruption window for
// certain: any store there reopens it.
TEST_CASE("mc6850 reset keeps output the guest has already handed over") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	for (int i = 0; i < 50; i++)
		uart_write(uart, static_cast<uint8_t>(i));
	CHECK(uart.host_receive_avail());

	// Master reset (divide_select=0b11)
	uart_control(uart, 0x03);

	// Still there, and still in order.
	CHECK(uart.host_receive_avail());
	CHECK(uart.host_receive_level_avail() == 50);
	for (int i = 0; i < 50; i++)
		CHECK(uart.host_receive() == static_cast<uint8_t>(i));
	CHECK_FALSE(uart.host_receive_avail());
}

TEST_CASE("mc6850 reset flushes RX queue") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// Host pushes bytes into RX queue
	for (int i = 0; i < 50; i++)
		uart.host_transmit(static_cast<uint8_t>(i));

	// Master reset
	uart_control(uart, 0x03);

	// RX queue should be drained — guest_receive stages nothing
	uart.guest_receive();
	CHECK(status().rdrf == 0);
}

TEST_CASE("mc6850 reset clears stale IRQ from previous run") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// Enable TX IRQ (tx_irq=0b01 → bits 5-6 = 0b01 → byte 0x20 with divide_select=0b00)
	uart_control(uart, 0b00100000);
	// TX IRQ should be asserted (TDRE=1, tx_irq=true)
	CHECK(uart.has_interrupt() == INTERRUPT_IRQ);

	// Master reset
	uart_control(uart, 0x03);

	// IRQ should be gone
	CHECK(uart.has_interrupt() == INTERRUPT_NONE);
	CHECK(status().irq == 0);
}

TEST_CASE("host_transmit_level_avail returns 0 when RX queue is full") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// Fill the RX queue to capacity (255 usable slots in 256-element ring buffer)
	for (int i = 0; i < CONSOLE_QUEUE_LEN - 1; i++) {
		CHECK(uart.host_transmit_level_avail() > 0);
		uart.host_transmit(static_cast<uint8_t>(i));
	}

	// Queue is now full — must report 0 available
	CHECK(uart.host_transmit_level_avail() == 0);

	// Drain one byte: has_interrupt() stages it, rx_read_fast_path() acks the read
	(void)uart.has_interrupt();
	CHECK(status().rdrf == 1);
	uart.rx_read_fast_path();

	// Should have space again (has_interrupt consumed 1 from queue)
	CHECK(uart.host_transmit_level_avail() == 1);
}

TEST_CASE("close loopback takes priority over queue loopback") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);

	// Both bits set: tx_irq=0b11 + divide_select=0b10
	uart_control(uart, 0b01100010);

	uart_write(uart, 'P');

	// Close loopback wins: byte in RX register immediately
	CHECK(status().rdrf == 1);
	CHECK(registers::peek(CONSOLE_RX_DATA) == 'P');
	CHECK_FALSE(uart.host_receive_avail());
}

// -----------------------------------------------------------------------
// Status register store
//
// sync_status() builds the status byte from the flags and then stores it.  A
// bus ISR fast path landing between the two changes the flags and stores its
// own byte, so the store has to fail, or the fast path's update is overwritten.
// Any other exception cancels the store too - the Q-edge handler, the tick
// timer - and then nothing else has stored the loop's update, so a failed store
// has to be retried.  On the device the failure comes from STREXB after an
// exception; here a fake store fails once, running the fast path in between if
// asked, as the interrupt would.
// -----------------------------------------------------------------------

static mc6850 *store_uart = nullptr;
static uint32_t store_commits = 0u;
static uint32_t store_opens = 0u;	// entries into the interrupts-off section
static bool fail_next_commit = false;		// an exception lands before the store
static bool fast_path_in_failure = false;	// ... and it is the read ISR's fast path

static uint8_t fake_open(volatile uint8_t *status)
{
	store_opens++;
	return *status;
}

static bool fake_commit(volatile uint8_t *status, const uint8_t value)
{
	store_commits++;
	if (fail_next_commit) {
		fail_next_commit = false;
		if (fast_path_in_failure) {
			fast_path_in_failure = false;
			store_uart->rx_read_fast_path();	// the guest reads RX data: the read ISR
		}
		return false;
	}
	*status = value;
	return true;
}

static void fake_abandon()
{
}

TEST_CASE("a fast path between building and storing the status is not overwritten") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);
	store_uart = &uart;
	mc6850::store = { fake_open, fake_commit, fake_abandon };

	uart.host_transmit('Z');
	(void)uart.has_interrupt();		// stages 'Z': RDRF=1
	registers::write(CONSOLE_TX_DATA) = static_cast<uint8_t>('A');
	uart.write_received();			// TDRE=0 until the loop takes the byte
	REQUIRE(status().rdrf == 1);
	REQUIRE(status().tdre == 0);

	// The loop builds TDRE=1 with RDRF still 1; the read ISR lands before the
	// store and clears RDRF.
	fail_next_commit = true;
	fast_path_in_failure = true;
	uart.guest_transmit('A');

	CHECK(status().rdrf == 0);
	CHECK(status().tdre == 1);

	mc6850::store = mc6850::native_store;
}

TEST_CASE("a status store cancelled by an unrelated exception is retried") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);
	store_uart = &uart;
	mc6850::store = { fake_open, fake_commit, fake_abandon };

	registers::write(CONSOLE_TX_DATA) = static_cast<uint8_t>('A');
	uart.write_received();			// TDRE=0 until the loop takes the byte
	REQUIRE(status().tdre == 0);

	// The loop builds TDRE=1; a Q-edge interrupt lands before the store.
	// Nothing else stores the update, so without a retry TDRE stays at 0.
	fail_next_commit = true;
	uart.guest_transmit('A');

	CHECK(status().tdre == 1);

	mc6850::store = mc6850::native_store;
}

TEST_CASE("an unchanged status is not stored") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);
	mc6850::store = { fake_open, fake_commit, fake_abandon };

	uart.guest_receive();
	const uint32_t before = store_commits;
	uart.guest_receive();			// nothing has changed

	CHECK(store_commits == before);

	mc6850::store = mc6850::native_store;
}

// The point of the pre-check: when nothing moved, interrupts are never held
// off at all.  sync_status() runs twice per E cycle per ACIA from Core 1's
// loop, so the common case has to be free of the CPSID/CPSIE pair.
TEST_CASE("an unchanged status does not enter the critical section") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);
	mc6850::store = { fake_open, fake_commit, fake_abandon };

	uart.guest_receive();			// settles the register
	const uint32_t opens = store_opens;
	const uint32_t commits = store_commits;

	for (int i = 0; i < 20; i++)
		uart.guest_receive();		// nothing changes, 20 times over

	CHECK(store_opens == opens);		// never opened
	CHECK(store_commits == commits);	// and so never stored

	mc6850::store = mc6850::native_store;
}

// ... and the converse, so the test above cannot pass by the section never
// being entered at all.
TEST_CASE("a changed status does enter the critical section") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);
	mc6850::store = { fake_open, fake_commit, fake_abandon };

	uart.guest_receive();
	const uint32_t opens = store_opens;

	uart.host_transmit('Q');
	(void)uart.has_interrupt();		// stages 'Q': RDRF goes 0 -> 1

	CHECK(store_opens > opens);
	CHECK(status().rdrf == 1);

	mc6850::store = mc6850::native_store;
}

// The pre-check compares against the REGISTER, not a remembered byte, so a
// register someone else corrupted is still corrected - and that correction
// must go through the critical section like any other store.
TEST_CASE("a corrupted register is corrected through the critical section") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);
	mc6850::store = { fake_open, fake_commit, fake_abandon };

	uart.guest_receive();
	REQUIRE(status().tdre == 1);
	const uint32_t opens = store_opens;

	registers::poke(CONSOLE_STATUS, 0x00u);	// behind the ACIA's back
	uart.guest_receive();

	CHECK(status().tdre == 1);		// put right
	CHECK(store_opens > opens);		// and properly guarded

	mc6850::store = mc6850::native_store;
}

// The byte is compared with the register, not with what was last stored: a
// register changed behind the ACIA's back is put right at the next update.
TEST_CASE("a status register changed behind the ACIA's back is corrected") {
	registers::clear();
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);
	uart.guest_receive();
	REQUIRE(status().tdre == 1);

	registers::poke(CONSOLE_STATUS, 0x00u);
	uart.guest_receive();

	CHECK(status().tdre == 1);
}
