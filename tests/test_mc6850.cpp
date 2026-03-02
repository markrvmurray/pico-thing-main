/**
 * Host-side unit tests for mc6850.cpp.
 *
 * Build: cmake -S .. -B build && cmake --build build (from tests/)
 *
 * The fake/ stubs replace:
 *   pico/stdlib.h        — defines uint
 *   pico/util/queue.h    — ring-buffer queue_t (no SDK spinlocks)
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
	uart.write_received();   // ISR step: clears TDRE
	uart.guest_transmit();   // for(;;) step: queues byte, restores TDRE
}

// -----------------------------------------------------------------------
// Construction
// -----------------------------------------------------------------------

TEST_CASE("mc6850 construction: TDRE=1, RDRF=0") {
	registers::clear();
	mc6850 uart;

	CHECK(status().tdre == 1);
	CHECK(status().rdrf == 0);
	CHECK(status().irq  == 0);
}

// -----------------------------------------------------------------------
// Master reset
// -----------------------------------------------------------------------

TEST_CASE("mc6850 master reset restores TDRE=1") {
	registers::clear();
	mc6850 uart;

	// Prime some pending state
	registers::write(CONSOLE_TX_DATA) = static_cast<uint8_t>('A');
	uart.write_received();
	REQUIRE(status().tdre == 0);

	// divide_select=0b11 triggers reset()
	registers::write(CONSOLE_CONTROL) = static_cast<uint8_t>(0x03u);
	uart.guest_control();

	CHECK(status().tdre == 1);
	CHECK(status().rdrf == 0);

	// update_task() after reset must leave TDRE=1
	uart.update_task();
	CHECK(status().tdre == 1);
}

// -----------------------------------------------------------------------
// write_received / guest_transmit handshake (the TX drop fix)
// -----------------------------------------------------------------------

TEST_CASE("write_received clears TDRE") {
	registers::clear();
	mc6850 uart;

	registers::write(CONSOLE_TX_DATA) = static_cast<uint8_t>('H');
	uart.write_received();

	CHECK(status().tdre == 0);
}

TEST_CASE("guest_transmit restores TDRE and queues byte") {
	registers::clear();
	mc6850 uart;

	registers::write(CONSOLE_TX_DATA) = static_cast<uint8_t>('H');
	uart.write_received();
	uart.guest_transmit();

	CHECK(status().tdre == 1);
	REQUIRE(uart.host_receive_avail());
	CHECK(uart.host_receive() == 'H');
}

TEST_CASE("update_task while write_pending does not restore TDRE") {
	registers::clear();
	mc6850 uart;

	registers::write(CONSOLE_TX_DATA) = static_cast<uint8_t>('X');
	uart.write_received();
	CHECK(status().tdre == 0);

	uart.update_task();   // must NOT restore TDRE while tx_write_pending
	CHECK(status().tdre == 0);

	uart.guest_transmit();
	CHECK(status().tdre == 1);
}

TEST_CASE("back-to-back writes: both bytes queued, neither dropped") {
	registers::clear();
	mc6850 uart;

	// PUTCHAR(LF) issues OUTCH(CR) then OUTCH(LF) — the original drop scenario
	uart_write(uart, '\r');
	uart_write(uart, '\n');

	CHECK(uart.receive_level() == 2);
	CHECK(uart.host_receive() == '\r');
	CHECK(uart.host_receive() == '\n');
}

TEST_CASE("sequential single-byte writes all arrive") {
	registers::clear();
	mc6850 uart;

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
	mc6850 uart;

	uart.host_transmit('Z');
	uart.guest_receive();

	CHECK(status().rdrf == 1);
	CHECK(registers::peek(CONSOLE_RX_DATA) == 'Z');
}

TEST_CASE("guest_receive with empty queue: RDRF=0") {
	registers::clear();
	mc6850 uart;

	uart.guest_receive();

	CHECK(status().rdrf == 0);
}

TEST_CASE("guest_receive with rts_deasserted skips staging") {
	registers::clear();
	mc6850 uart;

	// tx_irq=0b10 (bits 5-6) → rts_deasserted=true
	registers::write(CONSOLE_CONTROL) = static_cast<uint8_t>(0b01000000);
	uart.guest_control();

	uart.host_transmit('Q');
	uart.guest_receive();

	CHECK(status().rdrf == 0);
}

TEST_CASE("has_interrupt: stages byte and returns IRQ when rx_irq enabled") {
	registers::clear();
	mc6850 uart;

	// rx_irq = bit 7
	registers::write(CONSOLE_CONTROL) = static_cast<uint8_t>(0b10000000);
	uart.guest_control();

	uart.host_transmit('A');
	CHECK(uart.has_interrupt() == INTERRUPT_IRQ);
	CHECK(status().rdrf == 1);
}

TEST_CASE("has_interrupt: returns NONE when no IRQs enabled") {
	registers::clear();
	mc6850 uart;

	uart.host_transmit('B');
	CHECK(uart.has_interrupt() == INTERRUPT_NONE);
}

TEST_CASE("has_interrupt: tx irq fires when tx_irq enabled and buffer empty") {
	registers::clear();
	mc6850 uart;

	// tx_irq=0b01 (bits 5-6)
	registers::write(CONSOLE_CONTROL) = static_cast<uint8_t>(0b00100000);
	uart.guest_control();

	CHECK(uart.has_interrupt() == INTERRUPT_IRQ);
}

// -----------------------------------------------------------------------
// CTS / flow control
// -----------------------------------------------------------------------

TEST_CASE("TDRE remains 1 below CTS threshold") {
	registers::clear();
	mc6850 uart;

	// Fill to exactly the threshold (200 bytes)
	for (int i = 0; i < 200; i++)
		uart_write(uart, static_cast<uint8_t>(i));

	CHECK(status().tdre == 1);
	CHECK(status().cts  == 0);
}

TEST_CASE("CTS deasserts TDRE at threshold+1") {
	registers::clear();
	mc6850 uart;

	for (int i = 0; i < 201; i++)
		uart_write(uart, static_cast<uint8_t>(i));

	CHECK(status().tdre == 0);
	CHECK(status().cts  == 1);
}

TEST_CASE("TDRE recovers after draining below CTS threshold") {
	registers::clear();
	mc6850 uart;

	for (int i = 0; i < 201; i++)
		uart_write(uart, static_cast<uint8_t>(i));
	REQUIRE(status().tdre == 0);

	// Drain two bytes to bring level back to 199 (below 200)
	uart.host_receive();
	uart.host_receive();
	uart.update_task();

	CHECK(status().tdre == 1);
	CHECK(status().cts  == 0);
}
