/**
 * Host-side unit tests for uart_pump.cpp: the pump between an MC6850 and its
 * USB CDC port, driven with a real mc6850 and a fake port.
 *
 * The fake/ stubs are the ones test_mc6850 uses.
 */

#include <algorithm>
#include <cstdint>
#include <string>
#include <catch2/catch_test_macros.hpp>

#include "registers.h"
#include "pico_thing.h"
#include "mc6809.h"
#include "mc6850.h"
#include "uart_pump.h"

// Required by 'extern registers &reg' in registers.h.
registers &reg = registers::getInstance();

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

// A CDC port: whether the host has it open (DTR) and ready (RTS), what the
// host has sent for the guest, and what has reached the host.  write_room
// caps what one write accepts, as a full USB buffer would.
struct fake_port {
	bool dtr = true;
	bool rts = true;
	std::string from_host;
	std::string to_host;
	uint16_t write_room = 0xFFFFu;
};
static fake_port port;

static bool fake_connected(uint8_t) { return port.dtr; }
static bool fake_host_ready(uint8_t) { return port.rts; }

static uint16_t
fake_read(uint8_t, uint8_t *buf, const uint16_t len)
{
	const uint16_t n = static_cast<uint16_t>(std::min<size_t>(len, port.from_host.size()));
	port.from_host.copy(reinterpret_cast<char *>(buf), n);
	port.from_host.erase(0, n);
	return n;
}

static uint16_t
fake_write(uint8_t, const uint8_t *buf, const uint16_t len)
{
	const uint16_t n = std::min(len, port.write_room);
	port.to_host.append(reinterpret_cast<const char *>(buf), n);
	return n;
}

static constexpr cdc_port_io fake_io = { fake_connected, fake_host_ready, fake_read, fake_write };

static mc6850_status status()
{
	return mc6850_status{registers::peek(CONSOLE_STATUS)};
}

// A guest write to the data register, as the write ISR and Core 1's loop do it.
static void guest_send(mc6850 &uart, const uint8_t byte)
{
	registers::write(CONSOLE_TX_DATA) = byte;
	uart.write_received();
	uart.guest_transmit(byte);
}

static void pump(mc6850 &uart, uart_port_state &st, const bool suppress_read = false)
{
	uart_port_pump(uart, 0u, st, suppress_read, fake_io);
}

// -----------------------------------------------------------------------
// Guest -> host
// -----------------------------------------------------------------------

TEST_CASE("pump: output reaches an open, ready host") {
	registers::clear();
	port = fake_port{};
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);
	uart_port_state st{};

	guest_send(uart, 'H');
	guest_send(uart, 'i');
	pump(uart, st);
	(void)uart.has_interrupt();

	CHECK(port.to_host == "Hi");
	CHECK(status().tdre == 1);
}

TEST_CASE("pump: output to a closed port is discarded and the guest carries on") {
	registers::clear();
	port = fake_port{};
	port.dtr = false;
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);
	uart_port_state st{};

	guest_send(uart, 'A');
	guest_send(uart, 'B');
	pump(uart, st);
	(void)uart.has_interrupt();

	CHECK(port.to_host.empty());
	CHECK_FALSE(uart.host_receive_avail());
	CHECK(status().tdre == 1);

	// Once the host opens the port, only what is sent from then on arrives.
	port.dtr = true;
	guest_send(uart, 'C');
	pump(uart, st);
	CHECK(port.to_host == "C");
}

// The queue passing CTS_THRESHOLD raises CTS, and in the firmware
// has_interrupt() has already cached that by the time the pump discards.
TEST_CASE("pump: a closed port does not stall a guest that overfills the queue") {
	registers::clear();
	port = fake_port{};
	port.dtr = false;
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);
	uart_port_state st{};

	for (uint i = 0; i < mc6850::CTS_THRESHOLD + 1; i++)
		guest_send(uart, static_cast<uint8_t>(i));
	(void)uart.has_interrupt();
	REQUIRE(status().tdre == 0);

	pump(uart, st);
	(void)uart.has_interrupt();

	CHECK(status().tdre == 1);
	CHECK(status().cts == 0);
}

TEST_CASE("pump: a host with RTS down holds the output and stalls the guest") {
	registers::clear();
	port = fake_port{};
	port.rts = false;
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);
	uart_port_state st{};

	guest_send(uart, 'X');
	pump(uart, st);
	(void)uart.has_interrupt();

	CHECK(port.to_host.empty());
	CHECK(status().tdre == 0);
	CHECK(status().cts == 1);

	port.rts = true;
	pump(uart, st);
	(void)uart.has_interrupt();

	CHECK(port.to_host == "X");
	CHECK(status().tdre == 1);
	CHECK(status().cts == 0);
}

TEST_CASE("pump: a port told to ignore RTS writes to a host that never raises it") {
	registers::clear();
	port = fake_port{};
	port.rts = false;
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);
	uart_port_state st{};
	st.honour_host_rts = false;

	guest_send(uart, 'Y');
	pump(uart, st);
	(void)uart.has_interrupt();

	// This is the DriveWire server that does no flow control of its own: with
	// RTS honoured the byte would sit here for ever and the guest would stall.
	CHECK(port.to_host == "Y");
	CHECK(status().tdre == 1);
	CHECK(status().cts == 0);
}

TEST_CASE("pump: ignoring RTS still loses nothing when the USB buffer is full") {
	registers::clear();
	port = fake_port{};
	port.rts = false;
	port.write_room = 1u;
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);
	uart_port_state st{};
	st.honour_host_rts = false;

	guest_send(uart, 'a');
	guest_send(uart, 'b');
	pump(uart, st);
	(void)uart.has_interrupt();

	// Ignoring RTS removes one brake, not the other: the FIFO takes what it
	// can and CTS stalls the guest for the rest, so 'b' is held, not dropped.
	CHECK(port.to_host == "a");
	CHECK(status().tdre == 0);

	port.write_room = 0xFFFFu;
	pump(uart, st);
	(void)uart.has_interrupt();

	CHECK(port.to_host == "ab");
	CHECK(status().tdre == 1);
}

TEST_CASE("pump: a port still honouring RTS is unaffected by the other's switch") {
	registers::clear();
	port = fake_port{};
	port.rts = false;
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);
	uart_port_state honouring{};
	uart_port_state ignoring{};
	ignoring.honour_host_rts = false;

	// The switch lives in the per-port state, so one port's choice cannot
	// leak into the other's.
	CHECK(honouring.honour_host_rts);
	CHECK_FALSE(ignoring.honour_host_rts);

	guest_send(uart, 'Z');
	pump(uart, honouring);
	(void)uart.has_interrupt();

	CHECK(port.to_host.empty());
	CHECK(status().cts == 1);
}

TEST_CASE("pump: a full USB buffer holds the rest and stalls the guest until it drains") {
	registers::clear();
	port = fake_port{};
	port.write_room = 1u;
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);
	uart_port_state st{};

	guest_send(uart, 'a');
	guest_send(uart, 'b');
	pump(uart, st);
	(void)uart.has_interrupt();

	CHECK(port.to_host == "a");
	CHECK(status().tdre == 0);

	port.write_room = 0xFFFFu;
	pump(uart, st);
	(void)uart.has_interrupt();

	CHECK(port.to_host == "ab");
	CHECK(status().tdre == 1);
}

// -----------------------------------------------------------------------
// Host -> guest
// -----------------------------------------------------------------------

TEST_CASE("pump: host input reaches the guest only when it may") {
	registers::clear();
	port = fake_port{};
	port.from_host = "k";
	mc6850 uart(CONSOLE_CONTROL, CONSOLE_TX_DATA);
	uart_port_state st{};

	SECTION("delivered to an open port with the guest's RTS asserted") {
		pump(uart, st);
		(void)uart.has_interrupt();

		CHECK(port.from_host.empty());
		CHECK(status().rdrf == 1);
		CHECK(registers::peek(CONSOLE_RX_DATA) == 'k');
	}

	SECTION("left with the host while the guest has RTS deasserted") {
		uart.control_written_isr(0x40u);	// tx_irq 0b10: RTS deasserted
		uart.guest_control(0x40u);
		pump(uart, st);
		(void)uart.has_interrupt();

		CHECK(port.from_host == "k");
		CHECK(status().rdrf == 0);
	}

	SECTION("left with the host while reads are suppressed") {
		pump(uart, st, true);

		CHECK(port.from_host == "k");
	}

	SECTION("not read from a closed port") {
		port.dtr = false;
		pump(uart, st);

		CHECK(port.from_host == "k");
	}
}
