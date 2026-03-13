/**
 * Host-side unit tests for tick_timer.cpp.
 *
 * The fake/ stubs replace:
 *   pico/time.h     — repeating_timer struct, stub add_repeating_timer_us()
 *   registers.h     — simplified read/write proxy
 *   pico_thing.h    — SYSTEM_TIMER_* register offsets
 *   mc6809.h        — interrupt enum
 *
 * Tick timer register layout (offsets from $FFC0):
 *   0x08  SYSTEM_TIMER_CONTROL_13  (write: bit 0 = enable/disable)
 *   0x09  SYSTEM_TIMER_STATUS      (read:  bit 7 = IRQ pending, cleared on read)
 */

#include <cstdint>
#include <catch2/catch_test_macros.hpp>

#include "registers.h"
#include "pico_thing.h"
#include "mc6809.h"
#include "tick_timer.h"

// Required by 'extern registers &reg' in registers.h.
registers &reg = registers::getInstance();

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

static uint8_t status_reg()
{
	return registers::peek(SYSTEM_TIMER_STATUS);
}

// -----------------------------------------------------------------------
// Tests
// -----------------------------------------------------------------------

TEST_CASE("tick_timer: initial state", "[tick_timer]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);

	CHECK_FALSE(t.is_enabled());
	CHECK_FALSE(t.is_irq_pending());
	CHECK(t.has_interrupt() == INTERRUPT_NONE);
	CHECK(status_reg() == 0x00);
}

TEST_CASE("tick_timer: enable via control", "[tick_timer]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);

	t.control(0x01);
	CHECK(t.is_enabled());

	t.control(0x00);
	CHECK_FALSE(t.is_enabled());
}

TEST_CASE("tick_timer: bit 0 controls enable, bits 1-5 are countdown", "[tick_timer]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);

	t.control(0xC0);  // bits 6-7 only, bit 0 = 0
	CHECK_FALSE(t.is_enabled());
	CHECK(t.get_nmi_countdown() == 0);

	t.control(0x01);  // bit 0 = 1, no countdown
	CHECK(t.is_enabled());
	CHECK(t.get_nmi_countdown() == 0);
}

TEST_CASE("tick_timer: tick while disabled has no effect", "[tick_timer]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
	t.start();

	// Timer is disabled by default.
	t.simulate_tick();

	CHECK_FALSE(t.is_irq_pending());
	CHECK(t.has_interrupt() == INTERRUPT_NONE);
	CHECK(status_reg() == 0x00);
}

TEST_CASE("tick_timer: tick while enabled sets IRQ pending", "[tick_timer]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
	t.start();
	t.control(0x01);

	t.simulate_tick();

	CHECK(t.is_irq_pending());
	CHECK(t.has_interrupt() == INTERRUPT_IRQ);
	CHECK(status_reg() == 0x80);
}

TEST_CASE("tick_timer: multiple ticks while enabled", "[tick_timer]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
	t.start();
	t.control(0x01);

	t.simulate_tick();
	t.simulate_tick();
	t.simulate_tick();

	// Still pending (idempotent).
	CHECK(t.is_irq_pending());
	CHECK(t.has_interrupt() == INTERRUPT_IRQ);
	CHECK(status_reg() == 0x80);
}

TEST_CASE("tick_timer: acknowledge clears IRQ", "[tick_timer]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
	t.start();
	t.control(0x01);
	t.simulate_tick();

	t.acknowledge();

	CHECK_FALSE(t.is_irq_pending());
	CHECK(t.has_interrupt() == INTERRUPT_NONE);
	CHECK(status_reg() == 0x00);
}

TEST_CASE("tick_timer: tick after acknowledge re-asserts IRQ", "[tick_timer]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
	t.start();
	t.control(0x01);

	t.simulate_tick();
	CHECK(t.is_irq_pending());

	t.acknowledge();
	CHECK_FALSE(t.is_irq_pending());

	t.simulate_tick();
	CHECK(t.is_irq_pending());
	CHECK(t.has_interrupt() == INTERRUPT_IRQ);
	CHECK(status_reg() == 0x80);
}

TEST_CASE("tick_timer: disable clears pending IRQ", "[tick_timer]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
	t.start();
	t.control(0x01);
	t.simulate_tick();
	CHECK(t.is_irq_pending());

	t.control(0x00);

	CHECK_FALSE(t.is_enabled());
	CHECK_FALSE(t.is_irq_pending());
	CHECK(status_reg() == 0x00);
}

TEST_CASE("tick_timer: reset clears all state", "[tick_timer]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
	t.start();
	t.control(0x01);
	t.simulate_tick();

	t.reset();

	CHECK_FALSE(t.is_enabled());
	CHECK_FALSE(t.is_irq_pending());
	CHECK(t.has_interrupt() == INTERRUPT_NONE);
	CHECK(status_reg() == 0x00);
}

TEST_CASE("tick_timer: enable-disable-enable cycle has no stale IRQ", "[tick_timer]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
	t.start();

	t.control(0x01);
	t.simulate_tick();
	CHECK(t.is_irq_pending());

	t.control(0x00);
	CHECK_FALSE(t.is_irq_pending());

	t.control(0x01);
	// Re-enabling should not resurrect the old IRQ.
	CHECK_FALSE(t.is_irq_pending());
	CHECK(t.has_interrupt() == INTERRUPT_NONE);
}

TEST_CASE("tick_timer: reset then re-enable works cleanly", "[tick_timer]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
	t.start();
	t.control(0x01);
	t.simulate_tick();

	t.reset();
	t.control(0x01);
	CHECK(t.is_enabled());
	CHECK_FALSE(t.is_irq_pending());

	t.simulate_tick();
	CHECK(t.is_irq_pending());
	CHECK(t.has_interrupt() == INTERRUPT_IRQ);
}

TEST_CASE("tick_timer: different register offsets", "[tick_timer]")
{
	// Verify nothing is hardcoded to the default offsets.
	registers::clear();
	uint16_t ctrl_off = 0x0A;
	uint16_t stat_off = 0x0B;
	tick_timer t(reg, ctrl_off, stat_off);
	t.start();
	t.control(0x01);

	t.simulate_tick();

	CHECK(t.is_irq_pending());
	CHECK(registers::peek(stat_off) == 0x80);
	// Default offsets should be untouched.
	CHECK(registers::peek(SYSTEM_TIMER_STATUS) == 0x00);

	t.acknowledge();
	CHECK(registers::peek(stat_off) == 0x00);
}

TEST_CASE("tick_timer: acknowledge without prior tick is safe", "[tick_timer]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);

	// No tick fired, but 6809 reads the status register anyway.
	t.acknowledge();

	CHECK_FALSE(t.is_irq_pending());
	CHECK(status_reg() == 0x00);
}

// -----------------------------------------------------------------------
// NMI countdown tests
// -----------------------------------------------------------------------

TEST_CASE("tick_timer: countdown of 1 fires NMI after 1 tick", "[tick_timer][nmi]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);

	t.control(0x02);  // bits 1-5 = 1, countdown = 1
	CHECK(t.get_nmi_countdown() == 1);

	bool nmi = t.tick();
	CHECK(nmi);
	CHECK(t.get_nmi_countdown() == 0);
}

TEST_CASE("tick_timer: countdown of 5 fires NMI after exactly 5 ticks", "[tick_timer][nmi]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);

	t.control(0x0A);  // bits 1-5 = 5, countdown = 5
	CHECK(t.get_nmi_countdown() == 5);

	for (int i = 0; i < 4; i++) {
		CHECK_FALSE(t.tick());
		CHECK(t.get_nmi_countdown() == (uint8_t)(4 - i));
	}

	CHECK(t.tick());
	CHECK(t.get_nmi_countdown() == 0);
}

TEST_CASE("tick_timer: countdown of 31 (max) fires after 31 ticks", "[tick_timer][nmi]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);

	t.control(0x3E);  // bits 1-5 = 0x1F = 31
	CHECK(t.get_nmi_countdown() == 31);

	for (int i = 0; i < 30; i++)
		CHECK_FALSE(t.tick());

	CHECK(t.tick());
	CHECK(t.get_nmi_countdown() == 0);
}

TEST_CASE("tick_timer: tick with no countdown returns false", "[tick_timer][nmi]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);

	CHECK_FALSE(t.tick());
	CHECK_FALSE(t.tick());
}

TEST_CASE("tick_timer: countdown does not re-fire after expiry", "[tick_timer][nmi]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);

	t.control(0x02);  // countdown = 1
	CHECK(t.tick());   // fires

	// Subsequent ticks should not fire again.
	CHECK_FALSE(t.tick());
	CHECK_FALSE(t.tick());
}

TEST_CASE("tick_timer: countdown is independent of 50 Hz IRQ enable", "[tick_timer][nmi]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);

	// Countdown with IRQ disabled (bit 0 = 0)
	t.control(0x06);  // bits 1-5 = 3, bit 0 = 0
	CHECK_FALSE(t.is_enabled());
	CHECK(t.get_nmi_countdown() == 3);

	CHECK_FALSE(t.tick());
	CHECK_FALSE(t.tick());
	CHECK(t.tick());
}

TEST_CASE("tick_timer: countdown with IRQ enabled simultaneously", "[tick_timer][nmi]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
	t.start();

	// Enable IRQ (bit 0) and set countdown = 2 (bits 1-5)
	t.control(0x05);  // 0b00000101 → enable=1, countdown=2
	CHECK(t.is_enabled());
	CHECK(t.get_nmi_countdown() == 2);

	CHECK_FALSE(t.tick());
	CHECK(t.tick());

	// 50 Hz IRQ should still work independently
	t.simulate_tick();
	CHECK(t.is_irq_pending());
}

TEST_CASE("tick_timer: reset clears countdown", "[tick_timer][nmi]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);

	t.control(0x0A);  // countdown = 5
	CHECK(t.get_nmi_countdown() == 5);

	t.reset();
	CHECK(t.get_nmi_countdown() == 0);
	CHECK_FALSE(t.tick());
}

TEST_CASE("tick_timer: writing zero countdown does not arm", "[tick_timer][nmi]")
{
	registers::clear();
	tick_timer t(reg, SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);

	t.control(0x01);  // enable IRQ, bits 1-5 = 0
	CHECK(t.get_nmi_countdown() == 0);
	CHECK_FALSE(t.tick());
}
