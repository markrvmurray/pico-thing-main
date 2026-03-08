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

// Matches pico_thing.cpp constants.
static constexpr uint8_t PEND_NMI  = 1u << 0;
static constexpr uint8_t PEND_FIRQ = 1u << 1;
static constexpr uint8_t PEND_IRQ  = 1u << 2;

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
	volatile uint8_t pending = 0;
	tick_timer t(reg, pending, PEND_IRQ,
	             SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);

	CHECK_FALSE(t.is_enabled());
	CHECK_FALSE(t.is_irq_pending());
	CHECK(t.has_interrupt() == INTERRUPT_NONE);
	CHECK(status_reg() == 0x00);
}

TEST_CASE("tick_timer: enable via control", "[tick_timer]")
{
	registers::clear();
	volatile uint8_t pending = 0;
	tick_timer t(reg, pending, PEND_IRQ,
	             SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);

	t.control(0x01);
	CHECK(t.is_enabled());

	t.control(0x00);
	CHECK_FALSE(t.is_enabled());
}

TEST_CASE("tick_timer: only bit 0 of control matters", "[tick_timer]")
{
	registers::clear();
	volatile uint8_t pending = 0;
	tick_timer t(reg, pending, PEND_IRQ,
	             SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);

	t.control(0xFE);  // bit 0 = 0
	CHECK_FALSE(t.is_enabled());

	t.control(0xFF);  // bit 0 = 1
	CHECK(t.is_enabled());
}

TEST_CASE("tick_timer: tick while disabled has no effect", "[tick_timer]")
{
	registers::clear();
	volatile uint8_t pending = 0;
	tick_timer t(reg, pending, PEND_IRQ,
	             SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
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
	volatile uint8_t pending = 0;
	tick_timer t(reg, pending, PEND_IRQ,
	             SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
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
	volatile uint8_t pending = 0;
	tick_timer t(reg, pending, PEND_IRQ,
	             SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
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
	volatile uint8_t pending = PEND_IRQ;
	tick_timer t(reg, pending, PEND_IRQ,
	             SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
	t.start();
	t.control(0x01);
	t.simulate_tick();

	t.acknowledge();

	CHECK_FALSE(t.is_irq_pending());
	CHECK(t.has_interrupt() == INTERRUPT_NONE);
	CHECK(status_reg() == 0x00);
	CHECK((pending & PEND_IRQ) == 0);
}

TEST_CASE("tick_timer: acknowledge preserves other pending bits", "[tick_timer]")
{
	registers::clear();
	volatile uint8_t pending = PEND_IRQ | PEND_NMI | PEND_FIRQ;
	tick_timer t(reg, pending, PEND_IRQ,
	             SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
	t.start();
	t.control(0x01);
	t.simulate_tick();

	t.acknowledge();

	CHECK((pending & PEND_IRQ) == 0);
	CHECK((pending & PEND_NMI) != 0);
	CHECK((pending & PEND_FIRQ) != 0);
}

TEST_CASE("tick_timer: tick after acknowledge re-asserts IRQ", "[tick_timer]")
{
	registers::clear();
	volatile uint8_t pending = 0;
	tick_timer t(reg, pending, PEND_IRQ,
	             SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
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
	volatile uint8_t pending = 0;
	tick_timer t(reg, pending, PEND_IRQ,
	             SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
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
	volatile uint8_t pending = 0;
	tick_timer t(reg, pending, PEND_IRQ,
	             SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
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
	volatile uint8_t pending = 0;
	tick_timer t(reg, pending, PEND_IRQ,
	             SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
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
	volatile uint8_t pending = 0;
	tick_timer t(reg, pending, PEND_IRQ,
	             SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
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
	volatile uint8_t pending = 0;
	uint16_t ctrl_off = 0x0A;
	uint16_t stat_off = 0x0B;
	tick_timer t(reg, pending, PEND_IRQ, ctrl_off, stat_off);
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

TEST_CASE("tick_timer: different pending bit", "[tick_timer]")
{
	// Verify the timer clears the correct bit, not a hardcoded one.
	registers::clear();
	uint8_t MY_BIT = 1u << 5;
	volatile uint8_t pending = 0xFF;
	tick_timer t(reg, pending, MY_BIT,
	             SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);

	t.acknowledge();

	CHECK((pending & MY_BIT) == 0);
	CHECK((pending & ~MY_BIT) == (0xFF & ~MY_BIT));
}

TEST_CASE("tick_timer: acknowledge without prior tick is safe", "[tick_timer]")
{
	registers::clear();
	volatile uint8_t pending = PEND_IRQ;
	tick_timer t(reg, pending, PEND_IRQ,
	             SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);

	// No tick fired, but 6809 reads the status register anyway.
	t.acknowledge();

	CHECK_FALSE(t.is_irq_pending());
	CHECK(status_reg() == 0x00);
	CHECK((pending & PEND_IRQ) == 0);
}
