/**
 * Host-side unit tests for mc6840.cpp.
 *
 * Build: cmake -S .. -B build && cmake --build build (from tests/)
 * The CMakeLists.txt puts tests/fake/ first in the include path so
 * mc6840.cpp picks up stub Pico headers and the simplified registers class.
 *
 * Control register bit layout (mc6840_control):
 *   bit0  — special: CR3 = Timer Reset (TR); CR2 = CR1/CR3 select; CR1 = T1 prescaler
 *   bit1  — clk_sel (0 = E clock, 1 = external; both work in the emulation)
 *   bit2  — bits_8 (0 = 16-bit, 1 = dual 8-bit)
 *   bit3-5 — mode  (CONT_0=0, SINGLE_0=4, …)
 *   bit6  — irq_en
 *   bit7  — out_en
 *
 * MC6840 write offsets (relative to SYSTEM_TIMER_BASE):
 *   0 — CONTROL_13 (writes CR1 when CR2.bit0=1, else CR3)
 *   1 — CONTROL_2
 *   2..7 — Timer MSB/LSB for timers 1/2/3
 *
 * Timer reset: write CR3.bit0=1 via CONTROL_13 (with CR2.bit0=0).
 *   Clears all IRQ flags, stops all timers.  Re-write timer LSBs to restart.
 *
 * To run Timer N in CONT_0 16-bit mode with IRQ:
 *   Write CR2 = 0x42  (irq_en=1, mode=CONT_0, 16-bit; clk_sel bit is ignored)
 *   Write Timer-N MSB
 *   Write Timer-N LSB  ← triggers initialise(), sets running=true
 *   Period = latch_value + 1 ticks (datasheet N+1 rule)
 */

#include <sys/types.h>  // uint (POSIX, matches how test_srec.cpp handles it)
#include <cstdint>
#include <catch2/catch_test_macros.hpp>

// fake/ overrides come first (fake/registers.h, fake/pico_thing.h, …)
#include "registers.h"
#include "pico_thing.h"
#include "mc6809.h"
#include "mc6840.h"

// Required by the 'extern registers &reg' declaration in registers.h.
registers &reg = registers::getInstance();

// Convenience: write to a timer register via the same offset arithmetic
// that pico_thing.cpp uses.
static inline void tw(mc6840 &t, uint16_t reg_addr, uint8_t val)
{
	t.write(reg_addr - SYSTEM_TIMER_BASE, val);
}

static inline void tr(mc6840 &t, uint16_t reg_addr)
{
	t.read(reg_addr - SYSTEM_TIMER_BASE);
}

// CR2 value for CONT_0 16-bit mode, IRQ enabled.
// bit6=irq_en, bit1=clk_sel (harmless; E clock is always used), rest=0 → 0x42
static constexpr uint8_t CR2_CONT0_IRQ = 0x42u;

// Helper: construct a fresh mc6840 with cleared register state.
static mc6840 make_timer()
{
	registers::clear();
	return mc6840(registers::getInstance(), 1u);
}

// ============================================================
//  Write offset fix (was the pre-existing silent bug)
// ============================================================

TEST_CASE("MC6840 write() offsets are correct", "[mc6840][offset]")
{
	// Before the fix, all switch cases in write() used raw register
	// addresses (0x08-0x0F) but the parameter was already subtracted
	// (offset 0-7), so every write fell through to 'default: break'.
	// This test verifies the fix by checking that the timer actually starts
	// running after a proper initialisation sequence.

	auto t = make_timer();

	// Configure Timer 2: CR2 = CONT_0, 16-bit, irq_en, e_clk
	tw(t, SYSTEM_TIMER_CONTROL_2, CR2_CONT0_IRQ);

	// Write latch = 10 and start.
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_2_LSB, 10u);  // triggers initialise(CTR2)

	// Tick once — if write() was silently broken, the timer would
	// never have started and the counter would stay at 0.
	t.tick(1u);

	// After 1 tick the status register must NOT yet show irq (fires at 11, N+1=10+1).
	uint8_t status = registers::peek(SYSTEM_TIMER_STATUS);
	REQUIRE((status & 0x80u) == 0);  // composite irq flag not set

	// Tick the remaining 10 — timer should fire on the 11th tick total.
	for (int i = 0; i < 10; i++)
		t.tick(1u);

	status = registers::peek(SYSTEM_TIMER_STATUS);
	REQUIRE((status & 0x80u) != 0);  // irq flag set after 11 ticks
}

// ============================================================
//  CONT_0 fires at the correct interval
// ============================================================

TEST_CASE("MC6840 CONT_0 fires after exactly N+1 ticks", "[mc6840][cont0]")
{
	auto t = make_timer();
	tw(t, SYSTEM_TIMER_CONTROL_2, CR2_CONT0_IRQ);
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_2_LSB, 20u);   // latch = 20; fires at tick 21 (N+1)

	// 20 ticks — must not fire
	for (int i = 0; i < 20; i++)
		t.tick(1u);
	REQUIRE(t.has_interrupt() == INTERRUPT_NONE);

	// 21st tick — must fire
	t.tick(1u);
	REQUIRE(t.has_interrupt() == INTERRUPT_IRQ);
}

TEST_CASE("MC6840 CONT_0 fires with a 16-bit period", "[mc6840][cont0]")
{
	auto t = make_timer();
	tw(t, SYSTEM_TIMER_CONTROL_2, CR2_CONT0_IRQ);
	tw(t, SYSTEM_TIMER_2_MSB, 0x01u);   // latch = 0x0100 = 256; fires at tick 257 (N+1)
	tw(t, SYSTEM_TIMER_2_LSB, 0x00u);

	for (int i = 0; i < 256; i++)
		t.tick(1u);
	REQUIRE(t.has_interrupt() == INTERRUPT_NONE);

	t.tick(1u);
	REQUIRE(t.has_interrupt() == INTERRUPT_IRQ);
}

// ============================================================
//  has_interrupt() returns IRQ (not FIRQ — mc6840 uses IRQ; FIRQ is for the UART)
// ============================================================

TEST_CASE("MC6840 has_interrupt() returns INTERRUPT_IRQ", "[mc6840][interrupt]")
{
	auto t = make_timer();
	tw(t, SYSTEM_TIMER_CONTROL_2, CR2_CONT0_IRQ);
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_2_LSB, 5u);

	for (int i = 0; i < 6; i++)  // N+1 = 6 ticks for latch=5
		t.tick(1u);

	interrupt irq = t.has_interrupt();
	REQUIRE(irq == INTERRUPT_IRQ);
	REQUIRE(irq != INTERRUPT_FIRQ);
	REQUIRE(irq != INTERRUPT_NONE);
}

// ============================================================
//  has_interrupt() is level-sensitive (does not auto-clear)
// ============================================================

TEST_CASE("MC6840 IRQ stays pending until LSB is read", "[mc6840][interrupt]")
{
	auto t = make_timer();
	tw(t, SYSTEM_TIMER_CONTROL_2, CR2_CONT0_IRQ);
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_2_LSB, 5u);

	for (int i = 0; i < 6; i++)  // N+1 = 6
		t.tick(1u);

	REQUIRE(t.has_interrupt() == INTERRUPT_IRQ);
	REQUIRE(t.has_interrupt() == INTERRUPT_IRQ);  // still pending
	REQUIRE(t.has_interrupt() == INTERRUPT_IRQ);  // still pending
}

// ============================================================
//  read() LSB clears the IRQ flag
// ============================================================

TEST_CASE("MC6840 read() timer LSB clears IRQ", "[mc6840][read]")
{
	auto t = make_timer();
	tw(t, SYSTEM_TIMER_CONTROL_2, CR2_CONT0_IRQ);
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_2_LSB, 5u);

	for (int i = 0; i < 6; i++)  // N+1 = 6
		t.tick(1u);

	REQUIRE(t.has_interrupt() == INTERRUPT_IRQ);

	// Reading the LSB is the MC6840 IRQ-acknowledge mechanism.
	tr(t, SYSTEM_TIMER_2_LSB);

	REQUIRE(t.has_interrupt() == INTERRUPT_NONE);

	// Status register must also reflect the cleared state.
	uint8_t status = registers::peek(SYSTEM_TIMER_STATUS);
	REQUIRE((status & 0x80u) == 0);  // composite irq flag
	REQUIRE((status & 0x02u) == 0);  // irq2 flag (bit 1)
}

TEST_CASE("MC6840 reading MSB alone does not clear IRQ", "[mc6840][read]")
{
	auto t = make_timer();
	tw(t, SYSTEM_TIMER_CONTROL_2, CR2_CONT0_IRQ);
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_2_LSB, 5u);

	for (int i = 0; i < 6; i++)  // N+1 = 6
		t.tick(1u);

	REQUIRE(t.has_interrupt() == INTERRUPT_IRQ);
	tr(t, SYSTEM_TIMER_2_MSB);  // latch snapshot only — must not clear
	REQUIRE(t.has_interrupt() == INTERRUPT_IRQ);
}

// ============================================================
//  CONT_0 is continuous (reloads and fires again)
// ============================================================

TEST_CASE("MC6840 CONT_0 fires continuously", "[mc6840][cont0]")
{
	auto t = make_timer();
	tw(t, SYSTEM_TIMER_CONTROL_2, CR2_CONT0_IRQ);
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_2_LSB, 10u);

	for (int fire = 0; fire < 5; fire++) {
		for (int i = 0; i < 11; i++)  // N+1 = 11 ticks per period
			t.tick(1u);
		REQUIRE(t.has_interrupt() == INTERRUPT_IRQ);
		tr(t, SYSTEM_TIMER_2_LSB);  // acknowledge
		REQUIRE(t.has_interrupt() == INTERRUPT_NONE);
	}
}

// ============================================================
//  Multiple timers operate independently
// ============================================================

TEST_CASE("MC6840 Timer 2 and Timer 3 are independent", "[mc6840][multi]")
{
	// Timer 2: period 10, Timer 3: period 30.
	// We need CR1 to configure CTR3.  Write CR2.bit0=0 (selects CR3 via CONTROL_13).
	auto t = make_timer();

	// Timer 2: CONT_0, 16-bit, irq_en via CR2
	tw(t, SYSTEM_TIMER_CONTROL_2, CR2_CONT0_IRQ);
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_2_LSB, 10u);

	// Timer 3: CONT_0, 16-bit, irq_en via CONTROL_13 (CR2.bit0=0 → CR3)
	// CR2.bit0 is 0 (CR2_CONT0_IRQ & 1 = 0), so CONTROL_13 writes → CR3.
	tw(t, SYSTEM_TIMER_CONTROL_13, CR2_CONT0_IRQ);  // sets control[CTR3]
	tw(t, SYSTEM_TIMER_3_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_3_LSB, 30u);

	// After 11 ticks: Timer 2 fired (N+1=11), Timer 3 has not (fires at 31).
	for (int i = 0; i < 11; i++)
		t.tick(1u);

	uint8_t status = registers::peek(SYSTEM_TIMER_STATUS);
	REQUIRE((status & 0x02u) != 0);  // irq2 set
	REQUIRE((status & 0x04u) == 0);  // irq3 not set

	// Acknowledge Timer 2.
	tr(t, SYSTEM_TIMER_2_LSB);

	// After 20 more ticks (total 31): Timer 2 re-fired at tick 22, Timer 3 fired at 31.
	for (int i = 0; i < 20; i++)
		t.tick(1u);

	status = registers::peek(SYSTEM_TIMER_STATUS);
	REQUIRE((status & 0x02u) != 0);  // irq2 set again
	REQUIRE((status & 0x04u) != 0);  // irq3 now set
}

// ============================================================
//  IRQ-disabled timer does not interrupt
// ============================================================

TEST_CASE("MC6840 timer with irq_en=0 does not generate IRQ", "[mc6840][irq]")
{
	auto t = make_timer();

	// CR2 without irq_en: clk_sel=1 (harmless), irq_en=0 → 0x02
	tw(t, SYSTEM_TIMER_CONTROL_2, 0x02u);
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_2_LSB, 5u);

	for (int i = 0; i < 5; i++)
		t.tick(1u);

	// Counter wrapped but irq_en=0 so no interrupt.
	REQUIRE(t.has_interrupt() == INTERRUPT_NONE);
}

// ============================================================
//  Status register tracks irq bits correctly
// ============================================================

TEST_CASE("MC6840 status register irq bits", "[mc6840][status]")
{
	auto t = make_timer();
	tw(t, SYSTEM_TIMER_CONTROL_2, CR2_CONT0_IRQ);
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_2_LSB, 3u);

	for (int i = 0; i < 4; i++)  // N+1 = 4 ticks for latch=3
		t.tick(1u);

	// bit7 = composite irq, bit1 = irq2 (Timer 2)
	uint8_t s = registers::peek(SYSTEM_TIMER_STATUS);
	REQUIRE((s & 0x80u) != 0);
	REQUIRE((s & 0x02u) != 0);
	REQUIRE((s & 0x01u) == 0);  // irq1 not set
	REQUIRE((s & 0x04u) == 0);  // irq3 not set

	// After LSB read the status register is updated in place.
	tr(t, SYSTEM_TIMER_2_LSB);
	s = registers::peek(SYSTEM_TIMER_STATUS);
	REQUIRE((s & 0x80u) == 0);
	REQUIRE((s & 0x02u) == 0);
}

// ============================================================
//  Timer 1 SINGLE_0 dual 8-bit — NMI single-step mode
//
//  Control register $A6: out_en=1, SINGLE_0, bits_8=1, clk_sel=1, irq_en=0
//  (bit layout: bit7=out_en bit6=irq_en bits3-5=mode bit2=bits_8 bit1=clk_sel bit0=special)
//
//  Dual 8-bit SINGLE_0 semantics (real MC6840 and our emulation):
//    Phase 1 — UPPER counts the delay before NMI asserts.
//    Phase 2 — LOWER counts the NMI pulse width.
//  ASSIST09 programs $0701: 7 cycles of delay, 1 cycle of NMI pulse.
//
//  Setup sequence mirrors ASSIST09 ZMONT2:
//    Write CR2=0x01 (bit0=1 → next CONTROL_13 write goes to CR1)
//    Write CONTROL_13=0xA6 (→ CR1: single-shot, dual 8-bit, clk_sel, out_en)
//    Write CR2=0x00 (clear bit0)
// ============================================================

static mc6840 make_nmi_timer()
{
	registers::clear();
	mc6840 t(registers::getInstance(), 1u);
	tw(t, SYSTEM_TIMER_CONTROL_2, 0x01u);   // CR2.bit0=1 → route CONTROL_13 to CR1
	tw(t, SYSTEM_TIMER_CONTROL_13, 0xA6u);  // CR1: SINGLE_0, bits_8, clk_sel, out_en
	tw(t, SYSTEM_TIMER_CONTROL_2, 0x00u);   // clear CR2
	return t;
}

TEST_CASE("MC6840 Timer1 SINGLE_0 dual-8bit: NMI fires after UPPER delay", "[mc6840][nmi]")
{
	// UPPER=5 (delay), LOWER=2 (pulse width).
	auto t = make_nmi_timer();
	tw(t, SYSTEM_TIMER_1_MSB, 5u);
	tw(t, SYSTEM_TIMER_1_LSB, 2u);  // starts timer

	// Ticks 1-4: UPPER counting down (4 → 0 not yet), NMI must not fire.
	for (int i = 0; i < 4; i++) {
		t.tick(1u);
		REQUIRE(t.has_interrupt() == INTERRUPT_NONE);
	}

	// Tick 5: UPPER reaches 0 → NMI asserts.
	t.tick(1u);
	REQUIRE(t.has_interrupt() == INTERRUPT_NMI);
}

TEST_CASE("MC6840 Timer1 SINGLE_0 dual-8bit: NMI clears after LOWER pulse", "[mc6840][nmi]")
{
	// UPPER=3 (delay), LOWER=1 (pulse width).
	auto t = make_nmi_timer();
	tw(t, SYSTEM_TIMER_1_MSB, 3u);
	tw(t, SYSTEM_TIMER_1_LSB, 1u);

	// Advance through delay phase (3 ticks).
	for (int i = 0; i < 3; i++)
		t.tick(1u);

	// NMI asserts (UPPER just hit 0).
	t.tick(1u);
	REQUIRE(t.has_interrupt() == INTERRUPT_NMI);

	// Advance through pulse phase: LOWER counts from 1 to 0.
	t.tick(1u);

	// After LOWER reaches 0, NMI de-asserts and timer stops.
	t.tick(1u);
	REQUIRE(t.has_interrupt() == INTERRUPT_NONE);
}

TEST_CASE("MC6840 Timer1 SINGLE_0 dual-8bit: ASSIST09 $0701 values", "[mc6840][nmi]")
{
	// ASSIST09 CTRCE3: LDD #$0701 / STD PTMTM1 → UPPER=$07, LOWER=$01
	// "7 cycles DOWN + 1 cycle UP" = 7-tick delay then 1-tick NMI pulse.
	auto t = make_nmi_timer();
	tw(t, SYSTEM_TIMER_1_MSB, 0x07u);
	tw(t, SYSTEM_TIMER_1_LSB, 0x01u);

	// Ticks 1-6: UPPER counting (7→1), NMI must not fire.
	for (int i = 0; i < 6; i++) {
		t.tick(1u);
		REQUIRE(t.has_interrupt() == INTERRUPT_NONE);
	}

	// Tick 7: UPPER reaches 0 → NMI asserts.
	t.tick(1u);
	REQUIRE(t.has_interrupt() == INTERRUPT_NMI);

	// Advance through NMI pulse and beyond → NMI must clear.
	t.tick(1u);
	t.tick(1u);
	t.tick(1u);
	REQUIRE(t.has_interrupt() == INTERRUPT_NONE);
}

TEST_CASE("MC6840 Timer1 SINGLE_0 dual-8bit: single shot (does not repeat)", "[mc6840][nmi]")
{
	auto t = make_nmi_timer();
	tw(t, SYSTEM_TIMER_1_MSB, 2u);
	tw(t, SYSTEM_TIMER_1_LSB, 1u);

	// Run through the entire shot plus extra ticks.
	for (int i = 0; i < 20; i++)
		t.tick(1u);

	// After the single shot completes, NMI must stay clear.
	REQUIRE(t.has_interrupt() == INTERRUPT_NONE);
}

TEST_CASE("MC6840 Timer1 SINGLE_0 dual-8bit: re-arm fires again (ASSIST09 single-step pattern)", "[mc6840][nmi]")
{
	// ASSIST09 re-arms Timer 1 at the end of every NMI handler by writing
	// $07 then $01 to PTMTM1.  Verify that the second shot is identical to
	// the first: 6 ticks quiet, tick 7 = NMI, clears after the pulse.

	auto t = make_nmi_timer();

	// --- First shot ---
	tw(t, SYSTEM_TIMER_1_MSB, 0x07u);
	tw(t, SYSTEM_TIMER_1_LSB, 0x01u);

	// Run well past the end of the pulse so the timer has fully stopped.
	for (int i = 0; i < 12; i++)
		t.tick(1u);
	REQUIRE(t.has_interrupt() == INTERRUPT_NONE);  // clean starting state

	// --- Re-arm (mirrors what ASSIST09 does at the end of its NMI handler) ---
	tw(t, SYSTEM_TIMER_1_MSB, 0x07u);
	tw(t, SYSTEM_TIMER_1_LSB, 0x01u);

	// Ticks 1-6: delay phase, NMI must not fire.
	for (int i = 0; i < 6; i++) {
		t.tick(1u);
		REQUIRE(t.has_interrupt() == INTERRUPT_NONE);
	}

	// Tick 7: UPPER reaches 0 → NMI asserts.
	t.tick(1u);
	REQUIRE(t.has_interrupt() == INTERRUPT_NMI);

	// Run past the end of the pulse → NMI must clear.
	t.tick(1u);
	t.tick(1u);
	t.tick(1u);
	REQUIRE(t.has_interrupt() == INTERRUPT_NONE);
}

TEST_CASE("MC6840 Timer1 SINGLE_0 dual-8bit: re-arm during NMI pulse resets cleanly", "[mc6840][nmi]")
{
	// Edge case: if the Pico re-arms the timer while the NMI pulse is still
	// active (tick 8 in the $0701 sequence), initialise() must clear
	// nmi_pending and restart the delay counter correctly.

	auto t = make_nmi_timer();
	tw(t, SYSTEM_TIMER_1_MSB, 0x07u);
	tw(t, SYSTEM_TIMER_1_LSB, 0x01u);

	// Advance to tick 8: NMI pulse is active (UPPER=0, LOWER just hit 0).
	for (int i = 0; i < 8; i++)
		t.tick(1u);
	REQUIRE(t.has_interrupt() == INTERRUPT_NMI);  // in the pulse

	// Re-arm while NMI is still pending — initialise() must clear it.
	tw(t, SYSTEM_TIMER_1_MSB, 0x07u);
	tw(t, SYSTEM_TIMER_1_LSB, 0x01u);
	REQUIRE(t.has_interrupt() == INTERRUPT_NONE);  // cleared by initialise()

	// The new delay must run for the correct 7 ticks.
	for (int i = 0; i < 6; i++) {
		t.tick(1u);
		REQUIRE(t.has_interrupt() == INTERRUPT_NONE);
	}
	t.tick(1u);
	REQUIRE(t.has_interrupt() == INTERRUPT_NMI);
}

// ============================================================
//  CR1/CR3 routing via CONTROL_13
// ============================================================

TEST_CASE("MC6840 CR2.bit0=1 routes CONTROL_13 write to CR1", "[mc6840][routing]")
{
	// When CR2.bit0=1, a write to $FFC8 (CONTROL_13) goes to CR1, not CR3.
	// Verify by configuring Timer 1 via CR1 and checking it fires.
	auto t = make_timer();

	tw(t, SYSTEM_TIMER_CONTROL_2, 0x01u);           // CR2.bit0=1 → route to CR1
	tw(t, SYSTEM_TIMER_CONTROL_13, CR2_CONT0_IRQ);  // CR1 = $42
	tw(t, SYSTEM_TIMER_CONTROL_2, 0x00u);           // clear routing bit

	tw(t, SYSTEM_TIMER_1_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_1_LSB, 5u);  // starts CTR1

	for (int i = 0; i < 6; i++)  // N+1 = 6 ticks for latch=5
		t.tick(1u);

	uint8_t status = registers::peek(SYSTEM_TIMER_STATUS);
	REQUIRE((status & 0x01u) != 0);  // irq1 set — CR1 routing was correct
	REQUIRE((status & 0x04u) == 0);  // irq3 not set — CR3 was not touched
}

TEST_CASE("MC6840 CR2.bit0=0 routes CONTROL_13 write to CR3", "[mc6840][routing]")
{
	// When CR2.bit0=0 (the default), a write to $FFC8 goes to CR3.
	// Verify by configuring Timer 3 via CR3 and checking it fires.
	auto t = make_timer();

	// CR2 starts at $00 (bit0=0) from make_timer() — routes to CR3.
	tw(t, SYSTEM_TIMER_CONTROL_13, CR2_CONT0_IRQ);  // CR3 = $42

	tw(t, SYSTEM_TIMER_3_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_3_LSB, 5u);  // starts CTR3

	for (int i = 0; i < 6; i++)  // N+1 = 6 ticks for latch=5
		t.tick(1u);

	uint8_t status = registers::peek(SYSTEM_TIMER_STATUS);
	REQUIRE((status & 0x04u) != 0);  // irq3 set — CR3 routing was correct
	REQUIRE((status & 0x01u) == 0);  // irq1 not set — CR1 was not touched
}

// ============================================================
//  Latch readback
// ============================================================

TEST_CASE("MC6840 Timer 2 counter registers reflect written latch after initialise", "[mc6840][plumbing]")
{
	// Writing T2MSB then T2LSB triggers initialise(), which loads the
	// latch into the counter and writes it back to the read registers.
	auto t = make_timer();

	tw(t, SYSTEM_TIMER_CONTROL_2, 0x02u);  // clk_sel=1 (bit1), irq_en=0
	tw(t, SYSTEM_TIMER_2_MSB, 0x4Eu);
	tw(t, SYSTEM_TIMER_2_LSB, 0x1Fu);     // triggers initialise()

	REQUIRE(registers::peek(SYSTEM_TIMER_2_MSB) == 0x4Eu);
	REQUIRE(registers::peek(SYSTEM_TIMER_2_LSB) == 0x1Fu);
}

// ============================================================
//  Soft reset
// ============================================================


TEST_CASE("MC6840 timer reset clears pending IRQ", "[mc6840][reset]")
{
	// Timer reset (CR3.bit0=1) clears all IRQ flags immediately.
	auto t = make_timer();

	// Let Timer 2 fire (latch=3, fires at tick 4 with N+1).
	tw(t, SYSTEM_TIMER_CONTROL_2, CR2_CONT0_IRQ);
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_2_LSB, 3u);

	for (int i = 0; i < 4; i++)  // N+1 = 4 ticks
		t.tick(1u);

	REQUIRE(t.has_interrupt() == INTERRUPT_IRQ);

	// Timer reset: write CR3.bit0=1 via CONTROL_13.
	// CR2 was written with CR2_CONT0_IRQ (bit0=0) so CONTROL_13 routes to CR3.
	tw(t, SYSTEM_TIMER_CONTROL_13, 0x01u);  // CR3.bit0=1 → timer reset

	REQUIRE(t.has_interrupt() == INTERRUPT_NONE);
}

TEST_CASE("MC6840 timer reset does not itself assert any interrupt", "[mc6840][reset]")
{
	// Timer reset of an idle timer must not spuriously assert IRQ or NMI.
	auto t = make_timer();

	tw(t, SYSTEM_TIMER_CONTROL_2, CR2_CONT0_IRQ);
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_2_LSB, 100u);  // long period — won't fire

	t.tick(5u);  // a few ticks, well before underflow

	// Timer reset: CR3.bit0=1 via CONTROL_13 (CR2.bit0=0 routes there).
	tw(t, SYSTEM_TIMER_CONTROL_13, 0x01u);  // CR3.bit0=1 → timer reset

	REQUIRE(t.has_interrupt() == INTERRUPT_NONE);
}

// ============================================================
//  Writing MSB alone does not start the timer
// ============================================================

TEST_CASE("MC6840 writing T2MSB alone does not start Timer 2", "[mc6840][start]")
{
	// Initialise is only triggered by the LSB write.  Writing only the MSB
	// must leave the timer stopped and the IRQ flag clear.
	auto t = make_timer();

	tw(t, SYSTEM_TIMER_CONTROL_2, CR2_CONT0_IRQ);
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);  // MSB only — no initialise()

	for (int i = 0; i < 100; i++)
		t.tick(1u);

	REQUIRE(t.has_interrupt() == INTERRUPT_NONE);
	REQUIRE((registers::peek(SYSTEM_TIMER_STATUS) & 0x80u) == 0);
}

// ============================================================
//  Coherent 16-bit counter read
// ============================================================

TEST_CASE("MC6840 MSB read latches coherent LSB against concurrent tick", "[mc6840][latch]")
{
	// Latch = 5; period = 6 ticks (N+1).  After 4 ticks the counter is at 1.
	// Simulating what the 6809 does:
	//   1. DMA reads T2MSB (already in register as 0x00).
	//   2. ISR calls timer.read(T2MSB) which snapshots {0x00,0x01} and
	//      pre-loads the T2LSB register with the latched 0x01.
	//   3. Tick 5 fires: counter decrements 1 → 0 (NOT yet reloaded).
	//      Because latch_active=true, the T2LSB register is NOT updated.
	//   4. 6809 reads T2LSB — gets 0x01 (coherent with the snapshot MSB).
	//   5. After latch release, tick 6 detects counter==0 → reloads to 5.
	auto t = make_timer();
	tw(t, SYSTEM_TIMER_CONTROL_2, CR2_CONT0_IRQ);
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_2_LSB, 5u);         // latch = 5

	// Count down to 1 (4 ticks: 5→4→3→2→1).
	for (int i = 0; i < 4; i++)
		t.tick(1u);

	REQUIRE(registers::peek(SYSTEM_TIMER_2_MSB) == 0x00u);
	REQUIRE(registers::peek(SYSTEM_TIMER_2_LSB) == 0x01u);

	// ISR response: latch snapshot, pre-load T2LSB register.
	tr(t, SYSTEM_TIMER_2_MSB);

	// One more tick: counter decrements 1 → 0 (not yet reloaded; latch protects LSB).
	t.tick(1u);

	REQUIRE(registers::peek(SYSTEM_TIMER_2_MSB) == 0x00u);  // 0's MSB
	REQUIRE(registers::peek(SYSTEM_TIMER_2_LSB) == 0x01u);  // latched — NOT 0x00

	// Release latch.  Next tick detects counter==0 → reload to 5.
	tr(t, SYSTEM_TIMER_2_LSB);
	t.tick(1u);
	REQUIRE(registers::peek(SYSTEM_TIMER_2_LSB) == 0x05u);  // just reloaded
	t.tick(1u);
	REQUIRE(registers::peek(SYSTEM_TIMER_2_LSB) == 0x04u);  // first decrement: 5 → 4
}

// ============================================================
//  Timer 2 ack does not disturb Timer 1
// ============================================================

TEST_CASE("MC6840 Timer 2 IRQ ack does not affect Timer 1", "[mc6840][multi]")
{
	// Timer 1: period = 20 (via CR1).  Timer 2: period = 5 (via CR2).
	// After Timer 2 fires and is acknowledged, Timer 1 must continue
	// counting independently and fire at tick 20.
	auto t = make_timer();

	// Configure Timer 1 via CR1 routing.
	tw(t, SYSTEM_TIMER_CONTROL_2, 0x01u);
	tw(t, SYSTEM_TIMER_CONTROL_13, CR2_CONT0_IRQ);  // CR1 = $42
	tw(t, SYSTEM_TIMER_CONTROL_2, 0x00u);

	tw(t, SYSTEM_TIMER_1_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_1_LSB, 20u);  // starts CTR1

	// Configure Timer 2.
	tw(t, SYSTEM_TIMER_CONTROL_2, CR2_CONT0_IRQ);
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_2_LSB, 5u);   // starts CTR2

	// Tick 6: Timer 2 fires (latch=5, N+1=6).
	for (int i = 0; i < 6; i++)
		t.tick(1u);

	uint8_t status = registers::peek(SYSTEM_TIMER_STATUS);
	REQUIRE((status & 0x02u) != 0);  // irq2 set
	REQUIRE((status & 0x01u) == 0);  // irq1 not yet (only 6 of 21 ticks)

	// Acknowledge Timer 2.
	tr(t, SYSTEM_TIMER_2_MSB);
	tr(t, SYSTEM_TIMER_2_LSB);

	status = registers::peek(SYSTEM_TIMER_STATUS);
	REQUIRE((status & 0x02u) == 0);  // irq2 cleared
	REQUIRE((status & 0x01u) == 0);  // irq1 still not set

	// Tick 15 more (total 21): Timer 1 fires (latch=20, N+1=21).
	for (int i = 0; i < 15; i++)
		t.tick(1u);

	status = registers::peek(SYSTEM_TIMER_STATUS);
	REQUIRE((status & 0x01u) != 0);  // irq1 now set
	REQUIRE((status & 0x80u) != 0);  // composite set
}

// ============================================================
//  Latch value preserved across reloads
// ============================================================

TEST_CASE("MC6840 CONT_0 latch value is preserved across multiple reloads", "[mc6840][cont0]")
{
	// After 3 fire-and-ack cycles, reading the counter immediately after
	// the third ack must still give the original latch value (just reloaded).
	auto t = make_timer();
	tw(t, SYSTEM_TIMER_CONTROL_2, CR2_CONT0_IRQ);
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_2_LSB, 10u);  // latch = 10

	for (int fire = 0; fire < 3; fire++) {
		for (int i = 0; i < 11; i++)  // N+1 = 11 ticks per period (latch=10)
			t.tick(1u);
		REQUIRE(t.has_interrupt() == INTERRUPT_IRQ);
		tr(t, SYSTEM_TIMER_2_MSB);
		tr(t, SYSTEM_TIMER_2_LSB);  // acknowledge
	}

	// Snapshot immediately after the third ack (no additional ticks).
	// The counter was just reloaded from the latch to 10 on the 11th tick.
	tr(t, SYSTEM_TIMER_2_MSB);
	REQUIRE(registers::peek(SYSTEM_TIMER_2_MSB) == 0x00u);
	REQUIRE(registers::peek(SYSTEM_TIMER_2_LSB) == 0x0Au);  // latch = 10
	tr(t, SYSTEM_TIMER_2_LSB);  // release latch
}

// ============================================================
//  Timer 2 IRQ does not assert NMI
// ============================================================

TEST_CASE("MC6840 Timer 2 CONT_0 IRQ does not assert NMI", "[mc6840][isolation]")
{
	auto t = make_timer();
	tw(t, SYSTEM_TIMER_CONTROL_2, CR2_CONT0_IRQ);
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_2_LSB, 5u);

	for (int i = 0; i < 6; i++)  // N+1 = 6
		t.tick(1u);

	interrupt irq = t.has_interrupt();
	REQUIRE(irq == INTERRUPT_IRQ);
	REQUIRE(irq != INTERRUPT_NMI);   // NMI line must remain deasserted
}

// ============================================================
//  SINGLE_0 16-bit mode (newly implemented)
// ============================================================

// Control byte for SINGLE_0 16-bit mode with IRQ enabled:
//   bits 3-5 = mode = SINGLE_0 = 4 = 0b100 → bit5=1 → 0x20
//   bit6 = irq_en → 0x40
//   bit1 = clk_sel = 0 (E clock, verifies e_clk gate is gone)
// 0x20 | 0x40 = 0x60
static constexpr uint8_t CR_SINGLE0_16_IRQ = 0x60u;

TEST_CASE("MC6840 SINGLE_0 16-bit: IRQ fires after N+1 ticks and timer stops", "[mc6840][single0]")
{
	// latch = 4; fires at tick 5 (N+1).  Timer must stop and not repeat.
	auto t = make_timer();
	tw(t, SYSTEM_TIMER_CONTROL_2, CR_SINGLE0_16_IRQ);
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_2_LSB, 4u);   // latch=4, fires at tick 5 (N+1)

	// 4 ticks — must not fire yet
	for (int i = 0; i < 4; i++)
		t.tick(1u);
	REQUIRE(t.has_interrupt() == INTERRUPT_NONE);

	// 5th tick — terminal count detected, IRQ fires
	t.tick(1u);
	REQUIRE(t.has_interrupt() == INTERRUPT_IRQ);
}

TEST_CASE("MC6840 SINGLE_0 16-bit: does not repeat after firing", "[mc6840][single0]")
{
	auto t = make_timer();
	tw(t, SYSTEM_TIMER_CONTROL_2, CR_SINGLE0_16_IRQ);
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_2_LSB, 3u);   // latch=3, fires at tick 4

	// Run well past the firing point
	for (int i = 0; i < 20; i++)
		t.tick(1u);

	// Consume the single IRQ then verify no further interrupts
	tr(t, SYSTEM_TIMER_2_LSB);  // acknowledge
	REQUIRE(t.has_interrupt() == INTERRUPT_NONE);
}

TEST_CASE("MC6840 SINGLE_0 16-bit: 16-bit latch fires at correct count", "[mc6840][single0]")
{
	// latch = 0x0010 = 16; fires at tick 17 (N+1)
	auto t = make_timer();
	tw(t, SYSTEM_TIMER_CONTROL_2, CR_SINGLE0_16_IRQ);
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_2_LSB, 0x10u);

	for (int i = 0; i < 16; i++)
		t.tick(1u);
	REQUIRE(t.has_interrupt() == INTERRUPT_NONE);

	t.tick(1u);
	REQUIRE(t.has_interrupt() == INTERRUPT_IRQ);
}

TEST_CASE("MC6840 SINGLE_0 16-bit on Timer 2: no NMI asserted", "[mc6840][single0]")
{
	// Only Timer 1 is wired to NMI.  A 16-bit single-shot on Timer 2
	// must fire IRQ only.
	auto t = make_timer();
	tw(t, SYSTEM_TIMER_CONTROL_2, CR_SINGLE0_16_IRQ);
	tw(t, SYSTEM_TIMER_2_MSB, 0x00u);
	tw(t, SYSTEM_TIMER_2_LSB, 5u);

	for (int i = 0; i < 6; i++)  // N+1 = 6
		t.tick(1u);

	interrupt irq = t.has_interrupt();
	REQUIRE(irq == INTERRUPT_IRQ);
	REQUIRE(irq != INTERRUPT_NMI);
}
