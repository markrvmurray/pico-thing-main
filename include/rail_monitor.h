/**
 * Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <cstdint>

/*
 * The supply cutout's decision, as a pure function.
 *
 * WHAT IS ACTUALLY BEING MEASURED, because the old names said otherwise.
 * ADC6 (GPIO 46) sits on a 10k/10k divider across +5VP, which is U108's
 * output - a MIC29150 LDO fed from +VDC.  +5VP reaches the Pico through the
 * dual Schottky D109, which also ORs in USB VBUS, so:
 *
 *   - the Pico's own supply is +5VP minus a Schottky drop, and
 *   - with a USB cable in, the Pico is held up even if +VDC vanishes.
 *
 * The sense point is deliberately UPSTREAM of that diode.  Measured after it,
 * USB would hold the reading near 4.7V and mask the sag completely.
 *
 * +5VP is a good proxy for +VDC because U108 is an LDO: once in dropout its
 * output tracks its input down.  So a FALL in +5VP means +VDC is collapsing.
 *
 * It is NOT the guest's rail.  That is +5V from U104, a separate MIC29501,
 * and ADC6 cannot see it.  If U104 or its enable failed, this monitor would
 * read a contented 5V while the guest had no power at all - an accepted blind
 * spot, and the reason dropping /POWER (which disables U104 only) leaves the
 * Pico alive to report it.
 *
 * WHY RELATIVE RATHER THAN ABSOLUTE.  An absolute threshold is only as good
 * as the ADC's calibration, and version 2 once read 0.5 to 0.7V high at this
 * divider whenever the Pico served the bus (5V injected into the ADC-bank
 * pins, since clamped).  A near-constant error CANCELS in a difference:
 * tracking a slow baseline and tripping on a fall from it is immune to the
 * offset by construction.  The absolute floor stays as a backstop only.
 *
 * The decision used to be inline in the 20 ms ADC timer callback, where the
 * only way to exercise it was to collapse the real 5V rail - which is exactly
 * the thing not to do, because the board's LDOs misbehave right around the
 * voltage of interest.  Here, ctest covers the sags, recoveries, glitches,
 * baseline drift and the settle window, leaving the bench to prove only that
 * the GPIO actually opens.
 *
 * Deliberately free of any SDK dependency: no pico headers, nothing to fake,
 * no clock to stub.  Time arrives as a number and the caller owns the state.
 */

// What the caller must tell us.  All of it is state the caller owns and
// writes back from rail_judgement.
struct rail_state {
	bool armed;			// switch closed, and no trip yet
	uint64_t now_us;		// microseconds, any epoch
	uint64_t settle_until_us;	// no judging before this
	float volts;			// +5VP, as the ADC saw it
	uint32_t low_count;		// consecutive samples already counted low
	float baseline;			// the slow average; <= 0 means unseeded
	uint32_t good_count;		// consecutive healthy samples seen
};

// What we decided, and the state to store back.
struct rail_judgement {
	bool trip;			// open the switch, now
	uint32_t low_count;
	float baseline;
	uint32_t good_count;
};

struct rail_limits {
	// Absolute backstop.  A real collapse trips on the fall long before it
	// gets here, so this is the catch for a baseline that never
	// established - not the primary protection.
	float drop_volts;
	// Trip when the sample is this far below the baseline.  Zero or less
	// DISABLES the relative test, which is how the tests that pin the old
	// absolute-only behaviour still pin it.
	float fall_volts;
	// A sample at or above this may seed and update the baseline.  Below
	// it the baseline is frozen, so a sag cannot drag its own reference
	// down, and a USB-powered cold start with no +VDC at all never
	// establishes a baseline and so never trips on a fall.
	float healthy_volts;
	uint32_t low_samples;		// consecutive low samples that trip
	uint32_t good_samples;		// healthy samples to establish a baseline
	uint32_t baseline_div;		// baseline += (v - baseline) / this
};

/*
 * Two quirks are inherited from the original inline code and pinned by tests
 * rather than quietly fixed:
 *
 *   - while disarmed or inside the settle window NOTHING is updated, counter
 *     included.  So a sag that began before the window closed still has its
 *     earlier samples credited afterwards.
 *   - on tripping the counter is not reset either.  It does not matter today
 *     because the caller stops asking once it has tripped, but anything that
 *     re-arms must clear it.
 */
[[nodiscard]] inline rail_judgement
rail_judge(const rail_state &s, const rail_limits &l)
{
	// Not our business yet: the switch is open, we have already tripped, or
	// the bulk capacitance is still charging.
	if (!s.armed || s.now_us < s.settle_until_us)
		return { false, s.low_count, s.baseline, s.good_count };

	// Judge against the baseline as it stands, BEFORE this sample can
	// touch it.  Otherwise a sag updates the reference it is being
	// measured against and partly hides itself.
	const bool established = l.fall_volts > 0.0f
			      && s.good_count >= l.good_samples
			      && s.baseline > 0.0f;
	const bool below_floor = s.volts < l.drop_volts;
	const bool fallen = established
			 && s.volts < s.baseline - l.fall_volts;
	const bool low = below_floor || fallen;

	// Only a sample that is both healthy and not counted low may move the
	// baseline.  That keeps the reference anchored to how the rail looks
	// when it is well.
	float baseline = s.baseline;
	uint32_t good_count = s.good_count;
	if (!low && s.volts >= l.healthy_volts) {
		if (good_count < l.good_samples)
			good_count++;		// saturate; no wrap to worry about
		baseline = (baseline <= 0.0f)
			 ? s.volts		// first healthy sample seeds it
			 : baseline + (s.volts - baseline)
				      / static_cast<float>(l.baseline_div);
	}

	if (low) {
		const uint32_t n = s.low_count + 1u;
		return { n >= l.low_samples, n, baseline, good_count };
	}

	// One good sample is enough to forgive the run so far.
	return { false, 0u, baseline, good_count };
}
