/**
 * Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <iterator>

#include <catch2/catch_test_macros.hpp>

#include "rail_monitor.h"

// The firmware's values, so the tests exercise the shipped configuration
// rather than a convenient one.
static constexpr float DROP = 4.0f;
static constexpr uint32_t SAMPLES = 3u;
static constexpr uint64_t SETTLED = 1'000'000ull;	// 1 s in, past settling

// fall_volts 0 disables the relative test, so everything below pins the
// ORIGINAL absolute-only behaviour exactly as it shipped.
static constexpr rail_limits ABS = {
	/*drop*/ DROP, /*fall*/ 0.0f, /*healthy*/ 4.5f,
	/*low_samples*/ SAMPLES, /*good_samples*/ 50u, /*baseline_div*/ 250u
};

// Small good_samples and baseline_div keep the relative tests short and
// readable; the firmware's are 1 s and 5 s worth at 20 ms.
static constexpr rail_limits REL = {
	/*drop*/ DROP, /*fall*/ 0.4f, /*healthy*/ 4.5f,
	/*low_samples*/ SAMPLES, /*good_samples*/ 5u, /*baseline_div*/ 4u
};

// Run n healthy samples at `v` through the judge so the baseline settles on
// it, and hand back the state ready for the next sample.
static rail_state
seeded(const rail_limits &l, float v, uint32_t n = 40u)
{
	rail_state s{true, SETTLED, 0u, v, 0u, 0.0f, 0u};
	for (uint32_t i = 0; i < n; i++) {
		const rail_judgement j = rail_judge(s, l);
		REQUIRE(j.trip == false);
		s.low_count = j.low_count;
		s.baseline = j.baseline;
		s.good_count = j.good_count;
	}
	return s;
}

// Feed a sequence of voltages through the judge, carrying the counter forward
// as the firmware does, and report how many samples it took to trip (or 0).
static uint32_t
feed(const float *volts, size_t n, bool armed = true,
     uint64_t now = SETTLED, uint64_t settle_until = 0u)
{
	rail_state s{armed, now, settle_until, 0.0f, 0u, 0.0f, 0u};
	for (size_t i = 0; i < n; i++) {
		s.volts = volts[i];
		const rail_judgement j = rail_judge(s, ABS);
		s.low_count = j.low_count;
		if (j.trip)
			return static_cast<uint32_t>(i + 1);
	}
	return 0u;
}

TEST_CASE("rail: a healthy rail never trips", "[rail]")
{
	const float v[] = {5.0f, 5.0f, 4.99f, 5.01f, 5.0f, 4.5f, 5.0f};
	REQUIRE(feed(v, std::size(v)) == 0u);
}

TEST_CASE("rail: a sustained sag trips on the third low sample", "[rail]")
{
	const float v[] = {5.0f, 3.9f, 3.8f, 3.7f, 3.6f};
	REQUIRE(feed(v, std::size(v)) == 4u);	// samples 2,3,4 are the run
}

TEST_CASE("rail: a single-sample glitch does not trip", "[rail]")
{
	const float v[] = {5.0f, 3.0f, 5.0f, 3.0f, 5.0f, 3.0f, 5.0f};
	REQUIRE(feed(v, std::size(v)) == 0u);
}

TEST_CASE("rail: two low samples are not enough", "[rail]")
{
	const float v[] = {3.9f, 3.9f, 5.0f, 3.9f, 3.9f, 5.0f};
	REQUIRE(feed(v, std::size(v)) == 0u);
}

TEST_CASE("rail: one good sample forgives the run", "[rail]")
{
	// Two low, one good, two low: the counter must restart, so no trip.
	const float v[] = {3.9f, 3.9f, 4.1f, 3.9f, 3.9f};
	REQUIRE(feed(v, std::size(v)) == 0u);
	// And directly: a good sample with two already counted clears it.
	const rail_judgement good = rail_judge({true, SETTLED, 0u, 4.1f, 2u, 0.0f, 0u},
					       ABS);
	REQUIRE(good.trip == false);
	REQUIRE(good.low_count == 0u);
}

TEST_CASE("rail: the threshold is exclusive - exactly at it is not low",
	  "[rail]")
{
	const float v[] = {DROP, DROP, DROP, DROP};
	REQUIRE(feed(v, std::size(v)) == 0u);
	// A hair under, though, is low.
	const float under[] = {3.999f, 3.999f, 3.999f};
	REQUIRE(feed(under, std::size(under)) == 3u);
}

TEST_CASE("rail: a disarmed monitor never trips, whatever it sees", "[rail]")
{
	const float v[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	REQUIRE(feed(v, std::size(v), /*armed=*/false) == 0u);
}

TEST_CASE("rail: nothing is judged inside the settle window", "[rail]")
{
	const float v[] = {0.0f, 0.0f, 0.0f, 0.0f};
	// now < settle_until for every sample
	REQUIRE(feed(v, std::size(v), true, /*now=*/0u,
		     /*settle_until=*/SETTLED) == 0u);
}

TEST_CASE("rail: the settle window ends exactly at settle_until", "[rail]")
{
	// now == settle_until must already be judged: the firmware's test is
	// >=, so the boundary sample counts.
	const rail_judgement at = rail_judge({true, SETTLED, SETTLED, 3.0f, 2u, 0.0f, 0u},
					     ABS);
	REQUIRE(at.trip == true);
	const rail_judgement before = rail_judge({true, SETTLED - 1u, SETTLED,
						  3.0f, 2u, 0.0f, 0u}, ABS);
	REQUIRE(before.trip == false);
	REQUIRE(before.low_count == 2u);	// left alone, not reset
}

TEST_CASE("rail: the settle window preserves an in-progress run", "[rail]")
{
	// This is the inherited quirk, pinned deliberately: samples counted
	// before the window are still credited after it, because the window
	// leaves the counter alone rather than resetting it.
	rail_state s{true, 0u, SETTLED, 3.0f, 0u, 0.0f, 0u};
	for (int i = 0; i < 5; i++) {		// inside the window
		const rail_judgement j = rail_judge(s, ABS);
		REQUIRE(j.trip == false);
		s.low_count = j.low_count;
	}
	REQUIRE(s.low_count == 0u);		// nothing accumulated either

	// Now accumulate two, then re-enter a window, then leave it.
	s = rail_state{true, SETTLED, 0u, 3.0f, 0u, 0.0f, 0u};
	for (int i = 0; i < 2; i++)
		s.low_count = rail_judge(s, ABS).low_count;
	REQUIRE(s.low_count == 2u);
	s.now_us = 0u;				// back inside a window
	s.settle_until_us = SETTLED;
	const rail_judgement held = rail_judge(s, ABS);
	REQUIRE(held.trip == false);
	REQUIRE(held.low_count == 2u);		// carried, not cleared
	s.low_count = held.low_count;
	s.now_us = SETTLED;			// out again
	s.settle_until_us = 0u;
	REQUIRE(rail_judge(s, ABS).trip == true);
}

TEST_CASE("rail: a recovery after a trip is the caller's business", "[rail]")
{
	// On tripping the counter is NOT cleared.  Pinned because anything that
	// re-arms the monitor has to clear it itself.
	rail_state s{true, SETTLED, 0u, 3.0f, SAMPLES - 1u, 0.0f, 0u};
	const rail_judgement j = rail_judge(s, ABS);
	REQUIRE(j.trip == true);
	REQUIRE(j.low_count == SAMPLES);	// not reset
}

TEST_CASE("rail: a longer debounce needs proportionally more samples",
	  "[rail]")
{
	constexpr rail_limits TEN = {DROP, 0.0f, 4.5f, 10u, 50u, 250u};
	rail_state s{true, SETTLED, 0u, 3.0f, 0u, 0.0f, 0u};
	for (uint32_t i = 1; i < 10u; i++) {
		const rail_judgement j = rail_judge(s, TEN);
		REQUIRE(j.trip == false);
		REQUIRE(j.low_count == i);
		s.low_count = j.low_count;
	}
	REQUIRE(rail_judge(s, TEN).trip == true);
}

TEST_CASE("rail: one sample trips when the debounce is one", "[rail]")
{
	constexpr rail_limits ONE = {DROP, 0.0f, 4.5f, 1u, 50u, 250u};
	const rail_judgement j = rail_judge({true, SETTLED, 0u, 3.0f, 0u, 0.0f, 0u},
					    ONE);
	REQUIRE(j.trip == true);
	REQUIRE(j.low_count == 1u);
}

// ---------------------------------------------------------------------------
// The relative-drop detector.  This is the part that makes the cutout immune
// to the serving offset, so most of these are about the baseline rather than
// about any particular voltage.
// ---------------------------------------------------------------------------

// Step a state forward one sample and carry everything back.
static rail_judgement
step(rail_state &s, const rail_limits &l, float v)
{
	s.volts = v;
	const rail_judgement j = rail_judge(s, l);
	s.low_count = j.low_count;
	s.baseline = j.baseline;
	s.good_count = j.good_count;
	return j;
}

TEST_CASE("rail: the baseline needs good_samples healthy samples", "[rail]")
{
	rail_state s{true, SETTLED, 0u, 0.0f, 0u, 0.0f, 0u};
	for (uint32_t i = 1; i <= REL.good_samples; i++) {
		step(s, REL, 5.2f);
		REQUIRE(s.good_count == i);
	}
	// Seeded on the first healthy sample, and saturates after that.
	REQUIRE(s.baseline > 5.1f);
	step(s, REL, 5.2f);
	REQUIRE(s.good_count == REL.good_samples);
}

TEST_CASE("rail: a fall past fall_volts trips, after the debounce", "[rail]")
{
	rail_state s = seeded(REL, 5.2f);
	REQUIRE(step(s, REL, 4.7f).trip == false);	// 5.2 - 4.7 = 0.5 > 0.4
	REQUIRE(step(s, REL, 4.7f).trip == false);
	REQUIRE(step(s, REL, 4.7f).trip == true);
}

TEST_CASE("rail: a fall short of fall_volts never trips", "[rail]")
{
	rail_state s = seeded(REL, 5.2f);
	for (int i = 0; i < 20; i++)
		REQUIRE(step(s, REL, 4.9f).trip == false);  // only 0.3 down
}

TEST_CASE("rail: a sag cannot drag its own reference down", "[rail]")
{
	rail_state s = seeded(REL, 5.2f);
	const float was = s.baseline;
	step(s, REL, 4.7f);
	REQUIRE(s.baseline == was);		// frozen while counted low
	step(s, REL, 4.7f);
	REQUIRE(s.baseline == was);
}

TEST_CASE("rail: THE POINT - an inflated baseline still catches the fall",
	  "[rail]")
{
	// Serving the bus inflates both channels by 0.5 to 0.7V.  A baseline
	// seeded while serving sits near 5.7, and a 0.5V collapse lands at
	// 5.2 - which an absolute 4.0V floor would never see.  The relative
	// test catches it because the offset cancels in the difference.
	rail_state s = seeded(REL, 5.7f);
	REQUIRE(s.baseline > 5.6f);
	REQUIRE(step(s, REL, 5.2f).trip == false);
	REQUIRE(step(s, REL, 5.2f).trip == false);
	REQUIRE(step(s, REL, 5.2f).trip == true);
	// And prove the absolute-only configuration is blind to it.
	rail_state a = seeded(ABS, 5.7f);
	for (int i = 0; i < 20; i++)
		REQUIRE(step(a, ABS, 5.2f).trip == false);
}

TEST_CASE("rail: a rail below healthy never establishes a baseline", "[rail]")
{
	// 4.2V is above the 4.0 floor but below the 4.5 healthy line, so no
	// baseline forms and there is nothing to fall from.  This is also the
	// shape of a USB-powered cold start, where the Pico runs happily on
	// VBUS through D109 while +5VP is low or absent.
	rail_state s{true, SETTLED, 0u, 0.0f, 0u, 0.0f, 0u};
	for (int i = 0; i < 50; i++)
		REQUIRE(step(s, REL, 4.2f).trip == false);
	REQUIRE(s.baseline == 0.0f);
	REQUIRE(s.good_count == 0u);
}

TEST_CASE("rail: the floor still catches a rail that is simply gone", "[rail]")
{
	// No baseline, nothing to fall from - the absolute backstop earns its
	// keep here.
	rail_state s{true, SETTLED, 0u, 0.0f, 0u, 0.0f, 0u};
	REQUIRE(step(s, REL, 0.0f).trip == false);
	REQUIRE(step(s, REL, 0.0f).trip == false);
	REQUIRE(step(s, REL, 0.0f).trip == true);
}

TEST_CASE("rail: the baseline follows a slow drift while healthy", "[rail]")
{
	rail_state s = seeded(REL, 5.0f);
	const float low = s.baseline;
	for (int i = 0; i < 40; i++)
		REQUIRE(step(s, REL, 5.2f).trip == false);
	REQUIRE(s.baseline > low);
	REQUIRE(s.baseline > 5.15f);		// converged most of the way
}

TEST_CASE("rail: a recovered rail resumes moving the baseline", "[rail]")
{
	rail_state s = seeded(REL, 5.2f);
	step(s, REL, 4.9f);			// a dip, short of fall_volts
	const float during = s.baseline;
	for (int i = 0; i < 20; i++)
		step(s, REL, 5.3f);		// healthy again, and higher
	REQUIRE(s.baseline > during);
}

TEST_CASE("rail: the settle window freezes the baseline too", "[rail]")
{
	rail_state s = seeded(REL, 5.2f);
	const float was = s.baseline;
	s.now_us = 0u;				// back inside a window
	s.settle_until_us = SETTLED;
	for (int i = 0; i < 10; i++) {
		REQUIRE(step(s, REL, 5.9f).trip == false);
		REQUIRE(s.baseline == was);
	}
}

TEST_CASE("rail: a disarmed monitor updates nothing", "[rail]")
{
	rail_state s = seeded(REL, 5.2f);
	const float was = s.baseline;
	const uint32_t good = s.good_count;
	s.armed = false;
	for (int i = 0; i < 10; i++)
		REQUIRE(step(s, REL, 0.0f).trip == false);
	REQUIRE(s.baseline == was);
	REQUIRE(s.good_count == good);
}
