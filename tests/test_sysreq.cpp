/**
 * Host-side unit tests for the floating-point system request in sysreq.h.
 *
 * Built against the REAL registers.h, not fake/registers.h: the request reads
 * and writes 4-byte big-endian floats through the register proxies, and the
 * fake proxies have no float conversions to get right or wrong.  fake/ supplies
 * only pico.h.
 *
 * Operands are placed and results read back a byte at a time here, not through
 * the proxies' float conversions, so a mistake in those cannot cancel itself
 * out between writing an operand and reading the result.
 */

#include <cmath>
#include <cstdint>
#include <cstring>
#include <catch2/catch_test_macros.hpp>

#include "registers.h"
#include "sysreq.h"

// Required by 'extern registers &reg' in registers.h.
registers &reg = registers::getInstance();

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------

// SHARED ($FFD0), REGISTER_BUFFER_OFFSET in the firmware, where it passes the
// operands.  pico_thing.h is not host-buildable, so the offset is repeated.
static constexpr uint16_t BUFFER = 0x10u;

static void
put_float(const uint16_t at, const float f)
{
	uint32_t u;
	std::memcpy(&u, &f, sizeof u);
	for (uint16_t i = 0u; i < 4u; i++)
		reg[static_cast<uint16_t>(at + i)] = static_cast<uint8_t>(u >> (24u - 8u*i));
}

static uint8_t
byte_at(const uint16_t at)
{
	return static_cast<uint8_t>(reg[at]);
}

static float
get_float(const uint16_t at)
{
	uint32_t u = 0u;
	for (uint16_t i = 0u; i < 4u; i++)
		u = (u << 8) | byte_at(static_cast<uint16_t>(at + i));
	float f;
	std::memcpy(&f, &u, sizeof f);
	return f;
}

static sysreq_float_result
run(const uint8_t op, const float a, const float b)
{
	registers::clear();
	put_float(BUFFER, a);
	put_float(static_cast<uint16_t>(BUFFER + 4u), b);
	return sysreq_float(op, BUFFER);
}

// -----------------------------------------------------------------------
// Tests
// -----------------------------------------------------------------------

TEST_CASE("sysreq_float: add", "[sysreq][float]")
{
	const sysreq_float_result r = run(FLOAT_ADD, 1.5f, 2.25f);

	CHECK(r.status == RESPONSE_DONE);
	CHECK(r.response == 0u);
	CHECK(get_float(BUFFER) == 3.75f);
	CHECK(get_float(BUFFER + 4u) == 2.25f);	// second operand untouched
}

TEST_CASE("sysreq_float: subtract", "[sysreq][float]")
{
	const sysreq_float_result r = run(FLOAT_SUBTRACT, 1.5f, 2.25f);

	CHECK(r.status == RESPONSE_DONE);
	CHECK(r.response == FLOAT_RESPONSE_NEGATIVE);
	CHECK(get_float(BUFFER) == -0.75f);
}

TEST_CASE("sysreq_float: multiply", "[sysreq][float]")
{
	SECTION("negative product")
	{
		const sysreq_float_result r = run(FLOAT_MULTIPLY, 1.5f, -2.0f);

		CHECK(r.status == RESPONSE_DONE);
		CHECK(r.response == FLOAT_RESPONSE_NEGATIVE);
		CHECK(get_float(BUFFER) == -3.0f);
	}

	SECTION("zero product")
	{
		const sysreq_float_result r = run(FLOAT_MULTIPLY, 0.0f, 5.0f);

		CHECK(r.status == RESPONSE_DONE);
		CHECK(r.response == FLOAT_RESPONSE_ZERO);
		CHECK(get_float(BUFFER) == 0.0f);
	}
}

// Divide used to compute its quotient and never store it.
TEST_CASE("sysreq_float: divide stores its quotient", "[sysreq][float]")
{
	SECTION("exact quotient")
	{
		const sysreq_float_result r = run(FLOAT_DIVIDE, 7.0f, 2.0f);

		CHECK(r.status == RESPONSE_DONE);
		CHECK(r.response == 0u);
		CHECK(get_float(BUFFER) == 3.5f);
	}

	SECTION("inexact quotient")
	{
		const sysreq_float_result r = run(FLOAT_DIVIDE, 1.5f, 2.25f);

		CHECK(r.status == RESPONSE_DONE);
		CHECK(get_float(BUFFER) == 1.5f/2.25f);
	}
}

// Every operand used to be read as its first byte.  1.5 and 1.25 share theirs
// ($3F), so the old code added 63 to 63.
TEST_CASE("sysreq_float: operands are whole floats, not their first bytes", "[sysreq][float]")
{
	const sysreq_float_result r = run(FLOAT_ADD, 1.5f, 1.25f);

	CHECK(r.status == RESPONSE_DONE);
	CHECK(get_float(BUFFER) == 2.75f);
}

TEST_CASE("sysreq_float: result is stored big-endian", "[sysreq][float]")
{
	(void) run(FLOAT_ADD, 1.5f, 2.25f);	// 3.75 is $40700000

	CHECK(byte_at(BUFFER + 0u) == 0x40u);
	CHECK(byte_at(BUFFER + 1u) == 0x70u);
	CHECK(byte_at(BUFFER + 2u) == 0x00u);
	CHECK(byte_at(BUFFER + 3u) == 0x00u);
}

TEST_CASE("sysreq_float: zero result sets only the zero flag", "[sysreq][float]")
{
	const sysreq_float_result r = run(FLOAT_SUBTRACT, 2.25f, 2.25f);

	CHECK(r.status == RESPONSE_DONE);
	CHECK(r.response == FLOAT_RESPONSE_ZERO);
	CHECK(get_float(BUFFER) == 0.0f);
}

TEST_CASE("sysreq_float: division by zero gives a signed infinity", "[sysreq][float]")
{
	SECTION("positive")
	{
		const sysreq_float_result r = run(FLOAT_DIVIDE, 1.0f, 0.0f);

		CHECK(r.status == RESPONSE_DONE);
		CHECK(r.response == 0u);
		CHECK(std::isinf(get_float(BUFFER)));
		CHECK_FALSE(std::signbit(get_float(BUFFER)));
	}

	SECTION("negative")
	{
		const sysreq_float_result r = run(FLOAT_DIVIDE, -1.0f, 0.0f);

		CHECK(r.status == RESPONSE_DONE);
		CHECK(r.response == FLOAT_RESPONSE_NEGATIVE);
		CHECK(std::isinf(get_float(BUFFER)));
		CHECK(std::signbit(get_float(BUFFER)));
	}
}

TEST_CASE("sysreq_float: unknown operation leaves the buffer alone", "[sysreq][float]")
{
	const sysreq_float_result r = run(uint8_t('x'), 1.5f, 2.25f);

	CHECK(r.status == RESPONSE_UNKNOWN);
	CHECK(r.response == RESPONSE_NULL);
	CHECK(get_float(BUFFER) == 1.5f);
	CHECK(get_float(BUFFER + 4u) == 2.25f);
}
