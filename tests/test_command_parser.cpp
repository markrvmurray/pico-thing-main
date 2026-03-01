/**
 * Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Catch2 unit tests for the flex/bison command parser.
 *
 * Covers:
 *   - All command variants (happy path)
 *   - Keyword prefix matching and case-insensitivity
 *   - Range / validity error cases
 *   - Accumulator-leak regression (consecutive parses must not corrupt each
 *     other's byte / vector lists — requires the mid-rule _byte_count / _vec_count
 *     resets in command_parser.y)
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cstring>

extern "C" {
#include "command_parser.h"
}

/* Helper: copy the string literal into a mutable buffer (cmd_parse takes
 * char *, not const char *) and call cmd_parse. */
static bool
parse(const char *input_str, parsed_cmd_t *out)
{
	char buf[512];
	std::strncpy(buf, input_str, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';
	return cmd_parse(buf, out);
}

// ============================================================
//  Simple no-argument commands
// ============================================================

TEST_CASE("Simple no-argument commands", "[parser]")
{
	parsed_cmd_t cmd;

	SECTION("power")
	{
		REQUIRE(parse("power", &cmd));
		CHECK(cmd.tag == CMD_POWER);
	}
	SECTION("verbose")
	{
		REQUIRE(parse("verbose", &cmd));
		CHECK(cmd.tag == CMD_VERBOSE);
	}
	SECTION("status")
	{
		REQUIRE(parse("status", &cmd));
		CHECK(cmd.tag == CMD_STATUS);
	}
	SECTION("reset")
	{
		REQUIRE(parse("reset", &cmd));
		CHECK(cmd.tag == CMD_RESET);
	}
}

// ============================================================
//  Fixed 2-letter mnemonic matching
//  Each command accepts its full name OR exactly its registered alias.
// ============================================================

TEST_CASE("Fixed 2-letter mnemonic matching", "[parser][mnemonic]")
{
	parsed_cmd_t cmd;

	SECTION("'pw' matches power")
	{
		REQUIRE(parse("pw", &cmd));
		CHECK(cmd.tag == CMD_POWER);
	}
	SECTION("'vb' matches verbose (alias distinct from vector's 'vc')")
	{
		REQUIRE(parse("vb", &cmd));
		CHECK(cmd.tag == CMD_VERBOSE);
	}
	SECTION("'ex' matches examine")
	{
		REQUIRE(parse("ex dat", &cmd));
		CHECK(cmd.tag == CMD_EXAMINE_DAT);
	}
	SECTION("'mo' matches modify")
	{
		REQUIRE(parse("mo 1000 ab cd", &cmd));
		CHECK(cmd.tag == CMD_MODIFY);
	}
	SECTION("'st' matches status")
	{
		REQUIRE(parse("st", &cmd));
		CHECK(cmd.tag == CMD_STATUS);
	}
	SECTION("'sn' matches snippet")
	{
		REQUIRE(parse("sn 01", &cmd));
		CHECK(cmd.tag == CMD_SNIPPET);
	}
	SECTION("'lo' matches loop")
	{
		REQUIRE(parse("lo r 1000", &cmd));
		CHECK(cmd.tag == CMD_LOOP);
	}
	SECTION("'vc' matches vector")
	{
		REQUIRE(parse("vc 0001 0002 0003 0004 0005 0006 0007 0008", &cmd));
		CHECK(cmd.tag == CMD_VECTOR);
	}
	SECTION("'fi' matches fill")
	{
		REQUIRE(parse("fi all", &cmd));
		CHECK(cmd.tag == CMD_FILL_ALL);
	}
	SECTION("'ti' matches time")
	{
		REQUIRE(parse("ti", &cmd));
		CHECK(cmd.tag == CMD_TIME_QUERY);
	}
	SECTION("'ta' matches task")
	{
		REQUIRE(parse("ta", &cmd));
		CHECK(cmd.tag == CMD_TASK_QUERY);
	}
	SECTION("'ir' matches irq")
	{
		REQUIRE(parse("ir 0", &cmd));
		CHECK(cmd.tag == CMD_IRQ);
	}
	SECTION("'tr' matches trace")
	{
		REQUIRE(parse("tr", &cmd));
		CHECK(cmd.tag == CMD_TRACE);
	}
	SECTION("'ru' matches run")
	{
		REQUIRE(parse("ru", &cmd));
		CHECK(cmd.tag == CMD_RUN_LIST);
	}
	SECTION("'sp' matches stop")
	{
		REQUIRE(parse("sp", &cmd));
		CHECK(cmd.tag == CMD_STOP);
	}
	SECTION("'fq' matches freq")
	{
		REQUIRE(parse("fq 1.0", &cmd));
		CHECK(cmd.tag == CMD_FREQ);
	}
	SECTION("'rs' matches reset")
	{
		REQUIRE(parse("rs", &cmd));
		CHECK(cmd.tag == CMD_RESET);
	}
	SECTION("'ub' matches usb")
	{
		REQUIRE(parse("ub hello", &cmd));
		CHECK(cmd.tag == CMD_USB);
	}
	SECTION("old prefix 'po' is no longer accepted")
	{
		CHECK_FALSE(parse("po", &cmd));
	}
	SECTION("old prefix 'ver' is no longer accepted")
	{
		CHECK_FALSE(parse("ver", &cmd));
	}
	SECTION("old prefix 're' is no longer accepted (reset is 'rs')")
	{
		CHECK_FALSE(parse("re", &cmd));
	}
}

// ============================================================
//  Case-insensitivity of command keywords
// ============================================================

TEST_CASE("Command keyword case insensitivity", "[parser][case]")
{
	parsed_cmd_t cmd;

	SECTION("POWER")
	{
		REQUIRE(parse("POWER", &cmd));
		CHECK(cmd.tag == CMD_POWER);
	}
	SECTION("Power")
	{
		REQUIRE(parse("Power", &cmd));
		CHECK(cmd.tag == CMD_POWER);
	}
	SECTION("pOwEr")
	{
		REQUIRE(parse("pOwEr", &cmd));
		CHECK(cmd.tag == CMD_POWER);
	}
	SECTION("VERBOSE")
	{
		REQUIRE(parse("VERBOSE", &cmd));
		CHECK(cmd.tag == CMD_VERBOSE);
	}
	SECTION("STATUS")
	{
		REQUIRE(parse("STATUS", &cmd));
		CHECK(cmd.tag == CMD_STATUS);
	}
	SECTION("RESET")
	{
		REQUIRE(parse("RESET", &cmd));
		CHECK(cmd.tag == CMD_RESET);
	}
}

// ============================================================
//  Ambiguous prefixes → syntax error
// ============================================================

TEST_CASE("Ambiguous prefix produces syntax error", "[parser][prefix][error]")
{
	parsed_cmd_t cmd;

	SECTION("'s' matches status, snippet, stop")
	{
		CHECK_FALSE(parse("s", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("'v' matches verbose and vector")
	{
		CHECK_FALSE(parse("v", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("'ve' matches verbose and vector")
	{
		CHECK_FALSE(parse("ve", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("'f' matches fill and freq")
	{
		CHECK_FALSE(parse("f", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("'t' matches time, task, trace")
	{
		CHECK_FALSE(parse("t", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
}

// ============================================================
//  examine
// ============================================================

TEST_CASE("examine command", "[parser][examine]")
{
	parsed_cmd_t cmd;

	SECTION("examine dat")
	{
		REQUIRE(parse("examine dat", &cmd));
		CHECK(cmd.tag == CMD_EXAMINE_DAT);
	}
	SECTION("examine DAT (case-insensitive subcommand)")
	{
		REQUIRE(parse("examine DAT", &cmd));
		CHECK(cmd.tag == CMD_EXAMINE_DAT);
	}
	SECTION("examine start end — address range")
	{
		REQUIRE(parse("examine 0 100", &cmd));
		CHECK(cmd.tag    == CMD_EXAMINE);
		CHECK(cmd.examine.start == 0x0000);
		CHECK(cmd.examine.end   == 0x0100);
	}
	SECTION("examine larger range")
	{
		REQUIRE(parse("examine 1000 2000", &cmd));
		CHECK(cmd.examine.start == 0x1000);
		CHECK(cmd.examine.end   == 0x2000);
	}
	SECTION("mnemonic 'ex dat'")
	{
		REQUIRE(parse("ex dat", &cmd));
		CHECK(cmd.tag == CMD_EXAMINE_DAT);
	}
	SECTION("start == FFC0 is rejected (boundary)")
	{
		CHECK_FALSE(parse("examine FFC0 FFD0", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("end == 0 is rejected")
	{
		CHECK_FALSE(parse("examine 0 0", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("end <= start is rejected")
	{
		CHECK_FALSE(parse("examine 200 100", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
}

// ============================================================
//  modify
// ============================================================

TEST_CASE("modify command", "[parser][modify]")
{
	parsed_cmd_t cmd;

	SECTION("single byte")
	{
		REQUIRE(parse("modify 100 FF", &cmd));
		CHECK(cmd.tag          == CMD_MODIFY);
		CHECK(cmd.modify.start == 0x0100);
		CHECK(cmd.modify.count == 1);
		CHECK(cmd.modify.data[0] == 0xFF);
	}
	SECTION("four bytes")
	{
		REQUIRE(parse("modify 200 DE AD BE EF", &cmd));
		CHECK(cmd.tag          == CMD_MODIFY);
		CHECK(cmd.modify.start == 0x0200);
		CHECK(cmd.modify.count == 4);
		CHECK(cmd.modify.data[0] == 0xDE);
		CHECK(cmd.modify.data[1] == 0xAD);
		CHECK(cmd.modify.data[2] == 0xBE);
		CHECK(cmd.modify.data[3] == 0xEF);
	}
	SECTION("accumulator does not leak between calls (regression)")
	{
		/* First parse leaves _byte_count == 3; without the mid-rule reset
		 * in the grammar the second parse would produce count == 5. */
		parsed_cmd_t cmd2;
		REQUIRE(parse("modify 100 01 02 03", &cmd));
		REQUIRE(parse("modify 200 AA BB",    &cmd2));
		CHECK(cmd2.modify.count     == 2);
		CHECK(cmd2.modify.data[0]   == 0xAA);
		CHECK(cmd2.modify.data[1]   == 0xBB);
	}
	SECTION("start address > FFFE is rejected")
	{
		CHECK_FALSE(parse("modify FFFF 01", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("byte value > FF is rejected")
	{
		CHECK_FALSE(parse("modify 100 100", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
}

// ============================================================
//  snippet
// ============================================================

TEST_CASE("snippet command", "[parser][snippet]")
{
	parsed_cmd_t cmd;

	SECTION("two bytes")
	{
		REQUIRE(parse("snippet 12 34", &cmd));
		CHECK(cmd.tag            == CMD_SNIPPET);
		CHECK(cmd.snippet.count  == 2);
		CHECK(cmd.snippet.data[0] == 0x12);
		CHECK(cmd.snippet.data[1] == 0x34);
	}
	SECTION("maximum 16 bytes")
	{
		REQUIRE(parse("snippet 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10", &cmd));
		CHECK(cmd.tag           == CMD_SNIPPET);
		CHECK(cmd.snippet.count == 16);
		CHECK(cmd.snippet.data[15] == 0x10);
	}
	SECTION("17 bytes is rejected")
	{
		CHECK_FALSE(parse("snippet 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F 10 11", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("accumulator does not leak between snippet calls (regression)")
	{
		parsed_cmd_t cmd2;
		REQUIRE(parse("snippet 01 02 03 04 05",    &cmd));
		REQUIRE(parse("snippet AA BB",              &cmd2));
		CHECK(cmd2.snippet.count    == 2);
		CHECK(cmd2.snippet.data[0]  == 0xAA);
		CHECK(cmd2.snippet.data[1]  == 0xBB);
	}
}

// ============================================================
//  loop
// ============================================================

TEST_CASE("loop command", "[parser][loop]")
{
	parsed_cmd_t cmd;

	SECTION("loop r — byte read (LDA extended, opcode $B6)")
	{
		REQUIRE(parse("loop r 1000", &cmd));
		CHECK(cmd.tag          == CMD_LOOP);
		CHECK(cmd.loop.opcode  == 0xB6);
		CHECK(cmd.loop.address == 0x1000);
	}
	SECTION("loop R — word read (LDD extended, opcode $FC)")
	{
		REQUIRE(parse("loop R 2000", &cmd));
		CHECK(cmd.loop.opcode  == 0xFC);
		CHECK(cmd.loop.address == 0x2000);
	}
	SECTION("loop w — byte write (STA extended, opcode $B7)")
	{
		REQUIRE(parse("loop w 3000", &cmd));
		CHECK(cmd.loop.opcode  == 0xB7);
		CHECK(cmd.loop.address == 0x3000);
	}
	SECTION("loop W — word write (STD extended, opcode $FD)")
	{
		REQUIRE(parse("loop W 4000", &cmd));
		CHECK(cmd.loop.opcode  == 0xFD);
		CHECK(cmd.loop.address == 0x4000);
	}
	SECTION("loop rw — read-modify-write (NEG extended, opcode $70)")
	{
		REQUIRE(parse("loop rw 5000", &cmd));
		CHECK(cmd.loop.opcode  == 0x70);
		CHECK(cmd.loop.address == 0x5000);
	}
	SECTION("loop RW (uppercase alias for rw)")
	{
		REQUIRE(parse("loop RW ABCD", &cmd));
		CHECK(cmd.loop.opcode  == 0x70);
		CHECK(cmd.loop.address == 0xABCD);
	}
}

// ============================================================
//  vector
// ============================================================

TEST_CASE("vector command", "[parser][vector]")
{
	parsed_cmd_t cmd;

	SECTION("eight identical addresses")
	{
		REQUIRE(parse("vector 0130 0130 0130 0130 0130 0130 0130 0130", &cmd));
		CHECK(cmd.tag == CMD_VECTOR);
		for (int i = 0; i < 8; i++)
			CHECK(cmd.vector.vec[i] == 0x0130);
	}
	SECTION("eight distinct addresses — boundary check")
	{
		REQUIRE(parse("vector 1000 2000 3000 4000 5000 6000 7000 8000", &cmd));
		CHECK(cmd.vector.vec[0] == 0x1000);
		CHECK(cmd.vector.vec[7] == 0x8000);
	}
	SECTION("accumulator does not leak between vector calls (regression)")
	{
		/* Without the mid-rule _vec_count = 0 reset, the second parse would
		 * silently reuse the first call's values because _vec_count == 8
		 * causes every vec_item write to be suppressed (< 8 check). */
		parsed_cmd_t cmd2;
		REQUIRE(parse("vector 0001 0002 0003 0004 0005 0006 0007 0008", &cmd));
		REQUIRE(parse("vector AAAA BBBB CCCC DDDD EEEE FFFF 1111 2222", &cmd2));
		CHECK(cmd2.vector.vec[0] == 0xAAAA);
		CHECK(cmd2.vector.vec[7] == 0x2222);
	}
}

// ============================================================
//  fill
// ============================================================

TEST_CASE("fill command", "[parser][fill]")
{
	parsed_cmd_t cmd;

	SECTION("fill all — default zero fill")
	{
		REQUIRE(parse("fill all", &cmd));
		CHECK(cmd.tag        == CMD_FILL_ALL);
		CHECK(cmd.fill.start == 0x0000);
		CHECK(cmd.fill.end   == 0xFE00);
		CHECK(cmd.fill.value == 0x00);
	}
	SECTION("fill all with explicit value")
	{
		REQUIRE(parse("fill all FF", &cmd));
		CHECK(cmd.tag        == CMD_FILL_ALL_VALUE);
		CHECK(cmd.fill.value == 0xFF);
	}
	SECTION("fill ALL (case-insensitive)")
	{
		REQUIRE(parse("fill ALL", &cmd));
		CHECK(cmd.tag == CMD_FILL_ALL);
	}
	SECTION("fill range — default zero fill")
	{
		REQUIRE(parse("fill 0 1000", &cmd));
		CHECK(cmd.tag        == CMD_FILL);
		CHECK(cmd.fill.start == 0x0000);
		CHECK(cmd.fill.end   == 0x1000);
		CHECK(cmd.fill.value == 0x00);
	}
	SECTION("fill range with explicit value")
	{
		REQUIRE(parse("fill 0 1000 AA", &cmd));
		CHECK(cmd.tag        == CMD_FILL_VALUE);
		CHECK(cmd.fill.start == 0x0000);
		CHECK(cmd.fill.end   == 0x1000);
		CHECK(cmd.fill.value == 0xAA);
	}
	SECTION("end <= start is rejected")
	{
		CHECK_FALSE(parse("fill 200 100", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("fill value > FF is rejected")
	{
		CHECK_FALSE(parse("fill all 100", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
}

// ============================================================
//  time
// ============================================================

TEST_CASE("time command", "[parser][time]")
{
	parsed_cmd_t cmd;

	SECTION("time alone is a query")
	{
		REQUIRE(parse("time", &cmd));
		CHECK(cmd.tag == CMD_TIME_QUERY);
	}
	SECTION("year only — count == 1")
	{
		REQUIRE(parse("time 2026", &cmd));
		CHECK(cmd.tag       == CMD_TIME_SET);
		CHECK(cmd.time.year == 2026);
		CHECK(cmd.time.count == 1);
	}
	SECTION("year and month — count == 2, month is 0-based")
	{
		REQUIRE(parse("time 2026 2", &cmd));
		CHECK(cmd.time.year  == 2026);
		CHECK(cmd.time.month == 1);   /* February → 0-based index 1 */
		CHECK(cmd.time.count == 2);
	}
	SECTION("all six fields")
	{
		REQUIRE(parse("time 2026 2 20 10 30 45", &cmd));
		CHECK(cmd.tag           == CMD_TIME_SET);
		CHECK(cmd.time.year     == 2026);
		CHECK(cmd.time.month    == 1);   /* 0-based */
		CHECK(cmd.time.day      == 20);
		CHECK(cmd.time.hour     == 10);
		CHECK(cmd.time.minute   == 30);
		CHECK(cmd.time.second   == 45);
		CHECK(cmd.time.count    == 6);
	}
	SECTION("boundary year 2025 is accepted")
	{
		REQUIRE(parse("time 2025", &cmd));
		CHECK(cmd.time.year == 2025);
	}
	SECTION("boundary year 2061 is accepted")
	{
		REQUIRE(parse("time 2061", &cmd));
		CHECK(cmd.time.year == 2061);
	}
	SECTION("year 2024 (< 2025) is rejected")
	{
		CHECK_FALSE(parse("time 2024", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("year 2062 (> 2061) is rejected")
	{
		CHECK_FALSE(parse("time 2062", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("month 0 is rejected")
	{
		CHECK_FALSE(parse("time 2026 0", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("month 13 is rejected")
	{
		CHECK_FALSE(parse("time 2026 13", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("day 0 is rejected")
	{
		CHECK_FALSE(parse("time 2026 1 0", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("day 32 is rejected")
	{
		CHECK_FALSE(parse("time 2026 1 32", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("hour 24 is rejected")
	{
		CHECK_FALSE(parse("time 2026 1 1 24", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("minute 60 is rejected")
	{
		CHECK_FALSE(parse("time 2026 1 1 0 60", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("second 60 is rejected")
	{
		CHECK_FALSE(parse("time 2026 1 1 0 0 60", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
}

// ============================================================
//  go
// ============================================================

TEST_CASE("go command", "[parser][go]")
{
	parsed_cmd_t cmd;

	SECTION("go alone")
	{
		REQUIRE(parse("go", &cmd));
		CHECK(cmd.tag == CMD_GO);
	}
	SECTION("go with address")
	{
		REQUIRE(parse("go 1000", &cmd));
		CHECK(cmd.tag        == CMD_GO_ADDR);
		CHECK(cmd.go.address == 0x1000);
	}
	SECTION("go with max address FFFF")
	{
		REQUIRE(parse("go FFFF", &cmd));
		CHECK(cmd.go.address == 0xFFFF);
	}
}

// ============================================================
//  task
// ============================================================

TEST_CASE("task command", "[parser][task]")
{
	parsed_cmd_t cmd;

	SECTION("task alone is a query")
	{
		REQUIRE(parse("task", &cmd));
		CHECK(cmd.tag == CMD_TASK_QUERY);
	}
	SECTION("task 0 (minimum)")
	{
		REQUIRE(parse("task 0", &cmd));
		CHECK(cmd.tag           == CMD_TASK_SET);
		CHECK(cmd.task.tasknum  == 0);
	}
	SECTION("task F (maximum, hex 15)")
	{
		REQUIRE(parse("task F", &cmd));
		CHECK(cmd.task.tasknum == 15);
	}
	SECTION("task 10 (hex 16, out of range) is rejected")
	{
		CHECK_FALSE(parse("task 10", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
}

// ============================================================
//  irq
// ============================================================

TEST_CASE("irq command", "[parser][irq]")
{
	parsed_cmd_t cmd;

	SECTION("irq 0 (minimum)")
	{
		REQUIRE(parse("irq 0", &cmd));
		CHECK(cmd.tag          == CMD_IRQ);
		CHECK(cmd.irq.testnum  == 0);
	}
	SECTION("irq 5 (maximum)")
	{
		REQUIRE(parse("irq 5", &cmd));
		CHECK(cmd.irq.testnum == 5);
	}
	SECTION("irq 6 (out of range) is rejected")
	{
		CHECK_FALSE(parse("irq 6", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
}

// ============================================================
//  trace
// ============================================================

TEST_CASE("trace command", "[parser][trace]")
{
	parsed_cmd_t cmd;

	SECTION("trace alone")
	{
		REQUIRE(parse("trace", &cmd));
		CHECK(cmd.tag == CMD_TRACE);
	}
	SECTION("trace with length")
	{
		REQUIRE(parse("trace 100", &cmd));
		CHECK(cmd.tag           == CMD_TRACE_LEN);
		CHECK(cmd.trace.length  == 0x100);
	}
}

// ============================================================
//  run
// ============================================================

TEST_CASE("run command", "[parser][run]")
{
	parsed_cmd_t cmd;

	SECTION("run alone — list snippets")
	{
		REQUIRE(parse("run", &cmd));
		CHECK(cmd.tag == CMD_RUN_LIST);
	}
	SECTION("run with index")
	{
		REQUIRE(parse("run 2", &cmd));
		CHECK(cmd.tag       == CMD_RUN);
		CHECK(cmd.run.index == 2);
	}
	SECTION("run with index and ! (no-wait)")
	{
		REQUIRE(parse("run 2 !", &cmd));
		CHECK(cmd.tag       == CMD_RUN_NOWAIT);
		CHECK(cmd.run.index == 2);
	}
}

// ============================================================
//  stop
// ============================================================

TEST_CASE("stop command", "[parser][stop]")
{
	parsed_cmd_t cmd;

	SECTION("stop alone")
	{
		REQUIRE(parse("stop", &cmd));
		CHECK(cmd.tag == CMD_STOP);
	}
	SECTION("stop halt")
	{
		REQUIRE(parse("stop halt", &cmd));
		CHECK(cmd.tag == CMD_STOP_HALT);
	}
	SECTION("stop HALT (case-insensitive subcommand)")
	{
		REQUIRE(parse("stop HALT", &cmd));
		CHECK(cmd.tag == CMD_STOP_HALT);
	}
	SECTION("stop restart")
	{
		REQUIRE(parse("stop restart", &cmd));
		CHECK(cmd.tag == CMD_STOP_RESTART);
	}
	SECTION("stop stop")
	{
		REQUIRE(parse("stop stop", &cmd));
		CHECK(cmd.tag == CMD_STOP_STOP);
	}
}

// ============================================================
//  freq
// ============================================================

TEST_CASE("freq command", "[parser][freq]")
{
	using Catch::Matchers::WithinRel;
	parsed_cmd_t cmd;

	SECTION("freq 1.0 (floating-point token)")
	{
		REQUIRE(parse("freq 1.0", &cmd));
		CHECK(cmd.tag == CMD_FREQ);
		CHECK_THAT(cmd.freq.mhz, WithinRel(1.0f, 0.001f));
	}
	SECTION("freq 2 (integer token accepted as 2.0 MHz)")
	{
		REQUIRE(parse("freq 2", &cmd));
		CHECK(cmd.tag == CMD_FREQ);
		CHECK_THAT(cmd.freq.mhz, WithinRel(2.0f, 0.001f));
	}
	SECTION("freq 0.8 (minimum boundary)")
	{
		REQUIRE(parse("freq 0.8", &cmd));
		CHECK_THAT(cmd.freq.mhz, WithinRel(0.8f, 0.001f));
	}
	SECTION("freq 3.0 (maximum boundary)")
	{
		REQUIRE(parse("freq 3.0", &cmd));
		CHECK_THAT(cmd.freq.mhz, WithinRel(3.0f, 0.001f));
	}
	SECTION("freq 0.5 (below 0.8) is rejected")
	{
		CHECK_FALSE(parse("freq 0.5", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("freq 3.5 (above 3.0) is rejected")
	{
		CHECK_FALSE(parse("freq 3.5", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("freq 0 (integer zero) is rejected")
	{
		CHECK_FALSE(parse("freq 0", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
}

// ============================================================
//  usb
// ============================================================

TEST_CASE("usb command", "[parser][usb]")
{
	parsed_cmd_t cmd;

	SECTION("usb with single argument")
	{
		REQUIRE(parse("usb boot", &cmd));
		CHECK(cmd.tag == CMD_USB);
		CHECK(std::strcmp(cmd.usb.args, "boot") == 0);
	}
	SECTION("usb with multiple space-separated arguments")
	{
		REQUIRE(parse("usb msc -d /dev/sda", &cmd));
		CHECK(cmd.tag == CMD_USB);
		CHECK(std::strcmp(cmd.usb.args, "msc -d /dev/sda") == 0);
	}
	SECTION("usb without arguments is a syntax error")
	{
		CHECK_FALSE(parse("usb", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
}

// ============================================================
//  srecord
// ============================================================

TEST_CASE("srecord command", "[parser][srecord]")
{
	parsed_cmd_t cmd;

	SECTION("S1 data record")
	{
		REQUIRE(parse("S1030000FC", &cmd));
		CHECK(cmd.tag == CMD_SRECORD);
		CHECK(std::strncmp(cmd.srecord.line, "S1030000FC", 10) == 0);
	}
	SECTION("S0 header record — minimal")
	{
		REQUIRE(parse("S0030000FC", &cmd));
		CHECK(cmd.tag == CMD_SRECORD);
		CHECK(std::strncmp(cmd.srecord.line, "S0030000FC", 10) == 0);
	}
	SECTION("S0 header record — 104 chars, user-reported NitrOS-9 boot image")
	{
		// CC=0x32=50, addr=0000, "Pico-Thing NitrOS-9 boot image 2026-02-28 21:30"
		// 104 chars total — within poll_console's COMMAND_BUFFER_LEN-1=127
		const char *s0 =
			"S03200005069636F2D5468696E67204E6974724F532D3920626F6F74"
			"20696D61676520323032362D30322D32382032313A33302F";
		REQUIRE(parse(s0, &cmd));
		CHECK(cmd.tag == CMD_SRECORD);
		// Verify the full 104-char record is stored intact in srecord.line
		CHECK(std::strncmp(cmd.srecord.line, s0, 104) == 0);
		CHECK(cmd.srecord.line[104] == '\0');
	}
	SECTION("S9 end record")
	{
		REQUIRE(parse("S9030000FC", &cmd));
		CHECK(cmd.tag == CMD_SRECORD);
	}
}

// ============================================================
//  Error cases
// ============================================================

TEST_CASE("Unknown or empty input produces syntax error", "[parser][error]")
{
	parsed_cmd_t cmd;

	SECTION("unknown word")
	{
		CHECK_FALSE(parse("foobar", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("empty input")
	{
		CHECK_FALSE(parse("", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
	SECTION("whitespace only")
	{
		CHECK_FALSE(parse("   ", &cmd));
		CHECK(cmd.tag == CMD_SYNTAX_ERROR);
	}
}