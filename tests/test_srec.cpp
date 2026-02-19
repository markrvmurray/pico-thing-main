/**
 * Copyright (c) 2025 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sys/types.h>  // uint (POSIX)
#include <cstdint>

#include <catch2/catch_test_macros.hpp>

#include "srec.h"

// ============================================================
//  hex()
// ============================================================

TEST_CASE("hex() converts hex digit strings to unsigned integers", "[hex]")
{
    SECTION("single digit — all valid hex characters")
    {
        // decimal digits
        CHECK(hex("0", 1) == 0x0);
        CHECK(hex("9", 1) == 0x9);
        // uppercase A-F
        CHECK(hex("A", 1) == 0xA);
        CHECK(hex("F", 1) == 0xF);
        // lowercase a-f
        CHECK(hex("a", 1) == 0xa);
        CHECK(hex("f", 1) == 0xf);
    }

    SECTION("two-digit byte values")
    {
        CHECK(hex("00", 2) == 0x00);
        CHECK(hex("FF", 2) == 0xFF);
        CHECK(hex("ff", 2) == 0xFF);
        CHECK(hex("10", 2) == 0x10);
        CHECK(hex("AB", 2) == 0xAB);
        CHECK(hex("ab", 2) == 0xAB);
    }

    SECTION("four-digit word values")
    {
        CHECK(hex("0000", 4) == 0x0000);
        CHECK(hex("FFFF", 4) == 0xFFFF);
        CHECK(hex("1234", 4) == 0x1234);
        CHECK(hex("DEAD", 4) == 0xDEAD);
        CHECK(hex("dead", 4) == 0xDEAD);
    }

    SECTION("count=0 reads until null terminator")
    {
        CHECK(hex("00",       0) == 0x00);
        CHECK(hex("FF",       0) == 0xFF);
        CHECK(hex("1234",     0) == 0x1234);
        CHECK(hex("DEADBEEF", 0) == 0xDEADBEEF);
    }

    SECTION("invalid first character returns (uint)-1")
    {
        CHECK(hex("G0", 2) == (uint)-1);
        CHECK(hex("Z0", 2) == (uint)-1);
        CHECK(hex(" 0", 2) == (uint)-1);
        CHECK(hex("g0", 2) == (uint)-1);  // 'g' is just past 'f'
    }

    SECTION("invalid second character returns (uint)-1")
    {
        CHECK(hex("1G", 2) == (uint)-1);
        CHECK(hex("1 ", 2) == (uint)-1);
    }

    SECTION("empty string with count=0 returns (uint)-1")
    {
        // '\0' is not a valid hex digit
        CHECK(hex("", 0) == (uint)-1);
    }
}

// ============================================================
//  process_srecord()
//
//  S-record format (S1, 16-bit address):
//    S1 | CC | AAAA | DD…DD | XX
//       ^-- pos 2 is where the function starts reading
//  CC  = byte count = 2 (addr) + N (data) + 1 (checksum)
//  XX  = 0xFF - low_byte(CC + A_hi + A_lo + sum(data))
//
//  *count output is the raw CC byte; data byte count = *count - 3.
// ============================================================

TEST_CASE("process_srecord() parses Motorola S1 records", "[srec]")
{
    uint8_t  data[64] = {};
    uint8_t  count    = 0;
    uint8_t  checksum = 0;
    uint16_t address  = 0;

    // ----------------------------------------------------------
    // Valid records
    // ----------------------------------------------------------

    SECTION("no data bytes, address $0000")
    {
        // CC=03  AAAA=0000  (no data)
        // sum = 03+00+00 = 03  →  CS = FF-03 = FC
        const char *rec = "S1030000FC";
        REQUIRE(process_srecord(rec, &count, &address, data, &checksum));
        CHECK(count   == 3);
        CHECK(address == 0x0000);
        CHECK(checksum == 0xFC);
    }

    SECTION("three data bytes at address $0100")
    {
        // CC=06  AAAA=0100  data=[DE,AD,BE]
        // sum = 06+01+00+DE+AD+BE = 0x250  →  CS = FF-50 = AF
        const char *rec = "S1060100DEADBEAF";
        REQUIRE(process_srecord(rec, &count, &address, data, &checksum));
        CHECK(count   == 6);
        CHECK(address == 0x0100);
        CHECK(data[0] == 0xDE);
        CHECK(data[1] == 0xAD);
        CHECK(data[2] == 0xBE);
        CHECK(checksum == 0xAF);
        // caller-facing contract: data byte count = count - 3
        CHECK((count - 3) == 3);
    }

    SECTION("single data byte at address $1234")
    {
        // CC=04  AAAA=1234  data=[AB]
        // sum = 04+12+34+AB = 0xF5  →  CS = FF-F5 = 0A
        const char *rec = "S1041234AB0A";
        REQUIRE(process_srecord(rec, &count, &address, data, &checksum));
        CHECK(count   == 4);
        CHECK(address == 0x1234);
        CHECK(data[0] == 0xAB);
        CHECK(checksum == 0x0A);
    }

    SECTION("all-FF data — checksum arithmetic wraps through uint8_t")
    {
        // CC=05  AAAA=0000  data=[FF,FF]
        // sum = 05+00+00+FF+FF = 0x203  →  (uint8_t)0x203 = 03  →  CS = FF-03 = FC
        const char *rec = "S1050000FFFFFC";
        REQUIRE(process_srecord(rec, &count, &address, data, &checksum));
        CHECK(count   == 5);
        CHECK(address == 0x0000);
        CHECK(data[0] == 0xFF);
        CHECK(data[1] == 0xFF);
        CHECK(checksum == 0xFC);
    }

    SECTION("lowercase hex digits are accepted")
    {
        const char *rec = "S1060100deadbeaf";
        REQUIRE(process_srecord(rec, &count, &address, data, &checksum));
        CHECK(address == 0x0100);
        CHECK(data[0] == 0xDE);
        CHECK(data[1] == 0xAD);
        CHECK(data[2] == 0xBE);
    }

    // ----------------------------------------------------------
    // Checksum failures
    // ----------------------------------------------------------

    SECTION("wrong checksum returns false")
    {
        // Correct checksum is AF; supplying AE must fail
        const char *rec = "S1060100DEADBEAE";
        CHECK_FALSE(process_srecord(rec, &count, &address, data, &checksum));
    }

    // ----------------------------------------------------------
    // Invalid hex in each field
    // ----------------------------------------------------------

    SECTION("invalid hex in count field returns false")
    {
        const char *rec = "S1GG0000FC";
        CHECK_FALSE(process_srecord(rec, &count, &address, data, &checksum));
    }

    SECTION("invalid hex in address field returns false")
    {
        const char *rec = "S103GGGGFC";
        CHECK_FALSE(process_srecord(rec, &count, &address, data, &checksum));
    }

    SECTION("invalid hex in data field returns false")
    {
        // Third data byte position contains "GG"
        const char *rec = "S1060100DEADGGAF";
        CHECK_FALSE(process_srecord(rec, &count, &address, data, &checksum));
    }

    SECTION("invalid hex in checksum field returns false")
    {
        const char *rec = "S1030000GG";
        CHECK_FALSE(process_srecord(rec, &count, &address, data, &checksum));
    }
}
