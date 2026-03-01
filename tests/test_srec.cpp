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

// ============================================================
//  S0 header record tests
// ============================================================

TEST_CASE("process_srecord() parses Motorola S0 header records", "[srec][s0]")
{
    uint8_t  data[64] = {};
    uint8_t  count    = 0;
    uint8_t  checksum = 0;
    uint16_t address  = 0;

    SECTION("minimal S0 — no data bytes, address $0000")
    {
        // CC=03 AAAA=0000 (no data)  sum=03 → CS=FC
        const char *rec = "S0030000FC";
        REQUIRE(process_srecord(rec, &count, &address, data, &checksum));
        CHECK(count    == 3);
        CHECK(address  == 0x0000);
        CHECK(checksum == 0xFC);
        CHECK((count - 3) == 0);
    }

    SECTION("specific NitrOS-9 S0 header (CC=0x32, 47 data bytes) — user-reported")
    {
        // S03200005069636F2D5468696E67204E6974724F532D3920626F6F74
        //   20696D61676520323032362D30322D32382032313A33302F
        // CC=50, addr=0x0000, data="Pico-Thing NitrOS-9 boot image 2026-02-28 21:30"
        // checksum verified: 0xFF - ((uint8_t)(50 + sum_of_47_bytes)) = 0x2F
        const char *rec =
            "S03200005069636F2D5468696E67204E6974724F532D3920626F6F74"
            "20696D61676520323032362D30322D32382032313A33302F";
        REQUIRE(process_srecord(rec, &count, &address, data, &checksum));
        CHECK(count    == 0x32);          // CC = 50
        CHECK(address  == 0x0000);
        CHECK(checksum == 0x2F);
        CHECK((count - 3) == 47);
        // First four decoded bytes: 'P','i','c','o'
        CHECK(data[0] == 0x50);
        CHECK(data[1] == 0x69);
        CHECK(data[2] == 0x63);
        CHECK(data[3] == 0x6F);
        // Last decoded byte: '0' (0x30)
        CHECK(data[46] == 0x30);
    }

    SECTION("truncated S0 — NUL injected inside checksum field returns false")
    {
        // Simulate what poll_console does when an S0 record is longer than
        // COMMAND_BUFFER_LEN-1 = 127 chars: the string is NUL-terminated mid-record.
        // CC=0x3E=62, addr=0000, 59 zero data bytes, full checksum "C1".
        // Build the 128-char full record then cut it to 127 (drop the trailing '1').
        // At pos=126 hex() sees 'C' then '\0' → returns (uint)-1 → false.
        const char *full =
            "S03E0000"
            "00000000000000000000000000000000"  /* 32 chars = 16 bytes */
            "00000000000000000000000000000000"  /* 32 chars = 16 bytes */
            "000000000000000000000000000000"    /* 30 chars = 15 bytes */
            "000000"                            /* 6 chars  =  3 bytes, total 50 */
            "000000000000000000"                /* 18 chars =  9 bytes, total 59 */
            "C1";
        // The full record above is 8+118+2 = 128 chars.
        // Truncate by copying only the first 127 chars (drop the final '1').
        char buf[128];
        for (int i = 0; i < 127; i++) buf[i] = full[i];
        buf[127] = '\0';
        CHECK_FALSE(process_srecord(buf, &count, &address, data, &checksum));
    }
}

// ============================================================
//  S0 → S1 sequence: same data buffer
// ============================================================

TEST_CASE("process_srecord() S0 header does not corrupt subsequent S1 data", "[srec][sequence]")
{
    // process_command() in pico_thing.cpp uses a single static buffer[256] for
    // both S0 and S1 records.  This test verifies that parsing S0 (which fills
    // data[0..CC-4]) followed by S1 (which fills data[0..CC-4]) leaves the
    // S1 data intact and returns the correct S1 address and byte count.
    uint8_t  data[64] = {};
    uint8_t  count    = 0;
    uint8_t  checksum = 0;
    uint16_t address  = 0;

    // Step 1 — parse the user-reported S0 header (writes data[0..46])
    const char *s0 =
        "S03200005069636F2D5468696E67204E6974724F532D3920626F6F74"
        "20696D61676520323032362D30322D32382032313A33302F";
    REQUIRE(process_srecord(s0, &count, &address, data, &checksum));
    REQUIRE(count == 0x32);
    REQUIRE(data[0] == 0x50);  // 'P' — confirm S0 was parsed

    // Step 2 — parse first S1 from assist09.s19 using the same buffer
    // S113F400308DE7BE1F101F8B979D3384318C35EFF1
    // CC=0x13=19, addr=0xF400, 16 data bytes, checksum=0xF1
    const char *s1 = "S113F400308DE7BE1F101F8B979D3384318C35EFF1";
    REQUIRE(process_srecord(s1, &count, &address, data, &checksum));
    CHECK(count    == 0x13);
    CHECK(address  == 0xF400);
    CHECK(checksum == 0xF1);
    CHECK((count - 3) == 16);
    // data[0..15] must contain S1 bytes, NOT stale S0 ('P','i','c','o'…)
    CHECK(data[0]  == 0x30);
    CHECK(data[1]  == 0x8D);
    CHECK(data[2]  == 0xE7);
    CHECK(data[3]  == 0xBE);
    CHECK(data[4]  == 0x1F);
    CHECK(data[5]  == 0x10);
    CHECK(data[6]  == 0x1F);
    CHECK(data[7]  == 0x8B);
    CHECK(data[8]  == 0x97);
    CHECK(data[9]  == 0x9D);
    CHECK(data[10] == 0x33);
    CHECK(data[11] == 0x84);
    CHECK(data[12] == 0x31);
    CHECK(data[13] == 0x8C);
    CHECK(data[14] == 0x35);
    CHECK(data[15] == 0xEF);
}
