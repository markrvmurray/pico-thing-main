#include <cmoc.h>
#include "pico.h"
#include "ide_lib.h"

/*
 * DAT RAM Torture Test
 *
 * Tests for the Pico firmware DAT translation bug observed during
 * NitrOS-9 boot: writes to slot 6 ($C000-$DFFF) sometimes land
 * on page $07 (slot 7) at the same intra-page offset.
 *
 * The boot loop does:
 *   ldd  $FF00      ; read IDE data register (slot 7 / I/O space)
 *   std  ,x++       ; write to boot memory in slot 4/5/6
 * After ~76 sectors, page $07 at offset $0800-$09FF is overwritten
 * with bootfile data that should have gone to slot 6 (page $03).
 *
 * This program reproduces the exact bus activity and checks for bleed.
 *
 * Memory layout (identity DAT mapping, task 0):
 *   $0130-$3FFF  program code + data (slots 0-1)
 *   $8000-$9FFF  slot 4, page $04
 *   $A000-$BFFF  slot 5, page $05
 *   $C000-$DFFF  slot 6, page $06
 *   $E000-$FFFF  slot 7, page $07 (stack at $F000, I/O at $FF00+)
 */

#define DAT_REGS  ((uint8_t *)0xFE00)
#define DAT_TASK  ((uint8_t *)0xFFC0)

#define SENTINEL  ((uint8_t)0xA5)

/* ---------- helpers ---------- */

static void
print_dat(void)
{
	unsigned i;
	printf("  DAT: ");
	for (i = 0; i < 8; i++)
		printf("%02X ", *(DAT_REGS + i));
	printf("\n");
}

static void
fill_sentinel(uint8_t *addr, unsigned len)
{
	unsigned i;
	for (i = 0; i < len; i++)
		addr[i] = SENTINEL;
}

static unsigned
check_sentinel(uint8_t *addr, unsigned len, bool verbose)
{
	unsigned i, errors;
	errors = 0;
	for (i = 0; i < len; i++) {
		if (addr[i] != SENTINEL) {
			if (verbose && errors < 8)
				printf("    $%04X: $%02X (want $%02X)\n",
				    (unsigned)(addr + i), addr[i], SENTINEL);
			errors++;
		}
	}
	return errors;
}

/* Issue IDE IDENTIFY — makes 256 words (512 bytes) available at $FF00 */
static void
ide_start_identify(void)
{
	outb((uint8_t *)0xFF07, 0xA0);		/* DEVHEAD = master */
	while (inb((uint8_t *)0xFF08) & 0x80)	/* wait BSY clear */
		;
	outb((uint8_t *)0xFF08, 0xEC);		/* IDENTIFY command */
	while (!(inb((uint8_t *)0xFF08) & 0x08)) /* wait DRQ */
		;
}

/* Drain n words from IDE data register */
static void
ide_drain(unsigned n)
{
	uint16_t *ide = (uint16_t *)0xFF00;
	while (n--)
		(void)*ide;
}

/*
 * asm_rd256 — exact reproduction of boot_picothing.asm Rd256
 *
 *   ldd  ,y       ; Y = $FF00 (IDE data reg, 16-bit)
 *   std  ,x++     ; X = target buffer
 *   128 iterations = 256 bytes
 */
static void
asm_rd256(uint16_t *target)
{
	asm {
		pshs	x,y
		ldx	:target
		ldy	#$FF00
		ldb	#128
		pshs	b
rd256@		ldd	,y
		std	,x++
		dec	,s
		bne	rd256@
		leas	1,s
		puls	x,y
	}
}

/*
 * asm_rd256_from_ram — same tight loop but reading from a RAM
 * address instead of $FF00, to isolate IDE-specific effects.
 */
static void
asm_rd256_from_ram(uint16_t *source, uint16_t *target)
{
	asm {
		pshs	x,y
		ldx	:target
		ldy	:source
		ldb	#128
		pshs	b
rr256@		ldd	,y
		std	,x++
		dec	,s
		bne	rr256@
		leas	1,s
		puls	x,y
	}
}

/* ---------- tests ---------- */

/*
 * Test 1 — pure write (no IDE), baseline
 * Write a pattern to slot 6 at offset $0800, check that the
 * sentinel at slot 7 offset $0800 ($E800) is untouched.
 */
static unsigned
test1_pure_write(void)
{
	unsigned pass, i, errors, total;
	uint16_t *target;

	printf("\n[1] Pure write to $C800, sentinel at $E800\n");
	total = 0;
	for (pass = 0; pass < 20; pass++) {
		fill_sentinel((uint8_t *)0xE800, 256);
		target = (uint16_t *)0xC800;
		for (i = 0; i < 128; i++)
			target[i] = (uint16_t)(i + pass * 128);
		errors = check_sentinel((uint8_t *)0xE800, 256, (bool)(pass < 3));
		if (errors) {
			printf("  pass %u: %u errors\n", pass, errors);
			total += errors;
		}
	}
	printf("  %s (20 passes, %u errors)\n", total ? "FAIL" : "PASS", total);
	return total;
}

/*
 * Test 2 — IDE read + write (C compiler loop)
 * Read from $FF00 then write to $C800 in a C for-loop.
 * The C compiler won't generate the exact same instructions as
 * the handwritten Rd256, but the bus activity is similar.
 */
static unsigned
test2_ide_c_loop(void)
{
	unsigned pass, i, errors, total;
	uint16_t *ide, *target;

	printf("\n[2] IDE read + C-loop write to $C800\n");
	total = 0;
	ide = (uint16_t *)0xFF00;
	for (pass = 0; pass < 20; pass++) {
		fill_sentinel((uint8_t *)0xE800, 256);
		target = (uint16_t *)0xC800;
		ide_start_identify();
		for (i = 0; i < 128; i++)
			target[i] = *ide;
		ide_drain(128);
		errors = check_sentinel((uint8_t *)0xE800, 256, (bool)(pass < 3));
		if (errors) {
			printf("  pass %u: %u errors\n", pass, errors);
			total += errors;
		}
	}
	printf("  %s (20 passes, %u errors)\n", total ? "FAIL" : "PASS", total);
	return total;
}

/*
 * Test 3 — exact Rd256 assembly loop (boot pattern)
 * This is the tightest possible reproduction of the boot code.
 */
static unsigned
test3_asm_rd256(void)
{
	unsigned pass, errors, total;

	printf("\n[3] Assembly Rd256 ($FF00 -> $C800)\n");
	total = 0;
	for (pass = 0; pass < 20; pass++) {
		fill_sentinel((uint8_t *)0xE800, 256);
		ide_start_identify();
		asm_rd256((uint16_t *)0xC800);
		ide_drain(128);
		errors = check_sentinel((uint8_t *)0xE800, 256, (bool)(pass < 3));
		if (errors) {
			printf("  pass %u: %u errors\n", pass, errors);
			total += errors;
		}
	}
	printf("  %s (20 passes, %u errors)\n", total ? "FAIL" : "PASS", total);
	return total;
}

/*
 * Test 4 — Rd256 targeting each slot
 * For each slot 0-6, write 256 bytes at offset $0800
 * and check if the sentinel at slot 7 ($E800) is corrupted.
 */
static unsigned
test4_all_slots(void)
{
	unsigned slot, errors, total;
	uint16_t bases[] = { 0x0800, 0x2800, 0x4800, 0x6800,
	                     0x8800, 0xA800, 0xC800 };

	printf("\n[4] Rd256 to each slot, sentinel at $E800\n");
	total = 0;
	for (slot = 0; slot < 7; slot++) {
		if (bases[slot] >= 0x0130 && bases[slot] < 0x4000) {
			printf("  slot %u ($%04X): SKIP (code area)\n",
			    slot, bases[slot]);
			continue;
		}
		fill_sentinel((uint8_t *)0xE800, 256);
		ide_start_identify();
		asm_rd256((uint16_t *)bases[slot]);
		ide_drain(128);
		errors = check_sentinel((uint8_t *)0xE800, 256, true);
		printf("  slot %u ($%04X): %u errors%s\n",
		    slot, bases[slot], errors, errors ? " ***FAIL***" : "");
		total += errors;
	}
	printf("  %s (%u errors)\n", total ? "FAIL" : "PASS", total);
	return total;
}

/*
 * Test 5 — RAM-source tight loop (no IDE)
 * Same tight loop but reading from a RAM address in slot 7
 * ($E000) instead of $FF00.  If this also corrupts, the
 * bug is about any slot-7-read → slot-6-write transition,
 * not specifically IDE.
 */
static unsigned
test5_ram_source(void)
{
	unsigned pass, errors, total;

	printf("\n[5] RAM-source Rd256 ($E000 -> $C800), sentinel $E800\n");
	total = 0;
	/* Fill the source area with a known pattern */
	fill_sentinel((uint8_t *)0xE000, 256);
	for (pass = 0; pass < 20; pass++) {
		fill_sentinel((uint8_t *)0xE800, 256);
		asm_rd256_from_ram((uint16_t *)0xE000, (uint16_t *)0xC800);
		errors = check_sentinel((uint8_t *)0xE800, 256, (bool)(pass < 3));
		if (errors) {
			printf("  pass %u: %u errors\n", pass, errors);
			total += errors;
		}
	}
	printf("  %s (20 passes, %u errors)\n", total ? "FAIL" : "PASS", total);
	return total;
}

/*
 * Test 6 — non-identity DAT mapping (NitrOS-9 boot config)
 * Remap slot 6 → page $03 (like during boot: slots 4-6 = pages 1-3).
 * Write to $C800 should go to page $03, not page $07.
 * Sentinel at $E800 (page $07) checks for bleed.
 * Verify at $6800 (slot 3 = page $03, identity) confirms data arrived.
 */
static unsigned
test6_nonidentity(void)
{
	unsigned pass, i, errors, verify_ok, total;
	uint16_t *verify;
	uint8_t saved;

	printf("\n[6] Non-identity DAT (slot6=pg$03), Rd256 to $C800\n");
	total = 0;
	saved = *(DAT_REGS + 6);
	for (pass = 0; pass < 20; pass++) {
		fill_sentinel((uint8_t *)0xE800, 256);
		/* Clear verify area: slot 3 = page $03, offset $0800 */
		verify = (uint16_t *)0x6800;
		for (i = 0; i < 128; i++)
			verify[i] = 0;

		/* Remap slot 6 → page $03 */
		*(DAT_REGS + 6) = 0x03;

		ide_start_identify();
		asm_rd256((uint16_t *)0xC800);
		ide_drain(128);

		/* Restore slot 6 */
		*(DAT_REGS + 6) = saved;

		/* Check sentinel (page $07 offset $0800) */
		errors = check_sentinel((uint8_t *)0xE800, 256, (bool)(pass < 3));
		if (errors) {
			printf("  pass %u: %u sentinel errors\n", pass, errors);
			total += errors;
		}

		/* Verify data arrived at page $03 offset $0800 */
		verify_ok = 0;
		for (i = 0; i < 128; i++) {
			if (verify[i] != 0)
				verify_ok++;
		}
		if (verify_ok == 0 && pass < 3)
			printf("  pass %u: WARNING - no data at page $03\n", pass);
	}
	printf("  %s (20 passes, %u errors)\n", total ? "FAIL" : "PASS", total);
	return total;
}

/*
 * Test 7 — full boot simulation
 * Set up DAT like NitrOS-9 boot: slot 4=pg1, 5=pg2, 6=pg3, 7=pg7.
 * Read 88 IDE sectors into $8000+, incrementing by $0100 each time.
 * After all reads, check page $07 sentinels.
 */
static unsigned
test7_boot_sim(void)
{
	unsigned sector, errors, total;
	uint8_t saved4, saved5, saved6;
	uint16_t addr;

	printf("\n[7] Full boot simulation (88 sectors to $8000+)\n");
	total = 0;

	saved4 = *(DAT_REGS + 4);
	saved5 = *(DAT_REGS + 5);
	saved6 = *(DAT_REGS + 6);

	/* Fill page $07 sentinels — stop at $EE00 to avoid stack area */
	fill_sentinel((uint8_t *)0xE000, 0x0E00);

	/* Remap slots 4-6 to pages that don't alias with the program.
	 * Program uses slots 0-1 (pages 0-1), so use pages $08-$0A.
	 * Slot 7 (page $07) holds sentinels + stack + I/O.
	 */
	*(DAT_REGS + 4) = 0x08;
	*(DAT_REGS + 5) = 0x09;
	*(DAT_REGS + 6) = 0x0A;

	/* Read 88 sectors into $8000-$D7FF */
	for (sector = 0; sector < 88; sector++) {
		addr = 0x8000 + (sector * 0x100);
		ide_start_identify();
		asm_rd256((uint16_t *)addr);
		ide_drain(128);
	}

	/* Restore DAT */
	*(DAT_REGS + 4) = saved4;
	*(DAT_REGS + 5) = saved5;
	*(DAT_REGS + 6) = saved6;

	/* Check page $07 sentinels — stop at $EE00 to avoid stack bleed
	 * (stack at $F000 grows downward into $EFxx during function calls)
	 */
	errors = check_sentinel((uint8_t *)0xE000, 0x0800, true);
	if (errors)
		printf("  $E000-$E7FF: %u errors\n", errors);
	total += errors;

	errors = check_sentinel((uint8_t *)0xE800, 0x0200, true);
	if (errors)
		printf("  $E800-$E9FF: %u errors\n", errors);
	total += errors;

	errors = check_sentinel((uint8_t *)0xEA00, 0x0400, true);
	if (errors)
		printf("  $EA00-$EDFF: %u errors\n", errors);
	total += errors;

	printf("  %s (%u errors)\n", total ? "FAIL" : "PASS", total);
	print_dat();
	return total;
}

/*
 * Test 8 — DAT RAM integrity
 * After 100 IDE transfers, verify DAT RAM hasn't been corrupted.
 */
static unsigned
test8_dat_integrity(void)
{
	unsigned pass, i, errors, total;
	uint8_t before[8], after[8];

	printf("\n[8] DAT RAM integrity (100 IDE transfers)\n");
	total = 0;
	for (pass = 0; pass < 100; pass++) {
		for (i = 0; i < 8; i++)
			before[i] = *(DAT_REGS + i);
		ide_start_identify();
		asm_rd256((uint16_t *)0xC800);
		ide_drain(128);
		errors = 0;
		for (i = 0; i < 8; i++) {
			after[i] = *(DAT_REGS + i);
			if (after[i] != before[i]) {
				if (total < 5)
					printf("  pass %u: DAT[%u] $%02X -> $%02X\n",
					    pass, i, before[i], after[i]);
				errors++;
			}
		}
		total += errors;
	}
	printf("  %s (100 passes, %u errors)\n", total ? "FAIL" : "PASS", total);
	return total;
}

/*
 * Test 9 — sweep all page offsets
 * Rd256 to slot 6 at every $0100 offset; check which offsets
 * cause bleed into page $07.
 */
static unsigned
test9_offset_sweep(void)
{
	unsigned offset, errors, total;
	uint16_t target_addr, sentinel_addr;

	printf("\n[9] Offset sweep: Rd256 to slot 6, all offsets\n");
	total = 0;
	for (offset = 0; offset < 0x2000; offset += 0x0100) {
		target_addr = 0xC000 + offset;
		sentinel_addr = 0xE000 + offset;

		/* Skip if sentinel overlaps stack ($EF00+) or I/O */
		if (sentinel_addr >= 0xEF00)
			continue;

		fill_sentinel((uint8_t *)sentinel_addr, 256);
		ide_start_identify();
		asm_rd256((uint16_t *)target_addr);
		ide_drain(128);
		errors = check_sentinel((uint8_t *)sentinel_addr, 256, false);
		if (errors) {
			printf("  offset $%04X ($%04X): %u errors\n",
			    offset, target_addr, errors);
			total += errors;
		}
	}
	printf("  %s (%u errors)\n", total ? "FAIL" : "PASS", total);
	return total;
}

/*
 * Test 10 — execute Rd256 from slot 7
 *
 * The real NitrOS-9 boot code runs at $EA60+ (slot 7, page $07).
 * Tests 1-9 all execute from slot 0 ($0130+), which produces a
 * large address bus transition (000→110) between instruction fetch
 * and data write.  The boot code produces a minimal transition
 * (111→110, only bit 13 changes).
 *
 * This test copies the exact Rd256 machine code to $E000 (slot 7)
 * and calls it via JSR, reproducing the boot's bus pattern.
 */
static unsigned
test10_exec_from_slot7(void)
{
	unsigned pass, errors, total;
	uint8_t saved6;

	/*
	 * Rd256 machine code (15 bytes):
	 *   $C6 $80       ldb  #128
	 *   $34 $04       pshs b
	 *   $EC $A4       ldd  ,y
	 *   $ED $81       std  ,x++
	 *   $6A $E4       dec  ,s
	 *   $26 $F8       bne  rd256@
	 *   $32 $61       leas 1,s
	 *   $39           rts
	 */
	static const uint8_t rd256_code[] = {
		0xC6, 0x80,
		0x34, 0x04,
		0xEC, 0xA4,
		0xED, 0x81,
		0x6A, 0xE4,
		0x26, 0xF8,
		0x32, 0x61,
		0x39
	};

	printf("\n[10] Rd256 executed from slot 7 ($E000)\n");
	total = 0;

	/* Copy code to $E000 (page $07, offset $0000) */
	{
		unsigned i;
		uint8_t *dest = (uint8_t *)0xE000;
		for (i = 0; i < sizeof(rd256_code); i++)
			dest[i] = rd256_code[i];
	}

	saved6 = *(DAT_REGS + 6);

	for (pass = 0; pass < 20; pass++) {
		/* Sentinel at $E800 (page $07, offset $0800) */
		fill_sentinel((uint8_t *)0xE800, 256);

		ide_start_identify();

		/*
		 * Call the code at $E000 with X=$C800, Y=$FF00.
		 * Instruction fetches come from slot 7 (page $07).
		 * Data writes go to slot 6 ($C800 = page $06, offset $0800).
		 */
		asm {
			pshs	x,y
			ldx	#$C800
			ldy	#$FF00
			jsr	$E000
			puls	x,y
		}

		ide_drain(128);

		errors = check_sentinel((uint8_t *)0xE800, 256, (bool)(pass < 3));
		if (errors) {
			printf("  pass %u: %u errors\n", pass, errors);
			total += errors;
		}
	}

	*(DAT_REGS + 6) = saved6;
	printf("  %s (20 passes, %u errors)\n", total ? "FAIL" : "PASS", total);
	return total;
}

/*
 * Test 11 — execute Rd256 from slot 7 with non-identity DAT
 *
 * Like test 10, but with slot 6 remapped to page $03 (boot config).
 * This is the exact scenario that fails during NitrOS-9 boot:
 *   - Code fetches from slot 7 (page $07, $E000)
 *   - Data writes to slot 6 ($C800), mapped to page $03
 *   - Check page $07 offset $0800 ($E800) for bleed
 */
static unsigned
test11_slot7_nonidentity(void)
{
	unsigned pass, errors, total;
	uint8_t saved6;

	printf("\n[11] Rd256 from slot 7, slot6=pg$03 (boot scenario)\n");
	total = 0;

	/* Code already at $E000 from test 10 */

	saved6 = *(DAT_REGS + 6);

	for (pass = 0; pass < 20; pass++) {
		fill_sentinel((uint8_t *)0xE800, 256);

		/* Remap slot 6 → page $03 */
		*(DAT_REGS + 6) = 0x03;

		ide_start_identify();

		asm {
			pshs	x,y
			ldx	#$C800
			ldy	#$FF00
			jsr	$E000
			puls	x,y
		}

		ide_drain(128);

		/* Restore slot 6 before checking sentinel */
		*(DAT_REGS + 6) = saved6;

		errors = check_sentinel((uint8_t *)0xE800, 256, (bool)(pass < 3));
		if (errors) {
			printf("  pass %u: %u errors\n", pass, errors);
			total += errors;
		}
	}
	printf("  %s (20 passes, %u errors)\n", total ? "FAIL" : "PASS", total);
	return total;
}

/*
 * Test 12 — task switching between IDE transfers
 *
 * Set up task 1 with the same identity mapping as task 0.
 * Alternate between task 0 and task 1 between each sector read.
 * This exercises the Pico's DAT task register ($FFC0) under load.
 */
static unsigned
test12_task_switch(void)
{
	unsigned pass, errors, total;
	uint8_t *task1_dat;
	unsigned i;

	printf("\n[12] Task switching between IDE transfers\n");
	total = 0;

	/* Set up task 1 DAT regs as identity map (same as task 0) */
	task1_dat = DAT_REGS + 8;  /* task 1 base = $FE08 */
	for (i = 0; i < 8; i++)
		task1_dat[i] = (uint8_t)i;

	for (pass = 0; pass < 20; pass++) {
		fill_sentinel((uint8_t *)0xE800, 256);

		/* Read sector on task 0 */
		*DAT_TASK = 0;
		ide_start_identify();
		asm_rd256((uint16_t *)0xC800);
		ide_drain(128);

		/* Switch to task 1, read sector */
		*DAT_TASK = 1;
		ide_start_identify();
		asm_rd256((uint16_t *)0xC900);
		ide_drain(128);

		/* Switch back to task 0 */
		*DAT_TASK = 0;

		errors = check_sentinel((uint8_t *)0xE800, 256, (bool)(pass < 3));
		if (errors) {
			printf("  pass %u: %u errors\n", pass, errors);
			total += errors;
		}
	}
	printf("  %s (20 passes, %u errors)\n", total ? "FAIL" : "PASS", total);
	return total;
}

/*
 * Test 13 — rapid task switching during Rd256
 *
 * Use inline assembly to switch DAT task mid-transfer.
 * The Rd256 loop runs on task 0 but we flip to task 1 and back
 * between each word read.  Task 1 has slot 6 mapped to a
 * different page — if the Pico latches the wrong task during
 * the store, data lands on the wrong page.
 *
 * Machine code at $E000:
 *   ldb  #128              ; word count
 *   pshs b
 *   clra                   ; A=0 (task 0)
 *   sta  $FFC0             ; select task 0
 *   ldd  ,y                ; read IDE data reg
 *   std  ,x++              ; write to target (via task 0 mapping)
 *   lda  #1                ; A=1 (task 1)
 *   sta  $FFC0             ; briefly select task 1
 *   clra                   ; A=0
 *   sta  $FFC0             ; back to task 0
 *   dec  ,s
 *   bne  loop
 *   leas 1,s
 *   rts
 */
static unsigned
test13_task_switch_mid_transfer(void)
{
	unsigned pass, i, errors, total;
	uint8_t *task1_dat;

	static const uint8_t code[] = {
		0xC6, 0x80,             /* ldb  #128       */
		0x34, 0x04,             /* pshs b          */
		/* loop: */
		0x4F,                   /* clra             */
		0xB7, 0xFF, 0xC0,       /* sta  $FFC0      ; task 0 */
		0xEC, 0xA4,             /* ldd  ,y          */
		0xED, 0x81,             /* std  ,x++        */
		0x86, 0x01,             /* lda  #1          */
		0xB7, 0xFF, 0xC0,       /* sta  $FFC0      ; task 1 */
		0x4F,                   /* clra             */
		0xB7, 0xFF, 0xC0,       /* sta  $FFC0      ; task 0 */
		0x6A, 0xE4,             /* dec  ,s          */
		0x26, 0xEB,             /* bne  loop (-21)  */
		0x32, 0x61,             /* leas 1,s         */
		0x39                    /* rts              */
	};

	printf("\n[13] Task switch mid-transfer (task 0/1 flip per word)\n");
	total = 0;

	/* Copy code to $E000 */
	{
		uint8_t *dest = (uint8_t *)0xE000;
		for (i = 0; i < sizeof(code); i++)
			dest[i] = code[i];
	}

	/* Task 1: identity except slot 6 → page $10 (a distant page) */
	task1_dat = DAT_REGS + 8;
	for (i = 0; i < 8; i++)
		task1_dat[i] = (uint8_t)i;
	task1_dat[6] = 0x10;

	for (pass = 0; pass < 20; pass++) {
		fill_sentinel((uint8_t *)0xE800, 256);

		*DAT_TASK = 0;
		ide_start_identify();

		asm {
			pshs	x,y
			ldx	#$C800
			ldy	#$FF00
			jsr	$E000
			puls	x,y
		}

		*DAT_TASK = 0;
		ide_drain(128);

		errors = check_sentinel((uint8_t *)0xE800, 256, (bool)(pass < 3));
		if (errors) {
			printf("  pass %u: %u errors\n", pass, errors);
			total += errors;
		}
	}
	printf("  %s (20 passes, %u errors)\n", total ? "FAIL" : "PASS", total);
	return total;
}

/*
 * Test 14 — page restructuring between transfers
 *
 * Remap slot 6 to different pages between each sector read,
 * mimicking what would happen if kernel memory management
 * shuffled page assignments.  Verify data arrives at the
 * correct physical page each time.
 */
static unsigned
test14_page_restructure(void)
{
	unsigned pass, errors, total;
	uint8_t pages[] = { 0x03, 0x04, 0x05, 0x08, 0x10, 0x20 };
	uint8_t saved6, pg;
	uint16_t *verify_addr;

	printf("\n[14] Page restructuring: remap slot 6 between transfers\n");
	total = 0;
	saved6 = *(DAT_REGS + 6);

	for (pass = 0; pass < 6; pass++) {
		pg = pages[pass];

		fill_sentinel((uint8_t *)0xE800, 256);

		/* Remap slot 6 → target page */
		*(DAT_REGS + 6) = pg;

		ide_start_identify();
		asm_rd256((uint16_t *)0xC800);
		ide_drain(128);

		/* Restore identity to check sentinel and verify data */
		*(DAT_REGS + 6) = saved6;

		errors = check_sentinel((uint8_t *)0xE800, 256, true);
		if (errors) {
			printf("  pg $%02X: %u sentinel errors at $E800\n",
			    pg, errors);
			total += errors;
		}

		/* Verify data actually landed on the right page by reading
		 * through a slot temporarily mapped to that page.
		 * Use slot 3 (normally page $03) as a window.
		 */
		{
			uint8_t saved3 = *(DAT_REGS + 3);
			unsigned nonzero = 0, j;
			*(DAT_REGS + 3) = pg;
			verify_addr = (uint16_t *)0x6800; /* slot 3, offset $0800 */
			for (j = 0; j < 128; j++) {
				if (verify_addr[j] != 0)
					nonzero++;
			}
			*(DAT_REGS + 3) = saved3;
			if (nonzero == 0)
				printf("  pg $%02X: WARNING - no data arrived\n", pg);
			else
				printf("  pg $%02X: OK (%u/128 words nonzero)\n",
				    pg, nonzero);
		}
	}
	printf("  %s (%u errors)\n", total ? "FAIL" : "PASS", total);
	return total;
}

/*
 * Test 15 — task switch + page restructure + slot 7 execution
 *
 * The grand combination: execute Rd256 from slot 7 ($E000),
 * writing to slot 6 which is remapped to page $03,
 * while switching DAT task between sectors.
 * This combines all the conditions present during NitrOS-9 boot.
 */
static unsigned
test15_combined(void)
{
	unsigned pass, i, errors, total;
	uint8_t *task1_dat, saved6;

	printf("\n[15] Combined: slot7 exec + task switch + page remap\n");
	total = 0;

	/* Rd256 code should still be at $E000 from test 10 */

	/* Set up task 1: identity except slot 6 → page $03 */
	task1_dat = DAT_REGS + 8;
	for (i = 0; i < 8; i++)
		task1_dat[i] = (uint8_t)i;
	task1_dat[6] = 0x03;

	saved6 = *(DAT_REGS + 6);

	for (pass = 0; pass < 100; pass++) {
		fill_sentinel((uint8_t *)0xE800, 256);

		/* Remap slot 6 → page $03 on task 0 */
		*(DAT_REGS + 6) = 0x03;

		/* Briefly switch to task 1 and back (like SWI dispatch) */
		*DAT_TASK = 1;
		*DAT_TASK = 0;

		ide_start_identify();

		asm {
			pshs	x,y
			ldx	#$C800
			ldy	#$FF00
			jsr	$E000
			puls	x,y
		}

		ide_drain(128);

		/* Restore identity */
		*(DAT_REGS + 6) = saved6;

		errors = check_sentinel((uint8_t *)0xE800, 256, (bool)(pass < 3));
		if (errors) {
			printf("  pass %u: %u errors\n", pass, errors);
			total += errors;
		}
	}
	printf("  %s (100 passes, %u errors)\n", total ? "FAIL" : "PASS", total);
	return total;
}

/*
 * Test 16 — high-volume stress (1000 transfers with task/page churn)
 *
 * 1000 IDE transfers with random-ish task switching and page
 * remapping between each one.  This is the best chance to catch
 * an intermittent timing-dependent bug.
 */
static unsigned
test16_stress(void)
{
	unsigned pass, i, errors, total;
	uint8_t *task_dat, saved6;
	uint8_t pg;

	printf("\n[16] High-volume stress (1000 transfers, task/page churn)\n");
	total = 0;

	saved6 = *(DAT_REGS + 6);

	/* Set up tasks 1-7 with varying slot 6 mappings */
	for (i = 1; i < 8; i++) {
		task_dat = DAT_REGS + i * 8;
		task_dat[0] = 0;
		task_dat[1] = 1;
		task_dat[2] = 2;
		task_dat[3] = 3;
		task_dat[4] = 4;
		task_dat[5] = 5;
		task_dat[6] = (uint8_t)(0x08 + i);  /* different page per task */
		task_dat[7] = 7;
	}

	for (pass = 0; pass < 1000; pass++) {
		/* Rotate through safe pages (skip 0,1=program, 7=sentinel/stack) */
		{
			static const uint8_t safe_pages[] = {
				0x03, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x10, 0x20
			};
			pg = safe_pages[pass & 0x07];
		}

		fill_sentinel((uint8_t *)0xE800, 256);

		/* Remap slot 6 on task 0 */
		*(DAT_REGS + 6) = pg;

		/* Switch through a couple of tasks and back (tasks 1-7 only) */
		*DAT_TASK = (uint8_t)((pass % 7) + 1);
		*DAT_TASK = (uint8_t)((pass % 3) + 1);
		*DAT_TASK = 0;

		ide_start_identify();
		asm_rd256((uint16_t *)0xC800);
		ide_drain(128);

		/* Restore identity */
		*(DAT_REGS + 6) = saved6;

		errors = check_sentinel((uint8_t *)0xE800, 256, false);
		if (errors) {
			if (total < 8)
				printf("  pass %u (pg $%02X): %u errors\n",
				    pass, pg, errors);
			total += errors;
		}
	}
	printf("  %s (1000 passes, %u errors)\n", total ? "FAIL" : "PASS", total);
	return total;
}

/* ---------- main ---------- */

int
main(void)
{
	ConsoleOutHook oldCHROOT;
	unsigned total;

	initialiseFastSerial();
	oldCHROOT = setConsoleOutHook(newOutputRoutine);

	printf("\n=== DAT RAM Torture Test ===\n");

	/* Ensure task 0 and show initial DAT state */
	*(DAT_TASK) = 0;
	print_dat();

	total = 0;
	total += test1_pure_write();
	total += test2_ide_c_loop();
	total += test3_asm_rd256();
	total += test4_all_slots();
	total += test5_ram_source();
	total += test6_nonidentity();
	total += test7_boot_sim();
	total += test8_dat_integrity();
	total += test9_offset_sweep();
	total += test10_exec_from_slot7();
	total += test11_slot7_nonidentity();
	total += test12_task_switch();
	total += test13_task_switch_mid_transfer();
	total += test14_page_restructure();
	total += test15_combined();
	total += test16_stress();

	printf("\n=============================\n");
	printf("TOTAL ERRORS: %u\n", total);
	if (total == 0)
		printf("ALL TESTS PASSED\n");
	else
		printf("*** FAILURES DETECTED ***\n");
	printf("=============================\n");

	setConsoleOutHook(oldCHROOT);
	SYNC;
	return 0;
}
