#include <cmoc.h>
#include "pico.h"
#include "ide_lib.h"

/*
 * Prove the PATA write path on the drive's last sector, which is found from
 * IDENTIFY rather than assumed.  The sector is read and kept, a pattern is
 * written and read back, and the original is then written back and read
 * back in turn.  Nothing is written until the original is safely held, and
 * once the pattern may be on the disk the original goes back whatever the
 * pattern check found.
 */

#define BLOCK_WORD_COUNT	(256u)

#define IDENTIFY_CAPABILITIES	(49u)		/* bit 9 set: LBA supported */
#define IDENTIFY_LBA_SUPPORTED	(1u << 9)
#define IDENTIFY_LBA28_LOW	(60u)		/* words 60-61: LBA28 user sectors */
#define IDENTIFY_LBA28_HIGH	(61u)

/* In BSS rather than on the stack: four sectors is 2kB. */
static uint16_t identify[BLOCK_WORD_COUNT];
static uint16_t saved[BLOCK_WORD_COUNT];
static uint16_t pattern[BLOCK_WORD_COUNT];
static uint16_t check[BLOCK_WORD_COUNT];

/* Compare two sectors, reporting the first word that differs. */
static bool
same(const char *what, const uint16_t *expected, const uint16_t *got)
{
	unsigned i;

	for (i = 0; i < BLOCK_WORD_COUNT; i++)
		if (expected[i] != got[i])
			break;
	if (i == BLOCK_WORD_COUNT) {
		printf("%s: match\n", what);
		return true;
	}
	printf("%s: MISMATCH at word %u - expected %04X, read %04X\n",
	       what, i, expected[i], got[i]);
	return false;
}

static bool
write_test(uint32_t lba)
{
	unsigned i;
	int err;
	bool ok;

	if (ide_read_sector(lba, saved) != 0) {
		printf("Reading the original failed - nothing written\n");
		return false;
	}
	printf("Original read and kept\n");

	/* The high byte counts up and the low byte down, so every data line
	 * changes across the sector and a byte swap cannot pass. */
	for (i = 0; i < BLOCK_WORD_COUNT; i++)
		pattern[i] = (uint16_t)((i << 8) | (~i & 0xFFu));

	ok = true;
	err = ide_write_sector(lba, pattern);
	if (err == IDE_ERR_ADDRESS) {
		/* The drive did not latch this sector, so no command went out
		 * and the original was never touched. */
		printf("Address did not read back - nothing written\n");
		return false;
	}
	if (err != 0) {
		printf("Writing the pattern failed\n");
		ok = false;
	} else if (ide_read_sector(lba, check) != 0) {
		printf("Reading the pattern back failed\n");
		ok = false;
	} else if (!same("Pattern", pattern, check)) {
		ok = false;
	}

	if (ide_write_sector(lba, saved) != 0) {
		printf("RESTORING THE ORIGINAL FAILED - LBA %lu may hold the pattern\n", lba);
		return false;
	}
	if (ide_read_sector(lba, check) != 0) {
		printf("Reading back the restored original failed\n");
		return false;
	}
	if (!same("Restored original", saved, check))
		return false;

	return ok;
}

int
main(void)
{
	ConsoleOutHook oldCHROOT;
	uint32_t sectors, lba;
	bool ok;

	initialiseFastSerial();
	oldCHROOT = setConsoleOutHook(newOutputRoutine);

	ok = false;
	if (ide_identify(identify) != 0) {
		printf("IDENTIFY failed - nothing written\n");
	} else if ((identify[IDENTIFY_CAPABILITIES] & IDENTIFY_LBA_SUPPORTED) == 0u) {
		printf("Drive does not support LBA - nothing written\n");
	} else {
		sectors = ((uint32_t)identify[IDENTIFY_LBA28_HIGH]) * 0x10000ul
			+ (uint32_t)identify[IDENTIFY_LBA28_LOW];
		if (sectors == 0ul) {
			printf("Drive reports no LBA sectors - nothing written\n");
		} else {
			lba = sectors - 1ul;
			printf("LBA sectors: %lu - testing the last, LBA %lu ($%08lX)\n",
			       sectors, lba, lba);
			ok = write_test(lba);
		}
	}
	printf("Disk write test: %s\n", ok ? "PASS" : "FAIL");

	setConsoleOutHook(oldCHROOT);
	SYNC;
	return 0;
}
