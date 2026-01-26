#include <cmoc.h>
#include "pico.h"
#include "ide_lib.h"

void 
dump_buffer(uint16_t *buffer)
{
	uint8_t *ch;
	char print[64], *q;
	unsigned i, j, k;
	uint16_t *p;

	p = buffer;
	for (i = 0; i < 256; i += 8) {
		q = print;
		printf("%3u:", i);
		for (j = 0; j < 8; ++j) {
			ch = (uint8_t *)p;
			for (k = 0; k < 2; k++) {
				if (*ch >= (uint8_t)0x20 && *ch < (uint8_t)0x7F)
					*q++ = *ch;
				else
					*q++ = '.';
				ch++;
			}
			printf(" %04X", *p++);
		}
		print[16] = '\0';
		printf(" |%s|\n", print);
	}
}

int
main(void)
{
	unsigned i, j, k;
	int ide;
	ConsoleOutHook oldCHROOT;
	char print[128];
	uint8_t *r;
	uint16_t *p, buffer[256];
	uint32_t LBA_sectors, capacity;

	initialiseFastSerial();
	oldCHROOT = setConsoleOutHook(newOutputRoutine);
	// ==========================================================================}
	r = (uint8_t *)0xFF00;
	for (i = 0; i < 256; ++i) {
		printf("%04X: ", (unsigned)r);
		for (j = 0; j < 16; ++j)
			printf(" %02X", *r++);
		printf("\n");
		if (r >= (uint8_t *)0xFFC0)
			break;
	}
	// ==========================================================================}
	memset(buffer, 0, 512);
	ide = ide_identify(buffer);
	if (ide == 0)
		printf("Yay!\n");
	dump_buffer(buffer);
	printf("Cylinders: %u\n", buffer[1]);
	printf("Heads: %u\n", buffer[3]);
	printf("Sectors/Track: %u\n", buffer[5]);
	printf("LBA bit in: %04X - the LBA is %u\n", buffer[49], (buffer[49] & (1 << 9)) != 0);
	LBA_sectors = ((uint32_t)buffer[61] << 16) | (uint32_t)buffer[60];
	capacity = (LBA_sectors*2)/(1024*1024);
	printf("LBA sectors: %lu = %lu MB\n", LBA_sectors, capacity);
	strncpy(print, (char *)&buffer[10], 20);
	print[20] = '\0';
	printf("Serial number: '%s'\n", print);
	strncpy(print, (char *)&buffer[23], 8);
	print[8] = '\0';
	printf("Firmware revision: '%s'\n", print);
	strncpy(print, (char *)&buffer[27], 40);
	print[40] = '\0';
	printf("Model number: '%s'\n", print);
	// ==========================================================================}
	//memset(buffer, 0xFF, 512);
	for (i = 0; i < 512; i++)
		buffer[i] = (uint16_t)i + (uint16_t)129;
	dump_buffer(buffer);
	ide = ide_write_sector(0, buffer);
	if (ide == 0)
		printf("Yay?\n");
	memset(buffer, 0, 512);
	ide = ide_read_sector(0, buffer);
	if (ide == 0)
		printf("Yay again!!\n");
	dump_buffer(buffer);
	memset(buffer, 0, 512);
	ide = ide_read_sector(0, buffer);
	if (ide == 0)
		printf("Yay again again!!\n");
	dump_buffer(buffer);
	// ==========================================================================}
	for (i = 0; i < 2; i++) {
		timer_msleep(1000u);
		printf("%d\n", i);
	}
	// ==========================================================================}
	setConsoleOutHook(oldCHROOT);
	SYNC;
	return 0;
}
