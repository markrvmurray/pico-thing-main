#include <cmoc.h>
#include "pico.h"

int
main(void)
{
	float a, b, c;
	unsigned i, j;
	ConsoleOutHook oldCHROOT;
	uint8_t *r;

	initialiseFastSerial();
	oldCHROOT = setConsoleOutHook(newOutputRoutine);
	enable_printf_float();
	printf("\n");
	printf("Hello, World!\n");
	// ==========================================================================}
	a = 12.234567f;
	b = -17.4447f;
	c = a/b;
	printf("%f %f %f\n", a, b, c);
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
	SYNC;
	return 0;
}
