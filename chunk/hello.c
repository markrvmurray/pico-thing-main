#include <cmoc.h>

void
exit(int v)
{
	asm {
		sync
	}
}

void
initialiseFastSerial(void)
{
	asm {
		lda	#$08		; complete uart reset
		sta	$FFC8
R@0		lda	$FFC8
		bita	#$08
		bne	R@0
	}
}

void
newOutputRoutine(void)
{
	asm {
		pshs	b
		cmpa	#$0a
		bne	O@0
		lda	#$0d
		bsr	O@1
		lda	#$0a
O@0		bsr	O@1
		bra	O@2
O@1		ldb	$FFC8
		bitb	#%00000001
		beq	O@1		; branch if transmitter not empty
		sta	$FFC9
		rts
O@2		puls	b
	}
}

int
main(void)
{
	float a, b, c;
	unsigned i, j;
	unsigned char *p;
	ConsoleOutHook oldCHROOT;

	initialiseFastSerial();
	oldCHROOT = setConsoleOutHook(newOutputRoutine);
	enable_printf_float();
	printf("Hello, World!\n");
	a = 12.234567f;
	b = -17.4447f;
	c = a/b;
	printf("%f %f %f\n", a, b, c);
	p = (unsigned char *)0xFF00;
	for (i = 0; i < 256; ++i) {
		printf("%04X", (unsigned)p);
		for (j = 0; j < 16; ++j)
			printf(" %02X", *p++);
		printf("\n");
		if (p >= (unsigned char *)0xFFC0)
			break;
	}

	setConsoleOutHook(oldCHROOT);
	exit(0);
	return 0;
}
