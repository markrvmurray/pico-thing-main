#include <cmoc.h>
#include "pico.h"

struct sysvectors *sys_vector = (struct sysvectors *)0xFD00;

void
initialiseFastSerial(void)
{
	asm {
		EXTERN	INIT_CON
		EXTERN	INIT_AUX
		pshs	u
		lbsr	INIT_CON	; Console ACIA reset + install IRQ ISR + enable IRQ
		lbsr	INIT_AUX	; Auxiliary ACIA reset + enable IRQ
		puls	u
	}
}

void
newOutputRoutine(void)
{
	asm {
		EXTERN	PUTCHAR
		EXTERN	CON_PORT
		pshs	u
		ldu	#CON_PORT
		lbsr	PUTCHAR		; LF->CR+LF conversion, then interrupt-driven OUTCH
		puls	u
	}
}

uint8_t
inb(uint8_t *addr)
{
	uint8_t ret = *addr;
	return ret;
}

void
insw(uint8_t *addr, void *dest, uint16_t count)
{
	asm {
		ldx	:addr
		ldy	:dest
		ldd	:count
i@0		ldu	,x
		stu	,y++
		subd	#1
		bne	i@0
	}
}

void
outb(uint8_t *addr, uint8_t val)
{
	*addr = val;
}

void
outsw(uint8_t *addr, const void *src, uint16_t count)
{
	asm {
		ldx	:addr
		ldy	:src
		ldd	:count
o@0		ldu	,y++
		stu	,x
		subd	#1
		bne	o@0
	}
}

void timer_usleep_10(void)
{
	asm {
		pshs	a,b
		puls	a,b
	}
}

void timer_msleep(uint16_t milliseconds)
{
	asm {
		ldd	:milliseconds
m@0		bsr	d@0
		subd	#1
		bne	m@0
		bra	m@99
d@0		lbsr	d@1
d@1		lbsr	d@2
d@2		lbsr	d@3
d@3		lbsr	d@4
d@4		lbsr	d@5
d@5		lbsr	d@6
d@6		nop
		rts
m@99
	}
}
