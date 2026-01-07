#include <cmoc.h>
#include "pico.h"

struct sysvectors *sys_vector = (struct sysvectors *)0xFD00;

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

uint8_t
inb(uint8_t *addr)
{
	uint8_t ret = *addr;
	//printf(" inb: %04X %02X\n", addr, ret);
	return ret;
}

void
insw(uint8_t *addr, void *dest, uint16_t count)
{
	asm {
		pshs	d,x,y,u
		ldx	:addr
		ldy	:dest
		ldd	:count
i@0		ldu	,x
		stu	,y++
		subd	#1
		bne	i@0
		puls	d,x,y,u
	}
}

void
outb(uint8_t *addr, uint8_t val)
{
	*addr = val;
	//printf("outb: %04X %02X\n", addr, val);
}

void
outsw(uint8_t *addr, const void *src, uint16_t count)
{
	asm {
		pshs	d,x,y,u
		ldx	:addr
		ldy	:src
		ldd	:count
o@0		ldu	,y++
		stu	,x
		subd	#1
		bne	o@0
		puls	d,x,y,u
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
		pshs	d
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
m@99		puls	d
	}
}
