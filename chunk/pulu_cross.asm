; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; pulu_cross - one PULU that reads across the DAT-mapped/fixed boundary
;
; With U = $DFFC, PULU D,X,Y reads $DFFC-$DFFF through the DAT (task 0
; slot 6) and $E000-$E001 from the fixed window, in consecutive cycles, then
; the throwaway read of $E002.  Each canary word holds its own address, so a
; good pull is D=$DFFC X=$DFFE Y=$E000.  Anything else is complained about
; loudly on the console ACIA, with the words pulled, what memory holds when
; read again without crossing, and the pass count.
;
; Before the loop every mappable page ($00-$FE) is tagged through slot 1:
; offsets $0000 and $1FFC-$1FFF hold the page number then its complement.
; So a wrong word of the form pp,~pp names the page it really came from.
;
; The first thing a failure does is write $FFCB, which freezes an armed
; trace ring ("trace ring" first) and, on a PICO_THING_TRAP_TRIGGER build,
; pulses GPIO 39 for the scope a dozen cycles after the bad pull.
; Interrupts are masked throughout; code, stack and results are in the fixed
; window.
;
; Load with sp, start with go e100.  A dot every 65536 passes.  Before go,
; mo e200 sets the page mapped into slot 6 (default $01, the page the
; NitrOS-9 crash pulled from) and mo e201 01 keeps going after a failure
; instead of stopping.  Results with ex e200 e218:
;   E200 page  E201 keep-going  E202 passes (32-bit)  E206 failures
;   E208 D,X,Y pulled  E20E D,X,Y in memory  E214 passes at the failure

	INCLUDE	syslib.inc

Start	EXPORT

DATT0	EQU	$FE00		; task 0's eight slot entries
TRACE	EQU	$FFCB		; any write freezes an armed trace ring
CANARY	EQU	$DFFC		; four bytes above the fixed window, two in it

	SECTION	FIXED
Start	LDS	#$F000
	ORCC	#$50		; IRQ and FIRQ masked for the whole run
	LDA	#%00000011
	STA	ACIAC		; master reset, polled transmit from here on
	LDD	#0
	STD	passes
	STD	passes+2
	STD	fails
	LDB	DATT0+1
	PSHS	B		; slot 1's page, to put back
	CLRA			; tag pages $00-$FE through slot 1
tag	STA	DATT0+1
	TFR	A,B
	COMB			; D = page, ~page
	STD	$2000		; offset $0000
	STD	$3FFC		; offsets $1FFC-$1FFF
	STD	$3FFE
	INCA
	CMPA	#$FF		; $FF is the fixed window, not mappable
	BNE	tag
	PULS	B
	STB	DATT0+1
	LDA	page
	STA	DATT0+6		; the page under test into slot 6
	LDD	#CANARY		; each canary word is its own address
	STD	CANARY
	LDD	#CANARY+2
	STD	CANARY+2
	LDD	#CANARY+4
	STD	CANARY+4
	LDX	#m_start
	JSR	puts
	LDA	page
	JSR	hex8
	LDX	#m_crlf
	JSR	puts
loop	LDU	#CANARY
	PULU	D,X,Y		; $DFFC-$DFFF via the DAT, $E000-$E001 fixed
	CMPD	#CANARY
	LBNE	fail
	CMPX	#CANARY+2
	LBNE	fail
	CMPY	#CANARY+4
	LBNE	fail
	INC	passes+3
	BNE	loop
	INC	passes+2
	BNE	loop
	LDA	#'.
	JSR	putc		; heartbeat every 65536 passes
	INC	passes+1
	BNE	loop
	INC	passes
	BRA	loop

	ENDSECTION

* The failure path and printing, out of the way of the results at $E200.
	SECTION	FIXEDISR
fail	STA	TRACE		; first: freeze the ring, pulse the scope
	STD	got
	STX	got+2
	STY	got+4
	LDD	CANARY		; memory again, no read crossing the boundary
	STD	now
	LDD	CANARY+2
	STD	now+2
	LDD	CANARY+4
	STD	now+4
	LDD	passes
	STD	atpass
	LDD	passes+2
	STD	atpass+2
	INC	fails+1
	BNE	f1
	INC	fails
f1	LDX	#m_fail
	JSR	puts
	LDY	#got
	JSR	hex3
	LDX	#m_want
	JSR	puts
	LDX	#m_mem
	JSR	puts
	LDY	#now
	JSR	hex3
	LDX	#m_pass
	JSR	puts
	LDD	atpass
	JSR	hex16
	LDD	atpass+2
	JSR	hex16
	LDX	#m_crlf
	JSR	puts
	TST	cont
	LBNE	loop
halt	BRA	halt		; evidence stays put

* Three words at Y in hex, space separated.
hex3	LDD	,Y
	BSR	hex16
	LDA	#$20		; space
	BSR	putc
	LDD	2,Y
	BSR	hex16
	LDA	#$20		; space
	BSR	putc
	LDD	4,Y
	BRA	hex16

* D as four hex digits.
hex16	PSHS	B
	BSR	hex8
	PULS	A
* A as two hex digits.
hex8	PSHS	A
	LSRA
	LSRA
	LSRA
	LSRA
	BSR	nib
	PULS	A
nib	ANDA	#$0F
	ADDA	#'0
	CMPA	#'9
	BLS	putc
	ADDA	#'A-'9-1
* A to the console ACIA, polled.
putc	PSHS	B
p1	LDB	ACIAS
	BITB	#%00000010	; transmit data register empty
	BEQ	p1
	STA	ACIATX
	PULS	B,PC

* The NUL-terminated string at X.
puts	LDA	,X+
	BEQ	p2
	BSR	putc
	BRA	puts
p2	RTS

m_start	FCB	13,10
	FCC	"pulu_cross: PULU D,X,Y from $DFFC, slot 6 = page $"
	FCB	0
m_fail	FCB	13,10
	FCC	"*** PULU ACROSS $DFFF/$E000 READ WRONG ***"
	FCB	13,10
	FCC	"pulled "
	FCB	0
m_want	FCB	13,10
	FCC	"wanted DFFC DFFE E000"
	FCB	0
m_mem	FCB	13,10
	FCC	"memory "
	FCB	0
m_pass	FCB	13,10
	FCC	"pass   "
	FCB	0
m_crlf	FCB	13,10,0
	ENDSECTION

	SECTION	FIXEDDATA
page	FCB	$00
cont	FCB	0
passes	FDB	0,0
fails	FDB	0
got	FDB	0,0,0
now	FDB	0,0,0
atpass	FDB	0,0
	ENDSECTION
