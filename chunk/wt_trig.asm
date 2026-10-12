; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; wt_trig - write/verify torture with a trap on every miss
;
; Fills $2000-$3FFF with 16-bit stores of a changing sequence and verifies
; it, then pushes 64 words into block 2 and pulls them back.  On a miss it
; records the slot, the expected and actual words, prints S or P on the
; console ACIA, and executes an illegal opcode so the firmware's trap
; trigger (GPIO 39 on a PICO_THING_TRAP_TRIGGER build) pulses for the
; scope.  Interrupts are masked throughout.  Code and results live in the
; fixed window, so every fetch is a fixed-window read between DAT-range
; data accesses, which is the pattern the HD63C09E was seen to slip on.
;
; Load with sp, start with go e100, read results with ex e200 e214 and the
; vector counts with ex e220 e22e:
;   E200 passes  E202 S count  E204 P count  E206 X after the bad word
;   E208 S expected  E20A P pulled  E20C S actual  E20E P expected
;   E210 scratch  E212 P actual  E220.. vector counts (illegal first)
; The illegal-opcode vector is pointed at a counting handler; the other
; vectors keep syslib's defaults.

	INCLUDE	syslib.inc

Start	EXPORT

	SECTION	FIXED
Start	LDS	#$F000
	ORCC	#$50		; IRQ and FIRQ masked for the whole run
	LDX	#illegal
	STX	RSRVD+1		; illegal-opcode jump in the $FD00 table
loop	LDX	#$2000
	LDD	#$55AA
l1	STD	,X++		; fill with the sequence
	ADDD	#$1357
	EORA	#$A5
	CMPX	#$4000
	BNE	l1
	LDX	#$2000
	LDD	#$55AA
l2	CMPD	,X++		; verify it
	BNE	e1
	ADDD	#$1357
	EORA	#$A5
	CMPX	#$4000
	BNE	l2
	LDS	#$5FF0		; stack burst in block 2
	LDD	#$C3A5
	LDY	#64
l3	PSHS	D
	ADDD	#$1357
	EORA	#$A5
	LEAY	-1,Y
	BNE	l3
	LDY	#64
l4	EORA	#$A5		; unwind one step, pull, compare
	SUBD	#$1357
	PULS	X
	STD	scratch
	CMPX	scratch
	BNE	e2
	LEAY	-1,Y
	BNE	l4
	LDS	#$F000
	LDX	passes
	LEAX	1,X
	STX	passes
	LBRA	loop
e1	STX	s_xafter	; STD miss: X after the bad word
	PSHS	D
	LDA	#'S
	STA	ACIATX		; marker on the console ACIA
	PULS	D
	STD	s_expect
	LDD	-2,X
	STD	s_actual	; what RAM holds
	LDD	s_count
	ADDD	#1
	STD	s_count
	FCB	$15		; illegal opcode: trap -> trigger pulse
	LDS	#$F000
	LBRA	loop
e2	STX	p_pulled	; PSHS miss: the pulled word
	PSHS	D
	LDA	#'P
	STA	ACIATX
	PULS	D
	STD	p_expect
	LDD	-2,S
	STD	p_actual	; what RAM holds
	LDD	p_count
	ADDD	#1
	STD	p_count
	FCB	$15
	LDS	#$F000
	LBRA	loop
	ENDSECTION

	SECTION	FIXEDDATA
passes	FDB	0
s_count	FDB	0
p_count	FDB	0
s_xafter	FDB	0
s_expect	FDB	0
p_pulled	FDB	0
s_actual	FDB	0
p_expect	FDB	0
scratch	FDB	0
p_actual	FDB	0
	FDB	0,0,0,0,0,0	; pad to $E220
c_illeg	FDB	0
	ENDSECTION

	SECTION	FIXEDISR
illegal	INC	c_illeg+1		; 16-bit count, no register touched
	BNE	i1
	INC	c_illeg
i1	RTI
	ENDSECTION
