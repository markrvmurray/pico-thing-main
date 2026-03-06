;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; putdecd.asm — Print D as 1-5 decimal digits (no leading zeros).
; Preserves all registers and CC. Output via OUTCH.
;

	USE	syslib.inc

OUTCH	EXTERN

PUTDECD	EXPORT

	SECTION	TEXT

; -----------------------------------------------------------------------
; PUTDECD — print D as 1-5 decimal digits (no leading zeros).
; Preserves all registers and CC.
;
; Algorithm: for each power of 10 (10000, 1000, 100, 10), count how
; many times it divides into the value. Leading zeros are suppressed.
; The units digit is always printed.
;
; Stack frame during PD_SUB loop:
;   [S]   = digit counter (1-based)
;   [S+1] = power MSB
;   [S+2] = power LSB
;   [S+3] = "started printing" flag
; -----------------------------------------------------------------------
PUTDECD
	PSHS	CC,A,B,X,U
	TFR	D,X		; X = value to print
	LDU	#DECTAB		; U -> powers-of-10 table
	CLR	,-S		; push "started" flag (0 = suppress zeros)
PD_LP
	LDD	,U++		; D = next power of 10
	BEQ	PD_UNIT		; 0 sentinel -> print units digit
	PSHS	D		; push power onto stack
	TFR	X,D		; D = current value
	CLR	,-S		; push digit counter (0)
PD_SUB
	INC	,S		; digit++ (INC does not affect C)
	SUBD	1,S		; D -= power
	BCC	PD_SUB		; no borrow -> keep counting
	ADDD	1,S		; undo last subtraction
	TFR	D,X		; X = remainder
	LDA	,S		; A = digit count (1-based)
	LEAS	3,S		; drop digit counter + power
	DECA			; convert 1-based to 0-based
	BNE	PD_PRT		; non-zero digit -> print it
	TST	,S		; check started flag
	BEQ	PD_LP		; leading zero -> suppress
PD_PRT
	INC	,S		; mark: we have started printing
	ADDA	#'0'		; convert digit to ASCII
	LBSR	OUTCH
	BRA	PD_LP
PD_UNIT
	TFR	X,D		; D = remaining value (0-9 in B)
	ADDB	#'0'		; convert to ASCII
	TFR	B,A
	LBSR	OUTCH		; always print units digit
	LEAS	1,S		; drop started flag
	PULS	CC,A,B,X,U,PC

	ENDSECTION

; -----------------------------------------------------------------------
; Constant data
; -----------------------------------------------------------------------
	SECTION	CONSTANT

DECTAB	FDB	10000,1000,100,10,0

	ENDSECTION

	END
