;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; putdec.asm — Print B or D as decimal digits (no leading zeros).
; Preserves all registers and CC. Output via OUTCH (U = port).
;

	USE	syslib.inc

OUTCH	EXTERN

PUTDECB	EXPORT
PUTDECD	EXPORT

	SECTION	TEXT

; -----------------------------------------------------------------------
; PUTDECB — print B as 1-3 decimal digits (no leading zeros).
; Preserves all registers and CC.
; Zero-extends B into D and falls through to PUTDECD.
; -----------------------------------------------------------------------
PUTDECB
	PSHS	CC,A
	CLRA			; D = 0:B (zero-extend B to 16 bits)
	BSR	PUTDECD
	PULS	CC,A,PC

; -----------------------------------------------------------------------
; PUTDECD — print D as 1-5 decimal digits (no leading zeros).
; Preserves all registers and CC.
;
; Algorithm: double dabble (shift-and-add-3).
;
; A 5-byte shift register is built on the stack:
;
;   [S+0]  0000 TTTT   ten-thousands (BCD, low nibble only)
;   [S+1]  tttt hhhh   thousands | hundreds (BCD)
;   [S+2]  TTTT uuuu   tens | units (BCD)
;   [S+3]  MMMM MMMM   binary MSB
;   [S+4]  LLLL LLLL   binary LSB
;
; The binary value is shifted left one bit at a time (16 iterations).
; Before each shift, every BCD nibble >= 5 gets +3.  After 16 shifts
; the BCD bytes hold the decimal result.
;
; Y indexes the BCD buffer during printing so that U remains free
; as the port descriptor for OUTCH.
; -----------------------------------------------------------------------
PUTDECD
	PSHS	CC,A,B,X,Y
	; Build shift register: 3 BCD bytes (zeroed) + 2 binary bytes (D)
	PSHS	D		; [S+3..S+4] = binary value
	CLR	,-S		; [S+2] = tens | units
	CLR	,-S		; [S+1] = thousands | hundreds
	CLR	,-S		; [S+0] = ten-thousands

	LDX	#16		; shift counter

DD_LP
	; --- Add-3 pass: for each BCD nibble >= 5, add 3 ---

	; [S+0] low nibble (ten-thousands)
	LDA	,S
	TFR	A,B
	ANDB	#$0F
	CMPB	#5
	BLO	DD@1
	ADDA	#$03
DD@1	STA	,S

	; [S+1] low nibble (hundreds)
	LDA	1,S
	TFR	A,B
	ANDB	#$0F
	CMPB	#5
	BLO	DD@2
	ADDA	#$03
DD@2	; [S+1] high nibble (thousands)
	TFR	A,B
	ANDB	#$F0
	CMPB	#$50
	BLO	DD@3
	ADDA	#$30
DD@3	STA	1,S

	; [S+2] low nibble (units)
	LDA	2,S
	TFR	A,B
	ANDB	#$0F
	CMPB	#5
	BLO	DD@4
	ADDA	#$03
DD@4	; [S+2] high nibble (tens)
	TFR	A,B
	ANDB	#$F0
	CMPB	#$50
	BLO	DD@5
	ADDA	#$30
DD@5	STA	2,S

	; --- Shift entire 5 bytes left by 1 bit ---
	ASL	4,S
	ROL	3,S
	ROL	2,S
	ROL	1,S
	ROL	,S

	LEAX	-1,X
	BNE	DD_LP

	; --- Print BCD digits, suppress leading zeros ---
	LEAY	,S		; Y -> BCD buffer (stable across nested calls)
	CLRB			; B = started flag (0 = suppress leading zeros)

	LDA	,Y		; ten-thousands (low nibble of byte 0)
	ANDA	#$0F
	BSR	PD_DIG

	LDA	1,Y		; thousands (high nibble of byte 1)
	LSRA
	LSRA
	LSRA
	LSRA
	BSR	PD_DIG

	LDA	1,Y		; hundreds (low nibble of byte 1)
	ANDA	#$0F
	BSR	PD_DIG

	LDA	2,Y		; tens (high nibble of byte 2)
	LSRA
	LSRA
	LSRA
	LSRA
	BSR	PD_DIG

	LDA	2,Y		; units (low nibble of byte 2) — always printed
	ANDA	#$0F
	ADDA	#'0'
	LBSR	OUTCH

	LEAS	5,S		; drop shift register
	PULS	CC,A,B,X,Y,PC

; -----------------------------------------------------------------------
; PD_DIG — print one BCD digit with leading-zero suppression.
; In:  A = digit value (0-9), B = started flag.
; Out: B = 1 if a digit was printed, unchanged otherwise.
; -----------------------------------------------------------------------
PD_DIG
	TSTA
	BNE	PD@P		; non-zero digit — always print
	TSTB
	BEQ	PD@S		; zero and not started — suppress
PD@P	ADDA	#'0'
	LBSR	OUTCH
	LDB	#1		; mark: started
PD@S	RTS

	ENDSECTION

	END
