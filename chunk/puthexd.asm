;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; puthexd.asm — Print D (A:B) as four hex digits.
; Preserves all registers and CC. Output via OUTCH.
;

	USE	syslib.inc

OUTCH	EXTERN

PUTHEXD	EXPORT

	SECTION	TEXT

; -----------------------------------------------------------------------
; PUTHEXNIB — print low nibble of A as one hex digit.
; Internal helper. Clobbers A.
; -----------------------------------------------------------------------
PUTHEXNIB
	ANDA	#$0F
	CMPA	#10
	BLO	PHXN0
	ADDA	#('A'-10)
	BRA	PHXN1
PHXN0	ADDA	#'0'
PHXN1	LBSR	OUTCH
	RTS

; -----------------------------------------------------------------------
; PUTHEX8 — print A as two hex digits.
; Internal helper. Clobbers A.
; -----------------------------------------------------------------------
PUTHEX8
	PSHS	A
	LSRA
	LSRA
	LSRA
	LSRA
	BSR	PUTHEXNIB
	PULS	A
	ANDA	#$0F
	BRA	PUTHEXNIB	; tail call

; -----------------------------------------------------------------------
; PUTHEXD — print D (A:B) as four hex digits.
; Preserves all registers and CC.
; After PSHS CC,A,B: [S]=CC, [S+1]=A, [S+2]=B.
; -----------------------------------------------------------------------
PUTHEXD
	PSHS	CC,A,B
	BSR	PUTHEX8		; print A (high byte of D)
	LDA	2,S		; load original B (low byte of D)
	BSR	PUTHEX8		; print low byte
	PULS	CC,A,B,PC

	ENDSECTION

	END
