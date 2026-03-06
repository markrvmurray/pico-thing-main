;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; puthexb.asm — Print B as two hex digits.
; Preserves all registers and CC. Output via OUTCH.
;

	USE	syslib.inc

OUTCH	EXTERN

PUTHEXB	EXPORT

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
; PUTHEXB — print B as two hex digits.
; Preserves all registers and CC.
; -----------------------------------------------------------------------
PUTHEXB
	PSHS	CC,A
	TFR	B,A
	BSR	PUTHEX8
	PULS	CC,A,PC

	ENDSECTION

	END
