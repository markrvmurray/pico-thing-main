;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; putdecb.asm — Print B as 1-3 decimal digits (no leading zeros).
; Preserves all registers and CC. Output via OUTCH.
; Requires putdecd.obj at link time.
;

	USE	syslib.inc

PUTDECD	EXTERN

PUTDECB	EXPORT

	SECTION	TEXT

; -----------------------------------------------------------------------
; PUTDECB — print B as 1-3 decimal digits (no leading zeros).
; Preserves all registers and CC.
; Zero-extends B into D and calls PUTDECD.
; -----------------------------------------------------------------------
PUTDECB
	PSHS	CC,A
	CLRA			; D = 0:B (zero-extend B to 16 bits)
	BSR	PUTDECD
	PULS	CC,A,PC

	ENDSECTION

	END
