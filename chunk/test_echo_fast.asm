;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; Bare echo loop: receive a byte, send it back.
; No instrumentation, no sequence checking.
; Used to measure the maximum hardware echo rate.
;
; Cycle budget (1 MHz, no RDRF/TDRE spin):
;   LDA UARTS     5
;   BITA #S_RDRF  2
;   BEQ Loop      3
;   LDA UARTRX    5
;   LBSR OUTCH    9  (+OUTCH body 31)
;   BRA Loop      3
;   ─────────────────
;   Total:       58 cycles/byte  →  ~17 241 bytes/s @ 1 MHz

	PRAGMA	cescapes

	USE	uart.inc
	USE	syslib.inc

Start	EXPORT

	SECTION	TEXT

Start	LDS	#STACK
	LBSR	BSSCLR
	LBSR	INIT

Loop	LDA	UARTS
	BITA	#S_RDRF
	BEQ	Loop
	LDA	UARTRX
	LBSR	OUTCH
	BRA	Loop

	ENDSECTION

	END
