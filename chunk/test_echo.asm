;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; Simple UART echo: receive a byte, send it back.
; Uses the IRQ-driven io.asm driver with RTS/CTS flow control.
;
	PRAGMA	cescapes

	USE	ascii.inc
	USE	uart.inc
	USE	syslib.inc

Start	EXPORT

	SECTION	TEXT

Start	LDS	#STACK
	LBSR	BSSCLR
	LBSR	INIT		; install IRQ ISR, enable RX interrupt
Loop	LBSR	INCH		; non-blocking: C set = byte in A
	BCC	Loop		; nothing yet, spin
	LBSR	OUTCH		; echo it back (IRQ-driven TX)
	BRA	Loop

	ENDSECTION

	END
