;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
	PRAGMA	cescapes

	USE	ascii.inc
	USE	uart.inc
	USE	syslib.inc

Start	EXPORT

	SECTION	TEXT

Start	LDS	#STACK
	LBSR	BSSCLR
	LBSR	INIT
	LDA	#'X
	LBSR	OUTCH
	LDA	#'Y
	LBSR	OUTCH
	LDA	#'Z
	LBSR	OUTCH
	LBSR	PUTCRLF
	LBSR	PUTCRLF
S@0	LDX	#SHORT
	LBSR	PUTS
	LBSR	PUTCRLF
	LDX	#LONG
	LBSR	PUTS
	LBSR	PUTCRLF
	LBSR	PUTCRLF
S@1	BRA	S@1		Loop instead of syncing to allow interrupts to clear buffer

	ENDSECTION

	SECTION	CONSTANT

SHORT	FCN	"Hi!"

LONG	FCC	"Hello, World! 1\n"
	FCC	"Hello, World! 2\n"
	FCC	"Hello, World! 3\n"
	FCN	"Hello, World! 4\n"

	ENDSECTION

	END
