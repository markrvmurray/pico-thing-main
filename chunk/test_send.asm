;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;

; Print as fast as possible to try to provoke bugs

	PRAGMA	cescapes

	USE	ascii.inc
	USE	uart.inc
	USE	syslib.inc

Start	EXPORT

	SECTION	TEXT

Start	LDS	#STACK
	LBSR	BSSCLR
	LBSR	INIT
S@0	LDA	#SPACE
S@1	LBSR	OUTCH
	INCA
	CMPA	#DEL
	BLT	S@1
	LDA	#CR
	LBSR	OUTCH
	LDA	#LF
	LBSR	OUTCH
	BRA	S@0

	ENDSECTION

	SECTION	CONSTANT

SHORT	FCN	"Hi!"

LONG	FCC	"Hello, World! 1\n"
	FCC	"Hello, World! 2\n"
	FCC	"Hello, World! 3\n"
	FCC	"Hello, World! 4\n"

	ENDSECTION

	END
