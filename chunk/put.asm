;
; Copyright (c) 2025 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
	PRAGMA	cescapes

	USE	ascii.inc
	USE	uart.inc
	USE	syslib.inc

Start	EXPORT

	SECTION	TEXT

Start	LDS	#$8000
	LDA	#$22
	STA	SHARED+0
	LBSR	BSSCLR
	LDA	#$33
	STA	SHARED+1
	LBSR	INIT
	LDA	#$44
	STA	SHARED+2
	LBSR	PUTCRLF
	LDA	#$55
	STA	SHARED+3
	LBSR	PUTCRLF
	LDA	#$66
	STA	SHARED+4
S@0	LDX	#SHORT
	LBSR	PUTS
	LBSR	PUTCRLF
	LDX	#LONG
	LBSR	PUTS
	LBSR	PUTCRLF
	LBSR	PUTCRLF
HERE	BRA	HERE
S@2	BRA	S@0		Loop instead of syncing to allow interrupts to clear buffer

	ENDSECTION

	SECTION	CONSTANT

SHORT	FCN	"Hi!"

LONG	FCC	"Hello, World! 1\n"
	FCC	"Hello, World! 2\n"
	FCC	"Hello, World! 3\n"
	FCC	"Hello, World! 4\n"

	ENDSECTION

	END
