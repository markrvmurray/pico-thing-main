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
	ifdef	NOT_DEF
S@0	LBSR	INCH
	BCC	S@0
	STA	SHARED+15
	LDB	#$55
	STB	SHARED+3
	LBSR	OUTCH
	LDB	#$99
	STB	SHARED+4
S@1	BRA	S@0
	else
S@2	LBSR	PUTCRLF
	LDX	#BUFFER
	LBSR	GETS
	LBSR	PUTCRLF
	LBSR	PUTS
	BRA	S@2
STOP	BRA	STOP		Loop instead of sysncing to allow queues to flush
	endc

	ENDSECTION

	SECTION	BSS

BUFFER	RMB	256

	ENDSECTION

	END
