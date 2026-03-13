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
	LBSR	INIT_CON
S@0	LBSR	INCH
	BCC	S@0
	LBSR	OUTCH
	CMPA	#CR
S@1	BNE	S@0
	LBSR	PUTCRLF
	LDX	#BUFFER
	LBSR	GETS
	LBSR	PUTCRLF
	LBSR	PUTS
	LBSR	PUTCRLF
STOP	BRA	STOP		Loop instead of sysncing to allow queues to flush
	endc

	ENDSECTION

	SECTION	bss

BUFFER	RMB	256

	ENDSECTION

	END
