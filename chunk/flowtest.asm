;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; Flow control bench generator.
;
; Emits a deterministic, countable stream to one of the two ACIAs so that the
; host end can prove the CDC pump loses nothing.  The stream is a tag byte
; naming the port, then FT_TOTAL bytes of a plain 8-bit counter, so byte i of
; the count is (i AND $FF): any loss, duplication or reordering shows up at
; once, and the host needs no state but the index.
;
; Build with IO=poll (the default).  That matters: the polled OUTCH spins on
; S_TDRE, which is exactly the path the firmware stalls by clearing TDRE when
; the host cannot take more, so a stall here is the real mechanism rather than
; a ring buffer filling up.
;
; SHARED block layout (visible in the Pico's 'st' register dump at $FFD0):
;   +1  FT_PORT  (1)  set BEFORE running: 0 = console, non-zero = auxiliary
;   +2  FT_SENT  (2)  bytes of the count sent so far - watch this in 'st':
;                     frozen while the stream is held means the guest is
;                     stalled, which is the point of the RTS test
;   +4  FT_DONE  (1)  0 while running, $FF once the whole stream is out
;
; So a run is: mo ffd1 00   (console)  or  mo ffd1 01   (auxiliary)
;              then load and run this chunk, and SYNC ends it.

	PRAGMA	cescapes

	USE	ascii.inc
	USE	uart.inc
	USE	syslib.inc

Start	EXPORT

; NOT SHARED+0, +10 or +11: the firmware zeroes those three itself, in
; watchpoint_disarm() and on a watchpoint firing, so anything of ours living
; there would be cleared out from under us if a watchpoint or break were armed
; - intermittently, depending on whether one happened to be.
FT_PORT		EQU	SHARED+1	; which ACIA to emit on
FT_SENT		EQU	SHARED+2	; progress, 2 bytes
FT_DONE		EQU	SHARED+4	; completion flag

FT_TOTAL	EQU	8192		; bytes after the tag; several times both
					; the 512-byte pump buffer and the
					; 1024-byte ACIA queue, so the host has
					; to be throttled more than once

TAG_CON		EQU	'C		; tag emitted on the console port
TAG_AUX		EQU	'A		; tag emitted on the auxiliary port

	SECTION	TEXT

Start	LDS	#STACK			; our own stack
	LBSR	BSSCLR			; clear the bss as every chunk does
	CLR	FT_DONE			; not finished
	LDD	#0
	STD	FT_SENT			; nothing sent yet

	LDA	FT_PORT			; which port did the operator ask for
	BEQ	F@console
	LBSR	INIT_AUX		; U = auxiliary port descriptor
	LDA	#TAG_AUX
	BRA	F@tag
F@console
	LBSR	INIT_CON		; U = console port descriptor
	LDA	#TAG_CON
F@tag
	LBSR	OUTCH			; the tag tells the host which port it
					; opened, so no guessing from the node
					; name - see option 1 in the plan

	LDX	#0			; X = bytes of the count sent
	CLRB				; B = the counter itself
F@loop
	TFR	B,A			; next byte of the pattern
	LBSR	OUTCH			; spins on S_TDRE while the host holds
					; the output, which is the stall
	INCB
	LEAX	1,X
	STX	FT_SENT			; publish progress for 'st'
	CMPX	#FT_TOTAL
	BLO	F@loop

	LDA	#$FF
	STA	FT_DONE			; the whole stream is out
	SYNC				; hand back to the Pico

	ENDSECTION

	END
