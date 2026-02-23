;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; Instrumented UART echo: receive a byte, send it back.
; Checks that received bytes form a 0x00-0xFF cycling sequence
; (matching what serial_test.py sends) and classifies errors.
;
; Counters are in BSS. After the test, read them via 'ex <addr>' on the
; Pico console, using addresses from test_echo.map:
;
;   RX_COUNT   (2 bytes) - total bytes received
;   ERR_COUNT  (2 bytes) - sequence errors not classified as dup or skip
;   DUP_COUNT  (2 bytes) - recv == expected-1 (stale byte re-read)
;   SKIP_COUNT (2 bytes) - recv == expected+1 (one byte was lost)
;   S_BEFORE   (1 byte)  - UARTS captured just before last bad UARTRX read
;   S_AFTER    (1 byte)  - UARTS captured just after last bad UARTRX read
;   ERR_GOT    (1 byte)  - byte received on last error
;   ERR_EXP    (1 byte)  - byte expected on last error
;
	PRAGMA	cescapes

	USE	ascii.inc
	USE	uart.inc
	USE	syslib.inc

Start	EXPORT

	SECTION	TEXT

; Initialise, spin until the first byte to establish the sequence base.
Start	LDS	#STACK
	LBSR	BSSCLR
	LBSR	INIT

First	LDA	UARTS
	BITA	#S_RDRF
	BEQ	First
	LDA	UARTRX		; first byte - accept unconditionally
	INCA			; EXPECTED = first+1 (next byte we want to see)
	STA	EXPECTED
	DECA			; restore received byte for echo
	LBSR	OUTCH
	LDD	RX_COUNT
	ADDD	#1
	STD	RX_COUNT

; Main echo loop.
; UARTS is read before and after UARTRX so we can see the RDRF state
; on both sides of the read.  A = received byte throughout.
Loop	LDA	UARTS
	BITA	#S_RDRF
	BEQ	Loop		; spin

	STA	STAT_PRE	; status just before the data read

	LDA	UARTRX		; the byte

	PSHS	A		; save it briefly while we grab post-read status
	LDA	UARTS
	STA	STAT_POST
	PULS	A		; A = received byte again

	; --- Sequence check ---
	CMPA	EXPECTED
	BNE	Err

Good	; Correct byte.  Echo it, advance expected, count it.
	LBSR	OUTCH		; A still has the received byte
	INC	EXPECTED	; EXPECTED++ wraps naturally at $FF->$00
	BRA	Count

Err	; Wrong byte.  Save diagnostic info, classify, resync, still echo.
	STA	ERR_GOT
	LDA	EXPECTED
	STA	ERR_EXP
	LDA	STAT_PRE
	STA	S_BEFORE
	LDA	STAT_POST
	STA	S_AFTER

	; Dup: recv == expected-1  (same byte as last time - stale UARTRX)
	LDA	EXPECTED
	DECA
	CMPA	ERR_GOT
	BNE	Chk2
	LDD	DUP_COUNT
	ADDD	#1
	STD	DUP_COUNT
	BRA	Resync

	; Skip: recv == expected+1  (one byte was lost)
Chk2	LDA	EXPECTED
	ADDA	#1
	CMPA	ERR_GOT
	BNE	Chk3
	LDD	SKIP_COUNT
	ADDD	#1
	STD	SKIP_COUNT
	BRA	Resync

Chk3	LDD	ERR_COUNT
	ADDD	#1
	STD	ERR_COUNT

	; Resync: set EXPECTED to received+1 so errors don't cascade.
Resync	LDA	ERR_GOT
	ADDA	#1
	STA	EXPECTED
	LDA	ERR_GOT
	LBSR	OUTCH		; echo the (wrong) byte anyway

Count	LDD	RX_COUNT
	ADDD	#1
	STD	RX_COUNT
	LBRA	Loop

	ENDSECTION

	SECTION	BSS

RX_COUNT	RMB	2	; total bytes received
ERR_COUNT	RMB	2	; unclassified sequence errors
DUP_COUNT	RMB	2	; stale-byte re-reads (recv == expected-1)
SKIP_COUNT	RMB	2	; lost bytes (recv == expected+1)
EXPECTED	RMB	1	; expected value of next incoming byte
STAT_PRE	RMB	1	; UARTS before last UARTRX read (rolling)
STAT_POST	RMB	1	; UARTS after  last UARTRX read (rolling)
S_BEFORE	RMB	1	; UARTS before last *bad* UARTRX read
S_AFTER		RMB	1	; UARTS after  last *bad* UARTRX read
ERR_GOT		RMB	1	; byte received on last error
ERR_EXP		RMB	1	; byte expected on last error

	ENDSECTION

	END
