;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; Instrumented UART echo: receive a byte, send it back.
; Checks that received bytes form a 0x00-0xFF cycling sequence
; (matching what serial_test.py sends) and classifies errors.
;
; Counters are in the SHARED block ($FFD0-$FFDF) and appear directly
; in the 'st' command output.  Layout (15 bytes, 1 spare):
;
;   SHARED+0   RX_COUNT   (2 bytes) - total bytes received
;   SHARED+2   ERR_COUNT  (2 bytes) - sequence errors not classified as dup or skip
;   SHARED+4   DUP_COUNT  (2 bytes) - recv == expected-1 (stale byte re-read)
;   SHARED+6   SKIP_COUNT (2 bytes) - recv == expected+1 (one byte was lost)
;   SHARED+8   EXPECTED   (1 byte)  - expected value of next incoming byte
;   SHARED+9   (spare)
;   SHARED+10  (spare)
;   SHARED+11  S_BEFORE   (1 byte)  - UARTS snapshot at last error
;   SHARED+12  S_AFTER    (1 byte)  - UARTS snapshot at last error (ditto)
;   SHARED+13  ERR_GOT    (1 byte)  - byte received on last error
;   SHARED+14  ERR_EXP    (1 byte)  - byte expected on last error
;
; Cycle budget per byte (1 MHz, good-byte path):
;
; RX ISR (fires on each incoming byte):
;   IRQ entry   19  (stack CC,A,B,DP,X,Y,U,PC = 12 bytes + vector fetch)
;   ORCC I 3, LDA UARTS 5, BITA #RDRF 2, BNE RXIRQ 3            = 13
;   LDB RX_P_IN 5, INCB 2, ANDB 2, CMPB RX_P_OU 5, BEQ 3       = 17
;   PSHS B 6, LDA UARTRX 5, LDB RX_P_IN 5, LDX #RX_BUF 3       = 19
;   STA B,X 5, PULS B 6, STB RX_P_IN 5, RTI (E=1) 15            = 31
;                                                         Total  = 99
;
; Main loop (good-byte path):
;   LBSR INCH   9 + body 68  (see test_echo_fast.asm for breakdown) = 77
;   BCC Loop    3
;   CMPA EXPECTED 5, BNE not-taken 3                             =  8
;   LBSR OUTCH  9 + body 77  (see test_echo_fast.asm for breakdown) = 86
;   INC EXPECTED 7, BRA Count 3                                  = 10
;   LDD RX_COUNT 6, ADDD #1 4, STD RX_COUNT 6, LBRA Loop 5      = 21
;                                                       Subtotal = 205
;
; TX ISR (fires on TDRE after OUTCH enables it):
;   IRQ entry   19
;   ORCC I 3, LDA UARTS 5, BITA #RDRF 2, BNE not-taken 3        = 13
;   BITA #TDRE 2, BNE TXIRQ 3                                    =  5
;   LDB TX_P_OU 5, CMPB TX_P_IN 5, BEQ not-taken 3              = 13
;   LDX #TX_BUF 3, LDA B,X 5, STA UARTTX 5                      = 13
;   INCB 2, ANDB 2, STB TX_P_OU 5, RTI (E=1) 15                 = 24
;                                                         Total  = 87
;
; ──────────────────────────────────────────────────────────────────
;   Grand total: 391 cycles/byte  →  ~2 559 bytes/s @ 1 MHz
;
	PRAGMA	cescapes

	USE	ascii.inc
	USE	uart.inc
	USE	syslib.inc

Start	EXPORT

	SECTION	TEXT

; Variable layout in the SHARED block ($FFD0-$FFDF).
; BSSCLR does not touch SHARED, so we zero it explicitly at startup.
RX_COUNT	EQU	SHARED+0	; 2 bytes
ERR_COUNT	EQU	SHARED+2	; 2 bytes
DUP_COUNT	EQU	SHARED+4	; 2 bytes
SKIP_COUNT	EQU	SHARED+6	; 2 bytes
EXPECTED	EQU	SHARED+8	; 1 byte
S_BEFORE	EQU	SHARED+11	; 1 byte
S_AFTER		EQU	SHARED+12	; 1 byte
ERR_GOT		EQU	SHARED+13	; 1 byte
ERR_EXP		EQU	SHARED+14	; 1 byte

; Initialise, spin until the first byte to establish the sequence base.
Start	LDS	#STACK
	LBSR	BSSCLR

	; Zero the SHARED block (BSSCLR does not cover it).
	LDX	#SHARED
	LDB	#16
ShrClr	CLR	,X+
	DECB
	BNE	ShrClr

	LBSR	INIT

First	LBSR	INCH		; wait for first byte
	BCC	First
	INCA			; EXPECTED = first+1 (next byte we want to see)
	STA	EXPECTED
	DECA			; restore received byte for echo
	LBSR	OUTCH
	LDD	RX_COUNT
	ADDD	#1
	STD	RX_COUNT

; Main echo loop.  A = received byte throughout.
Loop	LBSR	INCH
	BCC	Loop		; no byte yet

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
	LDA	UARTS		; UART status snapshot at error time
	STA	S_BEFORE
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

	END
