;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;

* Control (write)
C_NONE		EQU	%00000000
C_RESET		EQU	%00000011
C_TX_IRQ	EQU	%00100000
C_RX_IRQ	EQU	%10000000
C_RTS_OFF	EQU	%01000000	; tx_irq=0b10: RTS deasserted, no TX irq
C_LOOPCLOSE	EQU	%01100000	; tx_irq=0b11: close loopback (TX->RX direct)
C_LOOPQUEUE	EQU	%00000010	; divide_select=0b10: queue loopback (TX->RX queue)


* Status (read)
S_RDRF		EQU	%00000001
S_TDRE		EQU	%00000010
S_IRQ		EQU	%10000000

* Port descriptor offsets (all routines take U = port descriptor)
AP_CTL		EQU	0	; control/status register address (2 bytes)
AP_DATA		EQU	2	; data register address (2 bytes)
AP_TXPIN	EQU	4	; TX ring write pointer (1 byte)
AP_TXPOU	EQU	5	; TX ring read pointer (1 byte)
AP_RXPIN	EQU	6	; RX ring write pointer (1 byte)
AP_RXPOU	EQU	7	; RX ring read pointer (1 byte)
AP_RXFL		EQU	8	; RX full flag (1 byte)
AP_ERRNO	EQU	9	; error code (1 byte)
AP_TXBUF	EQU	10	; TX ring buffer (64 bytes)
AP_RXBUF	EQU	74	; RX ring buffer (64 bytes)
AP_SIZE		EQU	138

* I/O routines (all take U = port descriptor)
INIT_CON	EXTERN
INIT_AUX	EXTERN
OUTCH		EXTERN
INCH		EXTERN
PUTS		EXTERN
PUTCHAR		EXTERN
PUTCRLF		EXTERN
PUTHEXB		EXTERN
PUTHEXD		EXTERN
PUTDECB		EXTERN
PUTDECD		EXTERN
GETS		EXTERN

* Port descriptor instances
CON_PORT	EXTERN
AUX_PORT	EXTERN

* ACIA register addresses (in PICO2 section of syslib)
ACIAC	EXTERN
ACIAS	EXTERN
ACIATX	EXTERN
ACIARX	EXTERN

AUXAC	EXTERN
AUXAS	EXTERN
AUXATX	EXTERN
AUXARX	EXTERN
