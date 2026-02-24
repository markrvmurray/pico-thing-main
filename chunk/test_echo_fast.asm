;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; Bare echo loop: receive a byte, send it back.
; No instrumentation, no sequence checking.
; Used to measure the maximum software echo rate.
;
; Cycle budget per byte (1 MHz, ring buffers not empty/full):
;
; RX ISR (fires on each incoming byte):
;   IRQ entry   19  (stack CC,A,B,DP,X,Y,U,PC = 12 bytes + vector fetch)
;   ORCC I 3, LDA UARTS 5, BITA #RDRF 2, BNE RXIRQ 3            = 13
;   LDB RX_P_IN 5, INCB 2, ANDB 2, CMPB RX_P_OU 5, BEQ 3       = 17
;   PSHS B 6, LDA UARTRX 5, LDB RX_P_IN 5, LDX #RX_BUF 3       = 19
;   STA B,X 5, PULS B 6, STB RX_P_IN 5, RTI (E=1) 15            = 31
;                                                         Total  = 99
;
; Main loop:
;   LBSR INCH   9 + body 68  (PSHS CC,B,U 9,
;                              LDB RX_P_OU 5, CMPB RX_P_IN 5, BEQ 3,
;                              LDU #RX_BUF 3, LDA B,U 5,
;                              INCB 2, ANDB 2, STB RX_P_OU 5,
;                              TST RX_FULL_FLAG 7, BEQ I@1 3,
;                              PULS CC 6, ORCC C 3,
;                              PULS B,U,PC 10)                   = 77
;   BCC Loop    3
;   LBSR OUTCH  9 + body 77  (PSHS CC,B,U 9, LDU #TX_BUF 3,
;                              LDB TX_P_IN 5, INCB 2, ANDB 2,
;                              CMPB TX_P_OU 5, BEQ not-taken 3,
;                              PSHS B 6, LDB TX_P_IN 5, STA B,U 5,
;                              PULS B 6, STB TX_P_IN 5,
;                              LDB IQ_RXTX 5, STB UARTC 5,
;                              PULS CC,B,U,PC 11)                = 86
;   BRA Loop    3
;                                                       Subtotal = 169
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
;   Grand total: 355 cycles/byte  →  ~2 817 bytes/s @ 1 MHz

	PRAGMA	cescapes

	USE	uart.inc
	USE	syslib.inc

Start	EXPORT

	SECTION	TEXT

Start	LDS	#STACK
	LBSR	BSSCLR
	LBSR	INIT

Loop	LBSR	INCH
	BCC	Loop
	LBSR	OUTCH
	BRA	Loop

	ENDSECTION

	END
