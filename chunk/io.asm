;
; Copyright (c) 2025 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
	PRAGMA	cescapes

	USE	ascii.inc
	USE	uart.inc
	USE	syslib.inc

INIT	EXPORT
PUTS	EXPORT
PUTCHAR	EXPORT
PUTCRLF	EXPORT
OUTCH	EXPORT
GETS	EXPORT
INCH	EXPORT

#NO_IRQ	EQU	1

	SECTION	TEXT

	IFDEF	NO_IRQ

INIT	BSR	URESET		reset the uart
	RTS

	ELSE

INIT	PSHS	CC,A,X
	BSR	URESET		reset the uart
	LDX	#IRQ_ISR
	LBSR	INITIRQ		install the interrrupt service routine
	LDA	IQ_RX		select receive interrupts, transmit interrupts are enabled when needed
	STA	UARTC
	PULS	CC
	ANDCC	I		enable interrupts in cpu
	PULS	A,X,PC

URESET	LDA	#C_RESET	complete uart reset
	STA	UARTC
	RTS

	ENDC

* Print a NUL-terminated string to the console.
* In: X points to the string.
* Out: All registers preserved
PUTS	PSHS	CC,A,X
P@0	LDA	,X+
	BEQ	P@1
	LBSR	PUTCHAR
	BRA	P@0
P@1	PULS	CC,A,X,PC

* Print byte, sends CR, LF for LF
* In: A - byte
* Out: All registers preserved
PUTCHAR PSHS	CC
	CMPA	#LF
	BNE	P@0
	PSHS	A
	LDA	#CR
	LBSR	OUTCH
	PULS	A
P@0	LBSR	OUTCH
	PULS	CC,PC

* Print CR, LF to the console.
* In: None
* Out: All registers preserved
PUTCRLF	PSHS	CC,A
	LDA	#LF
	BSR	PUTCHAR
	PULS	CC,A,PC

* Get a NUL-terminated string from the console. The CR is not stored.
* In: X points to the input buffer.
* Out: Y points to the NUL terminator. Y-X gives strlen()
GETS	PSHS	CC,A
	TFR	X,Y
G@0	LBSR	INCH
	BCC	G@0		nothing available yet
	CMPA	#CR
	BEQ	G@1
	CMPA	#LF
	BEQ	G@1
	LBSR	OUTCH
	STA	,Y+
	BRA	G@0
G@1	CLR	,Y
	BSR	PUTCRLF
	PULS	CC,A,PC

* OUTCH - output byte in A to UART
* No registers are changed.

* IRQ_ISR [ ] TX_P_IN == TX_P_OU  : buffer empty
* IRQ_ISR [ ] TX_P_IN != TX_P_OU  : next byte to transfer is at TX_P_OU, then TX_P_OU+
* OUTCH   [*] TX_P_IN+ == TX_P_OU : buffer full, so spinwait
* OUTCH   [*] TX_P_IN+ != TX_P_OU : Store byte in TX_P_IN, then TX_P_IN+

	IFDEF	NO_IRQ

OUTCH	PSHS	CC,B
O@0	LDB	UARTS
	BITB	#S_TDRE
	BEQ	O@0		branch if transmitter not empty
	STA	UARTTX
	PULS	CC,B,PC

	ELSE

OUTCH	PSHS	CC,B,U
	LDU	#TX_BUF
	LDB	TX_P_IN
	INCB
	ANDB	#%00001111	gives MOD 16
O@0	CMPB	TX_P_OU
	BEQ	O@0
	PSHS	B
	LDB	TX_P_IN
	STA	B,U
	PULS	B
	STB	TX_P_IN
	LDB	IQ_RXTX		receive and transmit are both irq-driven
	STB	UARTC
	PULS	CC,B,U,PC

	ENDC

* INCH - input byte from UART to A
* A contains returned value.
* C Flag set means a byte was received.
* No other registers or CC bits are changed.

* IRQ_ISR [ ] RX_P_IN+ == RX_P_OU : buffer full
* IRQ_ISR [ ] RX_P_IN+ != RX_P_OU : store byte in RX_P_IN, then RX_P_IN+
* INCH    [*] RX_P_IN == RX_P_OU  : buffer empty, so clear carry
* INCH    [*] RX_P_IN != RX_P_OU  : next byte to return is at RX_P_OU, then RX_P_OU+, set carry

	IFDEF	NO_IRQ

INCH	PSHS	CC
	LDA	UARTS
	BITA	#S_RDRF
	BEQ	I@0		branch if receiver not full
	LDA	UARTRX
	PULS	CC
	ORCC	C		carry set means got a byte
	RTS
I@0	PULS	CC
	ANDCC	C		carry clear means no byte
	RTS

	ELSE

INCH	PSHS	CC,B,U
	LDB	RX_P_OU
	CMPB	RX_P_IN
	BEQ	I@0		branch if buffer empty
	LDU	#RX_BUF
	LDA	B,U		get the byte from the buffer
	INCB
	ANDB	#%00001111	gives MOD 16
	STB	RX_P_OU
	PULS	CC
	ORCC	C		carry set means got a byte
	PULS	B,U,PC
I@0	PULS	CC
	ANDCC	C		carry clear means no byte
	PULS	B,U,PC

	ENDC

* =====================================================================
* IRQ_ISR For UART character input/output
*
* IRQ_ISR [*] TX_P_IN == TX_P_OU  : buffer empty
* IRQ_ISR [*] TX_P_IN != TX_P_OU  : next byte to transfer is at TX_P_OU, then TX_P_OU+
* OUTCH   [ ] TX_P_IN+ == TX_P_OU : buffer full, so spinwait
* OUTCH   [ ] TX_P_IN+ != TX_P_OU : Store byte in TX_P_IN, then TX_P_IN+
*
* IRQ_ISR [*] RX_P_IN+ == RX_P_OU : buffer full
* IRQ_ISR [*] RX_P_IN+ != RX_P_OU : store byte in RX_P_IN, then RX_P_IN+
* INCH    [ ] RX_P_IN == RX_P_OU  : buffer empty, so clear carry
* INCH    [ ] RX_P_IN != RX_P_OU  : next byte to return is at RX_P_OU, then RX_P_OU+, set carry

	IFDEF	NO_IRQ

IRQ_ISR	RTI

	ELSE

IRQ_ISR	ORCC	I
	LDA	UARTS		this doesn't clear the interrupt
	BITA	#S_TDRE		transmitter interrupt
	BNE	TXIRQ
	BITA	#S_RDRF		receiver interrupt
	BNE	RXIRQ
* FALLTHROUGH
	RTI

TXIRQ	LDB	TX_P_OU
	CMPB	TX_P_IN
	BEQ	TXEMPTY
	LDX	#TX_BUF
	LDA	B,X
	STA	UARTTX		clears the interrupt
	INCB
	ANDB	#%00001111	gives MOD 16
	STB	TX_P_OU
	RTI

TXEMPTY	LDA	IQ_RX		interrupts on receiver only
	STA	UARTC
	LDA	#$EA
	STA	ERRNO
	RTI

RXIRQ	LDB	RX_P_IN
	INCB
	ANDB	#%00001111	gives MOD 16
	CMPB	RX_P_OU
	BEQ	RXFULL		if the input buffer is full
	PSHS	B
	LDA	UARTRX		clears the interrupt
	LDB	RX_P_IN
	LDX	#RX_BUF
	STA	B,X
	PULS	B
	STB	RX_P_IN
	RTI

RXFULL	LDA	UARTRX		clear the interrupt
	LDA	#$BE		set errno for the user
	STA	ERRNO
	RTI

	ENDC

	ENDSECTION

	SECTION	CONSTANT

	IFNDEF	NO_IRQ

IQ_RX	FCB	C_RX_IRQ
IQ_RXTX	FCB	C_RX_IRQ|C_TX_IRQ

	ENDC

	ENDSECTION

	SECTION	BSS

ERRNO	RMB	1

TX_P_IN RMB     1
TX_P_OU RMB     1

RX_P_IN	RMB	1
RX_P_OU	RMB	1

TX_BUF	RMB	16
RX_BUF	RMB	16

	ENDSECTION

	END
