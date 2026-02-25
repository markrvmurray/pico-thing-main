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

; NO_IRQ		EQU	1
; RXFULL_SAFETY_NET	EQU	1

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

	ENDC

URESET	LDA	#C_RESET	complete uart reset
	STA	UARTC
	RTS

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
* OUTCH   [*] RX_FULL_FLAG != 0   : RTS deasserted, drain TX ring directly (no IQ_RXTX)

	IFDEF	NO_IRQ

OUTCH	PSHS	CC,B
O@0	LDB	UARTS
	BITB	#S_TDRE
	BEQ	O@0		branch if transmitter not empty
	STA	UARTTX
	PULS	CC,B,PC

	ELSE

OUTCH	PSHS	CC,B,X
	LDX	#TX_BUF
	LDB	TX_P_IN
	INCB
	ANDB	#%00001111	gives MOD 16
O@0	CMPB	TX_P_OU
	BEQ	O@0
	PSHS	B
	LDB	TX_P_IN
	STA	B,X
	PULS	B
	STB	TX_P_IN
	TST	RX_FULL_FLAG	RTS deasserted? drain TX ring buffer directly
	BNE	O@D
	LDB	IQ_RXTX		receive and transmit are both irq-driven
	STB	UARTC
	PULS	CC,B,X,PC
O@D	LDB	TX_P_OU		drain TX ring buffer directly (RTS deasserted, TXIRQ disabled)
	CMPB	TX_P_IN
	BEQ	O@X
O@W	LDA	UARTS		wait for TDRE before each write
	BITA	#S_TDRE
	BEQ	O@W
	LDA	B,X
	STA	UARTTX
	INCB
	ANDB	#%00001111
	STB	TX_P_OU
	BRA	O@D
O@X	PULS	CC,B,X,PC

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

INCH	PSHS	CC,B,X
	LDB	RX_P_OU
	CMPB	RX_P_IN
	BEQ	I@2		branch if buffer empty
	LDX	#RX_BUF
	LDA	B,X		get the byte from the buffer
	INCB
	ANDB	#%00001111	gives MOD 16
	STB	RX_P_OU
I@1	PULS	CC
	ORCC	C		carry set means got a byte
	PULS	B,X,PC
I@2	TST	RX_FULL_FLAG	buffer empty: was RTS deasserted?
	BEQ	I@3		no → nothing to do
	CLR	RX_FULL_FLAG	yes → re-assert RTS now that buffer has drained
	LDB	TX_P_IN
	CMPB	TX_P_OU
	BEQ	I@2A		TX empty → IQ_RX
	LDB	IQ_RXTX		TX has data → IQ_RXTX
	STB	UARTC
	BRA	I@3
I@2A	LDB	IQ_RX
	STB	UARTC
I@3	PULS	CC
	ANDCC	C		carry clear means no byte
	PULS	B,X,PC

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

IRQ_ISR	PSHS	A,B,X
	LDA	UARTS		this doesn't clear the interrupt
	BITA	#S_RDRF		receiver interrupt (checked first so RX has priority over TX)
	BNE	RXIRQ
	BITA	#S_TDRE		transmitter interrupt
	BNE	TXIRQ
* FALLTHROUGH
IRQ_RET	PULS	A,B,X
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
	BRA	IRQ_RET

TXEMPTY	LDA	IQ_RX		interrupts on receiver only
	STA	UARTC
	LDA	#$EA
	STA	ERRNO
	BRA	IRQ_RET

RXIRQ	LDB	RX_P_IN
	INCB
	ANDB	#%00001111	gives MOD 16
	CMPB	RX_P_OU

	IFDEF	RXFULL_SAFETY_NET
	BEQ	RXFULL		if the input buffer is full
	ELSE
	BEQ	IRQ_RET		if the input buffer is full
	ENDC

	PSHS	B		save next_in
	INCB			check: will buffer be full after this store?
	ANDB	#%00001111
	CMPB	RX_P_OU
	BNE	R@0		no room pressure → normal
	LDA	#1		buffer will be full: deassert RTS NOW, before LDA UARTRX
	STA	RX_FULL_FLAG	so guest_receive() sees rts_deasserted and skips staging
	LDA	IQ_RTSOFF
	STA	UARTC
R@0	LDA	UARTRX		clears the interrupt (no next-byte staged if rts_deasserted)
	LDB	RX_P_IN
	LDX	#RX_BUF
	STA	B,X
	PULS	B
	STB	RX_P_IN
	BRA	IRQ_RET

	IFDEF	RXFULL_SAFETY_NET

RXFULL	LDA	#1		safety net: RXIRQ should have caught this earlier
	STA	RX_FULL_FLAG
	LDA	IQ_RTSOFF
	STA	UARTC		deassert RTS BEFORE LDA UARTRX so guest_receive() skips staging

	ENDC

	LDA	UARTRX		clear the interrupt
	LDA	#$BE		set errno for the user
	STA	ERRNO

	IFDEF	RXFULL_SAFETY_NET

* Drain TX ring buffer. Cannot spin on TDRE inside ISR: if TDRE=0, stop
* early and leave remaining bytes for OUTCH O@D (main loop) to finish.
	LDX	#TX_BUF
F@0	LDB	TX_P_OU
	CMPB	TX_P_IN
	BEQ	IRQ_RET		TX ring buffer empty
	LDA	UARTS		check TDRE
	BITA	#S_TDRE
	BEQ	IRQ_RET		TDRE not set: defer remaining bytes to OUTCH O@D
	LDA	B,X		get byte
	STA	UARTTX		send it
	INCB
	ANDB	#%00001111
	STB	TX_P_OU
	BRA	F@0

	ENDC

	ENDC

	ENDSECTION

	SECTION	CONSTANT

	IFNDEF	NO_IRQ

IQ_RX	FCB	C_RX_IRQ
IQ_RXTX	FCB	C_RX_IRQ|C_TX_IRQ
IQ_RTSOFF	FCB	C_RX_IRQ|C_RTS_OFF	; rx irq on, rts deasserted (pauses Pico→6809)

	ENDC

	ENDSECTION

	SECTION	BSS

ERRNO	RMB	1

TX_P_IN RMB	1
TX_P_OU RMB	1

RX_P_IN	RMB	1
RX_P_OU	RMB	1

RX_FULL_FLAG	RMB	1	; non-zero when RXFULL fired and RTS is deasserted

TX_BUF	RMB	16
RX_BUF	RMB	16

	ENDSECTION

	END
