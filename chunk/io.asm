;
; Copyright (c) 2025 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; Port-parameterised I/O library.
;
; All routines take U = pointer to a port descriptor struct.
; Call INIT_CON or INIT_AUX to initialise a port and set U.
; U is preserved by all routines, so callers set it once.
;
	PRAGMA	cescapes

	USE	ascii.inc
	USE	uart.inc
	USE	syslib.inc

INIT_CON	EXPORT
INIT_AUX	EXPORT
CON_PORT	EXPORT
AUX_PORT	EXPORT
PUTS		EXPORT
PUTCHAR		EXPORT
PUTCRLF		EXPORT
OUTCH		EXPORT
GETS		EXPORT
INCH		EXPORT

; NO_IRQ		EQU	1
; RXFULL_SAFETY_NET	EQU	1

	SECTION	TEXT

* =====================================================================
* Port initialisation
*
* INIT_CON — initialise the console ACIA.
* INIT_AUX — initialise the auxiliary ACIA.
*
* On return U = port descriptor. CC: I bit cleared in IRQ mode.
* All other registers preserved.

	IFDEF	NO_IRQ

INIT_CON	PSHS	A,X,Y
	LDU	#CON_PORT
	LDX	#ACIAC
	LDY	#ACIATX
	BSR	INITPORT
	PULS	A,X,Y,PC

INIT_AUX	PSHS	A,X,Y
	LDU	#AUX_PORT
	LDX	#AUXAC
	LDY	#AUXATX
	BSR	INITPORT
	PULS	A,X,Y,PC

INITPORT
	STX	AP_CTL,U
	STY	AP_DATA,U
	CLR	AP_ERRNO,U
	LDA	#C_RESET
	STA	[AP_CTL,U]
	RTS

	ELSE

INIT_CON	PSHS	CC,A,B,X,Y
	LDU	#CON_PORT
	LDX	#ACIAC
	LDY	#ACIATX
	LDB	#1
	BSR	INITPORT
	PULS	CC
	ANDCC	I
	PULS	A,B,X,Y,PC

INIT_AUX	PSHS	CC,A,B,X,Y
	LDU	#AUX_PORT
	LDX	#AUXAC
	LDY	#AUXATX
	LDB	#2
	BSR	INITPORT
	PULS	CC
	ANDCC	I
	PULS	A,B,X,Y,PC

INITPORT
	STX	AP_CTL,U
	STY	AP_DATA,U
	CLR	AP_TXPIN,U
	CLR	AP_TXPOU,U
	CLR	AP_RXPIN,U
	CLR	AP_RXPOU,U
	CLR	AP_RXFL,U
	CLR	AP_ERRNO,U
	LDA	#C_RESET
	STA	[AP_CTL,U]
* Register IRQ_ISR once (on first port init)
	TST	PORT_FLAGS
	BNE	IP@1
	PSHS	B,X
	LDX	#IRQ_ISR
	LBSR	INITIRQ
	PULS	B,X
IP@1	ORB	PORT_FLAGS
	STB	PORT_FLAGS
* Enable receive interrupts on this port
	LDA	IQ_RX
	STA	[AP_CTL,U]
	RTS

	ENDC

* =====================================================================
* String I/O

* Print a NUL-terminated string to the port.
* In: U = port, X = string pointer.
* Out: All registers preserved.
PUTS	PSHS	CC,A,X
P@0	LDA	,X+
	BEQ	P@1
	LBSR	PUTCHAR
	BRA	P@0
P@1	PULS	CC,A,X,PC

* Print byte, sends CR, LF for LF.
* In: U = port, A = byte.
* Out: All registers preserved.
PUTCHAR PSHS	CC
	CMPA	#LF
	BNE	P@0
	PSHS	A
	LDA	#CR
	LBSR	OUTCH
	PULS	A
P@0	LBSR	OUTCH
	PULS	CC,PC

* Print CR, LF to the port.
* In: U = port.
* Out: All registers preserved.
PUTCRLF	PSHS	CC,A
	LDA	#LF
	BSR	PUTCHAR
	PULS	CC,A,PC

* Get a NUL-terminated string from the port. The CR is not stored.
* In: U = port, X = input buffer.
* Out: Y points to the NUL terminator. Y-X gives strlen().
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

* =====================================================================
* OUTCH — output byte in A to ACIA.
* In: U = port, A = byte.
* No registers are changed.

* IRQ_ISR [ ] TX_P_IN == TX_P_OU  : buffer empty
* IRQ_ISR [ ] TX_P_IN != TX_P_OU  : next byte to transfer is at TX_P_OU, then TX_P_OU+
* OUTCH   [*] TX_P_IN+ == TX_P_OU : buffer full, so spinwait
* OUTCH   [*] TX_P_IN+ != TX_P_OU : Store byte in TX_P_IN, then TX_P_IN+
* OUTCH   [*] RX_FULL_FLAG != 0   : RTS deasserted, drain TX ring directly (no IQ_RXTX)

	IFDEF	NO_IRQ

OUTCH	PSHS	CC,B
O@0	LDB	[AP_CTL,U]
	BITB	#S_TDRE
	BEQ	O@0		branch if transmitter not empty
	STA	[AP_DATA,U]
	PULS	CC,B,PC

	ELSE

OUTCH	PSHS	CC,B,X
	LEAX	AP_TXBUF,U
	LDB	AP_TXPIN,U
	INCB
	ANDB	#%00001111	gives MOD 16
O@0	CMPB	AP_TXPOU,U
	BEQ	O@0
	PSHS	B
	LDB	AP_TXPIN,U
	STA	B,X
	PULS	B
	STB	AP_TXPIN,U
	TST	AP_RXFL,U	RTS deasserted? drain TX ring buffer directly
	BNE	O@D
	LDB	IQ_RXTX		receive and transmit are both irq-driven
	STB	[AP_CTL,U]
	PULS	CC,B,X,PC
O@D	LDB	AP_TXPOU,U	drain TX ring buffer directly (RTS deasserted, TXIRQ disabled)
	CMPB	AP_TXPIN,U
	BEQ	O@X
O@W	LDA	[AP_CTL,U]	wait for TDRE before each write
	BITA	#S_TDRE
	BEQ	O@W
	LDA	B,X
	STA	[AP_DATA,U]
	INCB
	ANDB	#%00001111
	STB	AP_TXPOU,U
	BRA	O@D
O@X	PULS	CC,B,X,PC

	ENDC

* =====================================================================
* INCH — input byte from ACIA to A.
* In: U = port.
* A contains returned value.
* C flag set means a byte was received.
* No other registers or CC bits are changed.

* IRQ_ISR [ ] RX_P_IN+ == RX_P_OU : buffer full
* IRQ_ISR [ ] RX_P_IN+ != RX_P_OU : store byte in RX_P_IN, then RX_P_IN+
* INCH    [*] RX_P_IN == RX_P_OU  : buffer empty, so clear carry
* INCH    [*] RX_P_IN != RX_P_OU  : next byte to return is at RX_P_OU, then RX_P_OU+, set carry

	IFDEF	NO_IRQ

INCH	PSHS	CC
	LDA	[AP_CTL,U]
	BITA	#S_RDRF
	BEQ	I@0		branch if receiver not full
	LDA	[AP_DATA,U]
	PULS	CC
	ORCC	C		carry set means got a byte
	RTS
I@0	PULS	CC
	ANDCC	C		carry clear means no byte
	RTS

	ELSE

INCH	PSHS	CC,B,X
	LDB	AP_RXPOU,U
	CMPB	AP_RXPIN,U
	BEQ	I@2		branch if buffer empty
	LEAX	AP_RXBUF,U
	LDA	B,X		get the byte from the buffer
	INCB
	ANDB	#%00001111	gives MOD 16
	STB	AP_RXPOU,U
I@1	PULS	CC
	ORCC	C		carry set means got a byte
	PULS	B,X,PC
I@2	TST	AP_RXFL,U	buffer empty: was RTS deasserted?
	BEQ	I@3		no -> nothing to do
	CLR	AP_RXFL,U	yes -> re-assert RTS now that buffer has drained
	LDB	AP_TXPIN,U
	CMPB	AP_TXPOU,U
	BEQ	I@2A		TX empty -> IQ_RX
	LDB	IQ_RXTX		TX has data -> IQ_RXTX
	STB	[AP_CTL,U]
	BRA	I@3
I@2A	LDB	IQ_RX
	STB	[AP_CTL,U]
I@3	PULS	CC
	ANDCC	C		carry clear means no byte
	PULS	B,X,PC

	ENDC

* =====================================================================
* IRQ Service Routine — checks each initialised port.

	IFDEF	NO_IRQ

IRQ_ISR	RTS

	ELSE

IRQ_ISR	LDA	PORT_FLAGS
	BITA	#1
	BEQ	IS@A
	LDU	#CON_PORT
	BSR	ISR_PORT
IS@A	LDA	PORT_FLAGS
	BITA	#2
	BEQ	IS@D
	LDU	#AUX_PORT
	BSR	ISR_PORT
IS@D	RTS

ISR_PORT
	LDA	[AP_CTL,U]	this doesn't clear the interrupt
	BITA	#S_RDRF		receiver interrupt (checked first so RX has priority over TX)
	BNE	RXIRQ
	BITA	#S_TDRE		transmitter interrupt
	BNE	TXIRQ
* FALLTHROUGH
ISR_RET	RTS

TXIRQ	LDB	AP_TXPOU,U
	CMPB	AP_TXPIN,U
	BEQ	TXEMPTY
	LEAX	AP_TXBUF,U
	LDA	B,X
	STA	[AP_DATA,U]	clears the interrupt
	INCB
	ANDB	#%00001111	gives MOD 16
	STB	AP_TXPOU,U
	BRA	ISR_RET

TXEMPTY	LDA	IQ_RX		interrupts on receiver only
	STA	[AP_CTL,U]
	LDA	#$EA
	STA	AP_ERRNO,U
	BRA	ISR_RET

RXIRQ	LDB	AP_RXPIN,U
	INCB
	ANDB	#%00001111	gives MOD 16
	CMPB	AP_RXPOU,U

	IFDEF	RXFULL_SAFETY_NET
	BEQ	RXFULL		if the input buffer is full
	ELSE
	BEQ	ISR_RET		if the input buffer is full
	ENDC

	PSHS	B		save next_in
	INCB			check: will buffer be full after this store?
	ANDB	#%00001111
	CMPB	AP_RXPOU,U
	BNE	R@0		no room pressure -> normal
	LDA	#1		buffer will be full: deassert RTS NOW, before LDA ACIARX
	STA	AP_RXFL,U	so guest_receive() sees rts_deasserted and skips staging
	LDA	IQ_RTSOFF
	STA	[AP_CTL,U]
R@0	LDA	[AP_DATA,U]	clears the interrupt (no next-byte staged if rts_deasserted)
	LDB	AP_RXPIN,U
	LEAX	AP_RXBUF,U
	STA	B,X
	PULS	B
	STB	AP_RXPIN,U
	BRA	ISR_RET

	IFDEF	RXFULL_SAFETY_NET

RXFULL	LDA	#1		safety net: RXIRQ should have caught this earlier
	STA	AP_RXFL,U
	LDA	IQ_RTSOFF
	STA	[AP_CTL,U]	deassert RTS BEFORE LDA ACIARX so guest_receive() skips staging

	LDA	[AP_DATA,U]	clear the interrupt
	LDA	#$BE		set errno for the user
	STA	AP_ERRNO,U

* Drain TX ring buffer. Cannot spin on TDRE inside ISR: if TDRE=0, stop
* early and leave remaining bytes for OUTCH O@D (main loop) to finish.
	LEAX	AP_TXBUF,U
F@0	LDB	AP_TXPOU,U
	CMPB	AP_TXPIN,U
	BEQ	ISR_RET		TX ring buffer empty
	LDA	[AP_CTL,U]	check TDRE
	BITA	#S_TDRE
	BEQ	ISR_RET		TDRE not set: defer remaining bytes to OUTCH O@D
	LDA	B,X		get byte
	STA	[AP_DATA,U]	send it
	INCB
	ANDB	#%00001111
	STB	AP_TXPOU,U
	BRA	F@0

	ENDC

	ENDC

	ENDSECTION

	SECTION	CONSTANT

	IFNDEF	NO_IRQ

IQ_RX	FCB	C_RX_IRQ
IQ_RXTX	FCB	C_RX_IRQ|C_TX_IRQ
IQ_RTSOFF	FCB	C_RX_IRQ|C_RTS_OFF	; rx irq on, rts deasserted (pauses Pico->6809)

	ENDC

	ENDSECTION

	SECTION	BSS

CON_PORT	RMB	AP_SIZE
AUX_PORT	RMB	AP_SIZE

	IFNDEF	NO_IRQ
PORT_FLAGS	RMB	1		; bit 0 = CON initialised, bit 1 = AUX initialised
	ENDC

	ENDSECTION

	END
