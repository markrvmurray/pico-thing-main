	PRAGMA	cescapes

NUL	EQU	$00
EOT	EQU	$04
BEL	EQU	$07
BS	EQU	$08
TAB	EQU	$09
LF	EQU	$0A
VT	EQU	$0B
FF	EQU	$0C
CR	EQU	$0D
ESC	EQU	$1B
DEL	EQU	$FF

DEVICE	EQU	$FFC0	Base address of Pico register block
SHARED	EQU	$FFD0	Address of a shared buffer for moving bytes
START	EQU	$FFE0	Start address of code snippet
VECTORS	EQU	$FFF0	CPU reset and interrupt vectors

CHUNK	EQU	$0130

UARTC	equ	$FFC3
UARTS	equ	$FFC3
UARTTX	equ	$FFC4
UARTRX	equ	$FFC4

	ORG	CHUNK
	FDB	EUrtIRQ-CHUNK
UartIRQ	LDS	#$8000
	CLR	ERRNO
	CLR	UART
	LDA	#$22
	STA	UARTC
	LDX	#LONG
	BSR	PUTS
	LDA	#$88
	STA	ERRNO
	LDD	#$8000
	BSR	DELAY
	LDX	#SHORT
	BSR	PUTS
	LDA	#$98
	STA	ERRNO
	LDD	#$8000
	BSR	DELAY
	SYNC

DELAY	PSHS	CC,D
D@0	LBSR	D@1
	SUBD	#1
	BNE	D@0
	PULS	CC,D,PC
D@1	LBSR	D@2
D@2	LBSR	D@3
D@3	RTS

* Print a NUL-terminated string to the console.
* In: X points to the string.
* Out: X points to the byte after the NUL terminator.
PUTS	PSHS	CC,A
P@0	LDA	,X+
	BEQ	P@1
	BSR	OUTCH
	BRA	P@0
P@1	PULS	CC,A,PC

* Get a NUL-terminated string from the console. The CR is not stored.
* In: X points to the input buffer.
* Out: Y points to the NUL terminator. Y-X gives strlen()
GETS	PSHS	CC,A
	TFR	X,Y
G@0	BSR	INCH
	CMPA	#CR
	BEQ	G@1
	STA	,Y+
	BRA	G@0
G@1	CLR	,Y+
	PULS	CC,A,PC

* [ ] TX_P_IN == TX_P_OU => ISR: buffer empty
* [ ] TX_P_IN != TX_P_OU => ISR: next byte to transfer is at TX_P_OU, then TX_P_OU+
* [*] TX_P_IN+ == TX_P_OU => OUTCH: buffer full, so spinwait
* [*] TX_P_IN+ != TX_P_OU => OUTCH: Store byte in TX_P_IN, then TX_P_IN+

OUTCH	PSHS	CC,A,B,U
	LDU	#TX_BUF
	LDB	TX_P_IN
	INCB
	ANDB	#%00001111
O@0	CMPB	TX_P_OU
	BEQ	O@0
	PSHS	B
	LDB	TX_P_IN
	STA	B,U
	PULS	B
	STB	TX_P_IN
	LDA	#%10000000
	STA	UARTC
	PULS	CC
	ANDCC	#%11101111
	PULS	A,B,U,PC

* [ ] RX_P_IN+ == RX_P_OU => ISR: buffer full
* [ ] RX_P_IN+ != RX_P_OU => ISR: Store byte in RX_P_IN, then RX_P_IN+
* [*] RX_P_IN == RX_P_OU => INCH: buffer empty, so return "not available" status
* [*] RX_P_IN != RX_P_OU => INCH: next byte to return is at RX_P_OU, then RX_P_OU+

INCH	PSHS	B,U
	LDB	RX_P_OU
	CMPB	RX_P_IN
	BEQ	I@1		Branch if buffer empty
	LDU	#RX_BUF
	LDA	B,U		Get the byte from the buffer
	INCB
	ANDB	#%00001111
	STB	RX_P_OU
	ANDCC	#%11111110	Cleared carry means success
	PULS	B,U,PC
I@1	CLRA			NULL for no byte
	ORCC	#%00000001	Carry set marks the status
	PULS	B,U,PC

	FILL	$FF,$01E0-*

TX_P_IN	FCB	0
TX_P_OU	FCB	0

	FILL	$FF,$01E8-*

RX_P_IN	FCB	0
RX_P_OU	FCB	0

	FILL	$FF,$0200-*

TX_BUF	ZMB	16

	FILL	$FF,$0220-*

RX_BUF	ZMB	16

	FILL	$FF,$0240-*

* =====================================================================
* ISR For UART charater input/output
*
* [*] TX_P_IN == TX_P_OU => ISR: buffer empty, so come back later
* [*] TX_P_IN != TX_P_OU => ISR: next byte to transfer is at TX_P_OU, then TX_P_OU+
* [ ] TX_P_IN+ == TX_P_OU => OUTCH: buffer full, so spinwait
* [ ] TX_P_IN+ != TX_P_OU => OUTCH: Store byte in TX_P_IN, then TX_P_IN+
*
* [*] RX_P_IN+ == RX_P_OU => ISR: buffer full, so disable interrupts and do not read UARTRX register
* [*] RX_P_IN+ != RX_P_OU => ISR: Store byte in RX_P_IN, then RX_P_IN+
* [ ] RX_P_IN == RX_P_OU => INCH: buffer empty, so return "not available" status
* [ ] RX_P_IN != RX_P_OU => INCH: next byte to return is at RX_P_OU, then RX_P_OU+

ISR	LDA	UARTS
	BITA	#%00000010
	BNE	TXIRQ		Tx Interrupt
	BITA	#%00000001
	BNE	RXIRQ		Rx Interrupt
	BRA	NOT_ME

TXIRQ	LDU	#TX_BUF
	LDB	TX_P_OU
	CMPB	TX_P_IN
	BEQ	TXEMPTY
	LDA	B,U
	STA	UARTTX
	INCB
	ANDB	#%00001111
	STB	TX_P_OU
	BRA	ISR

TXEMPTY	LDA	UARTS
	ANDA	#%01111111
	STA	UARTC
	LDA	#$27
	STA	ERRNO
	RTI

RXIRQ	LDU	#RX_BUF
	LDB	RX_P_IN
	INCB
	ANDB	#%00001111
	CMPB	RX_P_OU
	BEQ	RXFULL
	PSHS	B
	LDB	RX_P_IN
	LDA	UARTRX
	STA	B,U
	PULS	B
	STB	RX_P_IN
	BRA	ISR

RXFULL	LDA	UARTS
	ANDA	#%11110111
	STA	UARTC
	RTI

NOT_ME	STA	UART
	LDA	#$76
	STA	ERRNO
	RTI

	FILL 	$FF,$02C0-*

SHORT	FCN	"Foo!\r\n"
LONG	FCC	"Hello, World! 1\r\n"
	FCC	"Hello, World! 2\r\n"
	FCC	"Hello, World! 3\r\n"
	FCN	"Hello, World! 4\r\n"

	FILL 	$FF,$0330-*

ERRNO	FCB	0
UART	FCB	0

	FILL 	$FF,$03A0-*

EUrtIRQ	EQU	*

	ORG	$FFF8
	FDB	ISR

	END	UartIRQ
