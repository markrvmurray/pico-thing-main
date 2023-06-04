;
; Copyright (c) 2025 Mark R V Murray.
;
; SPDX-License-Identifier: BSD-2-Clause
;
DATRAM	EQU	$FE00	Address of DAT RAM R/W block

DEVICE	EQU	$FFC0	Base address of Pico register block
SHARED	EQU	$FFD0	Address of a shared buffer for moving bytes
START	EQU	$FFE0	Start address of code snippet
VECTORS	EQU	$FFF0	CPU reset and interrupt vectors

CHUNK	EQU	$0130

UARTC	equ	$FFC8
UARTS	equ	$FFC8
UARTTX	equ	$FFC9
UARTRX	equ	$FFC9

* Used by the Pico2 to copy up to 16 bytes of data from the 6809's
* main memory.
	ORG	START
CopyIn	LDX	#SHARED
	LDU	#$AAAA	This address will be replaced by the Pico
	LDB	#$BB	This count will be replaced by the Pico
l@0	LDA	,U+
	STA	,X+
	DECB
	BNE	l@0
	SYNC

* Used by the Pico2 to copy up to 16 bytes of data to the 6809's
* main memory.
	ORG	START
CopyOut	LDX	#SHARED
	LDU	#$AAAA	This address will be replaced by the Pico
	LDB	#$BB	This count will be replaced by the Pico
l@0	LDA	,X+
	STA	,U+
	DECB
	BNE	l@0
	SYNC

* Instant stop. Proves that bus BS_SYNC is detected and acted upon.
	ORG	START
Sync	SYNC
	SYNC
	SYNC
	SYNC
	SYNC
	SYNC
	SYNC
	SYNC
	SYNC
	SYNC
	SYNC
	SYNC
	SYNC
	SYNC
	SYNC
	SYNC

* Looks good on the oscilloscope if the Pico2's PIO SM and DMA ISR are instrumented.
	ORG	START
Reads	LDA	>SHARED
	BRA	Reads
	SYNC
	SYNC
	SYNC
	SYNC
	SYNC
	SYNC
	SYNC
	SYNC
	SYNC
	SYNC
	SYNC

* Looks good on the oscilloscope if the Pico2's PIO SM and DMA ISR are instrumented.
	ORG	START
Writes	LDX	#SHARED
	CLRA
	CLRB
w@1	ANDA	#%00001111
	STB	A,X
	INCA
	INCB
	BRA	w@1
	SYNC
	SYNC
	SYNC

	ORG	START
StackEx	LDD	#$6677
	LDX	#$5A5A
	LDU	#$9889
	LDS	#START	Stack will be in the $FFDx buffer
	PSHS	D,X,U
	SYNC

	ORG	START
DatEx	CLRA
d@0	LDX	#DATRAM
d@1	COM	,X+
	INCA
	BNE	d@1
	BRA	d@0

* Set up the SWI vector to point to the ISR1 address, then chect that
* the stack frame is populated and that the sync is hit.
	ORG	START
TstSwi	LDS	#START
	LDD	#$55AA
	LDX	#$6677
	LDU	#$33CC
	SWI
ISR1	NOP
	SYNC

* The test driver will need to prepare the NMI vector, start/observe the run,
* trigger an !NMI, then check that the stack frame is correct after a BS_IRQ
* and that the SYNC was hit. Proves that !NMI works
	ORG	START
TstNMI	LDS	#START
	LDX	#$AABB
	CLRA
	CLRB
	LDU	#$CCDD
t@0	NOP
	BRA	t@0
ISR2	SYNC

* The test driver will need to prepare the IRQ vector, start/observe the run,
* trigger an !IRQ, then check that the stack frame is correct after a BS_IRQ
* and that the SYNC was hit. Proves that !IRQ works
	ORG	START
TstIRQ	LDS	#START
	LDX	#$ABAB
	ANDCC	#%11101111
	LDU	#$CDCD
t@0	NOP
	BRA	t@0
ISR3	SYNC

* The test driver will need to prepare the FIRQ vector, start/observe the run,
* trigger a !FIRQ, then check that the stack frame is correct after a BS_IRQ
* and that the SYNC was hit. Proves that !FIRQ works
	ORG	START
TstFIRQ	LDS	#START
	ANDCC	#%10111111
	ORCC	#%00001111
t@0	NOP
	BRA	t@0
	SYNC
	SYNC
	SYNC
	SYNC
ISR4	SYNC

* The test driver will need to copy_out the chunk, set the IRQ vector to ISR5 ($0200),
* and run the chunk in interactive mode
	ORG	CHUNK
	FDB	EUrtIRQ-CHUNK
UartIRQ	LDS	#$2000
	CLR	ERRNO
	CLR	EXITC
	LDA	#$40
	STA	,X
	ANDCC	#%11101111
	LDX	HELLO
u@0	LDA	,X+
	BEQ	FINISH
	BSR	OUTCH
	BRA	u@0
FINISH	LDA	#$88
	STA	ERRNO
	SYNC
OUTCH	PSHS	B,Y
	LDB	POSIN
	INCB
	ANDB	#$0F
o@0	CMPB	POSOUT
	BEQ	o@0
	LDY	BUFFER
	STA	B,Y
	STB	POSIN
	LDB	UARTS
	ANDB	#%00011111
	ORB	#%10000000
	STB	UARTC
	PULS	B,Y,PC
	FCB	$DE,$AD,$C0,$DE
	FCB	$DE,$AD,$C0,$DE
POSIN	FCB	0
POSOUT	FCB	0
	FCB	$DE,$AD,$C0,$DE
	FCB	$DE,$AD,$C0,$DE
BUFFER	ZMB	16
	FCB	$DE,$AD,$C0,$DE
	FCB	$DE,$AD,$C0,$DE
	FILL	$42,$0200-*-8
	FCB	$DE,$AD,$C0,$DE
	FCB	$DE,$AD,$C0,$DE
ISR5	LDX	#UARTC
	LDA	,X
	BITA	#%00010000
	BEQ	NOTME
	LDA	POSOUT
	CMPA	POSIN
	BEQ	EMPTY
	INCA
	ANDA	#$0F
	STA	POSOUT
	LDY	#BUFFER
	LDB	A,Y
	STB	1,X
	BRA	DONE
NOTME	LDA	#$76
	STA	ERRNO
	BRA	DONE
EMPTY	LDA	,X
	ANDA	#%01111111
	STA	,x
DONE	RTI
	FCB	$DE,$AD,$C0,$DE
	FCB	$DE,$AD,$C0,$DE
	FCB	$DE,$AD,$C0,$DE
	FCB	$DE,$AD,$C0,$DE
ABORT	LDA	#$B1
	STA	ERRNO
	SYNC
	FCB	$DE,$AD,$C0,$DE
	FCB	$DE,$AD,$C0,$DE
HELLO	FCS	'Hello, World! '
	FCS	'Hello, World! '
	FCS	'Hello, World! '
	FCS	'Hello, World! '
	FCB	$0C,$0A,$00
	FCB	$DE,$AD,$C0,$DE
	FCB	$DE,$AD,$C0,$DE
	FCB	$DE,$AD,$C0,$DE
	FCB	$DE,$AD,$C0,$DE
ERRNO	FCB	0
EXITC	FCB	0
	FCB	$DE,$AD,$C0,$DE
	FCB	$DE,$AD,$C0,$DE
	FCB	$DE,$AD,$C0,$DE
	FCB	$DE,$AD,$C0,$DE
EUrtIRQ	EQU	*

* A prepared stack frame needs to be set up. Once the RTI runs, the return
* address must point to the Cont offset. Check that the SYNC is hit.
	ORG	START
TstRti	LDS	#START-12
	NOP
	RTI
Cont	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	NOP
	SYNC

	ORG	$0000
	FCC	"COPY_IN"
	FCB	' '
	FCC	"COPY_OUT"
	FCB	' '
	FCC	"SYNC"
	FCB	' '
	FCC	"READS"
	FCB	' '
	FCC	"WRITES"
	FCB	' '
	FCC	"STACK_EX"
	FCB	' '
	FCC	"DAT_EX"
	FCB	' '
	FCC	"TEST_SWI"
	FCB	' '
	FCC	"TEST_NMI"
	FCB	' '
	FCC	"TEST_IRQ"
	FCB	' '
	FCC	"TEST_FIRQ"
	FCB	' '
	FCC	"UART_IRQ"
	FCB	' '
	FCC	"TEST_RTI"