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

TASK	EQU	DEVICE+$00

UARTC	equ	$FFC3
UARTS	equ	$FFC3
UARTTX	equ	$FFC4
UARTRX	equ	$FFC4

AUXC	equ	$FFC5
AUXS	equ	$FFC5
AUXTX	equ	$FFC6
AUXRX	equ	$FFC6

* Put these in a dedicated file!
NUL	equ	$00
LF	equ	$0A
CR	equ	$0D
ESC	equ	$1B

* Initialise the DAT RAM. Essential for anything outside the Pico
* Do this automatically?
DATInit	ORG	START
	CLR	TASK
	LDX	#DATRAM
	CLRA
D@0	STA	,X+
	INCA
	BNE	D@0
	SYNC

* Zero all processor registers
Zero	ORG	START
	LDD	#$0000
	ANDCC	#$00
	ORCC	I,F
	TFR	D,X
	TFR	D,Y
	TFR	D,U
	TFR	D,S
	SYNC

* Fill a block of memory with a constant byte
Fill	ORG	START
	LDA	#$AA	This value will be replaced by the Pico
	LDX	#$BBBB	This starting address will be replaced by the Pico
F@0	STA	,X+
	CMPX	#$CCCC	This terminal address will be replaced by the Pico
	BNE	F@0
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
Loop	LDA	>SHARED
	BRA	Loop
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

* Set up the SWI vector to point to the ISR1 address, then check that
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
	NOP
t@0	BRA	t@0
ISR2	SYNC

* The test driver will need to prepare the IRQ vector, start/observe the run,
* trigger an !IRQ, then check that the stack frame is correct after a BS_IRQ
* and that the SYNC was hit. Proves that !IRQ works
	ORG	START
TstIRQ	LDS	#START
	LDX	#$ABAB
	LDU	#$CDCD
	ANDCC	I
	NOP
t@0	BRA	t@0
ISR3	SYNC

* The test driver will need to prepare the FIRQ vector, start/observe the run,
* trigger a !FIRQ, then check that the stack frame is correct after a BS_IRQ
* and that the SYNC was hit. Proves that !FIRQ works
	ORG	START
TstFIRQ	LDS	#START
	ORCC	N,Z,V,C
	ANDCC	F
	NOP
	NOP
	NOP
	NOP
	NOP
t@0	BRA	t@0
ISR4	SYNC

* A prepared stack frame needs to be set up. Once the RTI runs, the return
* address must point to the Cont offset. SWI is used to dump the registers
* back onto the stack, then we sync.
	ORG	START
TstRti	LDS	#START-12
	NOP
	RTI
Cont	SWI
	NOP
	SYNC
RtiSwi	NOP
	RTI
	NOP
	NOP
	NOP
t@0	BRA	t@0

* NMI break target: infinite loop. The NMI vector is pointed here
* by the Pico's 'break' console command. The 6809 pushes its full
* register set onto the stack and spins. The pico can break the
* spin by setting the second-last byte to 0.
	ORG	START
Break	CLRA
	LDX	#SHARED
B@0	LDB	A,S
	STB	A,X
	INCA
	CMPA	#12	all registers on stack after NMI/IRQ/SWI
	BLT	B@0
B@1	BRA	B@1
	RTI

	ORG	CHUNK
	INCLUDEBIN	download

	ORG	CHUNK
	INCLUDEBIN	rel_picothing

********************************************************************
* dbgmon - Debug monitor routines for Pico-Thing
*
* Resides at $FC00-$FDFF in the kernel page (always mapped).
* Callable from anywhere via JSR to fixed entry points.
*
* Entry points (jump table):
*   $FC00  PrintChar  - print character in A
*   $FC03  Print2Hex  - print byte in A as 2 hex digits
*   $FC06  Print4Hex  - print D (A=hi, B=lo) as 4 hex digits
*   $FC09  PrintStr   - print null-terminated string at X
*   $FC0C  PrintCR    - print CR+LF
*   $FC0F  PrintRegs  - dump registers to ACIA
*   $FC12  IllegalOp  - 6309 illegal opcode trap handler (does not return)
*
* All routines preserve all registers unless noted.

                    org       $FC00

DAT.Task            equ       $FFC0
ACIA.Ctrl           equ       $FFC3
ACIA.Data           equ       $FFC4

********************************************************************
* Jump table - fixed entry points
*
JT.PChr             jmp       PrintChar
JT.P2Hx             jmp       Print2Hex
JT.P4Hx             jmp       Print4Hex
JT.PStr             jmp       PrintStr
JT.PCR              jmp       PrintCR
JT.PReg             jmp       PrintRegs
JT.IlOp             jmp       IllegalOp

Version             fcb       1,0,0

********************************************************************
* PrintChar - print character in A to ACIA
*
* Entry: A = character to print
* Exit:  all registers preserved
*
PrintChar           pshs      cc,a
PChr.Wait           lda       >ACIA.Ctrl read ACIA status
                    bita      #$02      TDRE set?
                    beq       PChr.Wait no, wait
                    lda       1,s       reload original A from stack
                    sta       >ACIA.Data send character
                    puls      cc,a,pc   restore CC and A and return

********************************************************************
* Print2Hex - print byte in A as 2 hex digits
*
* Entry: A = byte to print
* Exit:  all registers preserved
*
Print2Hex           pshs      cc,a
                    lsra                high nybble first
                    lsra
                    lsra
                    lsra
                    bsr       PNybble   print high nybble
                    lda       1,s       reload original A
                    bsr       PNybble   print low nybble
                    puls      cc,a,pc   restore CC and A and return

PNybble             anda      #$0F
                    adda      #'0
                    cmpa      #'9
                    bls       PNyb.Go
                    adda      #7        adjust for A-F
PNyb.Go             bra       PrintChar

********************************************************************
* Print4Hex - print 16-bit value in D as 4 hex digits
*
* Entry: D = 16-bit value (A=high byte, B=low byte)
* Exit:  all registers preserved
*
Print4Hex           pshs      cc,d
                    bsr       Print2Hex print high byte in A
                    tfr       b,a
                    bsr       Print2Hex print low byte (was in B)
                    puls      cc,d,pc   restore CC, D, and return

********************************************************************
* PrintStr - print null-terminated string
*
* Entry: X = pointer to null-terminated string
* Exit:  all registers preserved
*
PrintStr            pshs      cc,a,x
PStr.Loop           lda       ,x+
                    beq       PStr.Done
                    bsr       PrintChar
                    bra       PStr.Loop
PStr.Done           puls      cc,a,x,pc

********************************************************************
* PrintCR - print CR+LF
*
* Exit:  all registers preserved
*
PrintCR             pshs      cc,a
                    lda       #$0D
                    bsr       PrintChar
                    lda       #$0A
                    bsr       PrintChar
                    puls      cc,a,pc

********************************************************************
* PrintRegs - dump all registers to ACIA
*
* Prints:
* CC = xx   A = xx   B = xx   DP = xx
* X = xxxx   Y = xxxx   U = xxxx   S = xxxx

*
* The caller's registers are captured on entry via pshs.
* S printed is the value BEFORE the pshs (adjusted).
*
PrintRegs           pshs      cc,a,b,dp,x,y,u save all registers
* stack frame: 0=CC 1=A 2=B 3=DP 4-5=X 6-7=Y 8-9=U (10 bytes)
                    leax      str.CC,pcr
                    bsr       PrintStr
                    lda       0,s       CC
                    bsr       Print2Hex
                    leax      str.A,pcr
                    bsr       PrintStr
                    lda       1,s       A
                    bsr       Print2Hex
                    leax      str.B,pcr
                    bsr       PrintStr
                    lda       2,s       B
                    bsr       Print2Hex
                    leax      str.DP,pcr
                    bsr       PrintStr
                    lda       3,s       DP
                    bsr       Print2Hex
                    bsr       PrintCR
                    leax      str.X,pcr
                    bsr       PrintStr
                    lda       4,s       X high
                    ldb       5,s       X low
                    bsr       Print4Hex
                    leax      str.Y,pcr
                    bsr       PrintStr
                    lda       6,s       Y high
                    ldb       7,s       Y low
                    bsr       Print4Hex
                    leax      str.U,pcr
                    bsr       PrintStr
                    lda       8,s       U high
                    ldb       9,s       U low
                    bsr       Print4Hex
                    leax      str.S,pcr
                    bsr       PrintStr
                    tfr       s,d
                    addd      #10       adjust for pshs frame
                    bsr       Print4Hex
                    bsr       PrintCR
                    puls      cc,a,b,dp,x,y,u,pc

str.CC              fcn       "CC = "
str.A               fcn       "   A = "
str.B               fcn       "   B = "
str.DP              fcn       "   DP = "
str.X               fcn       "X = "
str.Y               fcn       "   Y = "
str.U               fcn       "   U = "
str.S               fcn       "   S = "

********************************************************************
* IllegalOp - 6309 illegal opcode / division-by-zero trap handler
*
* The 6309 vectors through $FFF0 on illegal opcode or /0.
* It pushes the entire register set (same as SWI):
*   S+0=CC S+1=A S+2=B S+3=DP S+4-5=X S+6-7=Y S+8-9=U S+10-11=PC
* PC points to the illegal opcode itself.
*
* Prints banner + all stacked registers + task register, then halts.
*
IllegalOp           orcc      i,f       mask interrupts
                    lbsr      PrintCR
                    leax      str.ILL,pcr
                    lbsr      PrintStr
* print stacked PC (most useful info)
                    leax      str.PC,pcr
                    lbsr      PrintStr
                    lda       10,s      PC high
                    ldb       11,s      PC low
                    lbsr      Print4Hex
                    lbsr      PrintCR
* dump stacked registers
                    leax      str.CC,pcr
                    lbsr      PrintStr
                    lda       0,s       CC
                    lbsr      Print2Hex
                    leax      str.A,pcr
                    lbsr      PrintStr
                    lda       1,s       A
                    lbsr      Print2Hex
                    leax      str.B,pcr
                    lbsr      PrintStr
                    lda       2,s       B
                    lbsr      Print2Hex
                    leax      str.DP,pcr
                    lbsr      PrintStr
                    lda       3,s       DP
                    lbsr      Print2Hex
                    lbsr      PrintCR
                    leax      str.X,pcr
                    lbsr      PrintStr
                    lda       4,s       X high
                    ldb       5,s       X low
                    lbsr      Print4Hex
                    leax      str.Y,pcr
                    lbsr      PrintStr
                    lda       6,s       Y high
                    ldb       7,s       Y low
                    lbsr      Print4Hex
                    leax      str.U,pcr
                    lbsr      PrintStr
                    lda       8,s       U high
                    ldb       9,s       U low
                    lbsr      Print4Hex
                    leax      str.S,pcr
                    lbsr      PrintStr
                    tfr       s,d
                    addd      #12       adjust for exception frame
                    lbsr      Print4Hex
                    lbsr      PrintCR
* print current task register
                    leax      str.TK,pcr
                    lbsr      PrintStr
                    lda       >DAT.Task read task register
                    lbsr      Print2Hex
                    lbsr      PrintCR
* halt forever
ILL.Halt            bra       ILL.Halt

str.ILL             fcn       "*** ILLEGAL OP ***"
str.PC              fcn       "  PC = "
str.TK              fcn       "  TK = "

                    fill      $FF,$FE00-* pad to end of region

	ORG	$0000
	FCC	"DAT_INIT"
	FCB	' '
	FCC	"ZERO"
	FCB	' '
	FCC	"BLOCK_FILL"
	FCB	' '
	FCC	"COPY_OUT"
	FCB	' '
	FCC	"COPY_IN"
	FCB	' '
	FCC	"SYNC"
	FCB	' '
	FCC	"LOOP_RW"
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
	FCC	"TEST_RTI"
	FCB	' '
	FCC	"BREAK"
	FCB	' '
	FCC	"DOWNLOAD"
	FCB	' '
	FCC	"NITROS9"
	FCB	' '
	FCC	"DBGMON"
