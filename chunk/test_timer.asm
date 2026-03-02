;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; test_timer.asm — MC6840 Timer 2 IRQ live test.
;
; Configures Timer 2 in CONT_0 16-bit mode at 50 Hz (20 000 E-clock cycles
; at the 1 MHz guest clock).  An IRQ handler acknowledges each tick and
; increments a 16-bit TICKS counter.  The main loop prints the hex tick
; count every 50 ticks (once per second) for 10 seconds, then stops.
;
; Timer 2 register addresses (from syslib.asm PICO2 section):
;   PTMTM2   = $FFCC — MSB latch write
;   PTMTM2+1 = $FFCD — LSB latch write (triggers initialise()), read acks IRQ
;
; CR2 = $42: e_clk=1 (bit1), irq_en=1 (bit6), mode=000 (CONT_0), 16-bit
; Period = 20 000 = $4E20 → 50 Hz at 1.0 MHz guest clock.
;
; Install the IRQ ISR via syslib INITIRQ.
;
; IRQ saves full register set (E=1, 12 bytes, 19 cycles entry, 15 cycles RTI).
; The ISR handler runs under JSR/RTS from IRQISR dispatcher.

	PRAGMA	cescapes

	USE	ascii.inc
	USE	uart.inc
	USE	syslib.inc

Start	EXPORT

; Control register 2 value: e_clk=1 (bit1), irq_en=1 (bit6), CONT_0, 16-bit.
CR2_CONT0	EQU	$42

; Timer 2 period for 50 Hz at 1 MHz: 20 000 = $4E20.
T2_PERIOD_MSB	EQU	$4E
T2_PERIOD_LSB	EQU	$20

TICKS_PER_SEC	EQU	50	; IRQ fires at 50 Hz

	SECTION	TEXT

; -----------------------------------------------------------------------
; IRQ handler — registered via INITIRQ, called by IRQISR dispatcher.
; Poll each device status register in priority order; handle the first
; active source found.  Spurious IRQ: return immediately.
;
; Extend by inserting additional BITA / BNE pairs before the final RTS.
; -----------------------------------------------------------------------
IRQ_DISPATCH
	INC	IRQ_CTR		; diagnostic: count all IRQ entries
	LDA	PTMSTA		; read PTM composite status (bit1=timer2)
	BITA	#$02		; timer 2 IRQ pending?
	BNE	TMR2_IRQ
	BRA	TkDone

; Timer 2 handler — called from IRQ_DISPATCH.
TMR2_IRQ
	LDA	PTMTM2+1	; read LSB — side-effect: clears irq2 in mc6840
	; Increment big-endian 16-bit TICKS (MSB at TICKS, LSB at TICKS+1).
	INC	TICKS+1		; bump low byte
	BNE	TkDone
	INC	TICKS		; carry into high byte
TkDone
	RTS

; -----------------------------------------------------------------------
; PUTHEXNIB — print nibble in A (0-15) as one hex digit.
; Preserves CC, A.
; -----------------------------------------------------------------------
PUTHEXNIB
	PSHS	CC,A
	CMPA	#10
	BLO	PHN0
	ADDA	#('A'-10)
	BRA	PHN1
PHN0	ADDA	#'0'
PHN1	LBSR	OUTCH
	PULS	CC,A,PC

; -----------------------------------------------------------------------
; PUTHEX8 — print A as two hex digits.
; Preserves CC, A, B.
; After PSHS CC,A,B: [S]=CC, [S+1]=A(original), [S+2]=B.
; -----------------------------------------------------------------------
PUTHEX8
	PSHS	CC,A,B
	LSRA			; shift high nibble down
	LSRA
	LSRA
	LSRA
	ANDA	#$0F
	BSR	PUTHEXNIB	; print high nibble
	LDA	1,S		; reload original A from stack
	ANDA	#$0F
	BSR	PUTHEXNIB	; print low nibble
	PULS	CC,A,B,PC

; -----------------------------------------------------------------------
; PUTHEX16 — print D as four hex digits.
; Preserves CC, A, B.
; After PSHS CC,A,B: [S]=CC, [S+1]=A(high byte), [S+2]=B(low byte).
; -----------------------------------------------------------------------
PUTHEX16
	PSHS	CC,A,B
	BSR	PUTHEX8		; print high byte (A)
	LDA	2,S		; load original B (low byte of D)
	BSR	PUTHEX8		; print low byte
	PULS	CC,A,B,PC

; -----------------------------------------------------------------------
; Main entry point.
; -----------------------------------------------------------------------
Start
	LDS	#STACK
	LBSR	BSSCLR
	LBSR	INIT

	LDX	#BannerStr
	LBSR	PUTS

	; Install IRQ dispatcher via syslib INITIRQ.
	LDX	#IRQ_DISPATCH
	LBSR	INITIRQ
	ANDCC	I		; clear I bit — enable IRQ at the CPU

	; Configure Timer 2: write CR2, then period MSB, then LSB.
	; Writing the LSB triggers mc6840::initialise(CTR2) which loads the
	; counter latch and sets running=true.
	LDA	#CR2_CONT0
	STA	PTMC2
	LDA	#T2_PERIOD_MSB
	STA	PTMTM2
	LDA	#T2_PERIOD_LSB
	STA	PTMTM2+1	; timer starts here

; -----------------------------------------------------------------------
; DIAGNOSTIC 1: Read back the timer counter immediately after init.
; Expected output: "I:4Exx" where xx is close to 0x20 (the LSB).
; Reading PTMTM2 (MSB) triggers timer.read(4) which latches the
; coherent counter snapshot; PTMTM2+1 then returns the latched LSB
; and (as a side-effect) clears any pending IRQ.
; -----------------------------------------------------------------------
	LDX	#InitStr
	LBSR	PUTS		; "I:"
	LDA	PTMTM2		; read MSB → triggers latch in mc6840
	LBSR	PUTHEX8		; print MSB (expected ~$4E)
	LDA	PTMTM2+1	; read LSB (coherent pair), clears any IRQ flag
	LBSR	PUTHEX8		; print LSB (expected ~$20)
	LBSR	PUTCRLF

; -----------------------------------------------------------------------
; DIAGNOSTIC 2: Timing loop — 65536 × 8 = 524288 cycles.
; LEAX -1,X: 5 cycles; BNE: 3 cycles → 8 cycles/iteration.
; At 1 MHz / 50 Hz timer: expect ~26 fires → TICKS ≈ 0x001A.
; If TICKS is 0 or 1: timer fires far too slowly.
; -----------------------------------------------------------------------
	LDX	#0		; X = 0 → LEAX wraps, gives 65536 iters
ZTimLoop
	LEAX	-1,X
	BNE	ZTimLoop
	LDX	#ZloopStr
	LBSR	PUTS		; "Z:"
	LDD	TICKS
	LBSR	PUTHEX16	; print TICKS (expected ~0x001A)
	LBSR	PUTCRLF

	; Set first tick boundary and iteration counter.
	LDD	#TICKS_PER_SEC
	STD	NEXTTICK
	LDA	#10
	STA	ITERS

; -----------------------------------------------------------------------
; Wait for TICKS to reach NEXTTICK, print hex count, repeat 10 times.
; WInner spins counting iterations.  When LOOPCNT overflows (~2 s) a
; diagnostic line "D:XX YYYY" is printed (IRQ_CTR, TICKS) so we can
; tell whether IRQ is firing and whether TICKS is advancing.
; -----------------------------------------------------------------------
WaitLoop
	LDD	#0
	STD	LOOPCNT		; reset counter for this wait period
WInner
	LDD	TICKS		; 16-bit read — atomic (6809 checks IRQ between insns)
	CMPD	NEXTTICK
	BCC	WPrint		; TICKS >= NEXTTICK (carry clear) → go print
	; Still waiting — count iterations
	LDD	LOOPCNT
	ADDD	#1
	STD	LOOPCNT
	BNE	WInner		; non-zero → keep spinning
	; LOOPCNT wrapped to $0000 — print diagnostic heartbeat
	LDX	#DiagStr
	LBSR	PUTS		; "D:"
	LDA	IRQ_CTR
	LBSR	PUTHEX8		; IRQ entry count as 2 hex digits
	LDA	#' '
	LBSR	OUTCH
	LDD	TICKS
	LBSR	PUTHEX16	; TICKS as 4 hex digits
	LBSR	PUTCRLF
	BRA	WInner		; keep waiting
WPrint
	LDX	#TickStr
	LBSR	PUTS		; "Tick: "
	LDD	TICKS
	LBSR	PUTHEX16	; print tick count as 4 hex digits
	LBSR	PUTCRLF

	LDD	NEXTTICK
	ADDD	#TICKS_PER_SEC
	STD	NEXTTICK

	DEC	ITERS
	BNE	WaitLoop

	; All done: stop timer and mask IRQ.
	CLR	PTMC2		; e_clk=0 stops timer
	ORCC	I		; set I bit — disable IRQ

	LDX	#DoneStr
	LBSR	PUTS

Done	BRA	Done		; spin

; -----------------------------------------------------------------------
; Constant strings
; -----------------------------------------------------------------------
	SECTION	CONSTANT

BannerStr
	FCN	"MC6840 Timer 2 IRQ test (50 Hz, 10 s)\n"
InitStr
	FCN	"I:"
ZloopStr
	FCN	"Z:"
DiagStr
	FCN	"D:"
TickStr
	FCN	"Tick: "
DoneStr
	FCN	"\nDone.\n"

; -----------------------------------------------------------------------
; BSS — zeroed by BSSCLR at startup.
; -----------------------------------------------------------------------
	SECTION	BSS

TICKS		RMB	2	; 16-bit tick counter (big-endian, ISR-maintained)
NEXTTICK	RMB	2	; tick count at which to print next
ITERS		RMB	1	; remaining iterations (counts down from 10)
IRQ_CTR		RMB	1	; diagnostic: total IRQ entries (wraps at 256)
LOOPCNT		RMB	2	; WaitLoop iteration counter (heartbeat)

	ENDSECTION

	END
