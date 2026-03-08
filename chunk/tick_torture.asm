;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; Tick Timer Torture Test
;
; Exercises the virtual 50 Hz tick timer at $FFC8-$FFC9.
;   $FFC8 write: bit 0 = enable (1) / disable (0)
;   $FFC9 read:  bit 7 = IRQ pending; reading clears it
;
; Tests:
;   1  Enable and first tick: IRQ fires after enable
;   2  Read-to-clear: reading $FFC9 clears pending IRQ
;   3  Disable stops ticks: no IRQs after disable
;   4  Disable clears pending: pending IRQ cleared on disable
;   5  IRQ latches while CPU masked: fires on unmask
;   6  Tick rate: ~50 ticks per second (timing loop)
;   7  Re-enable after disable: ticks resume
;   8  Rapid enable/disable cycling: no stale IRQs
;   9  Long-run stability: 500 ticks, verify count
;  10  Coexistence with ACIA: both IRQ sources work
;
; SHARED block layout (visible via Pico 'st' command):
;   +0  IRQ_COUNT  (2)  tick IRQ handler invocations
;   +2  IRQ_STATUS (1)  last $FFC9 value seen by handler
;   +4  IRQ_FLAGS  (1)  bit 0 = tick handler was called
;   +5  TEST_NUM   (1)  current test number
;   +6  PASS_COUNT (1)  tests passed
;   +7  FAIL_COUNT (1)  tests failed
;   +8  ACIA_COUNT (2)  ACIA IRQ handler invocations
;   +10 ACIA_DATA  (1)  last ACIA byte received
;   +11 ACIA_FLAGS (1)  bit 0 = ACIA handler was called
;

	PRAGMA	cescapes

	USE	ascii.inc
	USE	uart.inc
	USE	syslib.inc

Start	EXPORT

; Tick timer registers (same offsets as old PTM CR13/STA)
TICK_CTRL	EQU	PTMC13		; $FFC8 write: bit 0 = enable/disable
TICK_STAT	EQU	PTMSTA		; $FFC9 read: bit 7 = IRQ pending; cleared on read

; SHARED block layout
IRQ_COUNT	EQU	SHARED+0	; 2 bytes
IRQ_STATUS	EQU	SHARED+2	; 1 byte
			; +3 spare
IRQ_FLAGS	EQU	SHARED+4	; 1 byte
TEST_NUM	EQU	SHARED+5	; 1 byte
PASS_COUNT	EQU	SHARED+6	; 1 byte
FAIL_COUNT	EQU	SHARED+7	; 1 byte
ACIA_COUNT	EQU	SHARED+8	; 2 bytes
ACIA_DATA	EQU	SHARED+10	; 1 byte
ACIA_FLAGS	EQU	SHARED+11	; 1 byte

NTESTS		EQU	10

; Approximate cycle counts for timing loops at 1 MHz:
;   LEAX -1,X = 5 cycles; BNE = 3 cycles → 8 cycles/iter
;   65536 iters = 524288 cycles ≈ 0.524 s
;   At 50 Hz: expect ~26 ticks per 65536-iter loop

	SECTION	TEXT

; ===============================================================
; Entry point
; ===============================================================

Start	LDS	#STACK
	LBSR	BSSCLR

	; zero the SHARED block
	LDX	#SHARED
	LDB	#16
ShrClr	CLR	,X+
	DECB
	BNE	ShrClr

	; disable tick timer and reset ACIA
	CLR	TICK_CTRL
	LDA	#C_RESET
	STA	UARTC
	LDA	#C_NONE
	STA	UARTC

	; install our IRQ handler
	LDX	#IRQ_DISPATCH
	LBSR	INITIRQ

	; banner
	LDX	#Banner
	LBSR	PPuts

; ---------------------------------------------------------------
; Test 1: Enable and first tick
;         Enable the timer, unmask CPU, wait for first IRQ.
; ---------------------------------------------------------------
	LDA	#1
	STA	TEST_NUM
	LDX	#T1Msg
	LBSR	PPuts

	LBSR	ClrState
	; enable tick timer
	LDA	#$01
	STA	TICK_CTRL
	; unmask CPU IRQ
	ANDCC	#$EF
	; wait for first tick with timeout
	LDX	#$0000		; 65536 iterations (~0.5 s)
T1Wait	LDA	IRQ_FLAGS
	BITA	#1
	BNE	T1Got
	LEAX	-1,X
	BNE	T1Wait
	; timeout
	ORCC	#$50
	CLR	TICK_CTRL
	LBSR	PFail
	LDX	#FailTimeout
	LBSR	PPuts
	BRA	T1Done
T1Got	ORCC	#$50
	CLR	TICK_CTRL
	; verify handler was called
	LDD	IRQ_COUNT
	BNE	T1A
	LBSR	PFail
	LDX	#FailNoCount
	LBSR	PPuts
	BRA	T1Done
T1A	; verify status had bit 7 set
	LDA	IRQ_STATUS
	BMI	T1Pass
	LBSR	PFail
	LDX	#FailNoBit7
	LBSR	PPuts
	BRA	T1Done
T1Pass	LBSR	PPass
T1Done

; ---------------------------------------------------------------
; Test 2: Read-to-clear
;         Enable timer, wait for IRQ pending in status, read to
;         clear, verify status is now zero.
; ---------------------------------------------------------------
	LDA	#2
	STA	TEST_NUM
	LDX	#T2Msg
	LBSR	PPuts

	LBSR	ClrState
	; keep CPU masked, enable timer
	ORCC	#$50
	LDA	#$01
	STA	TICK_CTRL
	; poll status until bit 7 set (timer will fire even with CPU masked)
	LDX	#$0000
T2Poll	LDA	TICK_STAT	; this READ also clears it!
	BMI	T2Set
	LEAX	-1,X
	BNE	T2Poll
	CLR	TICK_CTRL
	LBSR	PFail
	LDX	#FailTimeout
	LBSR	PPuts
	BRA	T2Done
T2Set	; bit 7 was set — the read just cleared it
	; read again — should be zero now
	LDA	TICK_STAT
	BMI	T2Fail
	; pass — status cleared by read
	CLR	TICK_CTRL
	LBSR	PPass
	BRA	T2Done
T2Fail	CLR	TICK_CTRL
	LBSR	PFail
	LDX	#FailSticky
	LBSR	PPuts
T2Done

; ---------------------------------------------------------------
; Test 3: Disable stops ticks
;         Enable timer, count some ticks, disable, burn time,
;         verify count does not increase.
; ---------------------------------------------------------------
	LDA	#3
	STA	TEST_NUM
	LDX	#T3Msg
	LBSR	PPuts

	LBSR	ClrState
	LDA	#$01
	STA	TICK_CTRL
	ANDCC	#$EF
	; wait for 5 ticks
T3W1	LDD	IRQ_COUNT
	CMPD	#5
	BLO	T3W1
	; disable
	ORCC	#$50
	CLR	TICK_CTRL
	; save count
	LDD	IRQ_COUNT
	STD	SAVED_CNT
	; burn cycles with CPU unmasked (no ticks should arrive)
	ANDCC	#$EF
	LDX	#$0000		; 65536 iters
T3Burn	LEAX	-1,X
	BNE	T3Burn
	ORCC	#$50
	; compare
	LDD	IRQ_COUNT
	CMPD	SAVED_CNT
	BEQ	T3Pass
	LBSR	PFail
	LDX	#FailStillTick
	LBSR	PPuts
	BRA	T3Done
T3Pass	LBSR	PPass
T3Done

; ---------------------------------------------------------------
; Test 4: Disable clears pending
;         Enable timer, wait for pending flag in status, disable,
;         verify status is zero.
; ---------------------------------------------------------------
	LDA	#4
	STA	TEST_NUM
	LDX	#T4Msg
	LBSR	PPuts

	LBSR	ClrState
	ORCC	#$50
	LDA	#$01
	STA	TICK_CTRL
	; poll for pending (read clears, so we need a fresh tick)
	; wait long enough for at least one tick
	LDX	#$0000
T4Wait	LEAX	-1,X
	BNE	T4Wait
	; now disable — this should clear any pending IRQ
	CLR	TICK_CTRL
	; read status — should be zero
	LDA	TICK_STAT
	BMI	T4Fail
	LBSR	PPass
	BRA	T4Done
T4Fail	LBSR	PFail
	LDX	#FailSticky
	LBSR	PPuts
T4Done

; ---------------------------------------------------------------
; Test 5: IRQ latches while CPU masked
;         Enable timer with CPU I-bit set. Verify handler does
;         NOT fire. Unmask CPU. Verify handler fires.
; ---------------------------------------------------------------
	LDA	#5
	STA	TEST_NUM
	LDX	#T5Msg
	LBSR	PPuts

	LBSR	ClrState
	ORCC	#$50
	LDA	#$01
	STA	TICK_CTRL
	; wait long enough for tick to pend (~40 ms at 50 Hz)
	LDX	#$0000
T5Wait	LEAX	-1,X
	BNE	T5Wait
	; handler must not have fired
	LDA	IRQ_FLAGS
	BITA	#1
	BEQ	T5A
	CLR	TICK_CTRL
	LBSR	PFail
	LDX	#FailMasked
	LBSR	PPuts
	BRA	T5Done
T5A	; unmask — latched IRQ should fire
	ANDCC	#$EF
	; brief spin for delivery
	LDX	#$4000
T5W2	LDA	IRQ_FLAGS
	BITA	#1
	BNE	T5Got
	LEAX	-1,X
	BNE	T5W2
	; timeout
	ORCC	#$50
	CLR	TICK_CTRL
	LBSR	PFail
	LDX	#FailNoIRQ
	LBSR	PPuts
	BRA	T5Done
T5Got	ORCC	#$50
	CLR	TICK_CTRL
	LBSR	PPass
T5Done

; ---------------------------------------------------------------
; Test 6: Tick rate
;         Enable timer, run a calibrated loop (~1 second),
;         verify tick count is in range 40-60 (nominal 50).
; ---------------------------------------------------------------
	LDA	#6
	STA	TEST_NUM
	LDX	#T6Msg
	LBSR	PPuts

	LBSR	ClrState
	LDA	#$01
	STA	TICK_CTRL
	ANDCC	#$EF
	; burn ~1 second: 2 × 65536 × 8 cycles = 1048576 cycles
	LDY	#2
T6Outer	LDX	#$0000
T6Inner	LEAX	-1,X
	BNE	T6Inner
	LEAY	-1,Y
	BNE	T6Outer
	ORCC	#$50
	CLR	TICK_CTRL
	; check tick count
	LDD	IRQ_COUNT
	; print actual count
	LDX	#T6ActStr
	LBSR	PPuts
	LDD	IRQ_COUNT
	LBSR	PUTHEX16
	LDX	#T6ExpStr
	LBSR	PPuts
	; check range 40-60
	LDD	IRQ_COUNT
	CMPD	#40
	BLO	T6Fail
	CMPD	#60
	BHI	T6Fail
	LBSR	PPass
	BRA	T6Done
T6Fail	LBSR	PFail
	LDX	#FailRate
	LBSR	PPuts
T6Done

; ---------------------------------------------------------------
; Test 7: Re-enable after disable
;         Enable, get ticks, disable, re-enable, verify ticks
;         resume.
; ---------------------------------------------------------------
	LDA	#7
	STA	TEST_NUM
	LDX	#T7Msg
	LBSR	PPuts

	LBSR	ClrState
	LDA	#$01
	STA	TICK_CTRL
	ANDCC	#$EF
	; wait for 3 ticks
T7W1	LDD	IRQ_COUNT
	CMPD	#3
	BLO	T7W1
	; disable
	ORCC	#$50
	CLR	TICK_CTRL
	; save count
	LDD	IRQ_COUNT
	STD	SAVED_CNT
	; re-enable
	LDA	#$01
	STA	TICK_CTRL
	ANDCC	#$EF
	; wait for 3 more ticks beyond saved count
	LDD	SAVED_CNT
	ADDD	#3
	STD	TARGET_CNT
T7W2	LDD	IRQ_COUNT
	CMPD	TARGET_CNT
	BLO	T7W2
	ORCC	#$50
	CLR	TICK_CTRL
	LBSR	PPass
T7Done

; ---------------------------------------------------------------
; Test 8: Rapid enable/disable cycling
;         Toggle enable 100 times with brief delays. Verify no
;         stale IRQs fire after final disable.
; ---------------------------------------------------------------
	LDA	#8
	STA	TEST_NUM
	LDX	#T8Msg
	LBSR	PPuts

	LBSR	ClrState
	ORCC	#$50
	LDB	#100
T8Loop	LDA	#$01
	STA	TICK_CTRL	; enable
	NOP
	NOP
	NOP
	NOP
	CLR	TICK_CTRL	; disable
	DECB
	BNE	T8Loop
	; save count
	LDD	IRQ_COUNT
	STD	SAVED_CNT
	; unmask and burn time — no more ticks should arrive
	ANDCC	#$EF
	LDX	#$8000
T8Burn	LEAX	-1,X
	BNE	T8Burn
	ORCC	#$50
	; compare — count should not have increased
	LDD	IRQ_COUNT
	CMPD	SAVED_CNT
	BEQ	T8Pass
	LBSR	PFail
	LDX	#FailStale
	LBSR	PPuts
	BRA	T8Done
T8Pass	LBSR	PPass
T8Done

; ---------------------------------------------------------------
; Test 9: Long-run stability (500 ticks = 10 seconds)
;         Count 500 ticks, verify exact count.
; ---------------------------------------------------------------
	LDA	#9
	STA	TEST_NUM
	LDX	#T9Msg
	LBSR	PPuts

	LBSR	ClrState
	LDA	#$01
	STA	TICK_CTRL
	ANDCC	#$EF
	; wait for 500 ticks
T9Wait	LDD	IRQ_COUNT
	CMPD	#500
	BLO	T9Wait
	ORCC	#$50
	CLR	TICK_CTRL
	; print actual count
	LDX	#T9ActStr
	LBSR	PPuts
	LDD	IRQ_COUNT
	LBSR	PUTHEX16
	LBSR	PCrlf
	; allow 500-502 (handler might catch one extra before disable)
	LDD	IRQ_COUNT
	CMPD	#500
	BLO	T9Fail
	CMPD	#505
	BHI	T9Fail
	LBSR	PPass
	BRA	T9Done
T9Fail	LBSR	PFail
	LDX	#FailLongRun
	LBSR	PPuts
T9Done

; ---------------------------------------------------------------
; Test 10: Coexistence with ACIA
;          Enable tick timer AND ACIA RX IRQ. Send a char via
;          queue loopback. Verify both devices fire IRQs.
; ---------------------------------------------------------------
	LDA	#10
	STA	TEST_NUM
	LDX	#T10Msg
	LBSR	PPuts

	LBSR	ClrState
	; enable tick timer
	LDA	#$01
	STA	TICK_CTRL
	; enable ACIA queue loopback with RX IRQ
	LDA	#C_LOOPQUEUE|C_RX_IRQ
	STA	UARTC
	; unmask
	ANDCC	#$EF
	; send a byte via loopback
	LDA	#'Z'
	STA	UARTTX
	; wait for ACIA handler
	LDX	#$0000
T10W1	LDA	ACIA_FLAGS
	BITA	#1
	BNE	T10GotA
	LEAX	-1,X
	BNE	T10W1
	; timeout on ACIA
	ORCC	#$50
	CLR	TICK_CTRL
	LDA	#C_RESET
	STA	UARTC
	LBSR	PFail
	LDX	#FailACIA
	LBSR	PPuts
	BRA	T10Done
T10GotA
	; wait for at least 1 tick IRQ too
	LDX	#$0000
T10W2	LDD	IRQ_COUNT
	BNE	T10GotT
	LEAX	-1,X
	BNE	T10W2
	; timeout on tick
	ORCC	#$50
	CLR	TICK_CTRL
	LDA	#C_RESET
	STA	UARTC
	LBSR	PFail
	LDX	#FailTimeout
	LBSR	PPuts
	BRA	T10Done
T10GotT
	ORCC	#$50
	CLR	TICK_CTRL
	LDA	#C_RESET
	STA	UARTC
	; verify ACIA got 'Z'
	LDA	ACIA_DATA
	CMPA	#'Z'
	BEQ	T10A
	LBSR	PFail
	LDX	#FailData
	LBSR	PPuts
	BRA	T10Done
T10A	; verify tick count > 0
	LDD	IRQ_COUNT
	BNE	T10Pass
	LBSR	PFail
	LDX	#FailNoCount
	LBSR	PPuts
	BRA	T10Done
T10Pass	LBSR	PPass
T10Done

; ---------------------------------------------------------------
; Summary
; ---------------------------------------------------------------
	LDA	#C_NONE
	STA	UARTC

	LDX	#Summary
	LBSR	PPuts
	LDA	PASS_COUNT
	LBSR	PPrintA
	LDA	#'/'
	LBSR	POutch
	LDA	#NTESTS
	LBSR	PPrintA
	LDX	#SDone
	LBSR	PPuts

	LDA	FAIL_COUNT
	BEQ	AllPass
	LDX	#SomeFail
	LBSR	PPuts
	BRA	Done
AllPass	LDX	#AllOK
	LBSR	PPuts
Done	BRA	Done

; ===============================================================
; IRQ handler — dispatches tick timer and ACIA
; ===============================================================
IRQ_DISPATCH
	; check tick timer first
	LDA	TICK_STAT	; read and acknowledge tick timer
	BMI	TickIRQ		; bit 7 set = tick fired
	; check ACIA
	LDA	UARTS
	BITA	#S_RDRF
	BNE	AciaIRQ
	; spurious
	RTS

TickIRQ	; status register already read (and cleared) above
	STA	IRQ_STATUS
	LDA	#1
	STA	IRQ_FLAGS
	LDD	IRQ_COUNT
	ADDD	#1
	STD	IRQ_COUNT
	; check if ACIA also needs service (shared IRQ line)
	LDA	UARTS
	BITA	#S_RDRF
	BNE	AciaIRQ
	RTS

AciaIRQ	LDA	UARTRX		; read data clears ACIA IRQ
	STA	ACIA_DATA
	LDA	#1
	STA	ACIA_FLAGS
	LDD	ACIA_COUNT
	ADDD	#1
	STD	ACIA_COUNT
	RTS

; ===============================================================
; Helper routines
; ===============================================================

; Clear all handler state
ClrState
	LDX	#SHARED
	LDB	#16
CS@	CLR	,X+
	DECB
	BNE	CS@
	RTS

; Polled output: send byte in A
POutch	PSHS	B
P@W	LDB	UARTS
	BITB	#S_TDRE
	BEQ	P@W
	STA	UARTTX
	PULS	B,PC

; Polled puts: print NUL-terminated string at X, LF becomes CR+LF
PPuts	PSHS	A,X
P@L	LDA	,X+
	BEQ	P@D
	CMPA	#LF
	BNE	P@N
	LDA	#CR
	BSR	POutch
	LDA	#LF
P@N	BSR	POutch
	BRA	P@L
P@D	PULS	A,X,PC

; Print CR+LF
PCrlf	PSHS	A
	LDA	#CR
	BSR	POutch
	LDA	#LF
	BSR	POutch
	PULS	A,PC

; Print PASS
PPass	PSHS	A,X
	LDX	#PassMsg
	LBSR	PPuts
	INC	PASS_COUNT
	PULS	A,X,PC

; Print FAIL
PFail	PSHS	A,X
	LDX	#FailMsg
	LBSR	PPuts
	INC	FAIL_COUNT
	PULS	A,X,PC

; Print A as decimal 0-99
PPrintA	PSHS	B
	LDB	#'0'-1
PP@T	INCB
	SUBA	#10
	BCC	PP@T
	ADDA	#10+'0'
	CMPB	#'0'
	BEQ	PP@U
	EXG	A,B
	LBSR	POutch
	EXG	A,B
PP@U	LBSR	POutch
	PULS	B,PC

; Print nibble in A as hex digit
PUTHEXNIB
	PSHS	CC,A
	CMPA	#10
	BLO	PHN0
	ADDA	#('A'-10)
	BRA	PHN1
PHN0	ADDA	#'0'
PHN1	LBSR	POutch
	PULS	CC,A,PC

; Print A as 2 hex digits
PUTHEX8	PSHS	CC,A,B
	LSRA
	LSRA
	LSRA
	LSRA
	ANDA	#$0F
	BSR	PUTHEXNIB
	LDA	1,S
	ANDA	#$0F
	BSR	PUTHEXNIB
	PULS	CC,A,B,PC

; Print D as 4 hex digits
PUTHEX16
	PSHS	CC,A,B
	BSR	PUTHEX8
	LDA	2,S
	BSR	PUTHEX8
	PULS	CC,A,B,PC

	ENDSECTION

; ===============================================================
; String constants
; ===============================================================

	SECTION	CONSTANT

Banner	FCC	"\nTick Timer Torture Test (50 Hz)\n"
	FCC	"===============================\n\n"
	FCB	0

T1Msg	FCC	"Test  1: Enable and first tick ........ "
	FCB	0
T2Msg	FCC	"Test  2: Read-to-clear ................ "
	FCB	0
T3Msg	FCC	"Test  3: Disable stops ticks ........... "
	FCB	0
T4Msg	FCC	"Test  4: Disable clears pending ........ "
	FCB	0
T5Msg	FCC	"Test  5: IRQ latches while CPU masked .. "
	FCB	0
T6Msg	FCC	"Test  6: Tick rate (expect ~50) ........ "
	FCB	0
T6ActStr	FCC	"0x"
	FCB	0
T6ExpStr	FCC	" "
	FCB	0
T7Msg	FCC	"Test  7: Re-enable after disable ....... "
	FCB	0
T8Msg	FCC	"Test  8: Rapid enable/disable cycling .. "
	FCB	0
T9Msg	FCC	"Test  9: Long-run stability (500 ticks)  "
	FCB	0
T9ActStr	FCC	"count=0x"
	FCB	0
T10Msg	FCC	"Test 10: Coexistence with ACIA ......... "
	FCB	0

PassMsg	FCC	"PASS\n"
	FCB	0
FailMsg	FCC	"FAIL: "
	FCB	0
FailTimeout	FCC	"timeout waiting for IRQ\n"
	FCB	0
FailNoCount	FCC	"IRQ count is zero\n"
	FCB	0
FailNoBit7	FCC	"status bit 7 not set\n"
	FCB	0
FailSticky	FCC	"status not cleared by read\n"
	FCB	0
FailStillTick	FCC	"ticks continued after disable\n"
	FCB	0
FailMasked	FCC	"IRQ fired while CPU I-bit set\n"
	FCB	0
FailNoIRQ	FCC	"IRQ did not fire after unmask\n"
	FCB	0
FailRate	FCC	"tick rate out of range 40-60\n"
	FCB	0
FailStale	FCC	"stale IRQ after disable\n"
	FCB	0
FailLongRun	FCC	"tick count out of range\n"
	FCB	0
FailACIA	FCC	"ACIA IRQ did not fire\n"
	FCB	0
FailData	FCC	"wrong ACIA data byte\n"
	FCB	0
FailNoCount2	FCC	"tick count zero during coexistence\n"
	FCB	0
Summary	FCC	"\nResults: "
	FCB	0
SDone	FCC	" passed\n"
	FCB	0
AllOK	FCC	"ALL TESTS PASSED\n"
	FCB	0
SomeFail	FCC	"*** FAILURES DETECTED ***\n"
	FCB	0

	ENDSECTION

; ===============================================================
; BSS
; ===============================================================

	SECTION	BSS

SAVED_CNT	RMB	2
TARGET_CNT	RMB	2

	ENDSECTION

	END
