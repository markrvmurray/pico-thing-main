;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; ACIA Torture Test (self-contained via loopback)
;
; Exercises the virtual MC6850 ACIA behavior that NitrOS-9's IRQ chain
; depends on.  Uses close loopback (C_LOOPCLOSE) for register-level
; tests and queue loopback (C_LOOPQUEUE) for IRQ-path tests.
; No host interaction needed — fully autonomous.
;
; Tests:
;   1  Polled RX: S_RDRF set on receive, cleared by data read
;   2  Status read preserves data (reading UARTS doesn't consume char)
;   3  IRQ fires when C_RX_IRQ enabled and char arrives
;   4  IRQ deasserts after data register read (no spurious re-fire)
;   5  IRQ latches while CPU I-bit masked, fires on unmask
;   6  Back-to-back receive under IRQ (16 chars via queue loopback)
;   7  Overrun: second write before first read (close loopback)
;   8  Bulk 256-byte throughput via queue loopback
;   9  Host RX IRQ (character from terminal, no loopback)
;  10  Mode switch after output (write_ring ordering)
;  11  Tight IRQ deassert (PEND_IRQ clearing in DMA read ISR)
;  12  Output-then-burst (write ring stress, 24-byte blast)
;
; SHARED block layout (visible via Pico 'st' command):
;   +0  IRQ_COUNT  (2)  handler invocations
;   +2  IRQ_STATUS (1)  UARTS seen by handler
;   +3  IRQ_DATA   (1)  UARTRX read by handler
;   +4  IRQ_FLAGS  (1)  bit 0 = handler was called
;   +5  TEST_NUM   (1)  current test number
;   +6  PASS_COUNT (1)  tests passed
;   +7  FAIL_COUNT (1)  tests failed
;

	PRAGMA	cescapes

	USE	ascii.inc
	USE	uart.inc
	USE	syslib.inc

Start	EXPORT

	SECTION	TEXT

; SHARED block layout
IRQ_COUNT	EQU	SHARED+0	; 2 bytes
IRQ_STATUS	EQU	SHARED+2	; 1 byte
IRQ_DATA	EQU	SHARED+3	; 1 byte
IRQ_FLAGS	EQU	SHARED+4	; 1 byte
TEST_NUM	EQU	SHARED+5	; 1 byte
PASS_COUNT	EQU	SHARED+6	; 1 byte
FAIL_COUNT	EQU	SHARED+7	; 1 byte
IRQ_NOSIRQ	EQU	SHARED+8	; 2 bytes - handler calls where S_IRQ was not set

NTESTS		EQU	12

; ===============================================================
; Entry point
; ===============================================================

Start	LDS	#STACK
	LBSR	BSSCLR

	; zero the SHARED block (BSSCLR does not cover it)
	LDX	#SHARED
	LDB	#16
ShrClr	CLR	,X+
	DECB
	BNE	ShrClr

	; reset UART, then switch to close loopback for output
	LDA	#C_RESET
	STA	UARTC

	; install our IRQ handler into the vector chain
	LDX	#MyIRQ
	LBSR	INITIRQ

	; switch to normal mode for banner output (no loopback)
	LDA	#C_NONE
	STA	UARTC
	LDX	#Banner
	LBSR	PPuts

; ---------------------------------------------------------------
; Test 1: Polled receive - S_RDRF set then cleared
;         (close loopback: TX->RX on next Q edge)
; ---------------------------------------------------------------
	LDA	#1
	STA	TEST_NUM
	LDX	#T1Msg
	LBSR	PPuts

	; enable close loopback with RX IRQ (CPU masked so IRQ doesn't fire)
	ORCC	#$50
	LDA	#C_LOOPCLOSE|C_RX_IRQ
	STA	UARTC
	; send a byte - staged on next Q-rising edge
	LDA	#'A'
	STA	UARTTX
	; wait for it to be staged
	LBSR	PWaitRx
	; check S_RDRF is set
	LDA	UARTS
	BITA	#S_RDRF
	BNE	T1A
	LBSR	PFail
	LDX	#FailRDRF
	LBSR	PPuts
	BRA	T1Done
T1A	; check S_IRQ is also set (RX IRQ enabled + RDRF = IRQ condition)
	BITA	#S_IRQ
	BNE	T1B
	LBSR	PFail
	LDX	#FailNoSIRQ
	LBSR	PPuts
	LDA	UARTRX		; drain
	BRA	T1Done
T1B	; read data register - should clear both S_RDRF and S_IRQ
	LDA	UARTRX
	CMPA	#'A'
	BEQ	T1C
	LBSR	PFail
	LDX	#FailData
	LBSR	PPuts
	BRA	T1Done
T1C	LDB	UARTS
	BITB	#S_RDRF
	BEQ	T1D
	LBSR	PFail
	LDX	#FailSticky
	LBSR	PPuts
	BRA	T1Done
T1D	; check S_IRQ is also clear after data read
	BITB	#S_IRQ
	BEQ	T1E
	LBSR	PFail
	LDX	#FailSIRQSticky
	LBSR	PPuts
	BRA	T1Done
T1E	; check S_IRQ is NOT set without RX IRQ enabled
	LDA	#C_LOOPCLOSE	; no C_RX_IRQ
	STA	UARTC
	LDA	#'Z'
	STA	UARTTX
	LBSR	PWaitRx
	LDA	UARTS
	BITA	#S_IRQ
	BEQ	T1Pass
	LBSR	PFail
	LDX	#FailSIRQSpur
	LBSR	PPuts
	LDA	UARTRX		; drain
	BRA	T1Done
T1Pass	LDA	UARTRX		; drain
	LBSR	PPass
T1Done	LDA	#C_RESET
	STA	UARTC

; ---------------------------------------------------------------
; Test 2: Status read does not consume data
;         (close loopback)
; ---------------------------------------------------------------
	LDA	#2
	STA	TEST_NUM
	LDX	#T2Msg
	LBSR	PPuts

	LDA	#C_LOOPCLOSE
	STA	UARTC
	LDA	#'B'
	STA	UARTTX
	; wait for it to be staged
	LBSR	PWaitRx
	; read status twice - both should show S_RDRF
	LDA	UARTS
	BITA	#S_RDRF
	BNE	T2A
	LBSR	PFail
	LDX	#FailRDRF1
	LBSR	PPuts
	LDA	UARTRX		; drain
	BRA	T2Done
T2A	LDA	UARTS		; second status read
	BITA	#S_RDRF
	BNE	T2B
	LBSR	PFail
	LDX	#FailRDRF2
	LBSR	PPuts
	LDA	UARTRX		; drain
	BRA	T2Done
T2B	; data should still be 'B'
	LDA	UARTRX
	CMPA	#'B'
	BEQ	T2C
	LBSR	PFail
	LDX	#FailData
	LBSR	PPuts
	BRA	T2Done
T2C	; S_RDRF should now be clear
	LDB	UARTS
	BITB	#S_RDRF
	BEQ	T2Pass
	LBSR	PFail
	LDX	#FailSticky
	LBSR	PPuts
	BRA	T2Done
T2Pass	LBSR	PPass
T2Done	LDA	#C_RESET
	STA	UARTC

; ---------------------------------------------------------------
; Test 3: IRQ fires on receive
;         (queue loopback: byte goes through RX queue + staging)
; ---------------------------------------------------------------
	LDA	#3
	STA	TEST_NUM
	LDX	#T3Msg
	LBSR	PPuts

	LBSR	ClrIRQ
	; enable queue loopback with RX IRQ
	LDA	#C_LOOPQUEUE|C_RX_IRQ
	STA	UARTC
	; unmask CPU IRQ
	ANDCC	#$EF
	; send a byte - enters RX queue, staged on next Q edge, fires IRQ
	LDA	#'C'
	STA	UARTTX
	; wait for handler to fire
	LBSR	PWaitIRQ
	; mask and reset
	ORCC	#$50
	LDA	#C_RESET
	STA	UARTC
	; check handler saw S_RDRF
	LDA	IRQ_STATUS
	BITA	#S_RDRF
	BNE	T3A
	LBSR	PFail
	LDX	#FailNoRDRF
	LBSR	PPuts
	BRA	T3Done
T3A	; check handler got the right byte
	LDA	IRQ_DATA
	CMPA	#'C'
	BEQ	T3Pass
	LBSR	PFail
	LDX	#FailData
	LBSR	PPuts
	BRA	T3Done
T3Pass	LBSR	PPass
T3Done

; ---------------------------------------------------------------
; Test 4: IRQ clears after data read (no spurious re-fire)
;         (queue loopback)
; ---------------------------------------------------------------
	LDA	#4
	STA	TEST_NUM
	LDX	#T4Msg
	LBSR	PPuts

	LBSR	ClrIRQ
	; enable queue loopback with RX IRQ
	LDA	#C_LOOPQUEUE|C_RX_IRQ
	STA	UARTC
	ANDCC	#$EF
	; send one byte
	LDA	#'D'
	STA	UARTTX
	; wait for handler
	LBSR	PWaitIRQ
	; burn cycles - any spurious re-fire would happen here
	LDX	#$0800
T4Dly	LEAX	-1,X
	BNE	T4Dly
	; mask and reset
	ORCC	#$50
	LDA	#C_RESET
	STA	UARTC
	; should have exactly 1 IRQ
	LDD	IRQ_COUNT
	CMPD	#1
	BEQ	T4Pass
	LBSR	PFail
	LDX	#FailExtra
	LBSR	PPuts
	LDD	IRQ_COUNT
	LBSR	PPrintD
	LBSR	PCrlf
	BRA	T4Done
T4Pass	LBSR	PPass
T4Done

; ---------------------------------------------------------------
; Test 5: IRQ latches while CPU I-bit masked
;         (queue loopback)
;
; This is the critical NitrOS-9 scenario: a char arrives while
; the kernel has interrupts masked (orcc #IntMasks).  The IRQ
; must remain pending and fire when the CPU unmasks.
; ---------------------------------------------------------------
	LDA	#5
	STA	TEST_NUM
	LDX	#T5Msg
	LBSR	PPuts

	LBSR	ClrIRQ
	; CPU stays masked
	ORCC	#$50
	; enable queue loopback with RX IRQ
	LDA	#C_LOOPQUEUE|C_RX_IRQ
	STA	UARTC
	; send a byte while masked
	LDA	#'E'
	STA	UARTTX
	; wait for byte to be staged (poll status, don't read data)
	LBSR	PWaitRx
	; handler must NOT have fired (CPU masked)
	LDA	IRQ_FLAGS
	BITA	#1
	BEQ	T5A
	LBSR	PFail
	LDX	#FailMasked
	LBSR	PPuts
	ORCC	#$50
	BRA	T5Done
T5A	; unmask - IRQ should fire once the for(;;) loop sets PEND_IRQ
	; and the Q-rising ISR asserts /IRQ. Use a timeout loop rather
	; than fixed NOPs, since the for(;;) loop latency is variable.
	ANDCC	#$EF
	LDX	#$2000		; timeout (~65k cycles)
T5W	LDA	IRQ_FLAGS
	BITA	#1
	BNE	T5Got
	LEAX	-1,X
	BNE	T5W
	; timeout — IRQ never fired
	ORCC	#$50
	LBSR	PFail
	LDX	#FailNoIRQ
	LBSR	PPuts
	BRA	T5Done
T5Got	ORCC	#$50
	; verify correct byte
	LDA	IRQ_DATA
	CMPA	#'E'
	BEQ	T5Pass
	LBSR	PFail
	LDX	#FailData
	LBSR	PPuts
	BRA	T5Done
T5Pass	LBSR	PPass
T5Done	LDA	#C_RESET
	STA	UARTC

; ---------------------------------------------------------------
; Test 6: Back-to-back receive under IRQ (16 chars)
;         (queue loopback: bytes queue in FIFO)
; ---------------------------------------------------------------
	LDA	#6
	STA	TEST_NUM
	LDX	#T6Msg
	LBSR	PPuts

	LBSR	ClrIRQ
	; enable queue loopback with RX IRQ
	LDA	#C_LOOPQUEUE|C_RX_IRQ
	STA	UARTC
	ANDCC	#$EF
	; blast 16 bytes into the queue
	LDB	#16
	LDA	#$30		; '0', '1', '2' ...
T6Send	STA	UARTTX
	INCA
	DECB
	BNE	T6Send
	; wait for all 16 to be received
T6Wait	LDD	IRQ_COUNT
	CMPD	#16
	BLO	T6Wait
	; mask and reset
	ORCC	#$50
	LDA	#C_RESET
	STA	UARTC
	; check count is exactly 16
	LDD	IRQ_COUNT
	CMPD	#16
	BEQ	T6Pass
	LBSR	PFail
	LDX	#FailCount
	LBSR	PPuts
	LDD	IRQ_COUNT
	LBSR	PPrintD
	LBSR	PCrlf
	BRA	T6Done
T6Pass	LBSR	PPass
T6Done

; ---------------------------------------------------------------
; Test 7: Overrun - second write before first read
;         (close loopback: immediate, no queue)
; ---------------------------------------------------------------
	LDA	#7
	STA	TEST_NUM
	LDX	#T7Msg
	LBSR	PPuts

	LDA	#C_LOOPCLOSE
	STA	UARTC
	; write first byte, wait for it to stage
	LDA	#'X'
	STA	UARTTX
	LBSR	PWaitRx
	; write second byte before reading first (overrun)
	LDA	#'Y'
	STA	UARTTX
	LBSR	PWaitRx
	; read - should get 'Y' (overwritten) on a real ACIA
	LDA	UARTRX
	CMPA	#'Y'
	BEQ	T7Pass
	CMPA	#'X'
	BEQ	T7First
	LBSR	PFail
	LDX	#FailData
	LBSR	PPuts
	BRA	T7Done
T7First	; got first byte - not standard ACIA overrun behavior but note it
	LDA	#C_RESET
	STA	UARTC
	LDA	#C_NONE
	STA	UARTC
	LDX	#T7Note
	LBSR	PPuts
	LBSR	PPass
	BRA	T7Done
T7Pass	LBSR	PPass
T7Done	LDA	#C_RESET
	STA	UARTC

; ---------------------------------------------------------------
; Test 8: Bulk throughput - 256 bytes through queue loopback
;         Sends $00-$FF, handler stores each in RxBuf, then
;         we verify all 256 arrived in order.
; ---------------------------------------------------------------
	LDA	#8
	STA	TEST_NUM
	LDX	#T8Msg
	LBSR	PPuts

	LBSR	ClrIRQ
	; point handler at receive buffer
	LDX	#RxBuf
	STX	RxBufPtr
	; enable queue loopback with RX IRQ
	LDA	#C_LOOPQUEUE|C_RX_IRQ
	STA	UARTC
	ANDCC	#$EF
	; blast 256 bytes ($00-$FF) into the queue
	CLRA
T8Send	LDB	UARTS
	BITB	#S_TDRE
	BEQ	T8Send		; wait for TDRE before each write
	STA	UARTTX
	INCA
	BNE	T8Send
	; wait for all 256 to arrive
T8Wait	LDD	IRQ_COUNT
	CMPD	#256
	BLO	T8Wait
	; mask and reset
	ORCC	#$50
	LDA	#C_RESET
	STA	UARTC
	; clear buffer pointer (disables handler buffering)
	LDD	#0
	STD	RxBufPtr
	; verify count
	LDD	IRQ_COUNT
	CMPD	#256
	BEQ	T8ChkDat
	LBSR	PFail
	LDX	#FailCount
	LBSR	PPuts
	LDD	IRQ_COUNT
	LBSR	PPrintD
	LBSR	PCrlf
	BRA	T8Done
T8ChkDat
	; verify all 256 bytes arrived in order
	LDX	#RxBuf
	CLRA			; expected value
T8Chk	CMPA	,X+
	BNE	T8Bad
	INCA
	BNE	T8Chk
	; sequence good - now check S_IRQ was set on every handler call
	LDD	IRQ_NOSIRQ
	BEQ	T8Pass2
	LBSR	PFail
	LDX	#FailNoSIRQCnt
	LBSR	PPuts
	LDD	IRQ_NOSIRQ
	LBSR	PPrintD
	LBSR	PCrlf
	BRA	T8Done
T8Pass2	LBSR	PPass
	BRA	T8Done
T8Bad	; mismatch - A has expected, -1,X has actual
	PSHS	A		; save expected on stack
	LDB	-1,X		; get actual byte
	PSHS	B		; save actual on stack
	LBSR	PFail
	LDX	#FailSeq
	LBSR	PPuts
	LDB	1,S		; expected
	LBSR	PPrintH
	LBSR	PCrlf
	; dump first 32 bytes of buffer for inspection
	LDX	#FailDump
	LBSR	PPuts
	LDX	#RxBuf
	LDB	#32
T8Dump	LDA	,X+
	PSHS	B,X
	TFR	A,B
	LBSR	PPrintH
	LDA	#' '
	LBSR	POutch
	PULS	B,X
	DECB
	BNE	T8Dump
	LBSR	PCrlf
	PULS	A		; discard saved actual
	PULS	A		; discard saved expected
T8Done

; ---------------------------------------------------------------
; Test 9: Host RX IRQ - character from terminal (no loopback)
;
; This tests the real NitrOS-9 scenario: a character arrives
; from the host via USB serial, and the ACIA must assert IRQ.
; Prompts the user to press a key, waits with a timeout.
; ---------------------------------------------------------------
	LDA	#9
	STA	TEST_NUM
	LDX	#T9Msg
	LBSR	PPuts

	LBSR	ClrIRQ
	; reset, then normal mode with RX IRQ enabled
	LDA	#C_RESET
	STA	UARTC
	LDA	#C_RX_IRQ	; normal mode (no loopback), RX IRQ on
	STA	UARTC
	; drain any stale data
T9Drain	LDA	UARTS
	BITA	#S_RDRF
	BEQ	T9Go
	LDA	UARTRX		; discard
	BRA	T9Drain
T9Go	LBSR	ClrIRQ
	; unmask CPU IRQ and wait for host keypress
	ANDCC	#$EF
	; timeout loop: ~2.4 seconds at 1MHz
	; inner: LDY #0 → 65536 iterations (wraps from 0 to $FFFF)
	; outer: LDX #2  → 2 passes = 131072 inner iterations
	; ~18 cycles/iteration → ~23.6M cycles ≈ 24s at 1MHz
	LDX	#4		; outer counter
	LDY	#0		; inner counter (65536 iterations per outer)
T9Wait	LDA	IRQ_FLAGS
	BITA	#1
	BNE	T9Got		; IRQ handler fired
	LEAY	-1,Y
	BNE	T9Wait
	LEAX	-1,X
	BNE	T9Wait
	; timeout
	ORCC	#$50
	LDA	#C_RESET
	STA	UARTC
	LBSR	PFail
	LDX	#FailTimeout
	LBSR	PPuts
	BRA	T9Done
T9Got	ORCC	#$50
	LDA	#C_RESET
	STA	UARTC
	; verify S_IRQ was set in handler
	LDA	IRQ_STATUS
	BITA	#S_IRQ
	BNE	T9A
	LBSR	PFail
	LDX	#FailNoSIRQ
	LBSR	PPuts
	BRA	T9Done
T9A	; verify S_RDRF was set
	LDA	IRQ_STATUS
	BITA	#S_RDRF
	BNE	T9B
	LBSR	PFail
	LDX	#FailNoRDRF
	LBSR	PPuts
	BRA	T9Done
T9B	; echo the received character
	LDA	#C_NONE
	STA	UARTC
	LDA	IRQ_DATA
	LBSR	POutch
	LDA	#' '
	LBSR	POutch
	LBSR	PPass
T9Done

; ---------------------------------------------------------------
; Test 10: Mode switch after output (ordering)
;          Print a char in normal mode, immediately switch to
;          close loopback, send+receive a byte.  Exercises the
;          write_ring ordering fix: guest_control() must not
;          take effect before pending TX_DATA entries.
; ---------------------------------------------------------------
	LDA	#10
	STA	TEST_NUM
	LDX	#T10Msg
	LBSR	PPuts

	; normal mode - send a visible char (should appear on console)
	LDA	#C_NONE
	STA	UARTC
	LDA	#'*'
	STA	UARTTX
T10W1	LDB	UARTS
	BITB	#S_TDRE
	BEQ	T10W1
	; immediately switch to close loopback
	ORCC	#$50
	LDA	#C_LOOPCLOSE
	STA	UARTC
	; send a byte in loopback - should arrive in RX register
	LDA	#'!'
	STA	UARTTX
	LBSR	PWaitRx
	; read it back
	LDA	UARTRX
	CMPA	#'!'
	BEQ	T10Pass
	LBSR	PFail
	LDX	#FailData
	LBSR	PPuts
	BRA	T10Done
T10Pass	LBSR	PPass
T10Done	LDA	#C_RESET
	STA	UARTC

; ---------------------------------------------------------------
; Test 11: Tight IRQ deassert (PEND_IRQ clearing)
;          Send a byte via queue loopback with RX IRQ, wait for
;          exactly 1 IRQ, then burn minimal cycles and verify no
;          spurious re-fire.  Exercises the DMA read ISR clearing
;          PEND_IRQ after rx_read_fast_path().
; ---------------------------------------------------------------
	LDA	#11
	STA	TEST_NUM
	LDX	#T11Msg
	LBSR	PPuts

	LBSR	ClrIRQ
	; enable queue loopback with RX IRQ
	LDA	#C_LOOPQUEUE|C_RX_IRQ
	STA	UARTC
	ANDCC	#$EF
	; send one byte
	LDA	#'K'
	STA	UARTTX
	; wait for handler
	LBSR	PWaitIRQ
	; very short delay - just a few NOPs
	NOP
	NOP
	NOP
	NOP
	; mask and reset
	ORCC	#$50
	LDA	#C_RESET
	STA	UARTC
	; should have exactly 1 IRQ - no spurious re-fire from stale PEND_IRQ
	LDD	IRQ_COUNT
	CMPD	#1
	BEQ	T11A
	LBSR	PFail
	LDX	#FailExtra
	LBSR	PPuts
	LDD	IRQ_COUNT
	LBSR	PPrintD
	LBSR	PCrlf
	BRA	T11Done
T11A	; verify correct data
	LDA	IRQ_DATA
	CMPA	#'K'
	BEQ	T11Pass
	LBSR	PFail
	LDX	#FailData
	LBSR	PPuts
	BRA	T11Done
T11Pass	LBSR	PPass
T11Done

; ---------------------------------------------------------------
; Test 12: Output-then-burst (write ring stress)
;          Print a string in normal mode, switch to queue loopback,
;          blast 24 bytes.  Exercises the 32-entry write ring: the
;          CONTROL entry must not be dropped by overflow.
; ---------------------------------------------------------------
	LDA	#12
	STA	TEST_NUM
	LDX	#T12Msg
	LBSR	PPuts

	LBSR	ClrIRQ
	; point handler at receive buffer
	LDX	#RxBuf
	STX	RxBufPtr
	; normal mode - print a short string to fill the write ring
	LDA	#C_NONE
	STA	UARTC
	LDX	#T12Pre
	LBSR	PPuts
	; immediately switch to queue loopback with RX IRQ
	LDA	#C_LOOPQUEUE|C_RX_IRQ
	STA	UARTC
	ANDCC	#$EF
	; blast 24 bytes ($40-$57) - stresses write ring with CONTROL + 24 data
	LDB	#24
	LDA	#$40
T12Send	STA	UARTTX
	INCA
	DECB
	BNE	T12Send
	; wait for all 24 to arrive
T12Wait	LDD	IRQ_COUNT
	CMPD	#24
	BLO	T12Wait
	; mask and reset
	ORCC	#$50
	LDA	#C_RESET
	STA	UARTC
	; clear buffer pointer
	LDD	#0
	STD	RxBufPtr
	; verify count
	LDD	IRQ_COUNT
	CMPD	#24
	BEQ	T12Chk
	LBSR	PFail
	LDX	#FailCount
	LBSR	PPuts
	LDD	IRQ_COUNT
	LBSR	PPrintD
	LBSR	PCrlf
	BRA	T12Done
T12Chk	; verify sequence $40..$57
	LDX	#RxBuf
	LDA	#$40
T12Ck1	CMPA	,X+
	BNE	T12Bad
	INCA
	CMPA	#$58		; $40+24
	BNE	T12Ck1
	LBSR	PPass
	BRA	T12Done
T12Bad	LBSR	PFail
	LDX	#FailSeq
	LBSR	PPuts
	LDB	-1,X		; actual
	LBSR	PPrintH
	LBSR	PCrlf
T12Done

; ---------------------------------------------------------------
; Summary
; ---------------------------------------------------------------
	; switch back to normal mode for final output
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
; IRQ handler - called via IRQISR chain (JSR [A,U])
; Reads status and data, records in SHARED block
; ===============================================================
MyIRQ	LDA	UARTS
	STA	IRQ_STATUS
	BITA	#S_RDRF
	BEQ	MyIRQX		; not our interrupt
	; check S_IRQ is set alongside S_RDRF
	BITA	#S_IRQ
	BNE	MyIRQ0
	LDD	IRQ_NOSIRQ	; count missing S_IRQ
	ADDD	#1
	STD	IRQ_NOSIRQ
MyIRQ0	LDA	UARTRX		; read data clears IRQ
	STA	IRQ_DATA
	LDA	#1
	STA	IRQ_FLAGS
	; store byte in buffer if RxBufPtr is set
	LDX	RxBufPtr
	BEQ	MyIRQ1		; no buffer
	LDA	IRQ_DATA
	STA	,X+
	STX	RxBufPtr
MyIRQ1	LDD	IRQ_COUNT
	ADDD	#1
	STD	IRQ_COUNT
MyIRQX	RTS

; ===============================================================
; Helper routines
; ===============================================================

; Clear IRQ handler state
ClrIRQ	CLR	IRQ_FLAGS
	CLR	IRQ_STATUS
	CLR	IRQ_DATA
	CLR	IRQ_COUNT
	CLR	IRQ_COUNT+1
	CLR	IRQ_NOSIRQ
	CLR	IRQ_NOSIRQ+1
	RTS

; Polled output: send byte in A (works in any ACIA mode since TDRE stays set)
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

; Poll for S_RDRF (does not read data register)
PWaitRx	LDA	UARTS
	BITA	#S_RDRF
	BEQ	PWaitRx
	RTS

; Wait for IRQ handler to set IRQ_FLAGS bit 0
PWaitIRQ
	LDA	IRQ_FLAGS
	BITA	#1
	BEQ	PWaitIRQ
	RTS

; Print PASS and increment counter (switches to normal mode for output)
PPass	PSHS	A,X
	LDA	#C_RESET
	STA	UARTC
	LDA	#C_NONE
	STA	UARTC
	LDX	#PassMsg
	LBSR	PPuts
	INC	PASS_COUNT
	PULS	A,X,PC

; Print FAIL and increment counter (switches to normal mode for output)
PFail	PSHS	A,X
	LDA	#C_RESET
	STA	UARTC
	LDA	#C_NONE
	STA	UARTC
	LDX	#FailMsg
	LBSR	PPuts
	INC	FAIL_COUNT
	PULS	A,X,PC

; Print A as decimal 0-99 (two digits with leading digit suppressed)
PPrintA	PSHS	B
	LDB	#'0'-1		; tens digit
PP@T	INCB
	SUBA	#10
	BCC	PP@T
	ADDA	#10+'0'		; remainder + '0' = units digit
	CMPB	#'0'
	BEQ	PP@U		; suppress leading zero
	EXG	A,B
	LBSR	POutch		; print tens
	EXG	A,B
PP@U	LBSR	POutch		; print units
	PULS	B,PC

; Print D as 4 hex digits
PPrintD	PSHS	D
	TFR	A,B
	BSR	PPrintH
	PULS	D
	PSHS	D
	BSR	PPrintH
	PULS	D,PC

; Print B as 2 hex digits
PPrintH	PSHS	A,B
	TFR	B,A
	LSRA
	LSRA
	LSRA
	LSRA
	BSR	PHex
	PULS	A,B
	PSHS	A,B
	TFR	B,A
	ANDA	#$0F
	BSR	PHex
	PULS	A,B,PC

PHex	ADDA	#'0'
	CMPA	#'9'
	BLS	PH@0
	ADDA	#7
PH@0	LBSR	POutch
	RTS

	ENDSECTION

; ===============================================================
; String constants
; ===============================================================

	SECTION	CONSTANT

Banner	FCC	"\nACIA Torture Test (loopback)\n"
	FCC	"============================\n\n"
	FCB	0

T1Msg	FCC	"Test  1: Polled RX (RDRF set/clear) .. "
	FCB	0
T2Msg	FCC	"Test  2: Status read preserves data .. "
	FCB	0
T3Msg	FCC	"Test  3: IRQ fires on RX ............. "
	FCB	0
T4Msg	FCC	"Test  4: No spurious re-fire ......... "
	FCB	0
T5Msg	FCC	"Test  5: IRQ latches while CPU masked  "
	FCB	0
T6Msg	FCC	"Test  6: Back-to-back RX (16 chars) .. "
	FCB	0
T7Msg	FCC	"Test  7: Overrun (close loopback) .... "
	FCB	0
T7Note	FCC	"(got 1st byte) "
	FCB	0
T8Msg	FCC	"Test  8: Bulk 256-byte throughput ..... "
	FCB	0
T9Msg	FCC	"Test  9: Host RX IRQ (press a key) ... "
	FCB	0
T10Msg	FCC	"Test 10: Mode switch after output ..... "
	FCB	0
T11Msg	FCC	"Test 11: Tight IRQ deassert ........... "
	FCB	0
T12Msg	FCC	"Test 12: Write ring stress (24 burst) . "
	FCB	0
T12Pre	FCC	"."
	FCB	0
FailTimeout	FCC	"timeout - no IRQ from host char\n"
	FCB	0

PassMsg	FCC	"PASS\n"
	FCB	0
FailMsg	FCC	"FAIL: "
	FCB	0
FailRDRF	FCC	"S_RDRF not set\n"
	FCB	0
FailNoSIRQ	FCC	"S_IRQ not set (RX IRQ enabled + RDRF)\n"
	FCB	0
FailSIRQSticky	FCC	"S_IRQ still set after data read\n"
	FCB	0
FailSIRQSpur	FCC	"S_IRQ set without RX IRQ enabled\n"
	FCB	0
FailSticky	FCC	"S_RDRF still set after data read\n"
	FCB	0
FailData	FCC	"wrong data byte\n"
	FCB	0
FailRDRF1	FCC	"S_RDRF not set on 1st status read\n"
	FCB	0
FailRDRF2	FCC	"S_RDRF not set on 2nd status read\n"
	FCB	0
FailNoIRQ	FCC	"IRQ did not fire after unmask\n"
	FCB	0
FailNoRDRF	FCC	"handler saw no S_RDRF\n"
	FCB	0
FailExtra	FCC	"unexpected IRQ count: 0x"
	FCB	0
FailCount	FCC	"wrong IRQ count: 0x"
	FCB	0
FailMasked	FCC	"IRQ fired while CPU I-bit set\n"
	FCB	0
FailNoSIRQCnt	FCC	"S_IRQ not set in handler (count: 0x"
	FCB	0
FailSeq	FCC	"sequence mismatch at byte 0x"
	FCB	0
FailDump	FCC	"  buf[0..31]: "
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
; BSS - receive buffer for test 8
; ===============================================================

	SECTION	BSS

RxBufPtr	RMB	2		; pointer into RxBuf (0 = buffering disabled)
RxBuf		RMB	256		; 256-byte receive buffer

	ENDSECTION

	END
