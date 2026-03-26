;
; Copyright (c) 2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; Interrupt Test Chunk
;
; Tests all 6809 interrupt types by writing signature bytes
; into the SHARED buffer ($FFD0-$FFDF). The Pico monitors the
; buffer and delivers hardware interrupts (NMI, FIRQ, IRQ)
; after the software interrupts (SWI, SWI2, SWI3) complete.
;
; SHARED block layout:
;   +0  READY       (1)  6809 sets to 1 when vectors installed
;   +1  SWI_RESULT  (1)  $11 after SWI ISR runs
;   +2  SWI2_RESULT (1)  $22 after SWI2 ISR runs
;   +3  SWI3_RESULT (1)  $33 after SWI3 ISR runs
;   +4  NMI_RESULT  (1)  $AA after NMI ISR runs
;   +5  FIRQ_RESULT (1)  $BB after FIRQ ISR runs
;   +6  IRQ_RESULT  (1)  $CC after IRQ ISR runs
;   +7  DONE        (1)  6809 sets to 1 when entering final loop
;

	USE	syslib.inc

Start	EXPORT

; SHARED block layout
S_READY		EQU	SHARED+0
S_SWI		EQU	SHARED+1
S_SWI2		EQU	SHARED+2
S_SWI3		EQU	SHARED+3
S_NMI		EQU	SHARED+4
S_FIRQ		EQU	SHARED+5
S_IRQ		EQU	SHARED+6
S_DONE		EQU	SHARED+7

	SECTION	TEXT

; ===============================================================
; Entry point
; ===============================================================

Start	LDS	#STACK
	ORCC	#$FF		; mask all interrupts

	; Clear SHARED block
	LDX	#SHARED
	LDB	#16
Clr@	CLR	,X+
	DECB
	BNE	Clr@

	; Install interrupt vectors directly into CPU vector table
	LDD	#SWI3_ISR
	STD	>$FFF2		; SWI3 vector
	LDD	#SWI2_ISR
	STD	>$FFF4		; SWI2 vector
	LDD	#FIRQ_ISR
	STD	>$FFF6		; FIRQ vector
	LDD	#IRQ_ISR
	STD	>$FFF8		; IRQ vector
	LDD	#SWI_ISR
	STD	>$FFFA		; SWI vector
	LDD	#NMI_ISR
	STD	>$FFFC		; NMI vector

	; Signal ready
	LDA	#1
	STA	>S_READY

	; --- Software interrupts (self-triggered) ---

	SWI
	NOP
	NOP
	NOP
	NOP

	SWI2
	NOP
	NOP
	NOP
	NOP

	SWI3
	NOP
	NOP
	NOP
	NOP

	; --- Unmask IRQ and FIRQ for hardware interrupts ---
	ANDCC	#$AF		; clear I and F bits

	; Signal done — Pico can now deliver NMI, FIRQ, IRQ
	LDA	#1
	STA	>S_DONE

	; Infinite loop — Pico delivers interrupts during this
Done@	BRA	Done@

; ===============================================================
; Interrupt Service Routines
; ===============================================================

SWI_ISR		LDA	#$11
		STA	>S_SWI
		RTI

SWI2_ISR	LDA	#$22
		STA	>S_SWI2
		RTI

SWI3_ISR	LDA	#$33
		STA	>S_SWI3
		RTI

NMI_ISR		LDA	#$AA
		STA	>S_NMI
		RTI

FIRQ_ISR	LDA	#$BB
		STA	>S_FIRQ
		RTI

IRQ_ISR		LDA	#$CC
		STA	>S_IRQ
		RTI

	ENDSECTION

	END
