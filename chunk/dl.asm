;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; diskload.asm - Write a disk image to the PATA drive over serial.
;
; Protocol (host -> 6809, per 512-byte ATA sector):
;   SOH  LBA[23:16]  LBA[15:8]  LBA[7:0]  <512 bytes PackBits>  XOR-checksum
;   <- ACK on success, NAK on bad checksum or write error
;
; End of transfer:
;   EOT
;   <- ACK
;
; The host-side counterpart is scripts/send_disk.py.

	PRAGMA	cescapes

	USE	ascii.inc
	USE	uart.inc
	USE	syslib.inc

Start	EXPORT

; -----------------------------------------------------------------------
; Constants (SOH, EOT, ACK, NAK are in ascii.inc)
; -----------------------------------------------------------------------

; IDE task-file register offsets from ATAIDE
IDE.DATA	EQU	0	; 16-bit data register
IDE.ERROR	EQU	2
IDE.FEATURES	EQU	2
IDE.SECCNT	EQU	3
IDE.LBA0	EQU	4
IDE.LBA1	EQU	5
IDE.LBA2	EQU	6
IDE.DEVHEAD	EQU	7
IDE.STATUS	EQU	8
IDE.COMMAND	EQU	8

; IDE status bits
IDE.BSY		EQU	$80
IDE.DRQ		EQU	$08

; IDE commands
IDE.WRITE	EQU	$30

SECSIZE		EQU	512	; bytes per sector
WORDCNT		EQU	256	; 16-bit words per sector

	SECTION	TEXT

; -----------------------------------------------------------------------
; Entry point
; -----------------------------------------------------------------------

Start	LDS	#STACK
	LBSR	BSSCLR
	LBSR	INIT_CON
	LBSR	INIT_AUX

	; Print startup message on console (primary ACIA)
	LDU	#CON_PORT
	LEAX	S.HELLO,PCR
	LBSR	PUTS

	; All further I/O on auxiliary ACIA
	LDU	#AUX_PORT

	; Send ready signal
	LDA	#'R
	LBSR	OUTCH

; -----------------------------------------------------------------------
; Main loop: wait for SOH or EOT
; -----------------------------------------------------------------------

MainLoop
	BSR	RxByte		; blocking read
	CMPA	#EOT
	BEQ	Done
	CMPA	#SOH
	BNE	BadFrame

	; --- Read 3-byte LBA (big-endian) ---
	; Store in LBA+0 (bits 23:16), LBA+1 (15:8), LBA+2 (7:0)
	BSR	RxByte
	STA	LBA
	BSR	RxByte
	STA	LBA+1
	BSR	RxByte
	STA	LBA+2

	; --- Receive sector payload (PackBits RLE) ---
	LDX	#SECBUF
	LDY	#SECSIZE
	BSR	PackBits	; decode into X, Y = capacity

	; Check we got exactly 512 bytes
	CMPY	#0
	BNE	NakCont		; Y != 0 means short decode

	; --- Compute XOR checksum ---
	LDX	#SECBUF
	LDY	#SECSIZE
	CLRB			; B = running XOR
Chk@	EORB	,X+
	LEAY	-1,Y
	BNE	Chk@

	; --- Verify checksum ---
	BSR	RxByte		; expected checksum in A
	PSHS	B
	CMPA	,S+
	BNE	NakCont

	; --- IDE write sector ---
	BSR	IdeWrite
	BNE	NakCont

	; --- Success ---
	LDA	#ACK
	LBSR	OUTCH
	BRA	MainLoop

Done	LDA	#ACK
	LBSR	OUTCH
	SYNC
	BRA	Start		; restart on next run

BadFrame
	; Not SOH — send NAK and keep trying
NakCont	LDA	#NAK
	LBSR	OUTCH
	BRA	MainLoop

; -----------------------------------------------------------------------
; RxByte — blocking receive from aux ACIA
; Out: A = byte received
; -----------------------------------------------------------------------

RxByte	LBSR	INCH
	BCC	RxByte
	RTS

; -----------------------------------------------------------------------
; PackBits RLE decoder
; In:  X = destination buffer, Y = capacity (bytes)
; Out: Y = remaining capacity (0 = full sector decoded)
; Destroys: A, B, X, Y
; -----------------------------------------------------------------------

PackBits	BSR	RxByte		; control byte in A
	CMPA	#128
	BEQ	PBDone		; 128 = end marker

	TSTA
	BMI	PBRep		; >= 129 = repeat run

	; Literal run: A+1 bytes
	INCA			; count = A + 1
	TFR	A,B
PBLit	CMPY	#0
	BEQ	PBDone		; buffer full
	BSR	RxByte
	STA	,X+
	LEAY	-1,Y
	DECB
	BNE	PBLit
	BRA	PackBits

	; Repeat run: 257-A copies of next byte
	; A is 129..255; NEGA gives 256-A, INCA makes 257-A = count (2..128)
PBRep	NEGA
	INCA			; 257-A = correct repeat count
	TFR	A,B		; B = repeat count
	BSR	RxByte		; A = value to repeat
PBRpt	CMPY	#0
	BEQ	PBDone
	STA	,X+
	LEAY	-1,Y
	DECB
	BNE	PBRpt
	BRA	PackBits

PBDone	RTS

; -----------------------------------------------------------------------
; IdeWrite — write 512 bytes to IDE at LBA
; In:  LBA (3 bytes), SECBUF (512 bytes)
; Out: Z set on success, Z clear on error
; -----------------------------------------------------------------------

IdeWrite	LDX	#ATAIDE

	; Wait not busy
IWBsy1	LDA	IDE.STATUS,X
	BITA	#IDE.BSY
	BNE	IWBsy1

	; Set up LBA mode, drive 0, bits 27:24 = 0
	LDA	#$E0
	STA	IDE.DEVHEAD,X

IWBsy2	LDA	IDE.STATUS,X
	BITA	#IDE.BSY
	BNE	IWBsy2

	; Sector count = 1
	LDA	#1
	STA	IDE.SECCNT,X

	; LBA bytes
	LDA	LBA+2
	STA	IDE.LBA0,X
	LDA	LBA+1
	STA	IDE.LBA1,X
	LDA	LBA
	STA	IDE.LBA2,X

	; Issue write command
	LDA	#IDE.WRITE
	STA	IDE.COMMAND,X

	; Wait for DRQ
IWDrq	LDA	IDE.STATUS,X
	BITA	#IDE.DRQ
	BEQ	IWDrq

	; Write 256 words (512 bytes) — use U as loop counter
	LDY	#SECBUF
	PSHS	U
	LDU	#WORDCNT
IWLoop	LDD	,Y++
	STD	IDE.DATA,X
	LEAU	-1,U
	CMPU	#0
	BNE	IWLoop
	PULS	U

	; Wait not busy (write completes)
IWBsy3	LDA	IDE.STATUS,X
	BITA	#IDE.BSY
	BNE	IWBsy3

	; Return Z set (success)
	CLRA			; sets Z
	RTS

	ENDSECTION

; -----------------------------------------------------------------------
; Data
; -----------------------------------------------------------------------

	SECTION	CONSTANT

S.HELLO	FCN	"Disk imaging facility starting\n"

	ENDSECTION

	SECTION	bss

LBA	RMB	3		; 3-byte LBA (big-endian)
SECBUF	RMB	SECSIZE		; 512-byte sector buffer

	ENDSECTION

	END
