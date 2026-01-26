;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;

	PRAGMA	cescapes

	USE	ascii.inc
	USE	uart.inc
	USE	syslib.inc

Start	EXPORT

ATABASE	equ	$FF00
ATADATA	equ	$FF00+0
ATAERR	equ	$FF00+2+1
ATAFEAT	equ	$FF00+2+1
ATASECS	equ	$FF00+4+1
ATALO	equ	$FF00+6+1
ATAMID	equ	$FF00+8+1
ATAHI	equ	$FF00+10+1
ATADRV	equ	$FF00+12+1
ATASTAT	equ	$FF00+14+1
ATACOM	equ	$FF00+14+1


	SECTION	TEXT

Start	nop
*	ldd	ATADATA
	lda	ATAERR
	sta	SHARED
	ldb	ATACOM
	stb	SHARED+1
*	std	ATADATA
*	sta	ATAERR
*	sta	ATACOM
	bra	IdeTest
	sync

	ENDSECTION

	end
