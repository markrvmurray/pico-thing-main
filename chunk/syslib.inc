;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;

DEVICE	EXTERN
SHARED	EXTERN
SNIPPET	EXTERN

ACIAC	EXTERN
ACIAS	EXTERN
ACIATX	EXTERN
ACIARX	EXTERN

AUXAC	EXTERN
AUXAS	EXTERN
AUXATX	EXTERN
AUXARX	EXTERN

ATAIDE	EXTERN

TICKC	EXTERN
TICKS	EXTERN

BSSCLR	EXTERN

RSRVD	EXTERN
SWI3	EXTERN
SWI2	EXTERN
FIRQ	EXTERN
IRQ	EXTERN
SWI	EXTERN
NMI	EXTERN
RESET	EXTERN

INITIRQ	EXTERN
INITFIRQ	EXTERN

* Stack starts at the bottom of the DAT-mapped range, so the first push lands
* at $DFFF and it climbs from there.  With the DAT initialised to the identity
* list, blocks 0-6 are pages 0-6, so $0000-$DFFF is 56K of flat RAM and the
* stack is nowhere near even the largest chunk.  It also stays clear of the
* fixed window ($E000-$FDFF), where DBGMON and the test chunks' FIXED sections
* live.
STACK	EQU	$E000
