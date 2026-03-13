;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; Flat syslib for embeddable chunks.
;
; Provides the same exported symbols as syslib.asm, using the
; same section structure so addresses resolve correctly.  All
; fixed-address sections are empty (RMB 0 labels only) so no
; data is emitted at $FD00-$FFFF.
;
; Use with embed.ld + --format=raw to get a contiguous binary.

Start	EXTERN

; -----------------------------------------------------------------------
; TEXT section: stub ISR init (harmless no-ops for polled I/O)
; -----------------------------------------------------------------------

INITIRQ		EXPORT
INITFIRQ	EXPORT

	SECTION	TEXT

INITIRQ	RTS
INITFIRQ	RTS

	ENDSECTION

; -----------------------------------------------------------------------
; SYSVECTORS: empty labels only (no JMP table, no handler arrays)
; -----------------------------------------------------------------------

RSRVD	EXPORT
SWI3	EXPORT
SWI2	EXPORT
FIRQ	EXPORT
IRQ	EXPORT
SWI	EXPORT
NMI	EXPORT
RESET	EXPORT

	SECTION	SYSVECTORS

RSRVD	RMB	0
SWI3	RMB	0
SWI2	RMB	0
FIRQ	RMB	0
IRQ	RMB	0
SWI	RMB	0
NMI	RMB	0
RESET	RMB	0

	ENDSECTION

; -----------------------------------------------------------------------
; IO_DEVICES: empty label for ATAIDE base address
; -----------------------------------------------------------------------

ATAIDE	EXPORT

	SECTION	IO_DEVICES

ATAIDE	RMB	0

	ENDSECTION

; -----------------------------------------------------------------------
; PICO2: empty labels for device register addresses
; -----------------------------------------------------------------------

DEVICE	EXPORT
ACIAC	EXPORT
ACIAS	EXPORT
ACIATX	EXPORT
ACIARX	EXPORT
AUXAC	EXPORT
AUXAS	EXPORT
AUXATX	EXPORT
AUXARX	EXPORT
TICKC	EXPORT
TICKS	EXPORT
SHARED	EXPORT
SNIPPET	EXPORT

	SECTION	PICO2

DEVICE	RMB	3
ACIAC	RMB	0
ACIAS	RMB	1
ACIATX	RMB	0
ACIARX	RMB	1
AUXAC	RMB	0
AUXAS	RMB	1
AUXATX	RMB	0
AUXARX	RMB	1
	RMB	1
TICKC	RMB	1
TICKS	RMB	1
SHARED	RMB	16
SNIPPET	RMB	16

	ENDSECTION

; -----------------------------------------------------------------------
; VECTORS: empty label only
; -----------------------------------------------------------------------

VECTORS	EXPORT

	SECTION	VECTORS

VECTORS	RMB	0

	ENDSECTION

	END
