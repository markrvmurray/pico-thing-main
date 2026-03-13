;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
; Entry trampoline for embeddable chunks.
;
; The Pico firmware sets the reset vector to $0130 (CHUNK_ADDRESS).
; Library code (io_noirq, bss, etc.) may be placed before the
; program's Start label.  This trampoline sits at the front of the
; binary and jumps to the real entry point.

Start	EXTERN

	SECTION	STARTUP

	JMP	>Start

	ENDSECTION

	END
