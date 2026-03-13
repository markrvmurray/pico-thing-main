;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
BSSCLR	EXPORT

address_bss	EXTERN
length_bss	EXTERN

	SECTION	TEXT

* Clear the bss block.
* This always works, even if there is no bss in the rest of
* the program.
* All registers are preserved.
BSSCLR	PSHS	CC,D,X
	LDX	#address_bss
	LDD	#length_bss
	BEQ	B@99
B@0	CLR	,X+
	SUBD	#1
	BNE	B@0
B@99	PULS	CC,D,X,PC

	ENDSECTION

* Make sure there is always a bss, even if it has to be an
* empty one.
	SECTION	bss

	ENDSECTION

	END
