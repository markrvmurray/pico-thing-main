;
; Copyright (c) 2025-2026 Mark R V Murray
;
; SPDX-License-Identifier: BSD-2-Clause
;
	opt	c,ct

delay	pshs	a,b
	mul
	puls	a,b,pc
d@0	lbsr	d@1
d@1	lbsr	d@2
d@2	rts
