	PRAGMA	cescapes

	USE	syslib.inc

Start	EXPORT
IRQ_ISR	EXPORT

	SECTION	TEXT

Start	lds	#$8000		; Interrupts won't work without this
S@0	clr	SHARED
	clr	SHARED+1
	clr	SHARED+2
	clr	SHARED+3
S@1	inc	SHARED+3
	bne	S@1
	inc	SHARED+2
	bne	S@1
	inc	SHARED+1
	bne	S@1
	inc	SHARED
	bne	S@1
	bra	S@0

IRQ_ISR	rti

	ENDSECTION

	END
