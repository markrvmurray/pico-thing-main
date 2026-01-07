	PRAGMA	cescapes

	USE	syslib.inc

Start	EXPORT

	SECTION	TEXT

Start	lds	#STACK		; Interrupts won't work without this
	IFDEF	NOT_DEF
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
	ELSE
S@0	lda	>ATAIDE+3
	sta	SHARED
	bra	S@0
	ENDC

	ENDSECTION

	END
