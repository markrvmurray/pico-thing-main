BSSCLR	EXPORT

address_BSS	EXTERN
length_BSS	EXTERN

	SECTION	TEXT

* Clear the BSS block.
* This always works, even if there is no BSS in the rest of
* the program.
* All registers are preserved.
BSSCLR	PSHS	CC,D,X
	LDX	#address_BSS
	LDD	#length_BSS
	BEQ	B@99
B@0	CLR	,X+
	SUBD	#1
	BNE	B@0
B@99	PULS	CC,D,X,PC

	ENDSECTION

* Make sure there is always a BSS, even if it has to be an
* empty one.
	SECTION	BSS

	ENDSECTION

	END
