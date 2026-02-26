;
; cstart.asm  - Entry-point alias for C programs built with cmoc.
;
; cmoc generates 'program_start' as the top of the start section.
; syslib.asm's SYSVECTORS section expects 'Start' for the RESET vector.
; This tiny shim routes RESET → program_start.
;

program_start	EXTERN

		SECTION	TEXT
Start		EXPORT
Start		JMP	>program_start
		ENDSECTION
