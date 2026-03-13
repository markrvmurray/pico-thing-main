; Linker script for embeddable chunks (INCLUDEBIN in api.asm).
;
; Produces a contiguous binary from $0130.  The fixed-address
; sections are present (so device address symbols resolve) but
; syslib_flat.asm keeps them empty, so --format=raw output
; contains only the contiguous code region.

entry Start
define basesympat address_%s
define lensympat length_%s
section STARTUP load 0130
section TEXT
section CONSTANT
section BSSTART
section bss
section BSSEND
section SYSVECTORS load FD00
section IO_DEVICES load FF00
section PICO2 load FFC0
section VECTORS load FFF0
