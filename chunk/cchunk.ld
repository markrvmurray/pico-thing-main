; Linker script for C programs (compiled with cmoc) that use the io.asm
; interrupt-driven ACIA library.
;
; Section ordering:
;   start      load 0130  - cmoc startup (sets stack, calls INILIB then main)
;   code                  - cmoc compiled functions
;   constructors*         - C++ constructors (empty for C)
;   destructors*          - C++ destructors  (empty for C)
;   initgl*               - cmoc global-variable initializers
;   TEXT                  - lwasm text (io.asm + syslib.asm functions)
;   rodata                - cmoc string literals and real constants
;   rwdata                - cmoc initialised global variables
;   CONSTANT              - lwasm constants (io.asm IQ_* bytes)
;   bss,bss               - cmoc BSS (zero-init at runtime by INILIB)
;   BSS                   - lwasm BSS (io.asm ring buffers, zeroed in SREC)
;   program_end           - cmoc end marker
;   SYSVECTORS load FD00  - ISR dispatch code + handler tables
;   DAT_RAM    load FE00  - DAT RAM
;   IO_DEVICES load FF00  - IDE and other I/O devices
;   PICO2      load FFC0  - ACIA / timer register addresses
;   VECTORS    load FFF0  - CPU interrupt / reset vectors

; Use the patterns that cmoc's INILIB expects for BSS start/length.
define basesympat s_%s
define lensympat l_%s

section start load 0130
section code
section constructors_start
section constructors
section constructors_end
section destructors_start
section destructors
section destructors_end
section initgl_start
section initgl
section initgl_end
section TEXT
section rodata
section rwdata
section CONSTANT
section bss,bss
section BSS
section program_end
section SYSVECTORS load FD00
section DAT_RAM load FE00
section IO_DEVICES load FF00
section PICO2 load FFC0
section VECTORS load FFF0

entry program_start
