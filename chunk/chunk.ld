entry Start
define basesympat address_%s
define lensympat length_%s
section TEXT load 0130
section CONSTANT
section BSSTART
section BSS
section BSSEND
section SYSVECTORS load FD00
section DAT_RAM load FE00
section IO_DEVICES load FF00
section PICO2 load FFC0
section VECTORS load FFF0
