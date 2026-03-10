# Path to the cmoc standard library (derived from the cmoc binary location).
CMOC_LIBDIR := $(patsubst %/bin/cmoc,%/share/cmoc/lib,$(shell which cmoc))
# Libraries required by all cmoc-compiled programs.
CMOC_LIBS   := -lcmoc-crt-void -lcmoc-std-void \
               $(CMOC_LIBDIR)/float-ctor.mc6839_o -lcmoc-float-mc6839
# Common cmoc compilation flags.
CMOC_FLAGS  := --void-target --org=0x0130 --initial-s=0xf000 --mc6839

# Common .inc dependencies for assembly sources that USE them.
INCS := ascii.inc uart.inc syslib.inc

all: assist09.s19 assist09.bin \
     get.s19 get.bin put.s19 put.bin loop.s19 loop.bin \
     test_echo.s19 test_echo.bin test_echo_fast.s19 test_echo_fast.bin \
     test_send.s19 test_send.bin \
     acia_torture.s19 acia_torture.bin tick_torture.s19 tick_torture.bin \
     hello.srec hello.raw disk_test.srec disk_test.raw \
     diskload.srec diskload.raw dat_torture.srec dat_torture.raw

# -----------------------------------------------------------------------
# Libraries
# -----------------------------------------------------------------------

# ASM runtime library (IRQ-driven I/O).
CHUNK_LIB_OBJS := io.obj bss.obj syslib.obj putdec.obj puthex.obj
libchunk.a: $(CHUNK_LIB_OBJS)
	lwar -c $@ $^

# C runtime library (polling I/O).
CCHUNK_LIB_OBJS := io_noirq.obj syslib.obj cstart.obj
libcchunk.a: $(CCHUNK_LIB_OBJS)
	lwar -c $@ $^

# -----------------------------------------------------------------------
# Pattern rules
# -----------------------------------------------------------------------

# Assembly source → object file.
%.obj: %.asm $(INCS)
	lwasm --format=obj --list=$(basename $@).lst --output=$@ $<

# ASM program → s19 (linked against libchunk.a).
%.s19: %.obj libchunk.a
	lwlink --script=chunk.ld --map=$(basename $@).map --format=srec --output=$@ $< -L. -lchunk

# ASM program → bin (linked against libchunk.a).
%.bin: %.obj libchunk.a
	lwlink --script=chunk.ld --map=$(basename $@).map --format=decb --output=$@ $< -L. -lchunk

# C source → object file.
%.o: %.c pico.h
	cmoc $(CMOC_FLAGS) -c $<

# -----------------------------------------------------------------------
# Explicit rules (exceptions to the patterns above)
# -----------------------------------------------------------------------

# io_noirq.obj is io.asm assembled with NO_IRQ defined.
io_noirq.obj: io.asm $(INCS)
	lwasm -DNO_IRQ --format=obj --list=$(basename $@).lst --output=$@ $<

# cstart.obj has no .inc dependencies and no listing.
cstart.obj: cstart.asm
	lwasm --format=obj --output=$@ $<

# assist09 is assembled directly (no separate .obj step).
assist09.s19: assist09.asm
	lwasm --list=$(basename $@).lst --map=$(basename $@).map --format=srec --output=$@ $^

assist09.bin: assist09.asm
	lwasm --list=$(basename $@).lst --map=$(basename $@).map --format=decb --output=$@ $^

# Extra header dependencies for C objects.
diskload.o: ide_lib.h
ide_lib.o: ide_lib.h
disk_test.o: ide_lib.h
dat_torture.o: ide_lib.h

# -----------------------------------------------------------------------
# C program targets (linked against libcchunk.a)
# -----------------------------------------------------------------------

hello.srec: hello.o pico.o libcchunk.a
	lwlink --script=cchunk.ld --map=$(basename $@).map --format=srec \
		--output=$@ hello.o pico.o -L. -lcchunk -L$(CMOC_LIBDIR) $(CMOC_LIBS)

hello.raw: hello.o pico.o libcchunk.a
	lwlink --script=cchunk.ld --map=$(basename $@).map --format=raw \
		--output=$@ hello.o pico.o -L. -lcchunk -L$(CMOC_LIBDIR) $(CMOC_LIBS)

disk_test.srec: disk_test.o pico.o ide_lib.o libcchunk.a
	lwlink --script=cchunk.ld --map=$(basename $@).map --format=srec \
		--output=$@ disk_test.o pico.o ide_lib.o -L. -lcchunk -L$(CMOC_LIBDIR) $(CMOC_LIBS)

disk_test.raw: disk_test.o pico.o ide_lib.o libcchunk.a
	lwlink --script=cchunk.ld --map=$(basename $@).map --format=raw \
		--output=$@ disk_test.o pico.o ide_lib.o -L. -lcchunk -L$(CMOC_LIBDIR) $(CMOC_LIBS)

diskload.srec: diskload.o pico.o ide_lib.o libcchunk.a
	lwlink --script=cchunk.ld --map=$(basename $@).map --format=srec \
		--output=$@ diskload.o pico.o ide_lib.o -L. -lcchunk -L$(CMOC_LIBDIR) $(CMOC_LIBS)

diskload.raw: diskload.o pico.o ide_lib.o libcchunk.a
	lwlink --script=cchunk.ld --map=$(basename $@).map --format=raw \
		--output=$@ diskload.o pico.o ide_lib.o -L. -lcchunk -L$(CMOC_LIBDIR) $(CMOC_LIBS)

dat_torture.srec: dat_torture.o pico.o ide_lib.o libcchunk.a
	lwlink --script=cchunk.ld --map=$(basename $@).map --format=srec \
		--output=$@ dat_torture.o pico.o ide_lib.o -L. -lcchunk -L$(CMOC_LIBDIR) $(CMOC_LIBS)

dat_torture.raw: dat_torture.o pico.o ide_lib.o libcchunk.a
	lwlink --script=cchunk.ld --map=$(basename $@).map --format=raw \
		--output=$@ dat_torture.o pico.o ide_lib.o -L. -lcchunk -L$(CMOC_LIBDIR) $(CMOC_LIBS)

# -----------------------------------------------------------------------
clean:
	rm -f *.bin *.obj *.o *.lst *.map *.link *.s19 *.srec *.raw *.a
