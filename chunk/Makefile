# Path to the cmoc standard library (derived from the cmoc binary location).
CMOC_LIBDIR := $(patsubst %/bin/cmoc,%/share/cmoc/lib,$(shell which cmoc))
# Libraries required by all cmoc-compiled programs.
CMOC_LIBS   := -lcmoc-crt-void -lcmoc-std-void \
               $(CMOC_LIBDIR)/float-ctor.mc6839_o -lcmoc-float-mc6839
# Object files added to every C program link for interrupt-driven I/O.
C_IO_OBJS   := io.obj syslib.obj cstart.obj

all: assist09.s19 assist09.bin syslib.s19 syslib.bin get.s19 get.bin put.s19 put.bin loop.s19 loop.bin hello.srec hello.raw disk_test.srec disk_test.raw test_echo.s19 test_echo.bin test_echo_fast.s19 test_echo_fast.bin test_send.s19 test_send.bin test_timer.s19 test_timer.bin

assist09.s19: assist09.asm
	lwasm --list=assist09.lst --map=assist09.map --format=srec --output=$@ $^

assist09.bin: assist09.asm
	lwasm --list=assist09.lst --map=assist09.map --format=decb --output=$@ $^

get.s19: get.obj io.obj bss.obj syslib.obj
	lwlink --script=chunk.ld --map=get.map --format=srec --output=$@ $^

get.bin: get.obj io.obj bss.obj syslib.obj
	lwlink --script=chunk.ld --map=get.map --format=decb --output=$@ $^

get.obj: get.asm ascii.inc uart.inc syslib.inc
	lwasm --format=obj --list=get.lst --output=$@ $<

put.s19: put.obj io.obj bss.obj syslib.obj
	lwlink --script=chunk.ld --map=put.map --format=srec --output=$@ $^

put.bin: put.obj io.obj bss.obj syslib.obj
	lwlink --script=chunk.ld --map=put.map --format=decb --output=$@ $^

put.obj: put.asm ascii.inc uart.inc syslib.inc
	lwasm --format=obj --list=get.lst --output=$@ $<

test_send.s19: test_send.obj io.obj bss.obj syslib.obj
	lwlink --script=chunk.ld --map=test_send.map --format=srec --output=$@ $^

test_send.bin: test_send.obj io.obj bss.obj syslib.obj
	lwlink --script=chunk.ld --map=test_send.map --format=decb --output=$@ $^

test_send.obj: test_send.asm ascii.inc uart.inc syslib.inc
	lwasm --format=obj --list=test_send.lst --output=$@ $<

test_timer.s19: test_timer.obj io_noirq.obj bss.obj syslib.obj
	lwlink --script=chunk.ld --map=test_timer.map --format=srec --output=$@ $^

test_timer.bin: test_timer.obj io_noirq.obj bss.obj syslib.obj
	lwlink --script=chunk.ld --map=test_timer.map --format=decb --output=$@ $^

test_timer.obj: test_timer.asm ascii.inc uart.inc syslib.inc
	lwasm --format=obj --list=test_timer.lst --output=$@ $<

io_noirq.obj: io.asm uart.inc ascii.inc
	lwasm -DNO_IRQ --format=obj --list=io_noirq.lst --output=$@ $<

loop.s19: loop.obj bss.obj syslib.obj
	lwlink --script=chunk.ld --map=loop.map --format=srec --output=$@ $^

loop.bin: loop.obj bss.obj syslib.obj
	lwlink --script=chunk.ld --map=loop.map --format=decb --output=$@ $^

loop.obj: loop.asm bss.obj
	lwasm --format=obj --list=loop.lst --output=$@ $<

test_echo_fast.s19: test_echo_fast.obj io.obj bss.obj syslib.obj
	lwlink --script=chunk.ld --map=test_echo_fast.map --format=srec --output=$@ $^

test_echo_fast.bin: test_echo_fast.obj io.obj bss.obj syslib.obj
	lwlink --script=chunk.ld --map=test_echo_fast.map --format=decb --output=$@ $^

test_echo_fast.obj: test_echo_fast.asm uart.inc syslib.inc
	lwasm --format=obj --list=test_echo_fast.lst --output=$@ $<

test_echo.s19: test_echo.obj io.obj bss.obj syslib.obj
	lwlink --script=chunk.ld --map=test_echo.map --format=srec --output=$@ $^

test_echo.bin: test_echo.obj io.obj bss.obj syslib.obj
	lwlink --script=chunk.ld --map=test_echo.map --format=decb --output=$@ $^

test_echo.obj: test_echo.asm ascii.inc uart.inc syslib.inc
	lwasm --format=obj --list=test_echo.lst --output=$@ $<

io.obj: io.asm uart.inc ascii.inc
	lwasm --format=obj --list=io.lst --output=$@ $<

bss.obj: bss.asm
	lwasm --format=obj --list=bss.lst --output=$@ $<

syslib.s19: syslib.obj stub.obj
	lwlink --script=chunk.ld --map=syslib.map --format=srec --output=$@ $^

syslib.bin: syslib.obj stub.obj
	lwlink --script=chunk.ld --map=syslib.map --format=decb --output=$@ $^

syslib.obj: syslib.asm
	lwasm --format=obj --list=syslib.lst --output=$@ $<

stub.obj: stub.asm
	lwasm --format=obj --list=stub.lst --output=$@ $<

disk_test.srec: disk_test.o pico.o ide_lib.o $(C_IO_OBJS)
	lwlink --script=cchunk.ld --map=disk_test.map --format=srec \
		-L$(CMOC_LIBDIR) $(CMOC_LIBS) --output=$@ $^

disk_test.raw: disk_test.o pico.o ide_lib.o $(C_IO_OBJS)
	lwlink --script=cchunk.ld --map=disk_test.map --format=raw \
		-L$(CMOC_LIBDIR) $(CMOC_LIBS) --output=$@ $^

disk_test.o: disk_test.c pico.h
	cmoc --void-target --org=0x0130 --initial-s=0xf000 --mc6839 -c $<

hello.srec: hello.o pico.o $(C_IO_OBJS)
	lwlink --script=cchunk.ld --map=hello.map --format=srec \
		-L$(CMOC_LIBDIR) $(CMOC_LIBS) --output=$@ $^

hello.raw: hello.o pico.o $(C_IO_OBJS)
	lwlink --script=cchunk.ld --map=hello.map --format=raw \
		-L$(CMOC_LIBDIR) $(CMOC_LIBS) --output=$@ $^

hello.o: hello.c pico.h
	cmoc --void-target --org=0x0130 --initial-s=0xf000 --mc6839 -c $<

pico.o: pico.c pico.h
	cmoc --void-target --org=0x0130 --initial-s=0xf000 --mc6839 -c $<

cstart.obj: cstart.asm
	lwasm --format=obj --output=$@ $<

ide.o: ide.c ide.h pico.h
	cmoc --void-target --org=0x0130 --initial-s=0xf000 --mc6839 -c $<

ide_lib.o: ide_lib.c ide_lib.h pico.h
	cmoc --void-target --org=0x0130 --initial-s=0xf000 --mc6839 -c $<

clean:
	rm -f *.bin *.obj *.o *.lst *.map *.link *.s19 *.srec *.raw
