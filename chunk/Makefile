all: syslib.s19 syslib.bin get.s19 get.bin put.s19 put.bin loop.s19 loop.bin hello.srec hello.raw test_ide.srec test_ide.raw

get.s19: get.obj io.obj bss.obj syslib.obj
	lwlink --script=chunk.ld --map=get.map --format=srec --output=$@ $^

get.bin: get.obj io.obj bss.obj syslib.obj
	lwlink --script=chunk.ld --map=get.map --format=decb --output=$@ $^

get.obj: get.asm ascii.inc uart.inc
	lwasm --format=obj --list=get.lst --output=$@ $<

put.s19: put.obj io.obj bss.obj syslib.obj
	lwlink --script=chunk.ld --map=put.map --format=srec --output=$@ $^

put.bin: put.obj io.obj bss.obj syslib.obj
	lwlink --script=chunk.ld --map=put.map --format=decb --output=$@ $^

put.obj: put.asm ascii.inc uart.inc
	lwasm --format=obj --list=put.lst --output=$@ $<

loop.s19: loop.obj bss.obj syslib.obj
	lwlink --script=chunk.ld --map=loop.map --format=srec --output=$@ $^

loop.bin: loop.obj bss.obj syslib.obj
	lwlink --script=chunk.ld --map=loop.map --format=decb --output=$@ $^

loop.obj: loop.asm bss.obj
	lwasm --format=obj --list=loop.lst --output=$@ $<

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
	lwasm --format=obj --list=syslib.lst --output=$@ $<

hello.srec: hello.o pico.o ide_lib.o
	cmoc --void-target --org=0x0130 --initial-s=0xf000 --srec --mc6839 -o $@ $^

hello.raw: hello.o pico.o ide_lib.o
	cmoc --void-target --org=0x0130 --initial-s=0xf000 --raw --mc6839 -o $@ $^

hello.o: hello.c pico.h
	cmoc --void-target --org=0x0130 --initial-s=0xf000 --mc6839 -c $<

test_ide.srec: test_ide.o pico.o ide.o
	cmoc --void-target --org=0x0130 --initial-s=0xf000 --srec --mc6839 -o $@ $^

test_ide.raw: test_ide.o pico.o ide.o
	cmoc --void-target --org=0x0130 --initial-s=0xf000 --raw --mc6839 -o $@ $^

test_ide.o: test_ide.c pico.h ide.h
	cmoc --void-target --org=0x0130 --initial-s=0xf000 --mc6839 -c $<

pico.o: pico.c pico.h
	cmoc --void-target --org=0x0130 --initial-s=0xf000 --mc6839 -c $<

ide.o: ide.c ide.h pico.h
	cmoc --void-target --org=0x0130 --initial-s=0xf000 --mc6839 -c $<

ide_lib.o: ide_lib.c ide_lib.h pico.h
	cmoc --void-target --org=0x0130 --initial-s=0xf000 --mc6839 -c $<

clean:
	rm -f *.bin *.obj *.o *.lst *.map *.link *.s19 *.srec *.raw
