all: get.s19 get.bin put.s19 put.bin loop.s19 loop.bin hello.srec hello.raw

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

syslib.obj: syslib.asm
	lwasm --format=obj --list=syslib.lst --output=$@ $<

hello.srec: hello.c
	cmoc --void-target --org=0x0130 --initial-s=0xf000 --srec --mc6839 hello.c

hello.raw: hello.c
	cmoc --void-target --org=0x0130 --initial-s=0xf000 --raw --mc6839 hello.c

clean:
	rm -f *.bin *.obj *.lst *.map *.s19 *.srec *.raw
