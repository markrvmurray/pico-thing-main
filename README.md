# pico-thing-main

Firmware for the main RP2350 of the **Pico-Thing**, a single-board computer
built around a real Motorola MC6809E or Hitachi HD6309E.

A Pimoroni PGA2350 (RP2350B) sits on the 6809's bus as its supervisor. It
generates the E and Q clocks, serves the peripheral registers at the bottom of
the address space, drives the DAT memory-management unit, and hands the host
two USB serial ports for the guest's ACIAs. Bus cycles are served by PIO state
machines and DMA on Core 1, while Core 0 runs a command console for loading,
running, examining and tracing the guest.

The board boots **NitrOS-9**, from IDE or over **DriveWire**, in 6809 mode or
HD6309 native mode, at E clocks of up to 3 MHz.

## Related repositories

| Repository | What it is |
|---|---|
| [pico-thing](https://github.com/markrvmurray/pico-thing) | The hardware: KiCad schematics and PCB for every board revision (often behind the design work in progress) |
| [pico-thing-video](https://github.com/markrvmurray/pico-thing-video) | The second RP2350 on the bus: DVI video, USB keyboard and mouse, and sound |
| [nitros9](https://github.com/markrvmurray/nitros9) | NitrOS-9, with the Pico-Thing port in `level1/picothing` and `level2/picothing` |
| [usim, branch `pico-thing`](https://github.com/markrvmurray/usim/tree/pico-thing) | The simulator; `usim09pt` emulates the whole Pico-Thing, including a pseudo-terminal for DriveWire |

## Features

- **The 6809's clock**: E and Q from a PIO state machine, settable from 0.8
  to 3.0 MHz at the console.
- **Memory management (DAT)**: 32 tasks, each with eight 8 KB slots, which
  map guest pages to physical RAM pages. A slot holding `$FF` raises a DAT-fault
  NMI and records the fault and its task and block.
- **Two MC6850-compatible ACIAs**, one for the guest's console and one
  auxiliary, each presented to the host as a USB CDC serial port with
  hardware flow control.
- **A tick timer**: a 50 Hz IRQ, plus an E-cycle countdown NMI used for
  single-stepping.
- **A watchpoint** and **bus trace**: a one-shot or ring capture of every bus
  cycle, which can be armed to freeze when a given vector is taken.
- **PATA IDE** on the real bus at `$FF00`.
- **Real-time clock**: a DS12885, readable and settable from the console.
- **Supply supervision** (V2 and later): the Pico measures the external 5 V
  rail and only closes the load switch when it is good, then drops it if the
  rail sags.
- **NitrOS-9 boot loader** built into the firmware, with a small debug
  monitor (DBGMON) for the guest.

## Hardware

The firmware supports board revisions 1, 2 and 3, chosen at build time with
`PICO_THING_BOARD_VERSION` (default 2). The schematics and PCB are in
[pico-thing](https://github.com/markrvmurray/pico-thing). That repository is
often behind the design work in progress, so the firmware may support a
change to the board before it appears there.

The pins the revisions share:

```
GPIO 0-1    E, Q clock outputs
GPIO 2      R/!W
GPIO 3-8    A5-A0
GPIO 9-16   D7-D0
GPIO 17     !CS (the peripheral block is selected)
GPIO 18-22  BS, BA, BUSY, LIC, AVMA
GPIO 23-26  !HALT, !NMI, !FIRQ, !IRQ
GPIO 29     !RESET
GPIO 30-31  UART TX/RX: the firmware's own console
```

On version 2, GPIO 32-46 carry the task number, the scope trace outputs, the
power enable, the DAT fault input and NMI block, A13-A15 and the 5 V ADC. The
full map for every revision is in [`include/pico_thing.h`](include/pico_thing.h).

## Guest memory map

Addresses run from `$0000` at the top to `$FFFF` at the bottom.

| Range | Contents |
|---|---|
| `$0000-$DFFF` | DAT-mapped RAM: 7 slots of 8 KB per task |
| `$E000-$FDFF` | Fixed-window RAM (the last physical page, bypassing the DAT) |
| `$FE00-$FEFF` | DAT RAM: 32 tasks x 8 slots, one physical page number each |
| `$FF00-$FF09` | PATA IDE (the ATA register map, offset by one) |
| `$FFC0` | DAT task register |
| `$FFC2-$FFC3` | System request |
| `$FFC4-$FFC5` | Console ACIA (control/status, data) |
| `$FFC6-$FFC7` | Auxiliary ACIA (control/status, data) |
| `$FFC8-$FFC9` | Tick timer and NMI countdown |
| `$FFCA` | Watchpoint |
| `$FFCB` | Trace control |
| `$FFCC-$FFCD` | DAT fault, and its task and block |
| `$FFD0-$FFDF` | Shared data buffer |
| `$FFE0-$FFEF` | Snippet area (16-byte programs run by the firmware) |
| `$FFF0-$FFFF` | CPU vectors |

The registers are described in detail, with their bit layouts, in
[`docs/virtual-peripherals.md`](docs/virtual-peripherals.md).

## Getting started

### Prerequisites

- The [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk), with
  `PICO_SDK_PATH` pointing at it.
- An Arm toolchain: either `arm-none-eabi-gcc` or Arm's ATfE (clang). Both are
  supported.
- CMake 3.14 or later, flex, and bison 3.x. On macOS the system bison is
  2.3, which is too old: `brew install bison`, and the build finds it.
- [lwtools](http://lwtools.projects.l-w.ca) (`lwasm`, `lwlink`, `lwar`): the
  build assembles the 6809 snippets and the debug monitor.
- For the guest programs in `chunk/`: lwtools and
  [CMOC](http://perso.b2b2c.ca/~sarrazip/dev/cmoc.html).
- A CMSIS-DAP debug probe (for example a Raspberry Pi Debug Probe) and
  OpenOCD with RP2350 support.

### Build

```sh
export PICO_SDK_PATH=/path/to/pico-sdk
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

This produces `build/pico-thing-main.elf` and `build/pico-thing-main.uf2`.

To use ATfE instead of gcc, add `-DPICO_COMPILER=pico_arm_cortex_m33_clang` and set
`PICO_TOOLCHAIN_PATH` to the toolchain's directory. The assembler and binary
tools are taken from the same place, so set it in the environment that CMake
runs in, not only in the shell you build from.

### Flash

Copy the `.uf2` file to the board in BOOTSEL mode, or flash over SWD:

```sh
scripts/flash.sh build            # program, verify, reset and run
scripts/flash.sh build noreset    # program and verify only
scripts/flash.sh build reset      # just reset and run
```

`pico.cfg` is the repository's one OpenOCD configuration. It is pinned to a
particular probe by its adapter serial, so that it can never reach the video
board's probe when both are attached. **Change `adapter serial` in `pico.cfg`
to your own probe's serial** before using it; the same file works as CLion's
board configuration and with a bare `openocd -f pico.cfg`.

### Connect

The board presents three serial ports:

| Port | Connects to | Used for |
|---|---|---|
| The probe's UART bridge (GPIO 30/31, 115200 baud) | The firmware | The command console |
| USB CDC 0 | The console ACIA at `$FFC4` | The guest's terminal |
| USB CDC 1 | The auxiliary ACIA at `$FFC6` | DriveWire, or a second guest serial line |

Open the firmware console in any terminal program at 115200 8N1 and press
Return for a prompt; `help` lists every command.

## Using the console

Numbers are hexadecimal unless stated, keywords are case-insensitive, and
almost every command has a two-letter alias. An S-record pasted at the prompt
is loaded straight into guest memory.

| Area | Commands |
|---|---|
| CPU | `go [addr]`, `stop [halt\|restart]` (`sp`), `run [n]` (`ru`), `reset [level]` (`rs`), `freq <MHz>` (`fq`), `task [n]` (`ta`), `power [on\|off]` (`pw`) |
| Memory | `examine <s> <e>` (`ex`), `examine dat`, `modify <a> <b>...` (`mo`), `fill <s> <e> [v]` (`fi`), `vector [n]` (`vc`), `snippet [n]` (`sn`) |
| Tests | `ramtest`, `synctest [n]`, `adcdump`, `irq <0-9>` (`ir`), `loop <type> <addr>` (`lo`) |
| Debug | `trace on\|off\|ring\|stop\|<vector>` (`tr`), `break [addr\|r]` (`br`), `status [period]` (`st`), `verbose` (`vb`) |
| Host | `time` (`ti`), `rtc`, `usb` (`ub`), `flow [1\|2] [on\|off]` (`fl`), `nitros9` (`ni`) |

`reset` has graded levels, weakest first: `statistics`, `peripherals` (the
default), `vectors`, `run0`, `core1` and `reboot`. Each includes the ones
before it. `help` describes every command and option in full.

A typical session loads a program and starts it:

```
>>> fq 2.0                    E clock to 2 MHz
>>> (paste chunk/hello.srec)
>>> go 130
```

### Booting NitrOS-9

```
nitros9 [ide|drivewire] [auto|6809|6309]
```

`drivewire` may be shortened to `dw`. The CPU mode chooses the image to boot:
`auto` detects the CPU, `6809` keeps an HD6309 in 6809 mode for a 6809 image,
and `6309` boots an HD6309 in native mode.

The firmware loads its debug monitor at `$FC00` and the boot loader at
`$0130`, then starts the CPU. The boot loader prints to the console ACIA, so
watch CDC 0.

For a DriveWire boot, run a DriveWire server on the host attached to CDC 1
**before** starting the boot, with the NitrOS-9 disk image mounted as drive 0.
After the board is flashed or replugged, the server must reopen CDC 1; a boot
that prints "Boot failed" without reaching the disk format line usually means
it has not.

The NitrOS-9 port itself (`level1/picothing` and `level2/picothing`) lives in
[a NitrOS-9 fork](https://github.com/markrvmurray/nitros9). The boot loader is built there and embedded here as
`extra/rel_picothing`.

## Guest programs (`chunk/`)

`chunk/` holds 6809 programs to run on the guest: the ASSIST09 monitor, test
programs for the ACIAs, the timer, the DAT and the IDE disk, and some
examples. They are built separately:

```sh
cd chunk && make               # polled I/O (the default)
cd chunk && make IO=irq        # interrupt-driven ACIA I/O
cd chunk && make ASSIST09=1    # also build ASSIST09
```

Programs load at `$0130` with the stack in the fixed window. Load one by
pasting its `.s19`/`.srec` file at the console prompt (or with
`scripts/console.py --load`, which paces the lines), then `go 130`.

`acia_torture.asm` is a 12-test ACIA interrupt harness, useful for checking
a board at a new E clock.

## Configuration

The main CMake cache options:

| Option | Default | Meaning |
|---|---|---|
| `PICO_THING_BOARD_VERSION` | `2` | Board revision: 1, 2 or 3 |
| `PICO_THING_SYS_CLK_HZ` | `360000000` | RP2350 system clock. The ceiling on the 6809's E clock scales with it |
| `PICO_THING_VREG` | `VREG_VOLTAGE_1_50` | Core voltage, set before the clock is raised |
| `PICO_THING_VREG_UNLIMITED` | `ON` | Lift the SDK's 1.30 V limit so `PICO_THING_VREG` takes effect |
| `PICO_THING_CONSOLE_BAUD` | `115200` | Firmware console UART speed |
| `PICO_THING_POWER_RAIL_MIN_VOLTS` | `4.5` | External 5 V needed before the load switch closes |
| `PICO_THING_POWER_RAIL_DROP_VOLTS` | `4.0` | External 5 V below which the load switch opens |
| `PICO_THING_POWER_RAIL_SETTLE_MS` | `250` | Time the rail is given to settle before it is judged |
| `PICO_THING_DAT_FAULT_AUTO_BLOCK` | `1` | Block NMI automatically after a DAT fault (0 only for scope work) |
| `PICO_THING_LTO` | `OFF` | Link-time optimisation (not currently working) |

`PICO_THING_DIAG`, `PICO_THING_ISR_COST` and `PICO_THING_TESTPOINTS` turn on
measurement instruments that cost bus-serving time; leave them off in normal
use. CMake caches these values, so a tree configured with one clock keeps it
until the entry is changed with `-D` or removed with `-U`.

> **Clock and heat.** 360 MHz at 1.50 V is the tested default. 400 MHz at
> 2.00 V runs, but the die gets hot enough to need a heatsink. At high
> `clk_sys` the XIP flash clock is not reduced and can go out of
> specification.

## Host tests

The console parser, the ACIA, the tick timer, the S-record loader, the supply
monitor and the UART pump have unit tests that run on the build host:

```sh
cmake -S tests -B tests/build
cmake --build tests/build
for t in tests/build/test_*; do "$t" || break; done
```

## Repository layout

```
src/        Firmware: bus serving, devices, console (flex/bison parser), USB
include/    Headers; pico_thing.h has every pin and register address
extra/      6809 snippets and debug monitor, and the embedded NitrOS-9 boot loader
chunk/      Guest programs (built separately)
tests/      Host unit tests
scripts/    Flashing, console driving and bench tools
host/       Host-side USB test programs
docs/       Peripheral register reference and the inter-Pico link protocol
```

## Documentation

- [`docs/virtual-peripherals.md`](docs/virtual-peripherals.md): the
  registers at `$FFC0-$FFFF`, with bit layouts and usage.
- [`docs/pix-protocol.md`](docs/pix-protocol.md): the proposed 4-bit link
  between the two RP2350s (a specification; not yet implemented).

## Licence

MIT. See [LICENSE](LICENSE).
