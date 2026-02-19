# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Raspberry Pi Pico2 (RP2350/PGA2350) firmware project that acts as a **bus supervisor for a Motorola MC6809E microprocessor**. The Pico2 runs at 320 MHz and emulates peripheral devices (UART, timer) while managing bus cycles for the external 6809 CPU via PIO state machines and DMA.

## Build System

**Target board:** `pimoroni_pga2350` (set in CMakeLists.txt)

```bash
# Standard CMake build (requires PICO_SDK_PATH to be set)
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

Build outputs: `pico-thing.uf2` (flash to Pico2 via USB mass storage), `pico-thing.elf` (for debugger).

The build also:
- Assembles 6809 API snippets via `lwasm` and converts them to C byte arrays with `extra/api.py`
- Auto-generates PIO header files from `src/pico_thing.pio`

**6809 Assembly Code** (in `chunk/`): Built separately with its own Makefile:
```bash
cd chunk && make
```
Requires: `lwasm`, `lwlink`, `cmoc` (6809 toolchain).

**Debugging:** OpenOCD with `openocd_pico.cfg` or `pico-debug.cfg`. Serial console on UART (GPIO 30/31, 115200 baud) and USB CDC.

## Architecture

### Dual-Core Design

- **Core 0**: Console I/O (`poll_console()`), command processing, device state updates (slow path)
- **Core 1**: Real-time bus monitoring, PIO/DMA ISR handling, interrupt signaling (fast path)
- Cores coordinate through the `registers` singleton (64-byte aligned read/write arrays at guest address `$FFC0`)

### PIO State Machines (`src/pico_thing.pio`)

Three PIO programs handle timing-critical 6809 bus operations:
- `mc6809_clock`: Generates E/Q clock signals (quadrature pair, 4-cycle loop)
- `mc6809_bus_read`: Monitors address bus, returns data from `registers[]`
- `mc6809_bus_write`: Captures address+data during write cycles into `registers[]`

DMA channels feed address prefixes and data bytes to/from PIO FIFOs.

### Guest Memory Map

```
$0000–$FDBF   Main RAM (1MB, paged via DAT at $FE00)
$FE00–$FEFF   DAT RAM (256 bytes, 8kB page mapping for NitrOS9)
$FFC0–$FFFF   Emulated peripheral registers + vectors
  $FFC3–$FFC4   MC6850 UART (CONSOLE)
  $FFC8–$FFCF   MC6840 Timer (SYSTEM_TIMER)
  $FFD0–$FFDF   Shared data buffer
  $FFE0–$FFEF   Snippet execution area (16-byte programs)
  $FFF0–$FFFF   CPU vectors (RESET, NMI, FIRQ, IRQ, SWI*)
```

### Key Singletons (all in `src/`)

| Class | File | Responsibility |
|-------|------|----------------|
| `mc6809` | `mc6809.cpp` | Processor state machine (bus state, run state, LIC counting, interrupt management) |
| `registers` | `pico_thing.cpp` | 64-byte read/write register arrays; `ReadProxy`/`WriteProxy` handle 6809 big-endian conversion |
| `mc6850` | `mc6850.cpp` | UART emulation with TX/RX queues; generates IRQ |
| `mc6840` | `mc6840.cpp` | Three 16-bit timer counters; Timer 1 generates NMI |

### Snippet/Chunk System

Small 6809 programs are embedded in firmware as byte arrays:
- **Snippets** (`extra/api.asm`): ≤16 bytes, assembled into `api.inc`, run at `$FFE0`. Used for atomic operations: DATInit, Zero, BlockFill, CopyOut, CopyIn, Sync, LoopRW, and interrupt tests.
- **Chunks** (`chunk/*.asm`, `chunk/*.c`): Larger programs (ASSIST09 monitor, test programs), loaded at `$0130`.

### GPIO Assignment Summary

```
GPIO 0–1:   E/Q clock outputs (PIO)
GPIO 2:     R/!W input
GPIO 3–8:   Address bus A5–A0 (PIO)
GPIO 9–16:  Data bus D7–D0 (PIO)
GPIO 17:    !CS input
GPIO 18–22: BS, BA, BUSY, LIC, AVMA (processor status, ISR-driven)
GPIO 23–26: !HALT, !NMI, !FIRQ, !IRQ outputs
GPIO 29:    !RESET output
GPIO 30–31: UART TX/RX (console)
GPIO 46:    External power enable
```

## Key Files

- `src/pico_thing.cpp` — Main application (2000+ lines): PIO/DMA setup, ISR handlers, console command loop, device orchestration
- `src/mc6809.cpp` / `include/mc6809.h` — Processor state machine; `bus_state_t` and `run_state_t` enums are central to bus cycle handling
- `include/pico_thing.h` — All pin/register address constants; read this first when navigating the memory map
- `chunk/assist09.asm` — ASSIST09 monitor ROM (Motorola original); large file, treat carefully
- `chunk/syslib.asm` — System library, vector tables, device definitions for the 6809 side

## Development Notes

- The `registers` class exposes separate `read_registers[]` and `write_registers[]` arrays — reads and writes to the same guest address can go to different host-side buffers, enabling read-only enforcement.
- The `mc6809` state machine tracks `LIC` (Last Instruction Cycle) signals for cycle-accurate interrupt injection timing.
- DAT paging maps 8kB guest pages to 1MB physical RAM (NitrOS9 compatible); the mapping lives in `$FE00–$FEFF`.
- System clock is 320 MHz (`SYS_CLK_DEFAULT` in CMakeLists.txt); PIO timing depends on this.
