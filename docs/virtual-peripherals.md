# Virtual MC6850 ACIA and 50 Hz Tick Timer — Technical Reference

## 1. System Context

The Pico2 (RP2350) runs at 320 MHz and emulates peripheral devices for an external MC6809E CPU. The 6809 accesses peripherals through a 64-byte register window at guest addresses `$FFC0–$FFFF`. The Pico maps these to two 64-byte arrays: `read_registers[]` (what the 6809 reads) and `write_registers[]` (what the 6809 writes). Reads and writes to the same guest address can go to different host-side buffers.

All register offsets below are relative to `$FFC0` (the base of the 64-byte window). Guest address = `$FFC0 + offset`.

### Execution Contexts (Core 1 only)

There are three execution contexts on Core 1, listed from highest to lowest priority:

| Context | Priority | Trigger | Preempts |
|---------|----------|---------|----------|
| **DMA ISRs** | 0 (highest) | DMA transfer complete | Everything |
| **GPIO ISR** | 1 | E/Q clock edges | Thread mode only |
| **Thread mode (`for(;;)` loop)** | Lowest | Runs when no ISR active | Nothing |

DMA ISRs preempt everything. The GPIO ISR takes most of Core 1 CPU time; the `for(;;)` loop runs in the remainder. The `irq_dirty` optimization (see §2) allows `has_interrupt()` to return in ~2–4 cycles when no state has changed, keeping `for(;;)` loop overhead minimal.

Core 0 handles console I/O and host-side queue operations (putting bytes into the RX queue, reading bytes from the TX queue). Queues use a lock-free `SpscQueue` (single-producer single-consumer ring buffer with `std::atomic` head/tail), safe for cross-core use without spinlocks.

### Interrupt Delivery Pipeline

```
for(;;) loop                    GPIO ISR (Q-rising)           6809
─────────────                   ───────────────────           ────
has_interrupt() for each        reads pending_interrupts      sees /IRQ,
device → computes               asserts/deasserts GPIO        /FIRQ, /NMI
pending_interrupts               pins (/IRQ, /FIRQ, /NMI)     on next cycle
(single volatile write)         NMI: one-shot (clear flag)
                                IRQ/FIRQ: level (hold while set)
```

`pending_interrupts` is a single volatile byte with three flags:
- `PEND_NMI` (bit 0) — edge-triggered, cleared after one assertion
- `PEND_FIRQ` (bit 1) — level-triggered
- `PEND_IRQ` (bit 2) — level-triggered; also cleared by DMA read ISR on certain register reads

### Write Ring Buffer

The DMA write ISR cannot call slow methods (like `guest_control()`) directly. Instead, every write is enqueued into a 32-entry ring buffer (`write_ring[]`). The `for(;;)` loop drains this ring and dispatches to the appropriate handler. Some writes are *also* handled immediately in the DMA ISR for latency reasons (see per-device sections below).

---

## 2. Virtual MC6850 ACIA

### Source Files

- `src/mc6850.cpp` — implementation
- `include/mc6850.h` — header with data structures and class definition

### Register Map

Two instances exist: `fast_serial` (console) and `aux_serial` (auxiliary). Register offsets are passed to the constructor.

| Instance | Offset | Guest Addr | Read | Write |
|----------|--------|-----------|------|-------|
| Console  | `0x03` | `$FFC3` | Status Register | Control Register |
| Console  | `0x04` | `$FFC4` | RX Data Register | TX Data Register |
| Auxiliary| `0x05` | `$FFC5` | Status Register | Control Register |
| Auxiliary| `0x06` | `$FFC6` | RX Data Register | TX Data Register |

### Data Structures

**`mc6850_control` union** (written by 6809 to `$FFC3`):
- `divide_select` (bits 1:0) — `0b11` = master reset; `0b10` = ÷64 (repurposed as queue loopback mode)
- `word_select` (bits 4:2) — word length/parity (stored but not used in emulation)
- `tx_irq` (bits 6:5) — `0b01` = TX IRQ enabled; `0b10` = RTS deasserted; `0b11` = close loopback mode
- `rx_irq` (bit 7) — `1` = RX IRQ enabled

**`mc6850_status` union** (read by 6809 from `$FFC3`):
- `rdrf` (bit 0) — Receive Data Register Full
- `tdre` (bit 1) — Transmit Data Register Empty
- `dcd` (bit 2) — always 0
- `cts` (bit 3) — CTS deasserted (TX queue >200/256)
- `fe` (bit 4), `ovrn` (bit 5), `pe` (bit 6) — always 0
- `irq` (bit 7) — composite: `assert_receive_irq || assert_transmit_irq`

**Internal state booleans:**
- `transmit_buffer_empty` — TX queue has space
- `receive_buffer_full` — a byte is staged in `read_registers[CONSOLE_RX_DATA]`
- `transmit_irq` — TX IRQ enabled (from control register)
- `receive_irq` — RX IRQ enabled (from control register)
- `assert_transmit_irq` — `transmit_irq && transmit_buffer_empty && !tx_write_pending`
- `assert_receive_irq` — `receive_irq && receive_buffer_full`
- `cts_deasserted` — TX queue level > `CTS_THRESHOLD` (200)
- `rts_deasserted` — 6809 is signaling its ring buffer is full
- `loopback_close` — TX→RX register directly (no queues)
- `loopback_queue` — TX→RX queue (full staging pipeline)
- `tx_write_pending` (volatile) — set by DMA write ISR, cleared by `guest_transmit()`

**Caching state:**
- `cached_status_byte` — last value written to `reg[CONSOLE_STATUS]` by `sync_status()`. When the recomputed status byte matches, the register write is skipped.
- `cached_irq_result` — last return value of `has_interrupt()`. Returned immediately when `irq_dirty` is false.
- `irq_dirty` (volatile) — set by any state-changing method (`write_received()`, `guest_control()`, `guest_transmit()`, `rx_read_fast_path()`, `guest_receive()`, `host_transmit()`, `reset()`). Cleared by `has_interrupt()` after recomputation. When clean, `has_interrupt()` returns `cached_irq_result` in ~2–4 cycles, avoiding ~30–50 cycles of full status recomputation.

**Queues:**
- `tx` — 256-entry lock-free `SpscQueue`. 6809 writes go in (via `guest_transmit()`), host reads come out (via `host_receive()`). Cross-core safe.
- `rx` — 256-entry lock-free `SpscQueue`. Host writes go in (via `host_transmit()`), 6809 reads come out (via `guest_receive()`/`has_interrupt()`). Cross-core safe.

### Method Execution Contexts

| Method | Called From | Context |
|--------|------------|---------|
| `write_received()` | DMA write ISR | ISR (priority 0) |
| `rx_read_fast_path()` | DMA read ISR | ISR (priority 0) |
| `has_interrupt()` | `for(;;)` loop (every Q-rising edge) | Thread mode |
| `guest_control(val)` | `for(;;)` loop (write ring drain) | Thread mode |
| `guest_transmit()` | `for(;;)` loop (write ring drain) | Thread mode |
| `guest_receive()` | `for(;;)` loop (on rx_read_count change) | Thread mode |
| `host_transmit()` | Core 0 (console poll) | Cross-core |
| `host_receive()` | Core 0 (console poll) | Cross-core |

### `sync_status()` — The Status Register Rebuild

**Critical design rule:** `sync_status()` always builds the status register from scratch. It never reads the current `reg[CONSOLE_STATUS]` value and modifies it. This prevents a race where `rx_read_fast_path()` (running in a higher-priority DMA ISR) clears RDRF between a hypothetical read and write of the status register.

```
TDRE = transmit_buffer_empty && !cts_deasserted && !tx_write_pending
RDRF = receive_buffer_full
CTS  = cts_deasserted
IRQ  = assert_receive_irq || assert_transmit_irq
```

**Optimization:** The computed status byte is compared against `cached_status_byte`. If unchanged, the register write is skipped entirely. This is the common case during idle periods.

### TX Data Flow (6809 → Host)

```
6809 writes $FFC4          DMA write ISR              for(;;) loop              Core 0
────────────────           ──────────────             ────────────              ──────
STA UARTTX ──────────────→ write_received():          (drain write ring)        host_receive():
                           tx_write_pending=true       guest_transmit():         tx.get() → byte
                           sync_status() → TDRE=0      tx_write_pending=false    to USB/UART
                           enqueue to write_ring       tx.put(data)
                                                       sync_transmit() → TDRE=1
                                                       6809 unblocks
```

**Why `tx_write_pending` exists:** The 6809 can write two bytes back-to-back (e.g. CR then LF from PUTCHAR, 45 cycles apart). `write_registers[CONSOLE_TX_DATA]` is a single slot — the second write would overwrite the first before the `for(;;)` loop reads it. `write_received()` sets `tx_write_pending=true` and clears TDRE in the status register. The 6809's OUTCH poll loop (`LDB UARTS / BITB #S_TDRE / BEQ`) stalls until `guest_transmit()` clears `tx_write_pending` and restores TDRE=1.

### RX Data Flow (Host → 6809)

```
Core 0                    for(;;) loop                                    DMA read ISR
──────                    ────────────                                    ────────────
host_transmit():          has_interrupt() [every Q-rising]:               6809 reads $FFC4:
rx.put(byte) ──→          if !receive_buffer_full && rx.has_bytes():       rx_read_fast_path():
                            reg[CONSOLE_RX_DATA] = rx.get()                receive_buffer_full=false
                            receive_buffer_full = true                     assert_receive_irq=false
                          assert_receive_irq = receive_irq &&              sync_status() → RDRF=0
                            receive_buffer_full                            pending_interrupts &= ~PEND_IRQ
                          sync_status() → RDRF=1, IRQ=1
                          → pending_interrupts |= PEND_IRQ

                          guest_receive() [on rx_read_count change]:
                            stages next byte from rx queue if slot empty
```

**Two staging paths:** Both `has_interrupt()` and `guest_receive()` can stage a byte from the RX queue into `reg[CONSOLE_RX_DATA]`. Both check `receive_buffer_full` first to avoid double-consuming. `has_interrupt()` runs on every Q-rising edge (proactive); `guest_receive()` runs when `rx_read_count` changes (reactive, triggered by the 6809 actually reading the data register).

### `has_interrupt()` — Fast Path and Dirty Flag

`has_interrupt()` is called on every Q-rising edge. To avoid ~30–50 cycles of full recomputation each time, it uses an `irq_dirty` flag:

1. **Clean path** (`irq_dirty == false`): returns `cached_irq_result` immediately (~2–4 cycles).
2. **Dirty path** (`irq_dirty == true`): refreshes TX queue state (absorbing the work previously done by the removed `update_task()` method), stages RX bytes if needed, recomputes interrupt conditions, calls `sync_status()`, caches the result, and clears `irq_dirty`.

Every state-changing method sets `irq_dirty = true`. This ensures the next `has_interrupt()` call picks up the change, while all intervening calls on unchanged state return instantly.

**RTS flow control:** When `rts_deasserted=true`, neither `has_interrupt()` nor `guest_receive()` will stage bytes from the RX queue. Bytes accumulate in the queue until the 6809 re-asserts RTS (writes a control register value with `tx_irq != 0b10`).

### `rx_read_fast_path()` and PEND_IRQ Clearing

When the 6809 reads `$FFC4` (CONSOLE_RX_DATA), the DMA read ISR calls `rx_read_fast_path()` which immediately clears `receive_buffer_full` and `assert_receive_irq`, then rebuilds the status register. It also clears `PEND_IRQ` from `pending_interrupts` so the next Q-rising ISR deasserts `/IRQ`. Without this, the 6809 would see a stale `/IRQ` assertion for one or more cycles after acknowledging the interrupt.

### Control Register Processing and Write Ring Ordering

`guest_control(val)` is called from the `for(;;)` loop's write ring drain, **not** from the DMA write ISR. The control register value is passed as a parameter (not read from `write_registers` inside the method). This is critical for ordering: if the 6809 writes TX_DATA then CONTROL in quick succession, the write ring preserves FIFO order. The `for(;;)` loop processes TX_DATA first (via `guest_transmit()`), routing it through the current mode, then processes CONTROL (via `guest_control(val)`), which may change the mode. If `guest_control()` ran in the ISR, the mode change would take effect before the `for(;;)` loop processed the pending TX byte, routing it through the wrong path.

### Loopback Modes

**Close loopback** (`tx_irq == 0b11`, repurposed from BREAK):
- TX byte goes directly into `reg[CONSOLE_RX_DATA]`, bypassing both queues
- `receive_buffer_full` and `assert_receive_irq` are set immediately in `guest_transmit()`
- The 6809 sees RDRF=1 on the very next status read — no `has_interrupt()` lag
- Used for fast self-test (zero queue delay)

**Queue loopback** (`divide_select == 0b10`, repurposed from ÷64):
- TX byte enters the RX queue (not the TX queue)
- Delivered to the 6809 through the normal `has_interrupt()` / `guest_receive()` staging pipeline
- Tests the full interrupt delivery path including staging delay

### Reset

`divide_select == 0b11` triggers `reset()`: all state booleans cleared, status register set to `{tdre=1}` (TDRE=1, everything else 0). Queues are **not** flushed (commented out) — host-side bytes in flight are preserved.

---

## 3. 50 Hz Tick Timer

A minimal 50 Hz tick timer using the Pico SDK's hardware timer alarm, implemented as the `tick_timer` class.

### Register Map

| Offset | Guest Addr | Read | Write |
|--------|-----------|------|-------|
| `0x08` | `$FFC8` | — | Control (bit 0: enable/disable) |
| `0x09` | `$FFC9` | Status (bit 7: IRQ pending; cleared on read) | — |

Two registers, down from eight. All other offsets in the `$FFC8–$FFCF` range are unused.

### Source Files

- `src/tick_timer.cpp` — implementation
- `include/tick_timer.h` — class definition
- `tests/test_tick_timer.cpp` — unit tests (16 cases, 54 assertions)

### Implementation

The `tick_timer` class is instantiated in `src/pico_thing.cpp` as a file-scope object:

```cpp
static tick_timer system_tick(reg, pending_interrupts, PEND_IRQ,
                              SYSTEM_TIMER_CONTROL_13, SYSTEM_TIMER_STATUS);
```

The constructor takes references to the register singleton and `pending_interrupts` byte, plus the specific pending bit and register offsets. Nothing is hardcoded — a second tick timer at different offsets could be instantiated from the same class.

The timer is started once in `main_core1()` via `system_tick.start()`, which calls:

```cpp
add_repeating_timer_us(TICK_PERIOD_US, callback, this, &hw_timer);
```

The negative period (`-20000` µs) means the SDK schedules the next callback relative to the *scheduled* time of the previous one (not the completion time), preventing drift. The static `callback()` uses `rt->user_data` to recover `this`.

### Callback

Runs on an SDK timer alarm IRQ (Core 1, separate from the GPIO and DMA ISRs). If the timer is enabled, sets `irq_pending = true` and writes `$80` (bit 7) to the status register so the 6809 can see the IRQ source on the next read. Negligible CPU cost (~0.001% at 50 Hz).

### `control(val)` — `$FFC8` write

Called from the DMA write ISR. Bit 0 enables (1) or disables (0) the timer. Disabling immediately clears any pending IRQ and the status register.

### `acknowledge()` — `$FFC9` read

Called from the DMA read ISR when the 6809 reads `$FFC9`. Clears `irq_pending`, zeros the status register, and removes `PEND_IRQ` from `pending_interrupts` so `/IRQ` deasserts. This is the **only** way to acknowledge the timer IRQ.

### `has_interrupt()` — Interrupt Delivery

Called from the `for(;;)` loop alongside `fast_serial.has_interrupt()`. Returns `INTERRUPT_IRQ` if `irq_pending` is set, `INTERRUPT_NONE` otherwise. If pending, contributes to `pending_interrupts`, and the GPIO ISR asserts `/IRQ` on the next Q-rising edge.

### `reset()` — System Reset

Called from `peripheral_clear()` during system reset. Disables the timer and clears all state. The hardware alarm continues running (harmless — the callback checks `enabled`).

### 6809-Side Usage (NitrOS-9 Clock Module)

```asm
; Enable the tick timer
    LDA  #$01
    STA  $FFC8

; In the IRQ handler — acknowledge and check source
    LDA  $FFC9       ; bit 7 set? → tick timer IRQ; reading clears it
    BMI  tick_handler
    ; ... check other IRQ sources ...

; Disable the tick timer
    CLR  $FFC8
```

---

## 4. Interaction Between Devices

`fast_serial.has_interrupt()`, `aux_serial.has_interrupt()`, and `system_tick.has_interrupt()` are all called on every Q-rising edge in the `for(;;)` loop. Their return values are OR'd into `pending_interrupts`:

```cpp
pending_interrupts =
    (interrupt_refcount[INTERRUPT_NMI]  > 0 ? PEND_NMI  : 0) |
    (interrupt_refcount[INTERRUPT_FIRQ] > 0 ? PEND_FIRQ : 0) |
    (interrupt_refcount[INTERRUPT_IRQ]  > 0 ? PEND_IRQ  : 0);
```

If multiple devices assert `INTERRUPT_IRQ` simultaneously, the 6809 sees a single `/IRQ` assertion. The ISR must read each device's status register to determine the source(s).

`pending_interrupts` is recomputed from scratch each Q-rising edge by calling every device's `has_interrupt()`. DMA read ISRs also clear per-device state eagerly (e.g. `rx_read_fast_path()` clears RDRF and `PEND_IRQ`, `tick_timer::acknowledge()` clears `irq_pending` and `PEND_IRQ`) so the 6809 sees `/IRQ` deassert promptly after acknowledging an interrupt. The next `for(;;)` iteration then recomputes `pending_interrupts` from all sources — if any other device still has an interrupt pending, `/IRQ` stays asserted.
