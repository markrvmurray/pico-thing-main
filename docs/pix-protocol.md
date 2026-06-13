# PIX Link — 4-bit Bidirectional Inter-Pico Protocol

**Status:** specification / contract. Implementation pending.
**Parties:** *Supervisor* (this firmware, the MC6809 bus supervisor) and *Worker*
(a second RP2350, the graphics co-processor).
**Transport:** 4 open-drain data lines (PIX[3:0]) + 1 supervisor-sourced clock
(PIX_CLK). Half-duplex, DDR (one byte per clock period).

This document is the wire contract. The Worker firmware is a separate codebase;
anything here marked **MUST** is binding on both ends.

---

## 1. Physical Layer

### 1.1 Signals

| Signal   | Supervisor GPIO | Direction (supervisor view) | Notes |
|----------|-----------------|-----------------------------|-------|
| PIX0     | 32              | bidirectional, open-drain   | data nibble bit 0 |
| PIX1     | 33              | bidirectional, open-drain   | data nibble bit 1 |
| PIX2     | 34              | bidirectional, open-drain   | data nibble bit 2 |
| PIX3     | 35              | bidirectional, open-drain   | data nibble bit 3; doubles as Worker **attention** while clock parked (§5.3) |
| PIX_CLK  | 36              | **output only** (supervisor → worker) | gated clock, push-pull |
| GND      | —               | —                           | common ground **MUST** be shared |

Five wires plus ground. PIX_CLK is point-to-point push-pull (single driver, the
Supervisor). The four PIX lines are a shared open-drain bus.

On the Supervisor, all six pins live in PIO `pio2`'s GPIO window (16–47), so a
single PIO instance both generates PIX_CLK and drives/samples PIX[3:0]. The
Worker may assign its own pins freely within one of its PIO windows.

### 1.2 Open-drain signalling (wired-AND)

Each PIX line is **open-drain with an external pull-up**. A line is never driven
high. Signalling is by GPIO *direction*, with the output data latch held at 0:

| Intent      | Pin direction | Line voltage | Logic value |
|-------------|---------------|--------------|-------------|
| send a `1`  | input (hi-Z)  | pulled high  | **1** (recessive) |
| send a `0`  | output (driven 0) | low      | **0** (dominant)  |

The line value is the **wired-AND** of all drivers: any party driving `0` wins.
Consequences that the rest of the protocol relies on:

- Two parties driving `0` simultaneously is electrically safe — no contention,
  no short. (This is the whole point of the open-drain scheme.)
- A party can only ever *lose a `1`*: if you released a line (intending `1`) but
  read back `0`, someone else is driving it. This is the basis of both
  arbitration (§5) and per-nibble collision detection (§6).

### 1.3 Pull-ups and drive

External pull-ups are **mandatory** — the RP2350 internal pull-up (~50–80 kΩ) is
3–4× too slow against two-Pico + trace capacitance for a sub-µs half-cycle.

| PIX_CLK rate | Pull-up | Half-period | Pull-up τ (≈30 pF) |
|--------------|---------|-------------|--------------------|
| ≤ 1 MHz      | 2.2 kΩ  | ≥ 500 ns    | ~65 ns |
| ≤ 3 MHz      | 1.0 kΩ  | ≥ 167 ns    | ~30 ns |

Pull-ups go to **one** agreed 3V3 rail (the Supervisor's). Both ends configure
their PIX pads for high sink-drive (≥ 8 mA). At 2.2 kΩ a driven-low line sinks
1.5 mA; at 1 kΩ, 3.3 mA. Both 3V3 rails should be the same nominal voltage.

---

## 2. Clocking and Bit Timing

### 2.1 PIX_CLK ownership

PIX_CLK is generated **only** by the Supervisor and is **gated**: it toggles only
during an active transaction and is parked low when the bus is idle. The
Supervisor decides when to start and stop the clock, in **both** transfer
directions — when the Worker transmits (§5.3), the Supervisor still sources the
clock and the Worker drives data synchronised to it.

The clock is independent of the 6809 E clock and runs at the Supervisor's chosen
PIO divider, so PIX traffic is unaffected by 6809 run-state (halt/stop/reset).

### 2.2 DDR nibble timing

Eight bits per clock period, both edges carry a nibble:

```
            ___________             ___________
PIX_CLK ___|           |___________|           |___________  (parked low between
           ^rise       ^fall       ^rise       ^fall          transactions)
           |           |           |
   launch  lo nibble   hi nibble   lo nibble  ...
   sample             ^lo          ^hi         ^lo
                      (at fall)    (at rise)
```

- **Rising edge** carries the **low nibble** (data bits [3:0] on PIX[3:0]).
- **Falling edge** carries the **high nibble** (data bits [7:4] on PIX[3:0]).
- Byte assembled as `(high_nibble << 4) | low_nibble`.

**Source-synchronous, one half-cycle pipeline.** The transmitter launches a
nibble *on* the edge that opens its window and holds it for the half-period. The
receiver samples *on the next edge*, when the open-drain line has settled —
i.e. it reads the nibble that was launched on the previous edge. Both ends act
on the same edges, offset by one phase.

Bit→pin mapping is direct (line value = bit value); see §1.2. Transmitters that
drive via PIO PINDIRS load the **inverted** nibble into PINDIRS (1→input/`1`,
0→output/`0`) with the output latch held at 0.

---

## 3. Frame Format

A frame is the unit one party transmits before turning the bus around. Identical
layout in both directions.

```
  byte 0      byte 1     bytes 2 .. (1+LEN)        byte 2+LEN
┌─────────┬───────────┬───────────────────────┬───────────────┐
│   SOF   │    LEN     │       PAYLOAD         │     CRC8      │
└─────────┴───────────┴───────────────────────┴───────────────┘
   8 bits     8 bits        LEN × 8 bits           8 bits
```

| Field   | Size | Meaning |
|---------|------|---------|
| SOF     | 1 B  | Start-of-frame / role byte. Bit 7 = sender priority (0 = Supervisor, 1 = Worker), see §5.2. Bits [6:0] = frame type / sequence (see §4). |
| LEN     | 1 B  | Payload length in bytes. `1..255` = that many bytes; **`0` encodes 256** (a zero-length payload is never sent). |
| PAYLOAD | LEN B | Opaque to the link layer. Any byte value is legal, including `0x00` and `0xFF`. |
| CRC8    | 1 B  | CRC-8 over SOF, LEN, and PAYLOAD (see §7). |

Framing is **length-counted**: once SOF is seen, the receiver reads exactly
`LEN`+2 further bytes. There is **no byte-stuffing or escaping** — payload may
contain any value, including the idle pattern `0xF` nibbles. Idle is only
re-detected after the turnaround completes (§4).

Maximum frame on the wire = 1 (SOF) + 1 (LEN) + 256 (payload) + 1 (CRC) = 259
bytes = 259 clock periods.

---

## 4. Transaction and Turnaround

A **transaction** = one data frame followed by the peer's acknowledgement, all
under one continuous run of PIX_CLK. The Supervisor starts the clock at the
beginning and stops it after the ACK.

```
  ┌──────────────── PIX_CLK runs ────────────────┐
  │                                               │
  │  sender frame        gap        receiver ACK  │
  │ [SOF LEN .. CRC8]   (1 byte)   [ACK | NAK]    │
  │                                               │
park└───────────────────────────────────────────┘park
```

### 4.1 Turnaround

After the sender's CRC8 byte, the bus turns around:

1. **Gap (1 byte-time):** the sender releases all four PIX lines (all `1`, the
   `0xF` idle pattern). The Supervisor keeps clocking through the gap. This
   one-byte dead time covers the direction handover and line settling.
2. **ACK byte:** the receiver drives a single status byte:

   | Byte   | Name | Meaning |
   |--------|------|---------|
   | `0x06` | ACK  | frame received, CRC valid |
   | `0x15` | NAK  | CRC mismatch, framing error, or collision — sender **MUST** retransmit |

3. The Supervisor stops PIX_CLK; bus returns to idle.

On NAK or on no ACK within the ACK slot, the sender retransmits the same frame
(same SOF sequence bits). Retransmit policy (retry count, backoff) is
implementation-defined but **MUST** be bounded.

### 4.2 SOF sequence bits

SOF bits [6:0] carry a frame type and a 1-bit sequence number so a retransmitted
frame is distinguishable from a new one (guards against a lost ACK causing a
duplicate delivery). Type/sequence encoding is reserved for §8; v1 requires only
that the sequence bit toggles per *new* frame and is held across retransmits.

---

## 5. Arbitration — Supervisor Priority

Both ends may originate frames ("each packet from the supervisor will be
acknowledged by the worker and vice-versa"). The Supervisor is the clock master
and **always wins** a contended start.

### 5.1 Idle state

Bus idle ⇔ **PIX_CLK parked low** and all four PIX lines released (`0xF`).

### 5.2 Supervisor-initiated frame

The Supervisor simply starts PIX_CLK and transmits its frame (SOF bit 7 = 0).
The Worker, seeing clock edges begin, synchronises and receives.

### 5.3 Worker-initiated frame (attention + grant)

The Worker cannot clock, so it requests the bus:

1. **Attention:** while PIX_CLK is parked, the Worker pulls **PIX3 low** and
   holds it. With the clock parked no nibbles are in flight, so a static low on
   PIX3 is unambiguous as a request (not data).
2. **Detection:** the Supervisor detects PIX3-low (GPIO edge interrupt, serviced
   on **Core 0** — Core 1 is saturated with 6809 bus supervision).
3. **Grant:** when the Supervisor is willing to yield the bus, it starts
   PIX_CLK. The Worker releases PIX3 and begins transmitting its frame
   (SOF bit 7 = 1) on the first clock edge. The Supervisor reads the Worker's
   `LEN` byte as it arrives and clocks exactly `LEN`+2 more bytes, then drives
   the ACK and stops the clock (**adaptive clocking** — the Supervisor never
   needs to know the length in advance).

### 5.4 Simultaneous initiation

If the Worker is asserting attention (PIX3 low) and the Supervisor also wants the
bus, the Supervisor **just transmits its own frame**: it starts PIX_CLK and
drives its SOF. The first SOF bit the Worker observes is bit 7 = 0 (Supervisor,
dominant) pulling against the Worker's intended 1 → the Worker detects the
read-back mismatch (§6), abandons its request, and becomes the receiver. Its
attention request persists and is serviced after the Supervisor's transaction
completes. This is the supervisor-priority rule; no separate arbitration phase
is needed.

---

## 6. Collision Detection

Every transmitted nibble is **read back on the sampling edge and compared to the
intended value**. Because of wired-AND (§1.2), a mismatch can only appear on a
bit the transmitter sent as `1` (recessive) that some other party pulled to `0`.

- **During a frame** (point-to-point, single transmitter): a mismatch indicates
  desync, noise, or an unexpected second driver. The transmitter **MUST** abort
  the frame; the transaction resolves to NAK and is retransmitted.
- **At SOF** (the only normally-contended moment, §5.4): the mismatch *is* the
  arbitration outcome — the loser (always the Worker) silently drops to receiver.

The receiver independently validates with CRC8 (§7); collision detection and CRC
are complementary (one catches drive contention live, the other catches
corrupted-but-uncontended bytes).

---

## 7. CRC-8

- **Polynomial:** `0x07` (CRC-8/CCITT-ATM), MSB-first.
- **Initial value:** `0x00`.
- **No reflection, no final XOR.**
- **Coverage:** SOF, LEN, then PAYLOAD bytes in transmission order. The CRC byte
  itself is not included.

Reference (table-free) implementation, identical on both ends:

```c
uint8_t pix_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07)
                               : (uint8_t)(crc << 1);
    }
    return crc;
}
```

A 256-entry lookup table derived from the same polynomial is the expected
production form on both ends.

---

## 8. Reserved / Future (v1 leaves these open)

- **SOF type field** (bits [6:0]): a frame-type enumeration (data, control,
  read-request, etc.) is reserved here. v1 senders set the sequence bit and
  otherwise zero; v1 receivers ignore the type bits beyond bit 7 and the
  sequence bit.
- **Unsolicited low-latency Worker events:** v1 delivers Worker→Supervisor data
  via the attention/grant path (§5.3). If a dedicated event line is later
  needed, it is an additive change.
- **Flow control / windowing:** v1 is stop-and-wait (one frame, one ACK). A
  windowed mode may be layered on the SOF type field later without changing the
  physical or framing layers.

---

## 9. Throughput

One byte per PIX_CLK period:

| PIX_CLK | Raw    | Efficiency (256 B payload, 259 B frame + ~2 B ACK/gap) | Net |
|---------|--------|--------------------------------------------------------|-----|
| 1 MHz   | 1 MB/s | ~98%                                                   | ~0.98 MB/s |
| 3 MHz   | 3 MB/s | ~98%                                                   | ~2.9 MB/s  |

A full 256-byte transaction occupies ~261 clock periods (~261 µs at 1 MHz,
~87 µs at 3 MHz).

---

## 10. Supervisor Implementation Map (informative)

Not binding on the Worker; included so both authors share a mental model.

| Concern | Where |
|---------|-------|
| Pin constants (`GPIO_PIX_BASE = 32`, `PIX_CLK = 36`) | `include/pico_thing.h` |
| PIX_CLK gen SM + transceiver SM | `src/pico_thing.pio` (`pio2`, window 16–47, alongside `task_output`) |
| Nibble launch/sample, PINDIRS inversion, read-back collision | transceiver SM |
| DMA feed/drain of the transceiver FIFO | `src/pico_thing.cpp` |
| Turnaround, CRC8, ACK/NAK, attention IRQ | Core-0 ISR in `src/pico_thing.cpp` |
| CRC-8 | host-side unit-tested (`tests/`) |

`pio1` is full; `pio0` is reserved for the 6809 bus engine. PIX lives entirely on
`pio2`, which already runs in the 16–47 GPIO window for the TASK pins.
