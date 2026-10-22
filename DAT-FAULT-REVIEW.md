# DAT fault mechanism — firmware vs. hardware design review

Review of the firmware DAT fault implementation in this repo against the
hardware design in `../pico-thing` (v2) and `../pico-thing-1.0` (v1.0).
When it was written v2 was still on paper; it has since been built, with the
v1 kluge board's RAM decode incorporated as the final design, and v2 is the
board now in use.

## Since this review (updated 2026-10-22)

The findings below are kept as written, with dated notes where the design has
moved on.  In short:

- **U208's Fault is combinational and the GAL no longer raises NMI**
  (2026-10-11).  The registered `Fault.D`, clocked on Q rising, sampled the
  DAT RAM's page output before it was valid: at 2.5 MHz and above a DAT RAM
  that had been deselected read as page `$FF`, so it faulted spuriously.  U208
  now has `Fault = MMU_dat & Last_Block;` and `NMI = 'b'0;`, and Q201 (the
  PN2222A that pulled `~NMI` for the GAL) is removed.
- **The Pico raises the fault NMI itself**, through `pending_interrupts`, and
  NMIBlock is now enforced only by the Pico; no GAL equation reads it.
- **The fault is sampled by a PIO state machine** (`bus_fault`, on pio2) at Q
  falling.  Its one `in pins` takes FAULT, the task pins and A13-A15 in the
  same instant, which is L1's option 2 done in the Pico rather than the GAL.
- **The bus trace records GPIO 32-47** alongside GPIO 0-31, so Fault, NMIBlock,
  the task pins and A13-A15 are all visible in it.
- **The automatic task restore is gone.**  It was driven by the RTI opcode
  detector, removed on 2026-10-06; nothing restores the task after a fault
  NMI, so L6 no longer applies.
- **v2's FAULT and A13-A15 (GPIO 41, 43-45) are on the RP2350B's ADC pins,
  which are not 5 V tolerant.**  Clamped on the v2 board on 2026-10-19; v3
  moves them to GPIO 23-26.

Sources compared:

- `../pico-thing/docs/dat-mmu.tex` — the DAT MMU explainer
- `../pico-thing/GAL/MC6809_RAM_GLUE_1.PLD` (U209) and
  `MC6809_RAM_GLUE_2.PLD` (U208) — the authoritative DAT policy
- `../pico-thing/pico-thing.kicad_sch` — v2 pin assignment
- `../pico-thing-1.0/pico6809.kicad_sch` — v1.0 pin assignment
- `include/pico_thing.h`, `src/pico_thing.cpp` — this firmware

Pin assignments were derived from schematic symbol geometry, not from a
generated netlist. High confidence, but worth confirming against a netlist
before cutting copper.

## Checklist

Discrepancies:

- [x] **D1** — ~~v1 firmware fault pins collide with PIX0-3 and Trace-C on the v1.0 board~~ — **closed, not a problem**
- [x] **D2** — ~~Trace-C and Trace-D are swapped between the v2 schematic and the firmware~~ — **closed, not a problem**
- [x] **D3** — ~~`dat-mmu.tex` puts the Task register at `$FF00`; it is at `$FFC0`, and `$FF00` is the IDE~~ — **fixed**
- [x] **D4** — ~~`dat-mmu.tex` does not document the Pico-side fault mechanism at all~~ — **fixed**

Design limitations:

- [x] **L1** — ~~Fault is registered on Q, but the address and task pins are live; the pairing is only incidentally safe~~ — **fixed**
- [x] **L2** — ~~Task 0 has no memory protection, and `$FFCC` bit 0 is described wrongly for that case~~ — **fixed**
- [x] **L3** — ~~No lost-fault indicator; a second fault silently overwrites the first~~ — **fixed, first-fault-wins**
- [x] **L4** — ~~The page field can only ever hold 0-6; block 7 cannot fault~~ — **documented, and repurposed as a check on L1**
- [x] **L5** — ~~NMIBlock write-to-effect latency is asserted but unmeasured~~ — **no longer a GAL path**; what is left is Pico-internal
- [x] **L6** — ~~The automatic task restore assumes every interrupt resumes what it interrupted~~ — **moot, the restore was removed**

The hardware half of L1 (does the DAT lookup settle before Q rises?) has been
answered on the v2 board: **it does not**, at 2.5 MHz and above.  That was the
spurious-fault race described at the top, fixed in U208 on 2026-10-11.

The trace has been widened as this section once suggested: each row now carries
GPIO 32-47 as well as GPIO 0-31.

## What already matches

No action needed on these; recorded so they are not re-litigated.

- `Fault` and `NMIBlock` are both declared active high in
  `MC6809_RAM_GLUE_2.PLD` (`PIN 17 = Fault;`, `PIN 16 = NMIBlock`), which is
  what the firmware assumes.
- A13/A14/A15 land on GPIO 43/44/45 ascending on the v2 board, so the `A_HI`
  bitfield reads directly as the page number with A15 as its most significant
  bit. No reversal needed.  (Those pins are the ones that are not 5 V
  tolerant; see the top of this file.)
- The DAT index the doc derives for its worked example, `(25 << 3) | 4 = $CC`,
  is exactly what `SYSTEM_DAT_FAULT_LOC` produces.
- ~~The GAL raises NMI itself (`NMI.D` on U208), so the firmware is correct to
  latch only and never assert NMI for a fault.~~ **Superseded 2026-10-11:**
  U208's NMI output is `'b'0` and the Pico asserts the fault NMI.
- `ASSERT_NMI` / `DEASSERT_NMI` toggle pin direction rather than driving a
  level (`include/mc6809.h:34`), so `~NMI` stays open-drain.  The PN2222A it
  once wire-ORed with (Q201) has been removed, so the Pico is now the only
  driver.
- The v2 board pin map matches the firmware's version 2 map exactly, apart
  from D2 below.

## Discrepancies

### D1 — CLOSED, not a problem

**Resolution:** PIX was never used, and the trace pins are freely
re-allocatable. GPIO 32-36 on the v1.0 board are therefore available for the
kluge board, and the firmware's version 1 pin map stands as written — FAULT 32,
NMI_BLOCK 33, A13-A15 34-36, with the firmware's TRACE_C/TRACE_D at 38/39
simply re-labelling the board's Trace-E/Trace-F. No firmware change needed.

The original finding and the pin table are kept below as the record of what
the v1.0 schematic says, since the kluge board still has to land on these pins.

Original finding follows.

| GPIO | v1.0 board | Firmware v1 (`include/pico_thing.h:87`) |
|------|--------------------------------|-----------------------------------------|
| 32-35 | **PIX0-PIX3** | FAULT (32), NMI_BLOCK (33), A13-A14 (34-35) |
| 36 | **Trace-C** | A15 |
| 37 | Trace-D | *(left free)* |
| 38-39 | Trace-E, Trace-F | TRACE_C, TRACE_D |
| 40-44 | Task0-Task4 | same |
| 45 | TASK0 | same |
| 46 | POWER | same |

The task and power pins agree. Everything the firmware placed below GPIO 40
sits on the PIX video link and Trace-C — but PIX is unused and the trace pins
are re-allocatable, so both are available. GPIO 47 is the PSRAM `!CS` on the
PGA2350 and is not available.

Note the `PIX link protocol specification` (commit f542431) in this repo
describes a link that was never built; if it is ever revived it will need pins
other than 32-35.

### D2 — CLOSED, not a problem

**Resolution:** the trace pins are debug outputs whose names are arbitrary, and
the v1 board never brought them out to test points, so the firmware is free to
assign them as it likes. No change needed on either side.

Original finding, kept as the record of the mismatch:

The v2 schematic has GPIO 38 = Trace-D and GPIO 39 = Trace-C. The firmware has
`GPIO_TRACE_C = 38`, `GPIO_TRACE_D = 39` (`include/pico_thing.h:72`).

One detail for the bench: unlike v1, the **v2** schematic does route these two
to test points (adjacent to the Pico, on the GPIO 38 and 39 nets). Their
silkscreen names will therefore disagree with the firmware constants — the pin
labelled Trace-D carries what the firmware calls TRACE_C. Worth knowing when
probing a v2 board, but not worth a change.

### D3 — FIXED

**Resolution:** all 11 `FF00` references in `../pico-thing/docs/dat-mmu.tex`
changed to `FFC0` — two prose statements, the two software examples
(`CLR $FFC0`, `STA $FFC0`) and the five TikZ diagram labels. A new paragraph
after the Task-register description now makes the `IOS` / `CS_PICO` split
explicit: `$FFC0` is in the Pico's window, so a write to the Task register is
captured by the Pico's bus-write DMA, which drives the five TASK pins and the
`Root` pin the DAT GALs consume — and it is `Root`, not the Task register
directly, that gates the DAT RAM window. `dat-mmu.pdf` regenerated with the
three `pdflatex` passes from `build.sh`.

Original finding follows.

`docs/dat-mmu.tex` places the Task register at `$FF00` in thirteen places,
including every TikZ diagram and the worked software examples (`CLR $FF00` in
the reset-handler description, `STA $FF00` for a task switch).

The Task register is `SYSTEM_TASK`, offset 0 of `REGISTER_BASE`, i.e.
**`$FFC0`**. `MC6809_RAM_GLUE_1.PLD` confirms the split:

```
IOS       = ADDR:['h'FF00..'h'FFBF];   /* Regular I/O space */
CS_PICO   = ADDR:['h'FFC0..'h'FFFF];   /* Pico-supplied pseudo-memory and registers */
```

and `$FF00-$FF09` is the PATA IDE register map. A kernel writer following the
doc would write task numbers into the IDE's device-control register.

The doc needs correcting, not the firmware.

### D4 — FIXED

**Resolution:** added a new *The fault registers* subsection to
`../pico-thing/docs/dat-mmu.tex` covering `$FFCC` bit by bit (Fault,
R/!W, NMIBlock), `$FFCD`'s two fields, and the fact that `$FFCD` added to
`$FE00` is the address of the offending DAT slot. The *Hardware
implementation* subsection now quotes the `Fault.D` / `NMI.D` equations, the
wire-OR against the Pico's open-drain `~NMI`, and spells out the task-0
consequence. The NMI-handler paragraph no longer tells the reader to
reconstruct the fault from the pushed PC, and *Further reading* points at the
firmware. `dat-mmu.pdf` regenerated.

Note this documents L2 and L3 as current behaviour — if either is changed
later, this section needs revisiting.

Original finding follows.

`dat-mmu.tex` covers the GAL's NMI generation but never mentions the `Fault`
line to the Pico, the `NMIBlock` input, or the `$FFCC` / `$FFCD` registers.

Its "Software conventions" section tells the NMI handler to recover the
faulting address from the pushed PC and the Task register. That is both harder
and less exact than reading `$FFCD`, which gives `(task << 3) | block`
directly — the DAT RAM index of the offending slot.

Worth a new subsection covering the register pair, the read-clear behaviour,
and the NMIBlock masking bit.

## Design limitations

### L1 — FIXED

**Update 2026-10-22:** the sampler is now the `bus_fault` PIO state machine on
pio2, not an ISR branch.  It waits for Q falling and takes FAULT, the task pins
and A13-A15 in one `in pins`, so the whole record comes from one instant; the
ISR (`pio_bus_fault_irq_handler`) only unpacks it.  R/!W is still read live in
the ISR, because GPIO 2 is outside pio2's pin window.  The hardware half below
was answered on the board: the DAT lookup does **not** settle before Q rising
at 2.5 MHz and above, and U208's Fault is now combinational (see the top of this
file).

**Resolution (original):** the sampler moved from the Q-rising branch to the Q-falling
branch of `gpio_clock_eq_irq_handler`. Confirmed by MRVM that the 6809
guarantees BA, BS, R/!W and A0-A15 stable from before Q rises until ~20 ns
after E falls, so Q-falling sits mid-window: the GAL's registered outputs have
long settled, and the live bus pins still describe the same cycle. The margin
is now structural rather than a side effect of how much work the ISR happens
to do first. No new interrupt — the Q-falling branch was already enabled and
nearly empty.

R/!W now comes from `gpio_get(GPIO_RW)` rather than the `bus_pins` word, which
only exists in the Q-rising branch.

The hardware-side half of the original concern (failure mode 2 — whether the
DAT lookup settles before Q-rise, which would make the GAL's own capture a
cycle late) is **not** addressed by this and cannot be from firmware. It
needs the real t(PD) figures against the 6809's address-valid-to-Q timing,
especially at the 3 MHz the doc targets. Left as a hardware question.

Original finding follows.

`Fault.D` is a D flip-flop on U208, and pin 1 of that GAL is **Q**:

```
PIN 1 = Q;
Fault.AR = 'b'0;
Fault.SP = 'b'0;
Fault.D  = MMU_dat & Last_Block;
```

So Fault updates on Q-rising and describes the cycle whose address was valid
at that edge. A13-A15 and the task pins are live wires with no such register.

The firmware samples all of them in a single `gpio_hi_in` read from the
Q-rising ISR (`src/pico_thing.cpp:254`) — that is, right at the flop's own
clock edge. It works only because the ISR performs roughly 100 ns of other
work before reaching the sample, comfortably past the GAL's t(CO) plus the
RP2350 input synchroniser. **That margin is incidental, not designed.**

The failure mode is not a missed fault. If the sample lands early, or if the
DAT lookup does not settle before Q-rise, cycle N-1's fault flag is paired
with cycle N's address and task, and the reported location is *silently
wrong*.

Options, in increasing order of robustness:

1. Move the firmware sample to E-falling, where the bus trace already samples
   and every bus signal has settled.
2. Register A13-A15 and TASK in the GAL alongside Fault, so the whole record
   is captured on one edge and the firmware can read it at leisure.

Option 2 also fixes it for any future consumer of the fault signals.

### L2 — FIXED

**Update 2026-10-22:** the GAL equations quoted below are out of date: U208 now
has a combinational `Fault = MMU_dat & Last_Block;` and `NMI = 'b'0;`, and the
Pico raises the fault NMI.  The task-0 switch at the NMI vector fetch still
stands.  The sentence about `apply_rti()` does not: the automatic restore was
removed with the RTI detector, so nothing restores the faulting task (see L6).

**Resolution (original):** two halves.

*Hardware (MRVM, done).* `!Root` came out of `NMI.D` in
`MC6809_RAM_GLUE_2.PLD`, so every task including Task 0 raises an NMI on an
unassigned access. The two equations now differ only in `NMIBlock`. Recompiled
through CUPL, so `MC6809_RAM_GLUE_2.jed` and `.doc` match; the expanded
equations read

```
Fault.d => MMU_dat & PA13 & PA14 & PA15 & PA16 & PA17 & PA18 & PA19 & PA20
NMI.d   => MMU_dat & !NMIBlock & PA13 & ... & PA20
```

with `Last_Block` inlined as the eight-way AND of PA13-PA20. Both are
permanently enabled registered outputs clocked on pin 1 (Q), `.ar` and `.sp`
tied to 0. `Root`, `R_nW` and `E` remain declared but unread in this device.

*Firmware.* A fault NMI now takes its vector with the guest switched to task 0,
so the handler can reach the DAT RAM window — which is still Root-gated
(`MMU_cs = ADDR:[FE00..FEFF] & Root` in `MC6809_RAM_GLUE_1.PLD`, unchanged).

The switch happens in the `BUS_IRQ` case, not at fault-latch time, so that:

- the rest of the faulting instruction and the CPU's register push both
  complete under the faulting task's own mapping, leaving the stack frame
  where that task expects it;
- the faulting task is already on `task_stack` when the switch happens, so
  `apply_rti()` restores it. The handler must **not** restore the task itself.

A `dat_fault_nmi_pending` flag, set at latch time and cleared at the vector
fetch, keeps the timer and vector-watchpoint NMIs out of it — they are still
delivered in whatever task was running.

This also removes the L3 corollary about a Task 0 fault disarming the
mechanism with nothing to prompt a read: Task 0 faults now raise an NMI like
any other.

Original finding follows.

`NMI.D` includes `& !Root`, but `Fault.D` does not:

```
Fault.D = MMU_dat & Last_Block;
NMI.D   = MMU_dat & Last_Block & !Root & !NMIBlock;
```

Trace the chip selects for a task-0 access to an unassigned block:
`Last_Block` is true, so `CS_RAM3`'s first term needs `!Last_Block` and fails;
`MMU_fixed` is false; and `PADDR` is above the ranges of `CS_RAM0/1/2`. **No
RAM chip is selected at all.** The read returns a floating bus and the write
goes nowhere, with no NMI raised.

`$FFCC` bit 0 is the only record that such a fault ever happened. That is
presumably why `Fault.D` deliberately omits `!Root`, and it makes the register
considerably more valuable than its own comment suggests.

Two consequences:

- The header currently describes the register as identifying "a fault as the
  source of an NMI". That is wrong for task 0, and also wrong whenever
  NMIBlock is set — in both cases the bit sets with no NMI. Reword.
- Worth a kernel-side convention that task 0 polls `$FFCC`, since it is the
  only protection task 0 has.

### L3 — FIXED (first fault wins)

**Resolution:** MRVM's design, better than the overflow bit originally
suggested. Latching a fault now also asserts NMIBlock, and while NMIBlock is
set no further fault touches either register. The registers therefore hold the
fault that *started* a storm rather than the last one of it, and the guest
re-arms by writing zero to `$FFCC`.

The NMI for the first fault is unaffected: the GAL captures `NMI.D` at
Q-rising, before the Pico asserts NMIBlock at Q-falling of the same cycle.

**Update 2026-10-22:** the paragraph above describes the registered GAL NMI,
which no longer exists.  The Pico now raises the NMI itself as it latches the
fault, and turns off the sampler's interrupt until the guest clears `$FFCC`.
NMIBlock goes up `DAT_FAULT_BLOCK_DELAY` (32) Q-risings later.  First-fault-wins
is unchanged.

Consequence to watch, recorded under L2: a Task 0 fault latches and sets
NMIBlock but raises no NMI, so nothing prompts the kernel to read the
registers and the mechanism stays disarmed until something polls `$FFCC`.
Documented in `dat-mmu.tex`; see L2 for the option of gating the auto-set on
`!Root`.

Original finding follows.

`$FFCD` is overwritten wholesale by each fault, and `$FFCC` bit 0 is a single
flag. A second fault arriving before the guest reads leaves no trace: the
guest sees only the most recent location and cannot tell it missed one.

A sticky overflow bit in one of the spare bits (2-6) of `$FFCC` would cost
nothing — the sampler at `src/pico_thing.cpp:255` already does a
read-modify-write on that byte.

### L4 — DOCUMENTED, and it is more useful than it looked

**Resolution:** stated explicitly in both `include/pico_thing.h` and
`dat-mmu.tex` that the page field is always 0-6, with the reason.

The reason turns the quirk into something worth having. A page of 7 is not
merely unused, it is *unreachable*: `MMU_dat` covers `$0000-$DFFF` only, so a
block 7 access cannot assert the fault line at all. The Pico reads the fault
flag and the address pins in the same instant, and the flag can only be set by
a block 0-6 access — so if `$FFCD` ever reports page 7, the two came from
**different bus cycles**.

That is a free, always-on consistency check on exactly the failure this review
could not otherwise detect: the silently mislabelled location described under
L1.  (L1's hardware half has since been answered; see L1.) Page 7 is the observable symptom. Anything watching for it —
guest-side assertion, or a DEBUG counter in the sampler surfaced through the
status display — gives that open question a way to answer itself on real
hardware rather than from datasheet timing.

Not implemented, since a counter nothing reads is cruft. Worth adding when L1's
hardware half is actually investigated.

Original finding follows.

`MMU_dat = ADDR:['h'0000..'h'DFFF]` covers blocks 0-6 only; block 7 is
`MMU_fixed` and is hard-wired to physical page `$FF`, so it cannot fault.

Harmless, but the 3-bit page field in `$FFCD` never uses its full range, and
the register documentation implies 0-7.

### L5 — no longer a GAL path

**Update 2026-10-22:** U208 no longer reads NMIBlock (its NMI output is
`'b'0`), so the GAL-side latency below no longer exists.  NMIBlock is now
enforced by the Pico: the fault ISR ignores a sample whose NMIBlock pin was
high.  What is left is the delay from the guest's write to `$FFCC` to the
write ISR raising the pin, all inside the Pico and measurable with the bus
trace, which now records the NMIBlock pin.

Original finding follows.

The firmware sets `GPIO_NMI_BLOCK` in the write DMA ISR
(`src/pico_thing.cpp:448`), which fires around E-fall. The GAL samples
NMIBlock into the NMI flop at the next Q-rise, roughly a quarter to half a
cycle later — about 80-160 ns at 3 MHz.

Commit a86070e asserts that a guest masking faults is protected before its
next instruction can fault. That is probably true but has not been measured,
and it is the kind of thing that fails only under load, when a priority-0 DMA
ISR delays the write handler.

Measure it on the scope, or move the NMIBlock pin write somewhere with a
guaranteed deadline.

### L6 — MOOT, the restore was removed

**Update 2026-10-22:** the automatic task restore was driven by the RTI opcode
detector, which was removed on 2026-10-06 to free PIO space.  Nothing restores
the task after any interrupt now.  A fault NMI still switches the guest to
task 0 at the vector fetch, and from there the handler owns the task register
outright, with no countdown to race.

Original finding follows.

Raised by MRVM: **a fault NMI can never be resumed.** The MC6809E recognises
NMI only at an instruction boundary and has no bus-error or abort mechanism, so
by the time the handler runs the offending access has already returned a
floating bus (or written nowhere — see the chip-select trace under L2) and the
instruction has run to completion on that garbage. Demand paging is not
available; a fault is fatal to the process that caused it, and `$FFCC`/`$FFCD`
are post-mortem evidence rather than inputs to a repair.

The firmware's task bookkeeping does not know that. `BUS_IRQ` pushes the task
in force at the vector fetch, and `apply_rti()` restores it **unconditionally**
about 15 E-cycles after any RTI (`mc6809::apply_rti` sets `rti_task_countdown`,
and `tick_rti_countdown` applies it). For every other interrupt that is exactly
right. For a fault it means a handler that terminates the process and RTIs
onward finds the task register forced back to the **dead task** shortly after.

The delay is what makes this awkward rather than merely surprising: the restore
lands some cycles *after* the RTI has executed, so setting the task register in
the handler before the RTI does not help, and setting it immediately after is a
race against the countdown.

MRVM's intent is for the OS to RTI back into itself and sequence the cleanup
and the eventual task switch around the restore rather than across it. That
works, but it is a constraint on the handler's shape that is invisible from the
guest side — nothing in the register interface hints that a write to `$FFC0`
made in the wrong window will be silently undone.

Options, if it turns out to want firmware help rather than kernel discipline:

- suppress the restore for a fault NMI specifically — the pending flag that
  already distinguishes it (`dat_fault_nmi_pending`) could mark the task-stack
  entry as not-to-be-restored;
- give the guest a way to cancel a pending restore, so the handler can opt out
  explicitly before its RTI;
- leave it to the kernel, and document the timing precisely enough that
  "carefully done" is actionable — the doc now states the 15-cycle behaviour.

Currently documented, not changed.
