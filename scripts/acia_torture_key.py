#!/usr/bin/env python3
"""Run chunk/acia_torture end to end and expect 12/12.

One invocation does the whole recipe: identify the console, stop the guest,
load the S-records, open the guest's port, start the suite, press the key test
9 waits for, and read the verdict.  Exit status is 0 only on "ALL TESTS
PASSED", so it serves as a regression gate after any change to the ACIA, the
bus service path or the interrupt plumbing.

    P=/Users/markmurray/GitHub/PICO/venv/bin/python
    $P scripts/acia_torture_key.py                  the whole recipe
    $P scripts/acia_torture_key.py --watch-only     drive the console yourself
    $P scripts/acia_torture_key.py --power          power the guest on first

The S-records are reloaded every run, deliberately: **the suite cannot be
re-entered from RAM.** A "go 0191" after a completed run produces no output at
all - no error on the console either - so a --no-load option was tried, found
to do nothing useful, and removed.

Two ports, because the suite talks to neither one exclusively: commands go to
the debug probe's UART bridge, while the SUITE'S OWN OUTPUT is the guest
writing to its console ACIA, which surfaces on CDC 0.  Looking for the verdict
on the UART console finds nothing at all.

**The guest's port is opened BEFORE the suite starts.** A closed CDC port means
the firmware discards what the guest writes, so output that arrives before the
port opens is simply gone.

Test 9 is "Host RX IRQ (press a key)": the guest enables RX interrupts and
waits for a character from the host to raise one.  Unattended it times out, so
an otherwise clean run reports 11/12 - and the path it skips is host -> RDRF ->
IRQ -> handler, which is exactly what NitrOS-9 over DriveWire depends on.  The
key is pressed when test 9's BANNER appears, not at the start: tests 1-8 are
loopback tests that read RX themselves, and a character arriving early is
consumed by one of them and fails it.

Needs the bench venv for pyserial.  The console port is guarded by the same
signature check as console.py - see the warning there about the FT230X.

CDC 1 (/dev/cu.usbmodem68093) belongs to DriveWire and is never touched here.
"""

import argparse
import pathlib
import sys
import time

import serial

# console.py lives alongside this, and owns the console port's identity and its
# signature guard.  Importing it keeps one copy of both.  No bytecode, so the
# import does not litter scripts/ with a __pycache__ nobody asked for.
sys.dont_write_bytecode = True
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import console                                      # noqa: E402

# CDC 0, the guest's console ACIA.  Both CDC ports report identical vid/pid,
# serial and USB location, so the node is the only handle there is - and it is
# identified here by what came out of it, not by metadata.
GUEST_PORT = "/dev/cu.usbmodem68091"
SREC = "chunk/acia_torture.s19"
ENTRY = "0191"


def identify(ser, force):
    """One CR, and refuse to go further unless a Pico-Thing answers."""
    for n in range(3):
        ser.write(b"\r")
        ser.flush()
        reply = console.drain(ser, settle=0.4, limit=5.0)
        if any(s in reply for s in console.SIGNATURE):
            return True
        print(f"no signature in reply {n + 1}/3")
    if force:
        print("WARNING: no signature, continuing because --force")
        return True
    print("REFUSING to drive this port: no Pico-Thing signature. Check the"
          " port, or pass --force.")
    return False


def command(ser, cmd, settle=0.4, limit=15.0):
    print(f">> {cmd}")
    ser.write(cmd.encode() + b"\r")
    ser.flush()
    return console.drain(ser, settle=settle, limit=limit).decode(
        "utf-8", "replace")


def load_srec(ser, path):
    """Send an S-record file a line at a time, waiting for the prompt."""
    lines = [l.strip() for l in open(path) if l.strip()]
    print(f"loading {path}: {len(lines)} S-records, paced")
    bad = 0
    for n, line in enumerate(lines, 1):
        ser.write(line.encode() + b"\r")
        ser.flush()
        reply = console.drain(ser, settle=0.15, limit=5.0).decode(
            "utf-8", "replace")
        if "nvalid" in reply:
            bad += 1
            print(f"  line {n}/{len(lines)} REJECTED: {line}")
    print(f"load finished: {len(lines) - bad} accepted, {bad} rejected")
    return bad == 0


def watch(guest, char, banner, timeout):
    """Read the suite's output, pressing one key when it asks for one."""
    heard = bytearray()
    sent = False
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = guest.read(256)
        if not chunk:
            continue
        heard += chunk
        if not sent and banner.encode() in heard:
            time.sleep(0.05)		# let the guest reach its wait loop
            guest.write(char.encode())
            guest.flush()
            sent = True
            print(f"--- pressed {char!r} on seeing {banner!r} ---", flush=True)
        if b"Results:" in heard:
            time.sleep(0.5)		# the verdict lines trail the count
            heard += guest.read(512)
            break
    return bytes(heard), sent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--console", default=None,
                    help="the MAIN probe's UART bridge; found by adapter serial"
                         " if not given, since the device node moves")
    ap.add_argument("--guest", default=GUEST_PORT,
                    help="CDC 0, where the suite's own output appears")
    ap.add_argument("--srec", default=SREC)
    ap.add_argument("--entry", default=ENTRY)
    ap.add_argument("--char", default="A",
                    help="the key to press (default A, echoed by the test)")
    ap.add_argument("--banner", default="Test  9",
                    help="the text to wait for before pressing a key")
    ap.add_argument("--timeout", type=float, default=40.0,
                    help="give up this long after starting the suite")
    ap.add_argument("--power", action="store_true",
                    help="'power on' and 'run 0' first, then let it settle")
    ap.add_argument("--watch-only", action="store_true",
                    help="touch the console not at all: watch, press, report")
    ap.add_argument("--force", action="store_true",
                    help="drive the console even if the signature check fails")
    a = ap.parse_args()

    if a.console is None:
        a.console = console.find_console_port()
        if a.console is None:
            print("cannot find the main probe's console; see console.py --help")
            return 2

    try:
        guest = serial.Serial(a.guest, console.BAUD, timeout=0.2)
    except serial.SerialException as e:
        print(f"cannot open the guest port {a.guest}: {e}")
        return 2

    # Opened first and held open for the whole run: the firmware discards the
    # guest's output while nothing is attached.
    with guest:
        if a.watch_only:
            print(f"watching {a.guest} - start the suite yourself")
            heard, sent = watch(guest, a.char, a.banner, a.timeout)
        else:
            try:
                ctrl = serial.Serial(a.console, console.BAUD, timeout=0.2)
            except serial.SerialException as e:
                print(f"cannot open the console {a.console}: {e}")
                return 2
            with ctrl:
                console.drain(ctrl, settle=0.2, limit=2.0)	# pending output
                if not identify(ctrl, a.force):
                    return 2
                if a.power:
                    command(ctrl, "power on", settle=3.0, limit=20.0)
                    # DAT_INIT, and not optional: a freshly powered guest has
                    # no identity mapping, so the S-records load into RAM the
                    # CPU cannot then fetch from.  The symptom is a single
                    # junk character on the guest port and no banner at all.
                    command(ctrl, "run 0", settle=2.0, limit=20.0)
                command(ctrl, "sp")
                if not load_srec(ctrl, a.srec):
                    print("S-records were rejected - not starting")
                    return 1
                command(ctrl, f"go {a.entry}", settle=1.0, limit=20.0)
                heard, sent = watch(guest, a.char, a.banner, a.timeout)

    sys.stdout.write(heard.decode("utf-8", "replace"))
    if not heard:
        print("\nnothing came back on the guest port - was the suite running?")
        return 1
    if not sent:
        print(f"\nnever saw {a.banner!r}, so no key was pressed")
    if b"ALL TESTS PASSED" in heard:
        return 0
    print("\nNOT a clean run")
    return 1


if __name__ == "__main__":
    sys.exit(main())
