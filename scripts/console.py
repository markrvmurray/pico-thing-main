#!/usr/bin/env python3
"""Drive the Pico-Thing UART console, echoing everything verbatim.

The console is the debug probe's UART bridge, not the FT230X - that one is a
serial console into another machine entirely.

    P=/Users/markmurray/Documents/PICO/venv/bin/python
    $P scripts/console.py --listen 5            read only, writes nothing
    $P scripts/console.py --ping                one CR, identify the device
    $P scripts/console.py st flow               identify, then run commands
    $P scripts/console.py --load chunk/x.s19    paced S-record load

Needs the bench venv for pyserial.  Invoke it with that absolute interpreter
path and no "cd ... &&" prefix, so it matches a narrow Claude Code permission
rule of the form:

    "Bash(/Users/markmurray/Documents/PICO/venv/bin/python scripts/console.py:*)"

Safety: the ONLY byte sent before the device is identified is a single CR, and
if the reply carries no Pico-Thing signature nothing further is sent.  That
exists because a stray CR into an unidentified serial port turned out to be a
keystroke in somebody's pager on another machine.

S-records are sent one line at a time, waiting for the prompt between lines:
the console has no flow control and 32 bytes of RX FIFO, which is 2.78 ms of
runway at 115200, and Core 0 blocks for longer than that in several places.
"""
import argparse
import sys
import time

import serial

PORT = "/dev/cu.usbmodem202423102"
BAUD = 115200
SIGNATURE = (b">>>", b"Pico2 MC6809E", b"POWER:", b"Core0", b"Core1",
             b"Bus State", b"Run State")


def drain(ser, settle=0.4, limit=15.0):
    """Read until the stream has been quiet for `settle` seconds."""
    buf = bytearray()
    last = start = time.monotonic()
    while time.monotonic() - start < limit:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk
            last = time.monotonic()
        elif time.monotonic() - last >= settle:
            break
    return bytes(buf)


def show(label, data):
    print(f"--- {label}: {len(data)} bytes ---")
    if data:
        sys.stdout.write(data.decode("utf-8", "replace"))
        if not data.endswith(b"\n"):
            print()
    print(f"--- end {label} ---")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--listen", type=float, metavar="S",
                    help="READ ONLY for S seconds, then exit")
    ap.add_argument("--ping", action="store_true",
                    help="identify only: one CR, show the reply, stop")
    ap.add_argument("--force", action="store_true",
                    help="send commands even if the signature check fails")
    ap.add_argument("--load", metavar="FILE.s19",
                    help="send an S-record file one line at a time, waiting"
                         " for the prompt between lines (the console has no"
                         " flow control and 32 bytes of RX FIFO)")
    ap.add_argument("--timeout", type=float, default=15.0)
    ap.add_argument("--settle", type=float, default=0.4,
                    help="quiet time that ends a reply (default 0.4s). Raise"
                         " it for commands with internal sleeps, such as"
                         " break, which waits for the stacked frame to settle")
    ap.add_argument("command", nargs="*")
    a = ap.parse_args()

    print(f"opening {a.port} at {BAUD} 8N1, no flow control")
    try:
        ser = serial.Serial(a.port, BAUD, timeout=0.2, rtscts=False,
                            dsrdtr=False)
    except serial.SerialException as e:
        sys.exit(f"cannot open: {e}")

    with ser:
        if a.listen:
            print(f"LISTENING {a.listen}s - writing nothing")
            show("heard", drain(ser, settle=a.listen, limit=a.listen))
            return 0

        show("pending on open", drain(ser, settle=0.3, limit=2.0))

        # Up to three CRs, patiently: the bridge occasionally swallows the
        # first one just after the port is opened, and a bare prompt is all
        # we are waiting for.
        reply = b""
        for attempt in range(1, 4):
            print(f">> CR (identification {attempt}/3 - the only pre-check write)")
            ser.write(b"\r")
            ser.flush()
            reply = drain(ser, settle=1.0, limit=5.0)
            if reply:
                break
        show("reply", reply)

        if not any(s in reply for s in SIGNATURE):
            if not a.force:
                print("REFUSING to send commands: no Pico-Thing signature in"
                      " that reply. Check the port, or pass --force.")
                return 2
            print("WARNING: no signature, continuing because --force")

        if a.ping:
            print("identified; stopping here as asked")
            return 0

        if a.load:
            lines = [l.strip() for l in open(a.load) if l.strip()]
            print(f"loading {a.load}: {len(lines)} S-records, paced")
            bad = 0
            for n, line in enumerate(lines, 1):
                ser.write(line.encode() + b"\r")
                ser.flush()
                reply = drain(ser, settle=0.15, limit=5.0)
                text = reply.decode("utf-8", "replace")
                if "Invalid" in text or "nvalid" in text:
                    bad += 1
                    print(f"  line {n}/{len(lines)} REJECTED: {line}")
                    print(f"    {text.strip()}")
                elif n % 10 == 0 or n == len(lines):
                    print(f"  {n}/{len(lines)} accepted")
            print(f"load finished: {len(lines) - bad} accepted, {bad} rejected")
            if bad:
                return 1

        for cmd in a.command:
            print(f">> {cmd}")
            ser.write(cmd.encode() + b"\r")
            ser.flush()
            show(f"reply to {cmd!r}",
                 drain(ser, settle=a.settle, limit=a.timeout))
    return 0


if __name__ == "__main__":
    sys.exit(main())
