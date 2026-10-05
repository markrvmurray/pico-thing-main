#!/usr/bin/env python3
"""Log readings from SCPI multimeters on USB serial ports.

Written for the question "is the Pico's 3.3V rail really at 2V?", which the
firmware cannot answer about itself: every ADC reading is a ratio against the
reference, so a bad reference makes the board's own numbers wrong in exactly
the way that hides the fault.  An outside meter has no such problem.

    P=/Users/markmurray/Documents/PICO/venv/bin/python
    $P scripts/dmm.py --list                       what serial ports exist
    $P scripts/dmm.py --port /dev/cu.usbserial-A --id
    $P scripts/dmm.py --port /dev/cu.usbserial-A:3V3 \
                      --port /dev/cu.usbserial-B:5VP --duration 30

Each --port is PORT[:LABEL].  Readings go to stdout as CSV
(seconds, label, volts) so they can be pasted back or redirected.

THE METERS HERE ARE HANDHELDS AND CANNOT BE DRIVEN.  An Agilent U1252A and a
U1252B: they measure whatever the rotary knob is set to, so there is no
configuring them from this end and no --conf by default.  Consequences:

  - Set both knobs to DC volts before starting, or the log records ohms.
  - CONF? is queried once per meter and printed.  It is a READ of the knob
    position, not a command, and it is what makes a log self-describing -
    without it a column of numbers does not say what it measured.
  - They want LF line endings, which is what ask() sends.
  - FETC? or READ? to read; MEAS? answers *E, its command-error reply.  VAL1?,
    VAL2? and MEAS1? are also rejected.  CONF? and SYST:VERS? work.
  - 9600 baud over the IR-USB cable.
  - Handhelds sleep.  If a long run stops producing readings, auto-power-off
    is the first suspect; it is disabled from the front panel, not from here.

The SCPI spellings are still settable (--idn, --conf, --read, --func) so this
works with a bench meter too, where CONF:VOLT:DC would be worth passing.
"""

import argparse
import sys
import time

import serial
from serial.tools import list_ports


def ask(ser, cmd, settle=0.35):
    """Send one SCPI command and return its reply, or '' if it does not answer."""
    ser.reset_input_buffer()
    ser.write(cmd.encode() + b"\n")
    ser.flush()
    deadline = time.time() + settle + 2.0
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            if b"\n" in chunk:
                break
        elif buf:
            break
    return buf.decode("utf-8", "replace").strip()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", action="append", default=[], metavar="PORT[:LABEL]",
                    help="repeatable; one per meter")
    ap.add_argument("--baud", type=int, default=9600,
                    help="most bench meters default to 9600 (default 9600)")
    ap.add_argument("--idn", default="*IDN?")
    ap.add_argument("--conf", default="",
                    help="sent once at start; empty for a knob-driven handheld")
    ap.add_argument("--read", default="FETC?",
                    help="U1252x: FETC? fetches the displayed value, READ? also"
                         " works; MEAS? returns *E (default FETC?)")
    ap.add_argument("--func", default="CONF?",
                    help="read-only query of what the knob is set to; '' to skip")
    ap.add_argument("--interval", type=float, default=1.0)
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds; 0 means take a single reading per meter")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--id", action="store_true", help="identify only, take no readings")
    a = ap.parse_args()

    if a.list:
        for p in sorted(list_ports.comports(), key=lambda p: p.device):
            print(f"{p.device}  {p.description}  {p.hwid}")
        return 0

    if not a.port:
        print("no --port given; try --list")
        return 2

    meters = []
    for spec in a.port:
        dev, _, label = spec.partition(":")
        try:
            ser = serial.Serial(dev, a.baud, timeout=0.3)
        except serial.SerialException as e:
            print(f"cannot open {dev}: {e}")
            return 2
        idn = ask(ser, a.idn)
        print(f"# {dev} -> {label or dev}: {idn or 'NO REPLY to ' + a.idn}")
        if not idn:
            # Not fatal: a meter in the wrong mode may still measure.  But say
            # so, because an unlabelled silent port is how the wrong node gets
            # recorded as the right one.
            print(f"#   WARNING: {dev} did not identify itself")
        if a.func:
            func = ask(ser, a.func)
            print(f"#   knob: {func or 'no reply to ' + a.func}")
            if func and "VOLT" not in func.upper():
                print(f"#   WARNING: {label or dev} is not set to volts")
        if a.conf:
            ask(ser, a.conf)
        meters.append((ser, label or dev))

    if a.id:
        for ser, _ in meters:
            ser.close()
        return 0

    print("seconds,label,reading")
    start = time.time()
    try:
        while True:
            for ser, label in meters:
                reply = ask(ser, a.read)
                try:
                    value = float(reply.split(",")[0])
                    print(f"{time.time() - start:.1f},{label},{value:.5f}", flush=True)
                except ValueError:
                    print(f"{time.time() - start:.1f},{label},UNPARSED:{reply!r}",
                          flush=True)
            if a.duration <= 0.0 or time.time() - start >= a.duration:
                break
            time.sleep(a.interval)
    finally:
        for ser, _ in meters:
            ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
