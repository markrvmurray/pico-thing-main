#!/usr/bin/env python3
"""Host end of the Pico-Thing CDC flow-control bench test.

Pairs with chunk/flowtest.asm, which emits a tag byte naming its port and then
8192 bytes of a plain 8-bit counter.  This end verifies the stream is complete
and in order while manipulating DTR and RTS, which is the part that has never
been exercised on hardware: the host tests in tests/ cover the pump's logic
against a fake port, not the real tud_cdc_n_* wrappers.

Run it with the bench venv, which has pyserial:

    ~/GitHub/PICO/venv/bin/python scripts/flowtest.py --list
    ~/GitHub/PICO/venv/bin/python scripts/flowtest.py --port /dev/cu.usbmodem68091 rts
    ~/GitHub/PICO/venv/bin/python scripts/flowtest.py --port /dev/cu.usbmodem68091 plain

The node name identifies the port without guessing: macOS builds it as
usbmodem<iSerialNumber><bInterfaceNumber>, the serial number is "6809", and the
two CDC data interfaces are 1 (console) and 3 (auxiliary) by construction - see
ITF_NUM_CDC_0_DATA / ITF_NUM_CDC_1_DATA in src/usb_descriptors.c.  The tag byte
confirms it from the data, so a renumbering is caught rather than believed.
"""

import argparse
import sys
import time

try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    sys.exit("pyserial not found - run with ~/GitHub/PICO/venv/bin/python")

VID, PID = 0xCAFE, 0x4001
TOTAL = 8192			# must match FT_TOTAL in chunk/flowtest.asm
TAGS = {b"C": "console", b"A": "auxiliary"}
# The trailing digit of the device node is the USB interface number.
IFACE_PORT = {"1": "console", "3": "auxiliary"}


def find_ports():
    """Every Pico-Thing CDC node, with the port each one should be."""
    out = []
    for p in sorted(list_ports.comports(), key=lambda p: p.device):
        if p.vid == VID and p.pid == PID:
            out.append((p.device, IFACE_PORT.get(p.device[-1], "unknown")))
    return out


def read_exact(ser, want, deadline, progress=None):
    """Read until want bytes have arrived or the deadline passes."""
    buf = bytearray()
    while len(buf) < want and time.monotonic() < deadline:
        chunk = ser.read(min(4096, want - len(buf)))
        if chunk:
            buf += chunk
            if progress:
                progress(len(buf))
    return bytes(buf)


def check(data, label):
    """The stream must be (i & 0xFF) for every i.  Report the first break."""
    for i, byte in enumerate(data):
        if byte != (i & 0xFF):
            print(f"  FAIL {label}: byte {i} is {byte:#04x}, expected "
                  f"{i & 0xFF:#04x}")
            lo = max(0, i - 8)
            print(f"        got      {data[lo:i + 8].hex(' ')}")
            print(f"        expected "
                  + " ".join(f"{(j & 0xFF):02x}" for j in range(lo, i + 8)))
            return False
    if len(data) != TOTAL:
        print(f"  FAIL {label}: {len(data)} bytes of {TOTAL}"
              f" - {TOTAL - len(data)} missing")
        return False
    print(f"  PASS {label}: {len(data)} bytes, in order, none missing")
    return True


def open_port(path, rts):
    """Open with DTR up (so the firmware sees the port as open) and RTS set."""
    ser = serial.Serial()
    ser.port = path
    ser.baudrate = 115200		# advisory only; CDC ignores it
    ser.timeout = 0.2
    ser.rtscts = False			# we drive RTS by hand
    ser.dtr = True
    ser.rts = rts
    ser.open()
    ser.dtr = True			# again: some stacks clear it on open
    ser.rts = rts
    return ser


def await_tag(ser, deadline):
    """The generator's first byte says which port this really is."""
    while time.monotonic() < deadline:
        b = ser.read(1)
        if b:
            return b
    return b""


def run_plain(path, rts=True):
    """Baseline: an open, ready host must get the whole stream.

    With rts=False this is instead the "flow N off" case: a host that never
    says it can take data, which the firmware must write to anyway once that
    port has been told not to believe its RTS - the DriveWire situation.
    """
    print(f"plain: {path}, DTR up, RTS {'up' if rts else 'DOWN'} throughout")
    if not rts:
        print("  (expects 'flow <port> off' to have been set on the Pico)")
    with open_port(path, rts=rts) as ser:
        print("  start the generator on the guest now (ctrl-C to abort)")
        tag = await_tag(ser, time.monotonic() + 120.0)
        if not tag:
            print("  FAIL: no tag byte - did the generator run?")
            return False
        print(f"  tag {tag!r} = {TAGS.get(tag, 'UNKNOWN')} port")
        data = read_exact(ser, TOTAL, time.monotonic() + 120.0)
    return check(data, "plain")


def run_rts(path, hold):
    """Drop RTS part way through: output must be held, then lose nothing."""
    print(f"rts: {path}, RTS dropped for {hold}s after a quarter of the stream")
    with open_port(path, rts=True) as ser:
        print("  start the generator on the guest now (ctrl-C to abort)")
        tag = await_tag(ser, time.monotonic() + 120.0)
        if not tag:
            print("  FAIL: no tag byte - did the generator run?")
            return False
        print(f"  tag {tag!r} = {TAGS.get(tag, 'UNKNOWN')} port")

        first = read_exact(ser, TOTAL // 4, time.monotonic() + 120.0)
        print(f"  {len(first)} bytes in; dropping RTS")
        ser.rts = False
        time.sleep(hold)
        # Anything that arrives now came from buffers already in flight; it is
        # not a failure, but note it, because a host that sees the FULL stream
        # while RTS is down means the hold is not working at all.
        stray = ser.read(TOTAL)
        print(f"  while RTS was down: {len(stray)} bytes arrived"
              f" ({'in-flight buffers, expected' if len(stray) < TOTAL // 2 else 'TOO MANY - hold not working'})")
        print("  check 'st' on the console now:"
              " host RTS LOW - output held, guest stalled, bytes waiting")
        print("  raising RTS")
        ser.rts = True
        rest = read_exact(ser, TOTAL - len(first) - len(stray),
                          time.monotonic() + 120.0)
        data = first + stray + rest
    return check(data, "rts")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", nargs="?", choices=("plain", "rts"),
                    help="plain = baseline, rts = drop RTS mid-stream")
    ap.add_argument("--port", help="device node, e.g. /dev/cu.usbmodem68091")
    ap.add_argument("--hold", type=float, default=3.0,
                    help="seconds to hold RTS down (default 3)")
    ap.add_argument("--rts-low", action="store_true",
                    help="plain mode: hold RTS DOWN throughout, to test a port"
                         " set to 'flow <n> off'")
    ap.add_argument("--list", action="store_true",
                    help="list the Pico-Thing CDC nodes and exit")
    args = ap.parse_args()

    ports = find_ports()
    if args.list or not args.mode:
        if not ports:
            print(f"no Pico-Thing CDC ports ({VID:#06x}:{PID:#06x}) found")
            return 1
        print("Pico-Thing CDC ports:")
        for path, which in ports:
            print(f"  {path}  interface {path[-1]} -> {which}")
        print("\nThe port is confirmed from the generator's tag byte, not from"
              "\nthis mapping, so a renumbering will be reported rather than"
              "\nsilently believed.")
        return 0

    path = args.port
    if path is None:
        if len(ports) != 1:
            print("give --port; more than one candidate:")
            for p, which in ports:
                print(f"  {p} -> {which}")
            return 1
        path = ports[0][0]

    ok = (run_plain(path, rts=not args.rts_low) if args.mode == "plain"
          else run_rts(path, args.hold))
    return 0 if ok else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
