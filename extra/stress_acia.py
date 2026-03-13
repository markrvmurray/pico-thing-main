#!/usr/bin/env python3
"""
ACIA Flow Control Stress Test — Host Side

Receives a framed packet from the 6809 on USB CDC port 1 (auxiliary
ACIA), validates it, and echoes the entire frame back only if valid.
Corrupt or incomplete frames are discarded, preventing deadlocks.

Frame format (big-endian):
    SOF(0x02) | SEQ | LEN_H | LEN_L | PAYLOAD[LEN] | CHK | EOF(0x03)
    CHK = XOR of SEQ, LEN_H, LEN_L, and all payload bytes

Protocol:
    1. Host sends 'G' (go)
    2. 6809 replies 'R' (ready) — host reads until it sees 'R'
    3. 6809 sends framed packet; host validates and echoes if good

Usage:
    python3 extra/stress_acia.py /dev/tty.usbmodem*2
"""

import sys
import time
import serial

SOF = 0x02
EOF = 0x03
PAYLOAD_LEN = 4096
NUM_FRAMES = 16
TIMEOUT_S = 30


class BufferedPort:
    """Thin wrapper around a serial.Serial that keeps a pending-byte
    buffer, so leftover bytes from one read phase carry into the next."""

    def __init__(self, port):
        self._port = port
        self._buf = bytearray()

    def read_byte(self, deadline):
        """Return one byte, or None on timeout."""
        while True:
            if self._buf:
                b = self._buf[0]
                del self._buf[0]
                return b
            if time.time() >= deadline:
                return None
            raw = self._port.read(64)
            if raw:
                self._buf.extend(raw)

    def read_n(self, n, deadline):
        """Return exactly n bytes as a bytearray, or None on timeout."""
        while len(self._buf) < n:
            if time.time() >= deadline:
                return None
            raw = self._port.read(min(n - len(self._buf), 4096))
            if raw:
                self._buf.extend(raw)
        result = bytearray(self._buf[:n])
        del self._buf[:n]
        return result

    def push_back(self, data):
        """Push bytes back to the front of the buffer."""
        self._buf[0:0] = data


def recv_frame(bp, deadline, debug=False):
    """Receive and validate one frame from a BufferedPort.
    Returns (seq, payload) on success, or (None, reason_string)."""

    # Wait for SOF — discard any junk before it
    skipped = 0
    while True:
        b = bp.read_byte(deadline)
        if b is None:
            return None, "timeout waiting for SOF"
        if b == SOF:
            break
        if debug:
            print(f"  skip 0x{b:02X}", end="")
        skipped += 1
    if skipped and debug:
        print(f"  ({skipped} bytes skipped before SOF)")

    # SEQ
    b = bp.read_byte(deadline)
    if b is None:
        return None, "timeout reading SEQ"
    seq = b
    chk = seq

    # LEN (big-endian)
    b = bp.read_byte(deadline)
    if b is None:
        return None, "timeout reading LEN_H"
    chk ^= b
    length = b << 8
    b = bp.read_byte(deadline)
    if b is None:
        return None, "timeout reading LEN_L"
    chk ^= b
    length |= b

    if debug:
        print(f"  hdr: seq={seq} len={length}")

    if length > 65535:
        return None, f"length {length} too large"

    # PAYLOAD
    payload = bp.read_n(length, deadline)
    if payload is None:
        return None, f"timeout reading {length}-byte payload"
    for b in payload:
        chk ^= b

    if debug:
        head = ' '.join(f'{b:02X}' for b in payload[:16])
        tail = ' '.join(f'{b:02X}' for b in payload[-4:])
        print(f"  payload[0:16]: {head}")
        print(f"  payload[-4:]:  {tail}")

    # CHK
    b = bp.read_byte(deadline)
    if b is None:
        return None, "timeout reading CHK"
    if b != chk:
        if debug:
            print(f"  chk computed=0x{chk:02X} received=0x{b:02X}")
        return None, f"checksum mismatch: expected 0x{chk:02X}, got 0x{b:02X}"

    # EOF
    b = bp.read_byte(deadline)
    if b is None:
        return None, "timeout reading EOF"
    if b != EOF:
        return None, f"bad EOF marker: 0x{b:02X}"

    return seq, payload


def send_frame(port, seq, payload):
    """Send a framed packet."""
    length = len(payload)
    chk = seq ^ (length >> 8) ^ (length & 0xFF)
    for b in payload:
        chk ^= b

    frame = bytearray()
    frame.append(SOF)
    frame.append(seq)
    frame.append(length >> 8)
    frame.append(length & 0xFF)
    frame.extend(payload)
    frame.append(chk)
    frame.append(EOF)

    port.write(frame)
    port.flush()


def verify_payload(payload, expected_start):
    """Check that payload is the expected incrementing byte pattern.
    Returns (ok, error_count, first_error_str).
    expected_start is the first expected byte value (wraps at 256)."""
    errors = 0
    first_err = ""
    expected = expected_start
    for i, b in enumerate(payload):
        if b != (expected & 0xFF):
            if errors == 0:
                first_err = (f"@{i}: exp=0x{expected & 0xFF:02X} "
                             f"got=0x{b:02X}")
            errors += 1
            expected = b  # resync
        expected += 1
    return errors == 0, errors, first_err


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <serial-port>")
        print(f"  e.g. {sys.argv[0]} /dev/tty.usbmodem12102")
        sys.exit(1)

    port_name = sys.argv[1]
    print(f"Opening {port_name} ...")
    port = serial.Serial(port_name, timeout=0.05, write_timeout=30)
    port.dtr = True
    time.sleep(0.1)

    # Drain any stale data from previous runs
    port.reset_input_buffer()

    bp = BufferedPort(port)

    # Send 'G' (go) — 6809 replies with 'R' (ready)
    print("Sending start signal ('G') ...")
    port.write(b'G')
    port.flush()

    # Wait for 'R' — proves 6809 received 'G' and is about to send.
    # Any bytes after 'R' stay in the BufferedPort for recv_frame.
    sync_deadline = time.time() + 10.0
    synced = False
    while time.time() < sync_deadline:
        b = bp.read_byte(sync_deadline)
        if b is None:
            break
        if b == ord('R'):
            synced = True
            break
    if not synced:
        print("ERROR: never received 'R' from 6809")
        port.close()
        sys.exit(1)
    print("Synchronised with 6809")

    print(f"Expecting {NUM_FRAMES} x {PAYLOAD_LEN}-byte frames\n")

    start = time.time()
    deadline = start + TIMEOUT_S

    frames_ok = 0
    frames_fail = 0
    total_bytes = 0
    total_errors = 0
    expected_byte = 0  # tracks incrementing pattern across frames

    for expected_seq in range(NUM_FRAMES):
        seq, result = recv_frame(bp, deadline)

        if seq is None:
            print(f"[{expected_seq}] RX failed: {result}")
            frames_fail += 1
            break

        payload = result
        print(f"[{seq}] RX {len(payload)}B ", end="", flush=True)

        # Validate payload
        ok, errors, first_err = verify_payload(payload, expected_byte)
        expected_byte = (expected_byte + len(payload)) & 0xFF
        total_errors += errors
        total_bytes += len(payload)

        if not ok:
            print(f"ERR({errors}: {first_err}) ", end="")

        if seq != expected_seq:
            print(f"SEQ err(exp={expected_seq}) ", end="")

        # Echo the validated frame back
        send_frame(port, seq, payload)
        print("OK")
        frames_ok += 1

        # Extend deadline after each successful round-trip
        deadline = time.time() + TIMEOUT_S

    elapsed = time.time() - start
    port.close()

    # Summary
    print()
    print("--- Host Results ---")
    print(f"Frames OK:      {frames_ok} / {NUM_FRAMES}")
    print(f"Frames failed:  {frames_fail}")
    print(f"Total bytes:    {total_bytes}")
    print(f"Payload errors: {total_errors}")
    print(f"Elapsed:        {elapsed:.2f}s")
    if total_bytes > 0 and elapsed > 0:
        print(f"Throughput:     {total_bytes / elapsed:.0f} bytes/s")
    print()
    if frames_ok == NUM_FRAMES and total_errors == 0:
        print("PASS")
    else:
        print("*** FAIL ***")


if __name__ == "__main__":
    main()
