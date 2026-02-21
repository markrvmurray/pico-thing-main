#!/usr/bin/env python3
"""
serial_test.py -- USB CDC echo stress-tester for pico-mc6809.

Sends bytes (0x00–0xFF cycling) to the target running uart_echo.asm,
reads them back, and verifies the sequence. Reports throughput and a
latency histogram.

Usage:
    python serial_test.py /dev/cu.usbmodem... [--count N] [--chunk N] [--baud N]

Example:
    python serial_test.py /dev/cu.usbmodem1101 --count 50000 --chunk 64
"""

import argparse
import collections
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial is required: pip install pyserial")


def run(port: str, count: int, chunk: int, baud: int) -> int:
    """Return number of errors."""
    with serial.Serial(port, baudrate=baud, timeout=2.0) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        sent = 0
        received = 0
        errors = 0
        send_seq = 0   # next byte value to send (mod 256)
        recv_seq = 0   # next expected byte value (mod 256)
        latencies: list[float] = []   # round-trip times (seconds)
        bucket: collections.Counter[int] = collections.Counter()

        t_start = time.monotonic()
        last_send_time: float | None = None
        pending = 0   # bytes in flight

        while sent < count or received < sent:
            # --- Send a chunk if we have quota ---
            if sent < count:
                n = min(chunk, count - sent)
                payload = bytes((send_seq + i) & 0xFF for i in range(n))
                ser.write(payload)
                send_seq = (send_seq + n) & 0xFF
                sent += n
                if last_send_time is None:
                    last_send_time = time.monotonic()
                pending += n

            # --- Read whatever is available ---
            waiting = ser.in_waiting
            if waiting == 0:
                # Nothing yet; small sleep to avoid busy-spin
                time.sleep(0.001)
                # Timeout guard: if we've been waiting >5 s after last send, give up
                if last_send_time is not None and time.monotonic() - last_send_time > 5.0:
                    print(f"\nTimeout: only {received}/{sent} bytes received after 5 s stall.")
                    break
                continue

            data = ser.read(waiting)
            now = time.monotonic()
            for b in data:
                if b != recv_seq:
                    errors += 1
                    if errors <= 20:
                        print(f"  ERROR at offset {received}: expected {recv_seq:#04x}, got {b:#04x}")
                recv_seq = (recv_seq + 1) & 0xFF
                received += 1
                pending -= 1

            if last_send_time is not None and pending > 0:
                rtt = now - last_send_time
                latencies.append(rtt)
                ms = int(rtt * 1000)
                bucket[ms] += 1

            if received >= sent:
                last_send_time = None

        t_end = time.monotonic()
        elapsed = t_end - t_start or 1e-9
        throughput = received / elapsed

        print(f"\n{'='*50}")
        print(f"  Sent:       {sent}")
        print(f"  Received:   {received}")
        print(f"  Errors:     {errors}")
        print(f"  Elapsed:    {elapsed:.3f} s")
        print(f"  Throughput: {throughput:.0f} bytes/s ({throughput/1024:.1f} KB/s)")

        if latencies:
            avg_ms = sum(l * 1000 for l in latencies) / len(latencies)
            max_ms = max(latencies) * 1000
            print(f"\n  RTT (avg):  {avg_ms:.1f} ms")
            print(f"  RTT (max):  {max_ms:.1f} ms")
            print(f"\n  Latency histogram (ms bucket : count):")
            for ms in sorted(bucket):
                bar = '#' * min(60, bucket[ms])
                print(f"    {ms:4d} ms : {bar} ({bucket[ms]})")

        print(f"{'='*50}")
        return errors


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("port", help="Serial port, e.g. /dev/cu.usbmodem1101")
    parser.add_argument("--count", type=int, default=10000, help="Total bytes to send (default: 10000)")
    parser.add_argument("--chunk", type=int, default=64, help="Bytes per write call (default: 64)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    print(f"Serial echo test: port={args.port}  count={args.count}  chunk={args.chunk}  baud={args.baud}")
    errors = run(args.port, args.count, args.chunk, args.baud)
    sys.exit(0 if errors == 0 else 1)


if __name__ == "__main__":
    main()
