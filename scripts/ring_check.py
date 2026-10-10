#!/usr/bin/env python3
"""Check a bus-trace dump of a periodic guest loop for lost or doubled rows.

Usage: ring_check.py DUMP [--period N] [--end ROW]

DUMP is console output holding a `trace` dump.  Every row from --period up
to --end (default: the first row whose interrupt columns show a request) is
compared with the row one period earlier, on every column except the row
number.  A ring that wraps cleanly and keeps its two capture halves aligned
repeats exactly; a lost or doubled row anywhere breaks the repeat from that
row on, and the first break is reported.
"""
import argparse
import re
import sys

ROW = re.compile(r'^\s*(\d+)\s{2}(.*)$')


def rows(path):
    out = []
    with open(path, errors='replace') as f:
        for line in f:
            m = ROW.match(line.rstrip('\n'))
            if m and 'BUS_' in m.group(2):
                out.append((int(m.group(1)), m.group(2)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dump')
    ap.add_argument('--period', type=int, default=12)
    ap.add_argument('--end', type=int, default=None,
                    help='first row NOT checked (default: first interrupt request)')
    a = ap.parse_args()

    r = rows(a.dump)
    if not r:
        sys.exit('no trace rows found')
    nums = [n for n, _ in r]
    if nums != list(range(nums[0], nums[0] + len(nums))):
        sys.exit('row numbers are not consecutive - the dump itself is incomplete')

    # The H N F I column is the interrupt request lines; the first row with
    # any of them set is where the guest stops being periodic.
    end = a.end
    if end is None:
        end = len(r)
        for i, (_, text) in enumerate(r):
            fields = text.split()
            # ... b l v ! H N F I TSK Pg FM: H N F I are fields -7..-4
            if any(f != '.' for f in fields[-7:-3]):
                end = i
                break

    breaks = [i for i in range(a.period, end) if r[i][1] != r[i - a.period][1]]
    print(f'{len(r)} rows; checked rows {a.period}..{end - 1} against period {a.period}')
    if breaks:
        i = breaks[0]
        print(f'FIRST BREAK at row {r[i][0]} ({len(breaks)} rows differ):')
        print(f'  row {r[i - a.period][0]:5}: {r[i - a.period][1]}')
        print(f'  row {r[i][0]:5}: {r[i][1]}')
        sys.exit(1)
    print('PERIODIC: no row lost or doubled')


if __name__ == '__main__':
    main()
