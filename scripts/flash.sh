#!/usr/bin/env bash
#
# Flash (and optionally reset) the main board's RP2350 over SWD via OpenOCD.
#
# Usage:  scripts/flash.sh [BUILD_DIR] [ACTION]
#   BUILD_DIR  build tree holding pico-thing.elf (default: build)
#   ACTION     flash      program + verify + reset, then run   (default)
#              noreset    program + verify only; power-cycle yourself
#              reset      no programming, just reset and run
#
# The probe is not a choice: pico.cfg in this repo's root is pinned by serial
# to the main board's probe, so this script cannot touch the video board.  The
# old form "flash.sh BUILD_DIR main ACTION" is still accepted.

set -euo pipefail

cd "$(dirname "$0")/.."          # repo root

BUILD_DIR="${1:-build}"
shift || true
case "${1:-}" in
	main) shift ;;                # old probe argument, now implied
	video|pico)
		echo "error: this repo only flashes the main board; the probe is fixed by pico.cfg" >&2
		exit 1 ;;
esac
ACTION="${1:-flash}"
ELF="$BUILD_DIR/pico-thing.elf"
CFG="$PWD/pico.cfg"

command -v openocd >/dev/null || { echo "error: openocd not found" >&2; exit 1; }

case "$ACTION" in
	flash)
		[[ -f "$ELF" ]] || { echo "error: $ELF not found (build first)" >&2; exit 1; }
		exec openocd -f "$CFG" \
			-c "init; reset halt; sleep 100; program $ELF verify reset; sleep 200; shutdown" ;;
	noreset)
		[[ -f "$ELF" ]] || { echo "error: $ELF not found (build first)" >&2; exit 1; }
		exec openocd -f "$CFG" \
			-c "init; reset halt; sleep 100; program $ELF verify; shutdown" ;;
	reset)
		exec openocd -f "$CFG" -c "init; reset run; shutdown" ;;
	*)
		echo "error: unknown action '$ACTION' (flash|noreset|reset)" >&2; exit 1 ;;
esac
