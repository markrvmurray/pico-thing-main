#!/usr/bin/env bash
#
# Flash (and optionally reset) the RP2350 over SWD via OpenOCD + picoprobe.
#
# Usage:  scripts/flash.sh [BUILD_DIR] [PROBE] [ACTION]
#   BUILD_DIR  build tree holding pico-thing.elf (default: build)
#   PROBE      probe/config: main | video | pico   (default: main)
#              main  -> probe_main.cfg  (pinned by adapter serial to the main pico)
#              video -> probe_video.cfg (pinned to the Stamp XL video board)
#              pico  -> pico.cfg        (whichever single probe is attached)
#   ACTION     flash      program + verify + reset, then run   (default)
#              noreset    program + verify only; power-cycle yourself
#              reset      no programming, just reset and run
#
# The two probe cfgs live in the repo roots of pico-thing-main / pico-thing-video;
# OpenOCD finds them via -s, so this script works from either repo.

set -euo pipefail

cd "$(dirname "$0")/.."          # repo root

BUILD_DIR="${1:-build}"
PROBE="${2:-main}"
ACTION="${3:-flash}"
ELF="$BUILD_DIR/pico-thing.elf"

case "$PROBE" in
	main)  CFG=probe_main.cfg ;;
	video) CFG=probe_video.cfg ;;
	pico)  CFG=pico.cfg ;;
	*)     echo "error: unknown probe '$PROBE' (main|video|pico)" >&2; exit 1 ;;
esac

# Probe cfgs live in both sibling repos; search here first, then the sibling.
SEARCH=(-s . -s ../pico-thing-main -s ../pico-thing-video)

command -v openocd >/dev/null || { echo "error: openocd not found" >&2; exit 1; }

case "$ACTION" in
	flash)
		[[ -f "$ELF" ]] || { echo "error: $ELF not found (build first)" >&2; exit 1; }
		exec openocd "${SEARCH[@]}" -f "$CFG" \
			-c "init; reset halt; sleep 100; program $ELF verify reset; sleep 200; shutdown" ;;
	noreset)
		[[ -f "$ELF" ]] || { echo "error: $ELF not found (build first)" >&2; exit 1; }
		exec openocd "${SEARCH[@]}" -f "$CFG" \
			-c "init; reset halt; sleep 100; program $ELF verify; shutdown" ;;
	reset)
		exec openocd "${SEARCH[@]}" -f "$CFG" -c "init; reset run; shutdown" ;;
	*)
		echo "error: unknown action '$ACTION' (flash|noreset|reset)" >&2; exit 1 ;;
esac
