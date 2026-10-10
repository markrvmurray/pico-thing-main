cmake_minimum_required(VERSION 3.14)  # 3.14+ for BISON_TARGET DEFINES_FILE
set(PICO_BOARD pimoroni_pga2350)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Flex and bison run on the BUILD HOST; the generated .c files are
# cross-compiled for RP2350 by arm-none-eabi-gcc.
#
# macOS ships bison 2.3 which is too old.  Use Homebrew's bison 3.x.
# Install with: brew install bison
if(APPLE)
    find_program(_bison_homebrew
        NAMES bison
        PATHS /opt/homebrew/opt/bison/bin /usr/local/opt/bison/bin
        NO_DEFAULT_PATH
    )
    if(NOT _bison_homebrew)
        message(FATAL_ERROR
            "Bison 3.x not found.  Install with: brew install bison")
    endif()
    # Force-set so CLion's stale cache (pointing at /usr/bin/bison 2.3) is overridden.
    set(BISON_EXECUTABLE "${_bison_homebrew}" CACHE FILEPATH "Bison executable" FORCE)
endif()
find_package(FLEX  REQUIRED)
find_package(BISON 3.0 REQUIRED)

project(pico-thing C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Keep build trees out of Time Machine: they churn on every build and can be
# regenerated.  A sticky exclusion (no -p) is stored on the directory itself,
# so unlike a fixed-path one it needs no root; it goes if the directory is
# deleted and comes back at the next configure.  CMAKE_HOST_APPLE, not APPLE:
# this project cross-compiles for the RP2350, so APPLE is false here.
if(CMAKE_HOST_APPLE)
    find_program(_tmutil tmutil NO_CMAKE_FIND_ROOT_PATH)
    if(_tmutil)
        execute_process(COMMAND ${_tmutil} addexclusion "${CMAKE_BINARY_DIR}"
            RESULT_VARIABLE _tmutil_result OUTPUT_QUIET ERROR_QUIET)
        if(NOT _tmutil_result EQUAL 0)
            message(STATUS "Could not exclude ${CMAKE_BINARY_DIR} from Time Machine")
        endif()
    endif()
endif()

# Trick target
add_custom_target(clion_host_dummy ALL)

add_compile_definitions(DEBUG)

pico_sdk_init()

# ----- Flex / Bison targets -----
# flex/bison run on the BUILD HOST; the generated .c files are
# cross-compiled for RP2350 by arm-none-eabi-gcc (newlib has all needed symbols).
# DEFINES_FILE pins the header name to command_parser.tab.h to avoid a
# circular-include collision with our project header command_parser.h.
FLEX_TARGET(CmdLexer
	${CMAKE_CURRENT_LIST_DIR}/src/command_lexer.l
	${CMAKE_CURRENT_BINARY_DIR}/command_lexer.c
)

BISON_TARGET(CmdParser
	${CMAKE_CURRENT_LIST_DIR}/src/command_parser.y
	${CMAKE_CURRENT_BINARY_DIR}/command_parser.c
	DEFINES_FILE ${CMAKE_CURRENT_BINARY_DIR}/command_parser.tab.h
)

ADD_FLEX_BISON_DEPENDENCY(CmdLexer CmdParser)

add_executable(pico-thing
		src/pico_thing.cpp
		src/registers.cpp
		src/mc6809.cpp
		src/srec.cpp
		src/mc6850.cpp
		src/uart_pump.cpp
		src/tick_timer.cpp
		src/usb_descriptors.c
		src/usb.c
)

# Include both the project headers and generated ones
target_include_directories(pico-thing
	PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/include
	${CMAKE_CURRENT_BINARY_DIR}
)

pico_generate_pio_header(pico-thing ${CMAKE_CURRENT_LIST_DIR}/src/pico_thing.pio)

# ----- Embedded 6809 chunk blobs -----
# Binary blobs embedded into the snippet area by extra/api.asm via INCLUDEBIN.
#
# The .raw chunk blobs are built here, by delegating to the chunk makefile so
# the 6809 link recipe stays in one place.  They land in the source tree
# (chunk/*.raw) because api.asm's INCLUDEBIN paths are relative to
# extra/api.asm.
set(CHUNK_DIR ${CMAKE_CURRENT_LIST_DIR}/chunk)

find_program(MAKE_EXECUTABLE NAMES gmake make REQUIRED)

# Sources common to every embeddable chunk: the trampoline that is linked
# ahead of the program, everything in libchunk_flat.a, and the linker script.
set(CHUNK_COMMON_SOURCES
	${CHUNK_DIR}/trampoline.asm
	${CHUNK_DIR}/syslib_flat.asm
	${CHUNK_DIR}/io.asm
	${CHUNK_DIR}/bss.asm
	${CHUNK_DIR}/putdec.asm
	${CHUNK_DIR}/puthex.asm
	${CHUNK_DIR}/ascii.inc
	${CHUNK_DIR}/uart.inc
	${CHUNK_DIR}/syslib.inc
	${CHUNK_DIR}/embed.ld
	${CHUNK_DIR}/Makefile
)

# Assembly chunks embedded by api.asm; each <name> is built from <name>.asm.
set(CHUNK_BLOB_NAMES dl irq_test)

set(CHUNK_BLOBS "")
set(CHUNK_BLOB_SOURCES "")
set(CHUNK_BLOB_TARGETS "")
foreach(chunk ${CHUNK_BLOB_NAMES})
	list(APPEND CHUNK_BLOBS ${CHUNK_DIR}/${chunk}.raw)
	list(APPEND CHUNK_BLOB_SOURCES ${CHUNK_DIR}/${chunk}.asm)
	list(APPEND CHUNK_BLOB_TARGETS ${chunk}.raw)
endforeach()
list(JOIN CHUNK_BLOB_TARGETS " " CHUNK_BLOB_TARGETS_PRETTY)

# All blobs are built by ONE make invocation, not one per blob.  The chunk
# makefile shares intermediates between them (trampoline.obj, and the objects
# in libchunk_flat.a) and deletes the intermediate objects when it finishes,
# so two concurrent makes in that directory race and the link fails with
# "Can't open file trampoline.obj".  A single make process owns the directory
# and orders the work itself.
add_custom_command(OUTPUT ${CHUNK_BLOBS}
	DEPENDS ${CHUNK_BLOB_SOURCES} ${CHUNK_COMMON_SOURCES}
	COMMAND ${MAKE_EXECUTABLE} -C ${CHUNK_DIR} ${CHUNK_BLOB_TARGETS}
	COMMENT "Building 6809 chunk blobs: ${CHUNK_BLOB_TARGETS_PRETTY}"
	VERBATIM
)

# extra/rel_picothing is not generated by this build — it is maintained by
# hand.  If it is missing, fail loudly here rather than emitting a cryptic
# assembler error.
set(API_PREBUILT_BLOBS
	${CMAKE_CURRENT_LIST_DIR}/extra/rel_picothing
)
foreach(blob ${API_PREBUILT_BLOBS})
	if(NOT EXISTS ${blob})
		message(FATAL_ERROR
			"Required embedded blob is missing:\n  ${blob}\n"
			"This build does not generate it — it must be supplied by hand.")
	endif()
endforeach()

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/api.bin
	DEPENDS ${CMAKE_CURRENT_LIST_DIR}/extra/api.asm
			${CHUNK_BLOBS}
			${API_PREBUILT_BLOBS}
	COMMAND lwasm --format=decb
			--list=${CMAKE_CURRENT_BINARY_DIR}/api.lst
			--output=${CMAKE_CURRENT_BINARY_DIR}/api.bin
			${CMAKE_CURRENT_LIST_DIR}/extra/api.asm
	COMMENT "Assembling builtin code snippets"
)

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/api.inc
	DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/api.bin ${CMAKE_CURRENT_LIST_DIR}/extra/api.py
	COMMAND python3 ${CMAKE_CURRENT_LIST_DIR}/extra/api.py
			${CMAKE_CURRENT_BINARY_DIR}/api.bin
			${CMAKE_CURRENT_BINARY_DIR}/api.inc
	COMMENT "Converting snippets to byte list initialisers"
)

set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/api.inc PROPERTIES GENERATED TRUE)
target_sources(pico-thing
	PRIVATE
		${CMAKE_CURRENT_BINARY_DIR}/api.inc
		${FLEX_CmdLexer_OUTPUTS}
		${BISON_CmdParser_OUTPUTS}
)

# Board revision selects the GPIO map.  Both revisions carry the same signals:
# version 1 puts TASK[4:0], TASK0 and the power enable on GPIO 40-46 with the
# DAT fault signals on 32-36, version 2 puts the task pins on 32-37, the power
# enable on 40, the fault signals on 41-45 and the 5V ADC on 46.  Flashing the
# wrong one drives outputs into outputs, so pass -DPICO_THING_BOARD_VERSION=1
# when building for one of the older boards.
set(PICO_THING_BOARD_VERSION 2 CACHE STRING "Pico-Thing board revision (1 or 2)")
set_property(CACHE PICO_THING_BOARD_VERSION PROPERTY STRINGS 1 2)
if(NOT PICO_THING_BOARD_VERSION MATCHES "^[12]$")
	message(FATAL_ERROR
		"PICO_THING_BOARD_VERSION must be 1 or 2, not '${PICO_THING_BOARD_VERSION}'")
endif()
message(STATUS "Pico-Thing board version ${PICO_THING_BOARD_VERSION}")

# Version 2 reads the external 5V supply on ADC channel 6 and refuses to close
# the load switch below this.  A nominal 5V supply may sag 10%, so the default
# sits just above that: high enough to reject an absent or failing supply,
# low enough not to reject a working one under load.
set(PICO_THING_POWER_RAIL_MIN_VOLTS 4.5 CACHE STRING
	"Minimum external 5V, in volts, before the load switch is closed")
message(STATUS "Pico-Thing power rail minimum ${PICO_THING_POWER_RAIL_MIN_VOLTS}V")

# The supply monitor drops the load switch below this.  Deliberately under the
# arming threshold above: a rail sagging under load should not be switched off
# and straight back on again, so there is hysteresis between the two.
set(PICO_THING_POWER_RAIL_DROP_VOLTS 4.0 CACHE STRING
	"External 5V, in volts, below which the load switch is dropped")
message(STATUS "Pico-Thing power rail cutout ${PICO_THING_POWER_RAIL_DROP_VOLTS}V")

# How long the rail is given to come up before the monitor is allowed to judge
# it.  Closing the switch into the board's bulk capacitance pulls the supply
# down until it has charged, and how far and for how long depends on how much
# of the board is populated, so this wants raising as chips go in.
set(PICO_THING_POWER_RAIL_SETTLE_MS 250 CACHE STRING
	"Milliseconds the 5V rail is given to settle before it is judged")
message(STATUS "Pico-Thing power rail settle ${PICO_THING_POWER_RAIL_SETTLE_MS}ms")

# Whether a DAT fault automatically raises NMIBlock.  Zero suspends that for
# bring-up, so a faulting loop keeps re-raising the fault NMI instead of
# masking itself after the first one - which is what makes it visible on a
# scope.  A guest with a bad pointer will then NMI-storm, so this is a
# measurement setting, not one to leave on.
set(PICO_THING_DAT_FAULT_AUTO_BLOCK 1 CACHE STRING
	"Raise NMIBlock automatically on a DAT fault (0 for scope bring-up)")
# E cycles between a DAT fault and NMIBlock going up.  Has to outlast the
# faulting instruction, since the 6809 only acts on NMI at an instruction
# boundary - raise it further to widen the pulse for a scope.
set(PICO_THING_DAT_FAULT_BLOCK_DELAY 32 CACHE STRING
	"E cycles from a DAT fault to NMIBlock being raised")
message(STATUS "Pico-Thing DAT fault auto-block ${PICO_THING_DAT_FAULT_AUTO_BLOCK}, "
	"block delay ${PICO_THING_DAT_FAULT_BLOCK_DELAY} cycles")


# The UART console's baud rate, on GPIO 30/31.  This is the ONLY real baud rate
# in the firmware, and the UART is the ONLY console: stdio is wired to the UART
# alone (see pico_enable_stdio_usb 0 / pico_enable_stdio_uart 1 below), and the
# two USB CDC ports belong entirely to the guest's ACIAs.  Those ignore
# whatever line coding the host asks for, and the guest's own divide-select
# bits pick loopback and reset rather than a rate.
#
# 115200 because the cost of a faster rate falls on input, where there is no
# workaround: the UART has no flow control and it is the only place to paste
# anything, so bulk input cannot be sent by some better-behaved route.  32
# bytes of RX FIFO is 2.78 ms of runway at 115200 against 1.39 ms at 230400,
# and Core 0 blocks for longer than that in several places - the break settle,
# adcdump's sleeps, snippet timeouts - so characters pasted during one of
# those go over the side with nothing to throttle the sender.  S-records are
# checksummed (srec.cpp), so a mangled record is refused rather than loaded,
# which is the only protection there is.
#
# The price is paid on output.  Every printf blocks Core 0 once the 32-byte TX
# FIFO fills, and Core 0 is the loop that also runs usb_poll() and the CDC pump
# for the guest's two serial ports, so a trace dump or a long status page costs
# the guest DriveWire throughput.  A character costs 86.8 us here against 43.4
# us at 230400, which doubles that penalty.  Raise this to 230400 if printf
# latency ever hurts the guest more than slow pasting hurts.
#
# The rate comes from clk_peri, which must stay at 48 MHz: 115200 wants
# IBRD 26 / FBRD 3, giving 115177 baud, -0.02%.  `st` reports clk_peri, which
# is how a PLL change that silently drags it would be caught.
set(PICO_THING_CONSOLE_BAUD 115200 CACHE STRING
	"UART console baud rate on GPIO 30/31 (the CDC ports ignore baud entirely)")
message(STATUS "Pico-Thing UART console ${PICO_THING_CONSOLE_BAUD} baud")

# How fast the Pico itself runs, which is what caps the guest's E clock: the
# per-cycle work measured ~226 clk_sys cycles per E cycle, so the ceiling is
# this number divided by that - about 1.4 MHz at 320 MHz.  The regulator is
# already at VREG_VOLTAGE_1_30 before the clock is raised, and the raise is
# attempted with panic=false so an unachievable target falls back instead of
# hanging.  NOT adjusted here: the QMI divider for XIP flash, so a high enough
# clk_sys puts the flash clock out of spec.
set(PICO_THING_SYS_CLK_HZ 360000000 CACHE STRING
	"clk_sys in Hz; the default 360000000 pairs with VREG_VOLTAGE_1_50 (die 40-42C); 320000000 at VREG_VOLTAGE_1_30 is the conservative pair; 400000000 at VREG_VOLTAGE_2_00 works but the die climbs past 55C in minutes")
message(STATUS "Pico-Thing clk_sys ${PICO_THING_SYS_CLK_HZ} Hz")

# The core regulator, paired with the clock above.  VREG_VOLTAGE_1_30 is the
# SDK's VREG_VOLTAGE_MAX; anything above it is clamped by the regulator's own
# limit unless PICO_THING_VREG_UNLIMITED lifts it with
# vreg_disable_voltage_limit().  Any value of the hardware/vreg.h enum is
# accepted, 0.70V to 3.30V in the RP2350's steps.
#
# THE DEFAULT IS 360 MHz AT VREG_VOLTAGE_1_50 (the user's decision, 2026-10-10),
# measured the same day on this board, every reading with the guest rail OFF
# because with it on the ADC reference collapses and the thermometer reads
# with it:
#   320 MHz / 1.30V  E = 1.0..2.4 solid                die ~40C
#   360 MHz / 1.50V  E = 1.0..2.4 solid, 2.5 clean     die 40 -> 42C in 7 min
#   400 MHz / 2.00V  E = 1.0, 2.0 clean                die 46 -> 57C in 5 min, rising
# 360/1.50 is where the Pico has the E headroom with the die barely moving;
# 400/2.00 works but needs a heatsink and a thermometer in the loop before it
# is left running.  320/1.30 remains the conservative pair, one configure away.
set(PICO_THING_VREG VREG_VOLTAGE_1_50 CACHE STRING
	"core regulator setting, any enum from hardware/vreg.h (VREG_VOLTAGE_0_70 .. VREG_VOLTAGE_3_30); above VREG_VOLTAGE_1_30 needs PICO_THING_VREG_UNLIMITED=ON")
set_property(CACHE PICO_THING_VREG PROPERTY STRINGS
	VREG_VOLTAGE_0_70 VREG_VOLTAGE_0_75 VREG_VOLTAGE_0_80 VREG_VOLTAGE_0_85
	VREG_VOLTAGE_0_90 VREG_VOLTAGE_0_95 VREG_VOLTAGE_1_00 VREG_VOLTAGE_1_05
	VREG_VOLTAGE_1_10 VREG_VOLTAGE_1_15 VREG_VOLTAGE_1_20 VREG_VOLTAGE_1_25
	VREG_VOLTAGE_1_30 VREG_VOLTAGE_1_35 VREG_VOLTAGE_1_40 VREG_VOLTAGE_1_50
	VREG_VOLTAGE_1_60 VREG_VOLTAGE_1_65 VREG_VOLTAGE_1_70 VREG_VOLTAGE_1_80
	VREG_VOLTAGE_1_90 VREG_VOLTAGE_2_00 VREG_VOLTAGE_2_35 VREG_VOLTAGE_2_50
	VREG_VOLTAGE_2_65 VREG_VOLTAGE_2_80 VREG_VOLTAGE_3_00 VREG_VOLTAGE_3_15
	VREG_VOLTAGE_3_30)
option(PICO_THING_VREG_UNLIMITED
	"call vreg_disable_voltage_limit() so a PICO_THING_VREG above VREG_VOLTAGE_1_30 takes effect" ON)
message(STATUS "Pico-Thing vreg ${PICO_THING_VREG} unlimited ${PICO_THING_VREG_UNLIMITED}")

# The per-cycle ISR cost instrument costs about 21 clk_sys cycles per E cycle,
# which moves the E ceiling by roughly 0.13 MHz.  Set to 0 to get those back
# while keeping the rest of DEBUG.
# Default OFF: it is an instrument, and 0.13 MHz of E ceiling is the very thing
# the per-cycle redesign is trying to win back.  Turn it on for a measurement,
# not for a working build.
set(PICO_THING_ISR_COST 0 CACHE STRING
	"1 = measure the per-cycle ISR cost and report it in st, 0 = do not")
message(STATUS "Pico-Thing ISR cost instrument: ${PICO_THING_ISR_COST}")

# ---------------------------------------------------------------------------
# THE DIAGNOSTIC SWITCHES, split by WHAT THEY COST rather than by what they are.
#
# DEBUG is defined unconditionally above, and that is deliberate: it marks the
# things a plain user of a Pico-Thing would not care about - extra st sections,
# internal state variables, counters kept on Core 1's for(;;) loop and in rare
# error branches.  **The rule is that DEBUG must stay FREE on the per-cycle
# paths**, which is exactly what makes it safe to leave on; anything that costs
# cycles every E cycle gets its own switch below and defaults OFF.
#
#   DEBUG                    reporting surface        no per-cycle cost   ON
#   PICO_THING_TESTPOINTS    GPIO markers for a scope SIO write per ISR   off
#   PICO_THING_DIAG          per-cycle counters       ~9 cycles/E cycle   off
#   PICO_THING_ISR_COST      SysTick cost instrument  12-16 accesses/ISR  off
#
# Counters that are MECHANISM rather than statistics are never gated by any of
# these, however statistical they sound: rx_read_count/rx_processed_count and
# nmi_request_count/nmi_request_done carry work from an ISR to its consumer so
# a second event cannot be lost, _count_irq_ack[] is what the interrupt tests
# wait on, and core1_isr_count is Core 0's watchdog input - gating that would
# leave it unable to tell thread starvation from a hard fault.
#
# AND A GATED-OFF COUNTER MUST REPORT "off", NEVER 0.  An absent count has been
# read as a fault three times (the console ACIA Status column, st's Read IRQ
# after the address filter, and the LIC count), so a zero has to mean zero.
# ---------------------------------------------------------------------------

# Scope marker pins, driven high on entry to an ISR and low on exit, so a
# scope can see when each one ran and for how long.  Two SIO writes per entry
# on paths that run every E cycle, for a signal nobody needs unless they have
# probes on the board.
set(PICO_THING_TESTPOINTS 0 CACHE STRING
	"1 = drive the GPIO scope marker pins from the ISRs, 0 = do not")
message(STATUS "Pico-Thing ISR testpoints: ${PICO_THING_TESTPOINTS}")

# Per-cycle diagnostic counting: the LIC count, the E cycles with /IRQ
# asserted, and the console ACIA's status-read count.  About 9 of the Q-rising
# handler's ~110 clk_sys cycles, every E cycle, for numbers only a human
# reading st ever looks at.  Turn it on to debug, at a low guest clock where
# those cycles do not matter.
set(PICO_THING_DIAG 0 CACHE STRING
	"1 = keep the per-cycle diagnostic counters, 0 = leave them out")
message(STATUS "Pico-Thing per-cycle diagnostics: ${PICO_THING_DIAG}")

# The ADC clock in kHz.  48000 is the SDK's value and what the ADC calibration
# was taken against.  Lower it only to lengthen a conversion, which is how a
# drooping ADC_VREF is distinguished from a broken one.
set(PICO_THING_ADC_CLK_KHZ 48000 CACHE STRING
	"ADC clock in kHz; 48000 is standard, lower lengthens each conversion")
message(STATUS "Pico-Thing clk_adc ${PICO_THING_ADC_CLK_KHZ} kHz")

# The ADC calibration - reference voltage, divider ratio and converter zero
# error - is deliberately not settable here.  Those three are measurements of
# one physical board, not choices about how to build it, and as cache
# variables a build directory configured before they were measured keeps the
# nominal values and silently reports wrong volts and degrees.  They live in
# src/pico_thing.cpp, where the tracked value is the value that is used.

# Core 1 stack is 0x800 (2 KiB) by default — too small for three nested
# ISR levels (DMA prio-0 → GPIO prio-1 → thread) with FPU lazy stacking.
target_compile_definitions(pico-thing PRIVATE
	PICO_CORE1_STACK_SIZE=0x1000
	PICO_THING_BOARD_VERSION=${PICO_THING_BOARD_VERSION}
	PICO_THING_POWER_RAIL_MIN_VOLTS=${PICO_THING_POWER_RAIL_MIN_VOLTS}f
	PICO_THING_POWER_RAIL_DROP_VOLTS=${PICO_THING_POWER_RAIL_DROP_VOLTS}f
	PICO_THING_POWER_RAIL_SETTLE_MS=${PICO_THING_POWER_RAIL_SETTLE_MS}u
	PICO_THING_DAT_FAULT_AUTO_BLOCK=${PICO_THING_DAT_FAULT_AUTO_BLOCK}
	PICO_THING_DAT_FAULT_BLOCK_DELAY=${PICO_THING_DAT_FAULT_BLOCK_DELAY}u
	PICO_THING_CONSOLE_BAUD=${PICO_THING_CONSOLE_BAUD}
	SYS_CLK_DEFAULT=${PICO_THING_SYS_CLK_HZ}
	PICO_THING_VREG=${PICO_THING_VREG}
	PICO_THING_VREG_NAME="${PICO_THING_VREG}"
	PICO_THING_VREG_UNLIMITED=$<BOOL:${PICO_THING_VREG_UNLIMITED}>
	PICO_THING_ISR_COST=${PICO_THING_ISR_COST}
	PICO_THING_DIAG=${PICO_THING_DIAG}
	PICO_THING_TESTPOINTS=${PICO_THING_TESTPOINTS}
	PICO_THING_ADC_CLK_KHZ=${PICO_THING_ADC_CLK_KHZ}u
)

target_link_libraries(pico-thing
	pico_stdlib
	pico_multicore
	pico_aon_timer
	pico_unique_id
	hardware_adc
	hardware_dma
	hardware_irq
	hardware_pio
	hardware_vreg
	tinyusb_device
	tinyusb_board
)

pico_enable_stdio_usb(pico-thing 0)
pico_enable_stdio_uart(pico-thing 1)

# Link-time optimisation, across OUR translation units only - the SDK's static
# libraries are built without it and link normally alongside.
#
# WHERE THE GAIN IS, and is not.  The Q-rising handler is already fully
# inlined: a disassembly of it shows 137 instructions and ZERO out-of-line
# calls, because everything it touches (set_busy_lic_avma, count_lic, tick,
# trace_bla_store) is defined in a header.  LTO cannot improve it.
#
# Core 1's for(;;) loop is the opposite: on every pass it calls
# uart_port_pump() in uart_pump.cpp, mc6809::uart_task() in mc6809.cpp, and
# sync_status()/has_interrupt()/guest_transmit() in mc6850.cpp, none of which
# can inline today.  That loop costs roughly 77 clk_sys cycles per E cycle and
# is the term that decides where the guest's clock ceiling falls, so this is
# the place worth attacking.
#
# ThinLTO rather than full: for a tree this size it is as effective and builds
# far quicker.  CHECK AFTER TURNING IT ON that the hot code is still in RAM -
# the __time_critical_func and __no_inline_not_in_flash_func placements must
# survive, or the per-cycle paths start running from XIP flash and everything
# measured here becomes meaningless:
#
#   llvm-objdump -t pico-thing.elf | grep _Z18gpio_q_irq_handlerv
#
# should show it in .data, not .text.
option(PICO_THING_LTO "Link-time optimisation across the project's own sources" OFF)
if(PICO_THING_LTO)
	message(STATUS "Pico-Thing LTO: ThinLTO enabled")
	target_compile_options(pico-thing PRIVATE -flto=thin)
	target_link_options(pico-thing PRIVATE -flto=thin)
else()
	message(STATUS "Pico-Thing LTO: off")
endif()

pico_add_extra_outputs(pico-thing)
pico_set_binary_type(pico-thing copy_to_ram)
