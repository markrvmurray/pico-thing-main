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

# Whether the Pico restores the interrupted task itself, about 15 E cycles after
# it sees an RTI.  That is right for an interrupt which resumes what it
# interrupted and wrong for one which does not: a DAT fault handler that kills
# the faulting process finds the dead task forced back under it a moment after
# its RTI.  Off by default, so the task register belongs to the guest.  The task
# stack and the nesting depth are maintained either way, and the DAT fault's own
# switch to task 0 at the NMI vector fetch is separate and always happens.
set(PICO_THING_RTI_TASK_RESTORE 0 CACHE STRING
	"Restore the interrupted task ~15 E cycles after an RTI (0 = the guest owns the task register)")
message(STATUS "Pico-Thing RTI task restore ${PICO_THING_RTI_TASK_RESTORE}")

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
	PICO_THING_RTI_TASK_RESTORE=${PICO_THING_RTI_TASK_RESTORE}
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

pico_add_extra_outputs(pico-thing)
pico_set_binary_type(pico-thing copy_to_ram)
