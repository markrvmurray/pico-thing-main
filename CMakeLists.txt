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
		src/usb_descriptors.c
		src/usb.c
		src/mc6840.cpp
)

# Include both the project headers and generated ones
target_include_directories(pico-thing
	PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/include
	${CMAKE_CURRENT_BINARY_DIR}
)

pico_generate_pio_header(pico-thing ${CMAKE_CURRENT_LIST_DIR}/src/pico_thing.pio)

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/api.bin
	DEPENDS ${CMAKE_CURRENT_LIST_DIR}/extra/api.asm
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

target_link_libraries(pico-thing
	pico_stdlib
	pico_multicore
	pico_aon_timer
	pico_unique_id
	# hardware_adc
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
