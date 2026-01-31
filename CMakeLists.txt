cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico-thing C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Trick target
add_custom_target(clion_host_dummy ALL)

add_compile_definitions(DEBUG)

pico_sdk_init()

add_executable(pico-thing
		src/pico_thing.cpp
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
# pico_set_binary_type(pico-thing copy_to_ram)