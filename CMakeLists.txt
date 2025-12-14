cmake_minimum_required(VERSION 3.13)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico-mc6809 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(pico-mc6809
		src/mc6809.cpp
		src/srec.cpp
		src/uart.cpp
		src/usb_descriptors.c
		src/optional_usb.c
)

# Include both the project headers and generated ones
target_include_directories(pico-mc6809
	PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/include
	${CMAKE_CURRENT_BINARY_DIR}
)

pico_generate_pio_header(pico-mc6809 ${CMAKE_CURRENT_LIST_DIR}/src/mc6809.pio)

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
target_sources(pico-mc6809
	PRIVATE
	    ${CMAKE_CURRENT_LIST_DIR}/include/mc6809.h
		${CMAKE_CURRENT_LIST_DIR}/include/build_time.h
		${CMAKE_CURRENT_LIST_DIR}/include/srec.h
		${CMAKE_CURRENT_LIST_DIR}/include/uart.h
		${CMAKE_CURRENT_BINARY_DIR}/api.inc
)

target_link_libraries(pico-mc6809
	pico_stdlib
	pico_multicore
	pico_aon_timer
	pico_unique_id
	hardware_dma
	hardware_irq
	hardware_pio
	hardware_vreg
	tinyusb_device
	tinyusb_board
)

pico_enable_stdio_usb(pico-mc6809 0)
pico_enable_stdio_uart(pico-mc6809 1)

pico_add_extra_outputs(pico-mc6809)
# pico_set_binary_type(pico-mc6809 copy_to_ram)

