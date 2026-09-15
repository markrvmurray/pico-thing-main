cmake_minimum_required(VERSION 3.14)
project(pico-thing-main-tests CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

# Keep the test build tree out of Time Machine, as the firmware's is.  A sticky
# exclusion (no -p) is stored on the directory itself, so it needs no root; it
# goes if the directory is deleted and comes back at the next configure.
# CMAKE_HOST_APPLE to match the firmware build, where APPLE is false.
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

# macOS ships bison 2.3 which is too old; use Homebrew bison 3.x.
if(APPLE)
    find_program(_bison_homebrew
        NAMES bison
        PATHS /opt/homebrew/opt/bison/bin /usr/local/opt/bison/bin
        NO_DEFAULT_PATH
    )
    if(_bison_homebrew)
        set(BISON_EXECUTABLE "${_bison_homebrew}" CACHE FILEPATH "Bison executable" FORCE)
    endif()
endif()
find_package(FLEX  REQUIRED)
find_package(BISON 3.0 REQUIRED)

include(FetchContent)
FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.7.1
)
FetchContent_MakeAvailable(Catch2)

# Catch2's extra CMake modules supply catch_discover_tests()
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(CTest)
include(Catch)

# ---------------------------------------------------------------------------
# test_srec: pure host build, no pico-sdk dependency
# ---------------------------------------------------------------------------
add_executable(test_srec
    test_srec.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/srec.cpp
)
target_include_directories(test_srec PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(test_srec PRIVATE Catch2::Catch2WithMain)
catch_discover_tests(test_srec)

# ---------------------------------------------------------------------------
# test_command_parser: flex/bison parser compiled natively for the host.
#
# Re-runs flex/bison on the same source files used by the firmware build;
# the generated .c files land in the tests binary directory and are compiled
# with the host compiler (not arm-none-eabi-gcc).
# ---------------------------------------------------------------------------
FLEX_TARGET(CmdLexerTest
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/command_lexer.l
    ${CMAKE_CURRENT_BINARY_DIR}/command_lexer.c
)
BISON_TARGET(CmdParserTest
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/command_parser.y
    ${CMAKE_CURRENT_BINARY_DIR}/command_parser.c
    DEFINES_FILE ${CMAKE_CURRENT_BINARY_DIR}/command_parser.tab.h
)
ADD_FLEX_BISON_DEPENDENCY(CmdLexerTest CmdParserTest)

add_executable(test_command_parser
    test_command_parser.cpp
    cmd_parse_shim.c
    ${FLEX_CmdLexerTest_OUTPUTS}
    ${BISON_CmdParserTest_OUTPUTS}
)
target_include_directories(test_command_parser PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_BINARY_DIR}
)
target_link_libraries(test_command_parser PRIVATE Catch2::Catch2WithMain)
catch_discover_tests(test_command_parser)

# ---------------------------------------------------------------------------
# test_mc6850: host-native build using fake/ stub headers.
#
# fake/pico/util/queue.h provides a simple ring-buffer queue_t in place of
# the SDK's spinlock-protected multicore queue.
# ---------------------------------------------------------------------------
add_executable(test_mc6850
    test_mc6850.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/mc6850.cpp
)
target_include_directories(test_mc6850 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/fake   # fake Pico/registers stubs (first!)
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_compile_features(test_mc6850 PRIVATE cxx_std_17)
target_link_libraries(test_mc6850 PRIVATE Catch2::Catch2WithMain)
catch_discover_tests(test_mc6850)

# ---------------------------------------------------------------------------
# test_tick_timer: host-native build using fake/ stub headers.
#
# fake/pico/time.h stubs out repeating_timer and add_repeating_timer_us().
# simulate_tick() calls the callback directly for deterministic testing.
# ---------------------------------------------------------------------------
add_executable(test_tick_timer
    test_tick_timer.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/tick_timer.cpp
)
target_include_directories(test_tick_timer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/fake   # fake Pico/registers stubs (first!)
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_compile_features(test_tick_timer PRIVATE cxx_std_17)
target_link_libraries(test_tick_timer PRIVATE Catch2::Catch2WithMain)
catch_discover_tests(test_tick_timer)

# ---------------------------------------------------------------------------
# test_sysreq: the floating-point system request, against the REAL registers.h.
#
# include/ comes first so registers.h is the firmware's own: its float
# conversions are part of what is under test, and fake/registers.h has none.
# fake/ follows only to supply pico.h.
# ---------------------------------------------------------------------------
add_executable(test_sysreq
    test_sysreq.cpp
)
target_include_directories(test_sysreq PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include   # real registers.h (first!)
    ${CMAKE_CURRENT_SOURCE_DIR}/fake         # pico.h stub only
)
target_compile_features(test_sysreq PRIVATE cxx_std_17)
target_link_libraries(test_sysreq PRIVATE Catch2::Catch2WithMain)
catch_discover_tests(test_sysreq)

# ---------------------------------------------------------------------------
# test_uart_pump: the ACIA <-> USB CDC pump, with a real mc6850 and a fake
# port, using the same fake/ stubs as test_mc6850.
# ---------------------------------------------------------------------------
add_executable(test_uart_pump
    test_uart_pump.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/uart_pump.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/mc6850.cpp
)
target_include_directories(test_uart_pump PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/fake   # fake Pico/registers stubs (first!)
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_compile_features(test_uart_pump PRIVATE cxx_std_17)
target_link_libraries(test_uart_pump PRIVATE Catch2::Catch2WithMain)
catch_discover_tests(test_uart_pump)
