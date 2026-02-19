cmake_minimum_required(VERSION 3.14)
project(pico-mc6809-tests CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

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
# Future test targets for hardware-emulation classes (mc6850, mc6840, …)
# should be built in a separate cmake invocation with PICO_PLATFORM=host so
# the pico-sdk provides stub/no-op implementations of gpio_*, pio_*, etc.
# See: https://github.com/raspberrypi/pico-sdk/tree/master/test
# Build command:
#   cmake -S .. -B ../build/host-tests -DPICO_PLATFORM=host
#   cmake --build ../build/host-tests
# ---------------------------------------------------------------------------
