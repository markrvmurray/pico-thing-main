#pragma once
#include <cstdint>
// Minimal stub: register offsets used by mc6840.cpp and mc6850.cpp.

// Emulated UART (offsets from register base $FFC0)
#define CONSOLE_CONTROL		uint16_t(0x03u)
#define CONSOLE_STATUS		uint16_t(0x03u)
#define CONSOLE_TX_DATA		uint16_t(0x04u)
#define CONSOLE_RX_DATA		uint16_t(0x04u)

// Simple 50 Hz tick timer
#define SYSTEM_TIMER_CONTROL_13	uint16_t(0x08u)
#define SYSTEM_TIMER_STATUS	uint16_t(0x09u)
