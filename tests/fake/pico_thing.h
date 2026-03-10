#pragma once
#include <cstdint>
// Minimal stub: register offsets used by mc6840.cpp and mc6850.cpp.

// Emulated UARTs (offsets from register base $FFC0)
#define CONSOLE_CONTROL		uint16_t(0x03u)
#define CONSOLE_STATUS		uint16_t(0x03u)
#define CONSOLE_TX_DATA		uint16_t(0x04u)
#define CONSOLE_RX_DATA		uint16_t(0x04u)
#define AUX_CONTROL		uint16_t(0x05u)
#define AUX_STATUS		uint16_t(0x05u)
#define AUX_TX_DATA		uint16_t(0x06u)
#define AUX_RX_DATA		uint16_t(0x06u)

// Simple 50 Hz tick timer
#define SYSTEM_TIMER_CONTROL_13	uint16_t(0x08u)
#define SYSTEM_TIMER_STATUS	uint16_t(0x09u)
