#pragma once
#include <cstdint>
// Minimal stub: register offsets used by mc6840.cpp and mc6850.cpp.

// Emulated UART (offsets from register base $FFC0)
#define CONSOLE_CONTROL		uint16_t(0x03u)
#define CONSOLE_STATUS		uint16_t(0x03u)
#define CONSOLE_TX_DATA		uint16_t(0x04u)
#define CONSOLE_RX_DATA		uint16_t(0x04u)

#define SYSTEM_TIMER_BASE	uint16_t(0x08u)
#define SYSTEM_TIMER_CONTROL_13	uint16_t(0x08u)
#define SYSTEM_TIMER_CONTROL_2	uint16_t(0x09u)
#define SYSTEM_TIMER_STATUS	uint16_t(0x09u)
#define SYSTEM_TIMER_1_MSB	uint16_t(0x0Au)
#define SYSTEM_TIMER_1_LSB	uint16_t(0x0Bu)
#define SYSTEM_TIMER_2_MSB	uint16_t(0x0Cu)
#define SYSTEM_TIMER_2_LSB	uint16_t(0x0Du)
#define SYSTEM_TIMER_3_MSB	uint16_t(0x0Eu)
#define SYSTEM_TIMER_3_LSB	uint16_t(0x0Fu)
