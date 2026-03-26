/**
 * Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <cstdint>

#include "pico/time.h"
#include "interrupt.h"

// Forward declaration — full header included in tick_timer.cpp.
class registers;

class tick_timer {
	static constexpr int64_t TICK_PERIOD_US = -20000; // 50 Hz, negative = fixed interval
	registers &reg;
	uint16_t control_offset;
	uint16_t status_offset;
	struct repeating_timer hw_timer{};
	volatile bool irq_pending = false;
	volatile bool enabled = false;
	volatile uint8_t nmi_countdown = 0;

	static bool callback(repeating_timer *rt);

public:
	tick_timer(registers &r, uint16_t ctrl_off, uint16_t stat_off);

	void start();
	void stop();
	void control(uint8_t val);
	void acknowledge();
	void reset();
	[[nodiscard]] interrupt has_interrupt() const;
	[[nodiscard]] bool is_enabled() const { return enabled; }
	[[nodiscard]] bool is_irq_pending() const { return irq_pending; }
	[[nodiscard]] uint8_t get_nmi_countdown() const { return nmi_countdown; }

	// Called once per E-cycle (Q-rising ISR).  Returns true when the
	// NMI countdown expires, signalling the caller to set PEND_NMI.
	bool tick();

	// Simulate a hardware timer fire (calls the SDK callback path).
	// Used by unit tests; in production the SDK alarm calls callback() directly.
	void simulate_tick() { callback(&hw_timer); }
};
