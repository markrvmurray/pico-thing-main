/**
 * Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "mc6809.h"

// ANSI escape sequences
static constexpr const char *ANSI_CLEAR_SCREEN = "\033[2J";
static constexpr const char *ANSI_CURSOR_HOME  = "\033[H";

struct acia_status_t {
	uint8_t  interrupt_status;
	uint16_t tx_level;
	uint16_t rx_level;
#ifdef DEBUG
	unsigned control_count;
	unsigned tx_data_count;
	unsigned status_count;
	unsigned rx_data_count;
#endif
};

#define STATUS_MAX_INT_NEST_DEPTH 16u

struct status_snapshot_t {
	// Processor state
	enum run_state  run_state;
	enum bus_state  bus_state;
	enum bus_state  old_bus_state;
	uint8_t         task;
	uint16_t        task_stack_ptr;
	uint8_t         task_history[STATUS_MAX_INT_NEST_DEPTH];

	// GPIO pins
	bool power, reset, halt, nmi, firq, irq;

#ifdef DEBUG
	uint32_t lic_count;
	uint32_t rti_count;
	uint32_t last_lic;
	bool     lic_stalled;
#endif

	// Register blocks (raw bytes for hex dump / JSON array)
	uint8_t write_regs[64];
	uint8_t read_regs[64];

	// Tick timer
	bool tick_enabled;
	bool tick_irq_pending;

	// Core 1 diagnostics
	uint32_t core1_loop, core1_qrise, core1_irq_assert;
	uint32_t core1_write_ring, core1_read_irq;
	uint8_t  core1_last_write, core1_last_read;
	uint32_t core1_isr;
	uint32_t core1_stack_total, core1_stack_free;

	// ACIAs
	acia_status_t console_acia;
	acia_status_t aux_acia;
};

void status_gather(status_snapshot_t *s);
void status_print(const status_snapshot_t *s);
void status_json(const status_snapshot_t *s);
