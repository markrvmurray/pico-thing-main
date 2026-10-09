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
	// This ACIA's CDC port, as the pump sees it.  dtr: the host has the port
	// open.  host_rts: the host says it can take data.  honour_host_rts:
	// whether this port believes that.  held: output is waiting because it
	// cannot be written yet - the guest is stalled behind CTS.
	bool     dtr, host_rts, honour_host_rts, held;
#ifdef DEBUG
	unsigned control_count;
	unsigned tx_data_count;
	unsigned status_count;
	unsigned rx_data_count;
#endif
};


struct status_snapshot_t {
	// Processor state
	enum run_state  run_state;
	enum bus_state  bus_state;
	enum bus_state  old_bus_state;
	uint8_t         task;
	// Guest writes to the task register that no kernel path makes: from a
	// non-zero task, to a different non-zero task.  Expected to stay 0.
	uint32_t        task_write_odd_count;
	uint8_t         task_write_odd_from, task_write_odd_to;

	// GPIO pins
	bool power, reset, halt, nmi, firq, irq;
	uint16_t voltage, temperature;   // raw counts; voltage is 0 on version 1

	// Pico silicon
	uint8_t chip_revision;           // CHIP_ID.REVISION, raw
	uint8_t rom_version;             // bootrom version byte: 2 A2, 3 A3, 4 A4
	bool    qfn60;                   // PACKAGE_SEL: RP2350A (QFN-60), else RP2350B (QFN-80)

	// Clock tree as measured, in kHz
	uint32_t clk_sys_khz, clk_peri_khz, clk_usb_khz, clk_adc_khz;

#ifdef DEBUG
	uint32_t lic_count;
	uint32_t last_lic;
	bool     lic_stalled;
	float    rail_baseline;	// the cutout's slow reference, 0 = not established
	uint32_t rail_good;	// consecutive healthy samples behind it
	bool     rail_clipped;	// supply channel at full scale: no measurement
	bool     rail_floor_only;	// serving: absolute floor only, baseline frozen
	uint32_t write_ring_overflow;	// dropped guest-write dispatches: real loss
	uint32_t write_addr_worst;	// most ring entries one write interrupt drained
	uint32_t write_addr_coalesced;	// writes that shared an interrupt with another
	uint32_t write_addr_overflow;	// the address ring may have lapped
	uint8_t  busy_lic_avma;	// the pins, as the Q handler last read them
	uint8_t  pio2_irq;	// all eight of pio2's SM-visible IRQ flags
	uint32_t write_fdebug;	// the write PIO's FDEBUG: did a push ever stall?
	uint8_t  clock_pc;	// the E/Q clock SM's PC, to prove it is cycling
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
