/**
 * Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <cstdint>

class mc6850;

#define USB_BUFFER_SIZE 512

// Bytes on their way between an ACIA and its USB CDC port, in each direction.
struct uart_port_state {
	uint8_t rx_buf[USB_BUFFER_SIZE];
	uint8_t tx_buf[USB_BUFFER_SIZE];
	uint16_t rx_count = 0, rx_index = 0;
	uint16_t tx_count = 0, tx_index = 0;
};

// What the pump needs from a CDC port.  The firmware passes TinyUSB wrappers;
// the host tests pass a fake port.
struct cdc_port_io {
	bool (*connected)(uint8_t itf);		// the host has the port open (DTR)
	bool (*host_ready)(uint8_t itf);	// the host can take data (RTS)
	uint16_t (*read)(uint8_t itf, uint8_t *buf, uint16_t len);
	uint16_t (*write)(uint8_t itf, const uint8_t *buf, uint16_t len);
};

// Move what is waiting between serial and CDC port cdc_port, in both
// directions.  suppress_read leaves host input where it is.
void uart_port_pump(mc6850 &serial, uint8_t cdc_port, uart_port_state &st,
		    bool suppress_read, const cdc_port_io &io);
