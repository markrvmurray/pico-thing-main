/**
 * Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <algorithm>
#include <cstdint>

#include "pico/stdlib.h"

#include "registers.h"
#include "pico_thing.h"
#include "mc6809.h"
#include "mc6850.h"
#include "uart_pump.h"

void
uart_port_pump(mc6850 &serial, const uint8_t cdc_port, uart_port_state &st,
	       const bool suppress_read, const cdc_port_io &io)
{
	const bool connected = io.connected(cdc_port);

	// USB -> guest, only while the guest has RTS asserted.
	if (connected && !serial.is_rts_deasserted() && !suppress_read) {
		if (st.rx_count == 0u) {
			st.rx_count = io.read(cdc_port, st.rx_buf, USB_BUFFER_SIZE);
			st.rx_index = 0u;
		}
		const uint transmit_amount = std::min<uint>(serial.host_transmit_level_avail(),
							     st.rx_count - st.rx_index);
		for (uint i = 0u; i < transmit_amount; i++)
			serial.host_transmit(st.rx_buf[st.rx_index + i]);
		st.rx_index += transmit_amount;
		if (st.rx_index >= st.rx_count)
			st.rx_count = 0u;
	}

	// Guest -> USB.  With nobody listening - no terminal, or one that has not
	// raised DTR - the output is thrown away, as a real UART's would be with
	// nothing plugged in, so a guest printing to a closed port carries on
	// instead of stalling behind CTS.
	if (!connected) {
		while (serial.host_receive_avail())
			(void)serial.host_receive();
		st.tx_count = st.tx_index = 0u;
		serial.set_host_cts(false);
		return;
	}
	if (st.tx_count == 0u) {
		while (serial.host_receive_avail() && st.tx_count < USB_BUFFER_SIZE)
			st.tx_buf[st.tx_count++] = serial.host_receive();
		st.tx_index = 0u;
	}
	// A host that drops RTS cannot take more: hold the output, and let CTS
	// stall the guest until RTS is raised again.  Unless this port has been
	// told not to believe its RTS, in which case write regardless and let the
	// CDC FIFO and CTS do the throttling - see uart_port_state.
	if ((!st.honour_host_rts || io.host_ready(cdc_port)) && st.tx_index < st.tx_count) {
		st.tx_index += io.write(cdc_port, st.tx_buf + st.tx_index,
					static_cast<uint16_t>(st.tx_count - st.tx_index));
		if (st.tx_index >= st.tx_count)
			st.tx_count = 0u;
	}
	serial.set_host_cts(st.tx_count > 0u && st.tx_index < st.tx_count);
}
