/**
* Copyright (c) 2025 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "pico/stdlib.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"

#include "tusb.h"
#include "usb.h"
#include "registers.h"
#include "pico_thing.h"
#include "mc6809.h"
#include "mc6850.h"

const char *bus_state_string[] = {
	foreach_busstate(enum_list_strings)
};

const char *run_state_string[] = {
	foreach_runstate(enum_list_strings)
};

mc6809::mc6809() : task(0u), busy(false), lic(false), vma(false), trace{}
{
	old_bus_state = BUS_RUNNING;
	bus_state = BUS_RUNNING;
	run_state = RS_UNPOWERED;
	busy_lic_avma.byte = 0x00u;
	task_stack_ptr = 0u;
	e_freq = GUEST_CLK_DEFAULT;
#ifdef DEBUG
	_count_lic = 0u;
	_count_rti = 0u;
	for (auto &c : _count_irq_ack) c = 0u;
#endif
}

// Done once at start of run
void
mc6809::task_initialise()
{
	reg[SYSTEM_TASK] = static_cast<uint8_t>(0u);
	task_stack_ptr = 0x00u;
	task = 0u;
}

// Done when the guest changes the task register; task pins are
// driven by the PIO task_output program in pico_thing.cpp.
uint8_t
mc6809::task_change(uint8_t new_task)
{
	new_task &= TASK_PINS_MASK;
	task = new_task;
	return new_task;
}

// Return the active task number
uint8_t
mc6809::task_get() const
{
	return task;
}

bool
mc6809::assert_powered() const
{
	bool retval = run_state != RS_UNPOWERED;
	if (!retval)
		printf("MC6809 not powered\n");;
	return retval;
}

bool
mc6809::assert_stopped() const
{
	if (!assert_powered())
		return false;
	bool retval = run_state == RS_RESET;
	if (!retval)
		printf("MC6809 not stopped, currently in %s\n", run_state_string[run_state]);
	return retval;
}

void
mc6809::setup(enum run_state rs)
{
	assert(rs == RS_RESET);
	if (setup_callback)
		setup_callback();
	run_state = rs;
	old_bus_state = BUS_RUNNING;
	bus_state = BUS_RUNNING;
	busy_lic_avma.byte = 0x00u;
	task_stack_ptr = 0u;
	vma = false;
#ifdef DEBUG
	_count_lic = 0u;
	_count_rti = 0u;
	for (auto &c : _count_irq_ack) c = 0u;
	trace_pos = 0u;
#endif
}

void
mc6809::init()
{
	POWER_ASSERT;
	// 0.25s should be enough for power to stabilise
	sleep_ms(250u);

	// Assert reset until the guest is needed
	gpio_init(GPIO_RESET);
	gpio_set_dir(GPIO_RESET, GPIO_OUT);
	RESET_ASSERT;

	// Hold !halt high until needed
	gpio_init(GPIO_HALT);
	gpio_set_dir(GPIO_HALT, GPIO_OUT);
	HALT_DEASSERT;

	// The 3 interrupt outputs are going to pretend to be open-collector by
	// either going high impedance (becoming an input) or driving low
	// (becoming an output). Macros will be used to avoid confusion.
	gpio_init(GPIO_NMI);
	gpio_put(GPIO_NMI, false);
	gpio_set_drive_strength(GPIO_NMI, GPIO_DRIVE_STRENGTH_8MA);
	DEASSERT_NMI;
	gpio_init(GPIO_FIRQ);
	gpio_put(GPIO_FIRQ, false);
	gpio_set_drive_strength(GPIO_FIRQ, GPIO_DRIVE_STRENGTH_8MA);
	DEASSERT_FIRQ;
	gpio_init(GPIO_IRQ);
	gpio_put(GPIO_IRQ, false);
	gpio_set_drive_strength(GPIO_IRQ, GPIO_DRIVE_STRENGTH_8MA);
	DEASSERT_IRQ;

	setup(RS_RESET);
	task_initialise();
}

void
mc6809::deinit()
{
	// All pins driving the 6809 go low or hi-Z to avoid
	// backfeeding when external power is removed.

	// RESET: drive low (hold 6809 in reset), then go hi-Z
	gpio_put(GPIO_RESET, false);
	gpio_set_dir(GPIO_RESET, GPIO_IN);

	// HALT: drive low, then go hi-Z
	gpio_put(GPIO_HALT, false);
	gpio_set_dir(GPIO_HALT, GPIO_IN);

	// NMI/FIRQ/IRQ: already simulated open-collector —
	// ensure output latch is low, then go hi-Z (input)
	gpio_put(GPIO_NMI, false);
	DEASSERT_NMI;
	gpio_put(GPIO_FIRQ, false);
	DEASSERT_FIRQ;
	gpio_put(GPIO_IRQ, false);
	DEASSERT_IRQ;

	POWER_DEASSERT;
	old_bus_state = BUS_RUNNING;
	bus_state = BUS_RUNNING;
	run_state = RS_UNPOWERED;
}

void
mc6809::start()
{
	if (!assert_stopped())
		return;
	setup(RS_RESET);
	if (verbose)
		printf("Starting MC6809 from %04X at E = %.1f MHz\n\n", static_cast<uint16_t>(reg[REGISTER_VECTOR_RESET_OFFSET]), e_freq);
	RESET_DEASSERT;
	if (verbose) {
		sleep_ms(10u);
		printf("Started MC6809 bus state = %u = %s\n", bus_state, bus_state_string[bus_state]);
		printf("Started MC6809 run state = %u = %s\n", run_state, run_state_string[run_state]);
	}
}

bool
mc6809::start_with_timeout(const uint32_t timeout_ms, bool sync_means_stop)
{
	uint32_t i;
	uint32_t timeout_us = timeout_ms*1000u;

	timeout_us = MAX(1000u, timeout_us);
	timeout_us = MIN(1000000u, timeout_us);
	if (!assert_stopped())
		return false;
	if (verbose)
		printf("Starting MC6809 from %04X at E = %.1f MHz and timeout = %u us\n\n", static_cast<uint16_t>(reg[REGISTER_VECTOR_RESET_OFFSET]), e_freq, timeout_us);
	setup(RS_RESET);
#ifdef DEBUG
	uint32_t lic_start = _count_lic;
	uint32_t rti_start = _count_rti;
#endif
	RESET_DEASSERT;
	for (i = 0; i < 1000u*timeout_ms; i++) {
		sleep_us(1u);
		if (run_state == RS_SYNCED) {
			if (verbose)
				printf("Stopping MC6809 due to SYNC\n");
			break;
		}
	}
	bool ok = (run_state == RS_SYNCED);
	if (sync_means_stop && ok)
		RESET_ASSERT;
	if (!ok)
		RESET_ASSERT;
	if (verbose) {
		printf("MC6809 run took %u iterations\n", i);
#ifdef DEBUG
		printf("%lu LICs were counted\n", _count_lic - lic_start);
		printf("%lu RTIs were counted\n", _count_rti - rti_start);
#endif
		if (i == 1000u*timeout_ms)
			printf("Timeout was reached\n");
	}
	return ok;
}

void
mc6809::stop()
{
	RESET_ASSERT;
	if (verbose)
		printf("\n\nStopped MC6809\n");
}

void
mc6809::halt()
{
	HALT_ASSERT;
	if (verbose)
		printf("\n\nHalted MC6809\n");
}

void
mc6809::release()
{
	if (verbose)
		printf("Releasing MC6809\n");
	HALT_DEASSERT;
}

void
mc6809::apply_interrupts(uint32_t interrupt_refcount[NUM_INTERRUPTS])
{
	if (interrupt_refcount[INTERRUPT_NMI] > 0u) {
		ASSERT_NMI;
		interrupt_refcount[INTERRUPT_NMI] = 0u;
	} else
		DEASSERT_NMI;
	if (interrupt_refcount[INTERRUPT_FIRQ] > 0u) {
		ASSERT_FIRQ;
		interrupt_refcount[INTERRUPT_FIRQ] = 0u;
	} else
		DEASSERT_FIRQ;
	if (interrupt_refcount[INTERRUPT_IRQ] > 0u) {
		ASSERT_IRQ;
		interrupt_refcount[INTERRUPT_IRQ] = 0u;
	} else
		DEASSERT_IRQ;
}

// The guest just executed an RTI instruction
void
mc6809::apply_rti()
{
#ifdef DEBUG
	count_rti();
#endif
}

#define USB_BUFFER_SIZE 512

struct uart_port_state {
	uint8_t rx_buf[USB_BUFFER_SIZE];
	uint8_t tx_buf[USB_BUFFER_SIZE];
	uint16_t rx_count = 0, rx_index = 0;
	uint16_t tx_count = 0, tx_index = 0;
};

static uart_port_state port_state[2];

// Debug counters for aux ACIA (CDC port 1) diagnostics
static void
uart_port_pump(mc6850 &serial, uint8_t cdc_port, uart_port_state &st, bool suppress_read)
{
	// READ USB -> Guest (only if 6809 has asserted RTS)
	if (!serial.is_rts_deasserted() && !suppress_read) {
		if (st.rx_count == 0u) {
			st.rx_count = usb_cdc_read_n(cdc_port, st.rx_buf, USB_BUFFER_SIZE);
			st.rx_index = 0u;
		}
		uint uart_level_avail = serial.host_transmit_level_avail();
		uint transmit_amount = MIN(uart_level_avail, st.rx_count - st.rx_index);
		for (uint i = 0u; i < transmit_amount; i++)
			serial.host_transmit(st.rx_buf[st.rx_index + i]);
		st.rx_index += transmit_amount;
		if (st.rx_index >= st.rx_count)
			st.rx_count = 0u;
	}
	// WRITE Guest -> USB
	if (st.tx_count == 0u) {
		while (serial.host_receive_avail() && st.tx_count < USB_BUFFER_SIZE)
			st.tx_buf[st.tx_count++] = serial.host_receive();
		st.tx_index = 0u;
	}
	uint16_t written = usb_cdc_write_n(cdc_port, st.tx_buf + st.tx_index, st.tx_count - st.tx_index);
	st.tx_index += written;
	if (st.tx_index >= st.tx_count)
		st.tx_count = 0u;
	// Feed USB host readiness back to CTS
	bool cts = st.tx_count > 0u && st.tx_index < st.tx_count;
	serial.set_host_cts(cts);
}

void
mc6809::uart_task(bool suppress_cdc_read)
{
	if (MC6809.run_state >= RS_RUNNING) {
		uart_port_pump(fast_serial, 0, port_state[0], suppress_cdc_read);
		uart_port_pump(aux_serial, 1, port_state[1], false);
		usb_cdc_flush_n(0);
		usb_cdc_flush_n(1);
	} else if (MC6809.run_state == RS_RESET) {
		port_state[0] = {};
		port_state[1] = {};
	}
}

void
mc6809::preserve_state()
{
}

void
mc6809::restore_state()
{
}

void
mc6809::set_e_frequency(const float target_mhz, const pio_dma &pio_clock)
{
	e_freq = target_mhz;
	if (e_freq < GUEST_CLK_MIN)
		e_freq = GUEST_CLK_MIN;
	if (e_freq > GUEST_CLK_MAX)
		e_freq = GUEST_CLK_MAX;
	const auto sys_clock_rate = static_cast<float>(clock_get_hz(clk_sys));
	const float guest_divisor = sys_clock_rate/(4.0f*e_freq*1.0e6f);
	pio_sm_set_clkdiv(pio_clock.pio, pio_clock.sm, guest_divisor);
	if (verbose)
		printf("E/Q clock divisor = %f\n", guest_divisor);
}

float
mc6809::get_e_frequency() const
{
	return e_freq;
}
