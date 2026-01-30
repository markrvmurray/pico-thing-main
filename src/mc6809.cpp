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
#include "pico_thing.h"
#include "mc6809.h"
#include "mc6850.h"

const char *ba_bs_string[] = {
	foreach_busstate(enum_list_strings)
};

const char *run_state_string[] = {
	foreach_runstate(enum_list_strings)
};

mc6809::mc6809() : task(0u), busy(false), lic(false), vma(false), trace{}
{
	old_bus_state.state = BS_RUNNING_RESET;
	bus_state.state = BS_RUNNING_RESET;
	run_state = RS_STOPPED;
	busy_lic_avma.byte = 0x00u;
	task_stack_ptr = 0u;
	e_freq = GUEST_CLK_DEFAULT;
#ifdef DEBUG
	_count_lic = 0u;
#endif
}

// Done once at start of run
void
mc6809::task_initialise()
{
	reg[SYSTEM_TASK] = static_cast<uint8_t>(0u);
	task_stack_ptr = 0x00u;
	task = 0u;
	init_pins_range(GPIO_TASK_BASE, NUM_TASK_PINS);
	gpio_set_dir_out_masked64(TASK_PINS_MASK << GPIO_TASK_BASE);
	gpio_init(GPIO_TASK_0);
	gpio_set_dir(GPIO_TASK_0, GPIO_OUT);
	gpio_put_masked64(TASK_PINS_MASK << GPIO_TASK_BASE, 0LLu);
	gpio_put(GPIO_TASK_0, true);
}

// Done when the guest changes the task register
uint8_t
mc6809::task_change(uint8_t new_task)
{
	new_task &= TASK_PINS_MASK;
	task = new_task;
	gpio_put_masked64(TASK_PINS_MASK << GPIO_TASK_BASE, new_task << GPIO_TASK_BASE);
	gpio_put(GPIO_TASK_0, new_task == 0u);
	return new_task;
}

// The guest just executed an RTI instruction
void
mc6809::apply_rti()
{
}

// Return the active task number
uint8_t
mc6809::task_get() const
{
	return task;
}

bool
mc6809::assert_stopped() const
{
	bool retval = run_state == RS_STOPPED;
	if (!retval)
		printf("MC6809 not stopped, currently in %s\n", run_state_string[run_state]);
	return retval;
}

void
mc6809::setup(enum run_state rs)
{
	assert(rs == RS_STOPPED);
	run_state = rs;
	old_bus_state.state = BS_RUNNING_RESET;
	bus_state.state = BS_RUNNING_RESET;
	busy_lic_avma.byte = 0x00u;
	task = 0u;
	task_stack_ptr = 0u;
	vma = false;
#ifdef DEBUG
	_count_lic = 0u;
	trace_pos = 0u;
#endif
	task_initialise();
}

void
mc6809::init()
{
	RESET_ASSERT;
	HALT_DEASSERT;
	DEASSERT_NMI;
	DEASSERT_FIRQ;
	DEASSERT_IRQ;
	setup(RS_STOPPED);
}

void
mc6809::start()
{
	if (!assert_stopped())
		return;
	setup(RS_STOPPED);
	if (verbose)
		printf("Starting MC6809 from %04X at E = %.1f MHz\n\n", static_cast<uint16_t>(reg[REGISTER_VECTOR_RESET_OFFSET]), e_freq);
	RESET_DEASSERT;
	if (verbose) {
		sleep_ms(10u);
		printf("Started MC6809 bus state = %u = %s\n", bus_state.state, ba_bs_string[bus_state.state]);
		printf("Started MC6809 run state = %u = %s\n", run_state, run_state_string[run_state]);
	}
}

void
mc6809::start_with_timeout(const uint32_t timeout_ms, bool sync_means_stop)
{
	uint32_t i;
	uint32_t timeout_us = timeout_ms*1000u;

	timeout_us = MAX(1000u, timeout_us);
	timeout_us = MIN(1000000u, timeout_us);
	if (!assert_stopped())
		return;
	if (verbose)
		printf("Starting MC6809 from %04X at E = %.1f MHz and timeout = %u us\n\n", static_cast<uint16_t>(reg[REGISTER_VECTOR_RESET_OFFSET]), e_freq, timeout_us);
	setup(RS_STOPPED);
#ifdef DEBUG
	uint32_t lic_start = _count_lic;
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
	if (sync_means_stop && run_state == RS_SYNCED)
		RESET_ASSERT;
	if (verbose) {
		printf("MC6809 run took %u iterations\n", i);
#ifdef DEBUG
		printf("%lu LICs were counted\n", _count_lic - lic_start);
#endif
		if (i == 1000u*timeout_ms)
			printf("Timeout was reached\n");
	}
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

#define USB_BUFFER_SIZE 64

void
mc6809::uart_task()
{
	static uint8_t receive_buffer[USB_BUFFER_SIZE], transmit_buffer[USB_BUFFER_SIZE];
	static uint16_t bytes_read = 0u, bytes_read_index = 0u, bytes_write = 0u, bytes_write_index = 0u;

	if (MC6809.run_state == RS_STARTED || MC6809.run_state == RS_SYNCED) {
		// READ USB -> Guest
		if (bytes_read == 0u) {
			bytes_read = usb_cdc_read(receive_buffer, USB_BUFFER_SIZE);
			bytes_read_index = 0u;
		}
		uint uart_level_avail = fast_serial.host_transmit_level_avail();
		uint transmit_amount = MIN(uart_level_avail, bytes_read - bytes_read_index);
		for (uint i = 0u; i < transmit_amount; i++)
			fast_serial.host_transmit(receive_buffer[bytes_read_index + i]);
		bytes_read_index += transmit_amount;
		if (bytes_read_index >= bytes_read)
			bytes_read = 0u;
		// WRITE Guest -> USB
		if (bytes_write == 0u) {
			while (fast_serial.host_receive_avail() && bytes_write < USB_BUFFER_SIZE)
				transmit_buffer[bytes_write++] = fast_serial.host_receive();
			bytes_write_index = 0u;
		}
		bytes_write_index += usb_cdc_write(transmit_buffer + bytes_write_index, bytes_write - bytes_write_index);
		if (bytes_write_index >= bytes_write)
			bytes_write = 0u;
	} else if (MC6809.run_state == RS_STOPPED) {
		bytes_read = 0u;
		bytes_read_index = 0u;
		bytes_write = 0u;
		bytes_write_index = 0u;
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
