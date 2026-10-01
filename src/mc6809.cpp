/**
* Copyright (c) 2025 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <cinttypes>
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
#include "uart_pump.h"

// Set from CMake with -DPICO_THING_RTI_TASK_RESTORE=1 to have the Pico put the
// interrupted task back after an RTI.  Off by default: see apply_rti().
#ifndef PICO_THING_RTI_TASK_RESTORE
#define PICO_THING_RTI_TASK_RESTORE 0
#endif

const char *bus_state_string[] = {
	foreach_busstate(enum_list_strings)
};

const char *run_state_string[] = {
	foreach_runstate(enum_list_strings)
};

// The one instance.  See the note in mc6809.h: a class member rather than a
// function-local static, so no use site pays for an initialisation guard.
mc6809 mc6809::instance;

mc6809::mc6809() : task(0u), busy(false), lic(false), vma(false), trace{}
{
	old_bus_state = BUS_RUNNING;
	bus_state = BUS_RUNNING;
	run_state = RS_UNPOWERED;
	busy_lic_avma.byte = 0x00u;
	task_stack_ptr = 0u;
	nesting_depth = 0u;
	rti_task_countdown = 0u;
	rti_pending_task = 0u;
	e_freq = GUEST_CLK_DEFAULT;
#ifdef DEBUG
	_count_lic = 0u;
	_count_rti = 0u;
#endif
	for (auto &c : _count_irq_ack) c = 0u;
}

// Done once at start of run
void
mc6809::task_initialise()
{
	reg[SYSTEM_TASK] = static_cast<uint8_t>(0u);
	task_stack_ptr = 0x00u;
	task = 0u;
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
	nesting_depth = 0u;
	rti_task_countdown = 0u;
	rti_pending_task = 0u;
	vma = false;
#ifdef DEBUG
	_count_lic = 0u;
	_count_rti = 0u;
#endif
	for (auto &c : _count_irq_ack) c = 0u;
	// If a one-shot trace is armed for this run, restart its fill cleanly.
	// An interrupt-triggered config (trace_irq_trigger) persists across restarts.
	if (trace_armed)
		trace_arm();
}

void
mc6809::init()
{
	// The 5V rail is brought up by the caller, which also confirms it
	// actually came up before anything is driven into the guest logic.

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

	// RP2350 pads come out of reset with their pull-downs enabled.  The
	// interrupt lines deassert by being released, and a released line with a
	// pull-down works against the pull-up meant to hold it inactive; !HALT is
	// released at power-down too.  Pull up instead - driving low to assert
	// still wins.
	static constexpr uint8_t released_pins[] = { GPIO_HALT, GPIO_NMI, GPIO_FIRQ, GPIO_IRQ };
	for (const uint8_t pin : released_pins)
		gpio_set_pulls(pin, true, false);

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

	// No pull either, so the pull-ups set by init() do not feed the guest
	// logic once its rail is gone.
	static constexpr uint8_t released_pins[] = { GPIO_HALT, GPIO_NMI, GPIO_FIRQ, GPIO_IRQ };
	for (const uint8_t pin : released_pins)
		gpio_disable_pulls(pin);

	// The caller drops the 5V rail once every pin is released.
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
		printf("%" PRIu32 " LICs were counted\n", _count_lic - lic_start);
		printf("%" PRIu32 " RTIs were counted\n", _count_rti - rti_start);
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

// The PIO RTI detector has spotted an RTI opcode fetch.  The RTI has not
// executed yet, so the interrupt is unwound here - nesting depth and task
// stack - but any task restore has to wait out the instruction: 15 E cycles
// for a full frame, 6 for FIRQ, so 15 covers both.
//
// With PICO_THING_RTI_TASK_RESTORE off, which is the default, no restore is
// armed and the task register stays where the handler left it.  Putting the
// interrupted task back suits an interrupt that resumes what it interrupted,
// but not a handler that does not: a DAT fault cannot be retried, so its
// handler kills the faulting process, and an automatic restore would force the
// dead task back under it about 15 E cycles after its RTI - too late to
// pre-empt by writing the task register first, and racing it afterwards.
void
mc6809::apply_rti()
{
#ifdef DEBUG
	count_rti();
#endif
	if (nesting_depth > 0u) {
		nesting_depth--;
		if (task_stack_ptr > 0u) {
			const uint8_t interrupted_task =
				task_stack[task_stack_ptr.fetch_sub(1u, std::memory_order_relaxed) - 1u];
#if PICO_THING_RTI_TASK_RESTORE
			rti_pending_task = interrupted_task;
			rti_task_countdown = 15u;
#else
			(void) interrupted_task;	// the guest owns the task register
#endif
		}
		if (nesting_depth == 0u && rti_task_countdown == 0u)
			run_state = RS_RUNNING;
	}
}

// The deferred task restore's countdown has expired: put the interrupted task
// back.  Reached from tick_rti_countdown(), which is inline in the header
// because it runs on every Q-rising; this half runs once per RTI at most, so
// it stays out of line and out of the ISR's instruction footprint.
void
mc6809::rti_task_expired()
{
	task = rti_pending_task;
	reg[SYSTEM_TASK] = rti_pending_task;
	if (task_pin_callback)
		task_pin_callback(rti_pending_task);
	if (nesting_depth == 0u)
		run_state = RS_RUNNING;
}

static uart_port_state port_state[2];

uart_port_state &
uart_port_get(const uint8_t port)
{
	return port_state[port & 1u];
}

// The pump's view of the TinyUSB CDC ports.
static constexpr cdc_port_io usb_ports = {
	usb_cdc_connected_n, usb_cdc_host_rts_n, usb_cdc_read_n, usb_cdc_write_n
};

void
mc6809::uart_task(bool suppress_cdc_read)
{
	if (MC6809.run_state == RS_UNPOWERED)
		return;
	// Guest to host whenever the guest is powered, not only while it runs.  A
	// run that reaches SYNC is put straight back into reset, and the next start
	// resets the ACIAs and empties their queues, so gating this on running lost
	// everything a short run sent.
	//
	// Host to guest only while it runs, and anything already staged for a
	// guest that is not running is DISCARDED - deliberately, on both ports.
	//
	// For CDC 0 that is obvious: it is also the command console, so a guest in
	// reset must not take what was typed at the prompt.
	//
	// For CDC 1 it is a protocol decision rather than an obvious one, and it
	// was taken explicitly (the user, 2026-10-01) rather than left to fall out
	// of sharing this code with CDC 0.  A guest that has stopped has no live
	// DriveWire transaction, so bytes the server sent meanwhile belong to a
	// conversation that no longer exists; handing them to the guest when it
	// restarts would prefix a fresh session with stale data and desynchronise
	// it.  Losing them is the lesser fault, and the server retries.
	//
	// So this is not an oversight to be tidied up into "keep CDC 1's input".
	const bool running = MC6809.run_state >= RS_RUNNING;
	uart_port_pump(fast_serial, 0, port_state[0], suppress_cdc_read || !running, usb_ports);
	uart_port_pump(aux_serial, 1, port_state[1], !running, usb_ports);
	usb_cdc_flush_n(0);
	usb_cdc_flush_n(1);
	if (!running)
		for (uart_port_state &st : port_state)
			st.rx_count = st.rx_index = 0u;
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
