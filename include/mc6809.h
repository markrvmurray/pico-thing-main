/**
* Copyright (c) 2025 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <atomic>

// Guest E/Q clocks in MHz
#define GUEST_CLK_DEFAULT 1.0f
#define GUEST_CLK_MIN 0.8f
#define GUEST_CLK_MAX 3.0f

// RESET is active-low.
#define RESET_HOLD_US 500u
#define RESET_ASSERT do { gpio_put(GPIO_RESET, false); sleep_us(RESET_HOLD_US); } while (0)
#define RESET_DEASSERT do { sleep_us(RESET_HOLD_US); gpio_put(GPIO_RESET, true); } while (0)
#define RESET_IS_ASSERTED (!gpio_get(GPIO_RESET))

// HALT is active-low.
#define HALT_ASSERT gpio_put(GPIO_HALT, false);
#define HALT_DEASSERT gpio_put(GPIO_HALT, true);
#define HALT_IS_ASSERTED (!gpio_get(GPIO_HALT))

// POWER is active-high.
#define POWER_ASSERT gpio_put(GPIO_POWER, true);
#define POWER_DEASSERT gpio_put(GPIO_POWER, false);
#define POWER_IS_ASSERTED (gpio_get(GPIO_POWER))

// Interrupts are simulated open-collector. Becoming an output causes
// a drive-low, becoming an input sets high impedance.
#define ASSERT_NMI gpio_set_dir(GPIO_NMI, GPIO_OUT)
#define DEASSERT_NMI gpio_set_dir(GPIO_NMI, GPIO_IN)
#define NMI_IS_ASSERTED ((gpio_get_dir(GPIO_NMI) == GPIO_OUT) && !gpio_get(GPIO_NMI))
#define ASSERT_FIRQ gpio_set_dir(GPIO_FIRQ, GPIO_OUT)
#define DEASSERT_FIRQ gpio_set_dir(GPIO_FIRQ, GPIO_IN)
#define FIRQ_IS_ASSERTED ((gpio_get_dir(GPIO_FIRQ) == GPIO_OUT) && !gpio_get(GPIO_FIRQ))
#define ASSERT_IRQ gpio_set_dir(GPIO_IRQ, GPIO_OUT)
#define DEASSERT_IRQ gpio_set_dir(GPIO_IRQ, GPIO_IN)
#define IRQ_IS_ASSERTED ((gpio_get_dir(GPIO_IRQ) == GPIO_OUT) && !gpio_get(GPIO_IRQ))

#include "interrupt.h"

#define stringify(a) x_stringify(a)
#define x_stringify(a) #a

#define enum_list(e) e,
#define enum_list_strings(s) #s,

// Pure 2-bit pin observation of BA/BS
#define foreach_busstate(busstate) \
busstate(BUS_RUNNING)      \
busstate(BUS_IRQ)          \
busstate(BUS_SYNC)         \
busstate(BUS_HALT)

enum bus_state : uint8_t {
	foreach_busstate(enum_list)
	BUS_STATE_COUNT
};

// System-level state machine with memory
#define foreach_runstate(runstate) \
runstate(RS_UNPOWERED) \
runstate(RS_RESET)     \
runstate(RS_RUNNING)   \
runstate(RS_IN_IRQ)    \
runstate(RS_SYNCED)    \
runstate(RS_HALTED)

enum run_state : uint8_t {
	foreach_runstate(enum_list)
	RUN_STATE_COUNT
};

extern const char *bus_state_string[];
extern const char *run_state_string[];

union BLA {
	struct {
		uint8_t BUSY: 1;
		uint8_t LIC: 1;
		uint8_t AVMA: 1;
		uint8_t : 5;
	} bit;
	uint8_t byte;
};

// Bus trace: the raw GPIO state captured per E cycle at E-falling — the low
// word (GPIO 0-31) and the high word (GPIO 32-47) together, so the DAT fault
// signals, the task pins and A13-A15 are visible alongside the bus.  One-shot
// fill of trace_size cycles; decoded on demand via bus_pins and high_bus_pins.
#define TRACE_SIZE 4096u

class mc6809 {
private:
	volatile enum bus_state old_bus_state;
	volatile enum bus_state bus_state;
	volatile enum run_state run_state;
	volatile BLA busy_lic_avma{};
	static constexpr uint8_t MAX_INT_NEST_DEPTH = 16u;
	volatile uint8_t task;
	volatile uint8_t nesting_depth = 0;
	// Where the depth count goes wrong, counted rather than hidden.  Neither
	// was visible before: a push dropped at MAX_INT_NEST_DEPTH used to skip
	// the depth increment as well, so the count silently stopped rising
	// while every RTI kept lowering it, and an RTI with no matching push did
	// nothing at all.
	volatile uint32_t depth_overflows = 0;	// task history full, push dropped
	volatile uint32_t depth_underflows = 0;	// RTI with the depth already 0
	volatile uint32_t depth_declares = 0;	// times the OS resynchronised us
	volatile uint8_t rti_task_countdown = 0;	// E-cycle countdown for deferred task restore
	uint8_t rti_pending_task = 0;			// task to restore when countdown expires
	void rti_task_expired();			// the countdown reached zero: put the task back
	volatile uint8_t task_stack[MAX_INT_NEST_DEPTH]{};
	std::atomic<uint16_t> task_stack_ptr{0u};
	volatile bool busy, lic, vma;
	using task_pin_cb_t = void(*)(uint8_t);
	task_pin_cb_t task_pin_callback = nullptr;
#ifdef DEBUG
	volatile uint32_t _count_lic;
	volatile uint32_t _count_rti;
#endif
	// Interrupt acknowledge (vector fetch) counts.  Not DEBUG-only: the
	// interrupt tests hold each request asserted until its count moves, so
	// this has to be maintained in every build.
	volatile uint32_t _count_irq_ack[NUM_INTERRUPTS];
	// Bus trace.  Filled by DMA from two PIO state machines, not by the CPU —
	// see the bus_capture programs.  This class owns the buffers and the
	// trigger mask; the PIO and DMA plumbing lives in pico_thing.cpp and is
	// reached through the callbacks below, so this header stays free of SDK
	// dependencies for the host-side unit tests.
	static constexpr unsigned trace_size = TRACE_SIZE;
	volatile uint32_t trace[trace_size];
	// GPIO 32-47 only, so 16 bits is the whole of the high bank on a PGA2350.
	volatile uint16_t trace_hi[trace_size];
	// The BUSY/LIC/AVMA the Q-rising handler actually USED for each cycle,
	// taken from the DMA mirror.  The two buffers above are the pins as a PIO
	// state machine saw them; this one is what reached the software, so a
	// disagreement between them is a fault in the mirror's DMA path rather
	// than in the sampling.  Written by the CPU, not by DMA.
	volatile uint8_t trace_bla[trace_size];
	// One past the highest row stored, for the dump's footer.
	volatile uint32_t trace_bla_index = 0u;
	volatile bool trace_armed = false;	// capture requested (may still be waiting on a trigger)
	uint8_t trace_irq_trigger = 0u;		// bitmask of (1<<interrupt) vectors that arm capture
	using trace_cb_t = void (*)();
	using trace_count_cb_t = uint32_t (*)();
	trace_cb_t trace_arm_callback = nullptr;
	trace_cb_t trace_stop_callback = nullptr;
	trace_count_cb_t trace_count_callback = nullptr;
	float e_freq;
	using setup_callback_t = void(*)();
	setup_callback_t setup_callback = nullptr;
	void task_initialise();
	void setup(enum run_state rs);
	// This is a singleton class
	mc6809(); // no public constructor
	~mc6809() = default; // no public destructor
public:
	// The one instance, defined in mc6809.cpp and reached through the MC6809
	// macro below.  It is a static data member rather than a function-local
	// static inside getInstance() because a function-local static with a
	// non-trivial constructor carries a thread-safe-initialisation guard, and
	// that guard is consulted at EVERY use site: load the guard byte, test
	// it, branch past the initialiser, then do the work.  There were 26 such
	// sequences in the Q-rising handler alone, which runs two million times a
	// second at 1 MHz E.  A class member is initialised once before main and
	// has no guard, and its address is a link-time constant.
	//
	// It cannot be constant-initialised - the constructor assigns to volatile
	// members, which constant evaluation does not allow - so it is still
	// dynamically initialised at startup.  Nothing else's constructor may
	// touch it: no translation unit in this tree does.
	//
	// A static member can use the private constructor, so there is still no
	// way to make a second one.
	static mc6809 instance;
	// Kept for any caller that wants the old spelling; no guard, no local
	// static, and it folds away to the address of the member above.
	static mc6809& getInstance() { return instance; }
	mc6809(const mc6809&) = delete;
	mc6809& operator=(const mc6809&) = delete;
	void set_busy_lic_avma(uint8_t value)
	{
		busy_lic_avma.byte = value;
		busy = busy_lic_avma.bit.BUSY;
		lic = busy_lic_avma.bit.LIC;
		vma = busy_lic_avma.bit.AVMA;
	}
	[[nodiscard]] uint8_t get_busy_lic_avma() const { return busy_lic_avma.byte; }
	[[nodiscard]] bool get_busy() const { return busy; }
	[[nodiscard]] bool get_lic() const { return lic; }
	[[nodiscard]] bool get_vma() const { return vma; }
	// --- Bus trace control (one-shot capture of one GPIO word per E cycle) ---
	void set_trace_callbacks(trace_cb_t arm, trace_cb_t stop, trace_count_cb_t count)
		{ trace_arm_callback = arm; trace_stop_callback = stop; trace_count_callback = count; }
	void trace_arm() { trace_bla_index = 0u; trace_armed = true; if (trace_arm_callback) trace_arm_callback(); }
	[[nodiscard]] bool trace_is_armed() const { return trace_armed; }
	void trace_stop() { trace_armed = false; if (trace_stop_callback) trace_stop_callback(); }
	void trace_clear() { trace_stop(); trace_irq_trigger = 0u; }
	void trace_set_irq_trigger(interrupt i) { trace_irq_trigger |= static_cast<uint8_t>(1u << i); }
	[[nodiscard]] uint32_t trace_count() const
		{ return trace_count_callback ? trace_count_callback() : 0u; }
	[[nodiscard]] bool trace_is_full() const { return trace_count() >= trace_size; }
	// Armed and still filling.  A hardware-triggered capture reads as active
	// with a count of zero while it waits for its trigger.
	[[nodiscard]] bool trace_is_active() const { return trace_armed && !trace_is_full(); }
	// Buffer bases, for the DMA write addresses.
	[[nodiscard]] volatile uint32_t *trace_base() { return trace; }
	[[nodiscard]] volatile uint16_t *trace_hi_base() { return trace_hi; }
	[[nodiscard]] uint32_t trace_word(uint32_t i) const { return trace[i]; }
	[[nodiscard]] uint16_t trace_hi_word(uint32_t i) const { return trace_hi[i]; }
	[[nodiscard]] uint8_t trace_bla_byte(uint32_t i) const { return trace_bla[i]; }
	[[nodiscard]] uint32_t trace_bla_count() const { return trace_bla_index; }
	// Store this cycle's mirror value at the row the capture DMA is about to
	// fill, so the two columns line up whatever happened before the guest
	// started.  An index of its own does NOT work: the E/Q clock runs while
	// the guest is in reset, so a counter rewound at trace_arm() has already
	// saturated by the time the guest executes anything, and the column then
	// shows thousands of static reset-state samples against unrelated rows.
	void trace_bla_store(uint32_t i, uint8_t bla)
	{
		if (i < trace_size) {
			trace_bla[i] = bla;
			if (i + 1u > trace_bla_index)
				trace_bla_index = i + 1u;
		}
	}
	void trace_irq_event(interrupt i)
	{
		if ((trace_irq_trigger & (1u << i)) && !trace_armed)
			trace_arm();
	}
#ifdef DEBUG
	void clear_lic_count() { _count_lic = 0u; }
	[[nodiscard]] uint32_t get_lic_count() const { return _count_lic; }
	void count_lic() { if (lic && bus_state == BUS_RUNNING) _count_lic++; }
	void clear_rti_count() { _count_rti = 0u; }
	[[nodiscard]] uint32_t get_rti_count() const { return _count_rti; }
	void count_rti() { _count_rti++; }
#endif
	void clear_irq_ack_count() { for (auto &c : _count_irq_ack) c = 0u; }
	[[nodiscard]] uint32_t get_irq_ack_count(interrupt i) const { return _count_irq_ack[i]; }
	[[nodiscard]] uint32_t get_irq_ack_total() const { uint32_t t = 0; for (int i = 0; i < NUM_INTERRUPTS; i++) if (i != INTERRUPT_RESET) t += _count_irq_ack[i]; return t; }
	void count_irq_ack(interrupt i) { _count_irq_ack[i]++; }
	void init();
	void deinit();
	void set_setup_callback(setup_callback_t cb) { setup_callback = cb; }
	// Called from the DMA write ISR on every guest write to $FFC0, so both
	// of these are here rather than out of line: task_get() compiled to four
	// bytes reached by a call, and task_change() to ten.
	uint8_t task_change(uint8_t new_task)
	{
		new_task &= TASK_PINS_MASK;
		task = new_task;
		return new_task;
	}
	void set_task_pin_callback(task_pin_cb_t cb) { task_pin_callback = cb; }
	void apply_rti();
	// Called once per Q-rising from the ISR - two million times a second at
	// 1 MHz E - so the idle test is inline here and only the expiry, which
	// happens once per RTI at most, is a call.  With
	// PICO_THING_RTI_TASK_RESTORE off the countdown is never armed at all,
	// and this is one load and one branch.
	void tick_rti_countdown()
	{
		if (rti_task_countdown == 0u)
			return;
		if (--rti_task_countdown == 0u)
			rti_task_expired();
	}

	[[nodiscard]] bool assert_powered() const;
	[[nodiscard]] bool assert_stopped() const;
	[[nodiscard]] uint8_t task_get() const { return task; }
	void start();
	[[nodiscard]] bool start_with_timeout(uint32_t timeout_ms, bool sync_means_stop = true);
	static void stop();
	static void halt();
	static void release();
	static void uart_task(bool suppress_cdc_read = false);
	void preserve_state();
	void restore_state();
	void set_e_frequency(float target_mhz, const pio_dma &pio_clock);
	[[nodiscard]] bool is_running() const { return run_state == RS_RUNNING || run_state == RS_IN_IRQ; }
	[[nodiscard]] bool is_in_irq() const { return run_state == RS_IN_IRQ; }
	[[nodiscard]] bool is_synced() const { return run_state == RS_SYNCED; }
	[[nodiscard]] bool is_halted() const { return run_state == RS_HALTED; }
	[[nodiscard]] bool is_stopped() const { return run_state == RS_RESET; }
	[[nodiscard]] bool is_unpowered() const { return run_state == RS_UNPOWERED; }
	[[nodiscard]] float get_e_frequency() const;
	[[nodiscard]] enum run_state get_run_state() const { return run_state; }
	[[nodiscard]] enum bus_state get_bus_state() const { return bus_state; }
	[[nodiscard]] enum bus_state get_old_bus_state() const { return old_bus_state; }
	[[nodiscard]] uint8_t get_task() const { return task; }
	[[nodiscard]] uint8_t get_nesting_depth() const { return nesting_depth; }
	struct depth_errors { uint32_t overflows, underflows, declares; };
	[[nodiscard]] depth_errors get_depth_errors() const
		{ return { depth_overflows, depth_underflows, depth_declares }; }
	// The OS declaring where it really is.  Trims the task history to match:
	// entries above the declared depth describe interrupts that, as far as
	// the OS is concerned, are over.
	void declare_depth(uint8_t depth)
	{
		nesting_depth = depth;
		task_stack_ptr = (depth < MAX_INT_NEST_DEPTH) ? depth : MAX_INT_NEST_DEPTH;
		depth_declares++;
		if (depth == 0u && rti_task_countdown == 0u)
			run_state = RS_RUNNING;
	}
	[[nodiscard]] uint16_t get_task_stack_ptr() const { return task_stack_ptr; }
	[[nodiscard]] uint8_t get_task_history(uint16_t i) const { return (i < MAX_INT_NEST_DEPTH) ? task_stack[i] : 0u; }
	friend void gpio_q_irq_handler();
	friend void dma_bus_read_irq_handler();
	friend void dma_bus_write_irq_handler();
	friend void process_command(char *buf);
};

// Tidy up the singleton access semantics
// auto MC6809() { return &processor::getInstance(); }
// processor *MC6809 = processor::getInstance();
#define MC6809 mc6809::instance
