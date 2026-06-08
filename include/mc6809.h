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

// Bus trace: one raw gpio_get_all() word captured per E cycle at Q-rising.
// One-shot fill of trace_size words; decoded on demand via the bus_pins union.
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
	volatile uint8_t rti_task_countdown = 0;	// E-cycle countdown for deferred task restore
	uint8_t rti_pending_task = 0;			// task to restore when countdown expires
	volatile uint8_t task_stack[MAX_INT_NEST_DEPTH]{};
	std::atomic<uint16_t> task_stack_ptr{0u};
	volatile bool busy, lic, vma;
	using task_pin_cb_t = void(*)(uint8_t);
	task_pin_cb_t task_pin_callback = nullptr;
#ifdef DEBUG
	volatile uint32_t _count_lic;
	volatile uint32_t _count_rti;
	volatile uint32_t _count_irq_ack[NUM_INTERRUPTS];
#endif
	// Bus trace (always compiled; gated at runtime by trace_enabled).
	static constexpr unsigned trace_size = TRACE_SIZE;
	volatile uint32_t trace[trace_size];
	volatile uint32_t trace_pos = 0u;	// words captured so far (also the write index)
	volatile bool trace_enabled = false;	// capture in progress
	volatile bool trace_full = false;	// one-shot fill completed
	uint8_t trace_irq_trigger = 0u;		// bitmask of (1<<interrupt) vectors that arm capture
	float e_freq;
	using setup_callback_t = void(*)();
	setup_callback_t setup_callback = nullptr;
	void task_initialise();
	void setup(enum run_state rs);
	// This is a singleton class
	mc6809(); // no public constructor
	~mc6809() = default; // no public destructor
public:
	static mc6809& getInstance() {
		static mc6809 instance;
		return instance;
	}
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
	void trace_arm() { trace_pos = 0u; trace_full = false; trace_enabled = true; }
	void trace_stop() { trace_enabled = false; }
	void trace_clear() { trace_enabled = false; trace_full = false; trace_pos = 0u; trace_irq_trigger = 0u; }
	void trace_set_irq_trigger(interrupt i) { trace_irq_trigger |= static_cast<uint8_t>(1u << i); }
	[[nodiscard]] bool trace_is_full() const { return trace_full; }
	[[nodiscard]] bool trace_is_active() const { return trace_enabled; }
	[[nodiscard]] uint32_t trace_count() const { return trace_pos; }
	[[nodiscard]] uint32_t trace_word(uint32_t i) const { return trace[i]; }
	// Capture hook: called at E-falling from the EQ ISR, when address, data,
	// R/W and status are all settled. One-shot — stops when the buffer fills.
	void trace_capture(uint32_t gpio_word)
	{
		// Only capture cycles of a running CPU — skip the reset-held idle bus
		// so an armed one-shot isn't consumed before the guest actually runs.
		if (!trace_enabled || run_state < RS_RUNNING)
			return;
		trace[trace_pos++] = gpio_word;
		if (trace_pos >= trace_size) {
			trace_enabled = false;
			trace_full = true;
		}
	}
	// Named-interrupt trigger: called from the EQ ISR when a vector is taken.
	void trace_irq_event(interrupt i)
	{
		if ((trace_irq_trigger & (1u << i)) && !trace_enabled && !trace_full)
			trace_arm();
	}
#ifdef DEBUG
	void clear_lic_count() { _count_lic = 0u; }
	[[nodiscard]] uint32_t get_lic_count() const { return _count_lic; }
	void count_lic() { if (lic && bus_state == BUS_RUNNING) _count_lic++; }
	void clear_rti_count() { _count_rti = 0u; }
	[[nodiscard]] uint32_t get_rti_count() const { return _count_rti; }
	void count_rti() { _count_rti++; }
	void clear_irq_ack_count() { for (auto &c : _count_irq_ack) c = 0u; }
	[[nodiscard]] uint32_t get_irq_ack_count(interrupt i) const { return _count_irq_ack[i]; }
	[[nodiscard]] uint32_t get_irq_ack_total() const { uint32_t t = 0; for (int i = 0; i < NUM_INTERRUPTS; i++) if (i != INTERRUPT_RESET) t += _count_irq_ack[i]; return t; }
	void count_irq_ack(interrupt i) { _count_irq_ack[i]++; }
#endif
	void init();
	void deinit();
	void set_setup_callback(setup_callback_t cb) { setup_callback = cb; }
	uint8_t task_change(uint8_t new_task);
	void set_task_pin_callback(task_pin_cb_t cb) { task_pin_callback = cb; }
	void apply_rti();
	void tick_rti_countdown();	// called per Q-rising from ISR

	[[nodiscard]] bool assert_powered() const;
	[[nodiscard]] bool assert_stopped() const;
	[[nodiscard]] uint8_t task_get() const;
	void start();
	[[nodiscard]] bool start_with_timeout(uint32_t timeout_ms, bool sync_means_stop = true);
	static void stop();
	static void halt();
	static void release();
	static void apply_interrupts(uint32_t interrupt_refcount[NUM_INTERRUPTS]);
	static void uart_task(bool suppress_cdc_read = false);
	void preserve_state();
	void restore_state();
	void set_e_frequency(float target_mhz, const pio_dma &pio_clock);
	[[nodiscard]] bool is_running() const { return run_state == RS_RUNNING || run_state == RS_IN_IRQ; };
	[[nodiscard]] bool is_in_irq() const { return run_state == RS_IN_IRQ; };
	[[nodiscard]] bool is_synced() const { return run_state == RS_SYNCED; };
	[[nodiscard]] bool is_halted() const { return run_state == RS_HALTED; };
	[[nodiscard]] bool is_stopped() const { return run_state == RS_RESET; }
	[[nodiscard]] bool is_unpowered() const { return run_state == RS_UNPOWERED; }
	[[nodiscard]] float get_e_frequency() const;
	[[nodiscard]] enum run_state get_run_state() const { return run_state; }
	[[nodiscard]] enum bus_state get_bus_state() const { return bus_state; }
	[[nodiscard]] enum bus_state get_old_bus_state() const { return old_bus_state; }
	[[nodiscard]] uint8_t get_task() const { return task; }
	[[nodiscard]] uint8_t get_nesting_depth() const { return nesting_depth; }
	[[nodiscard]] uint16_t get_task_stack_ptr() const { return task_stack_ptr; }
	[[nodiscard]] uint8_t get_task_history(uint16_t i) const { return (i < MAX_INT_NEST_DEPTH) ? task_stack[i] : 0u; }
	friend void gpio_clock_eq_irq_handler();
	friend void dma_bus_read_irq_handler();
	friend void dma_bus_write_irq_handler();
	friend void process_command(char *buf);
};

// Tidy up the singleton access semantics
// auto MC6809() { return &processor::getInstance(); }
// processor *MC6809 = processor::getInstance();
#define MC6809 mc6809::getInstance()
