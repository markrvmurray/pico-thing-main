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
#define ASSERT_FIRQ gpio_set_dir(GPIO_FIRQ, GPIO_OUT)
#define DEASSERT_FIRQ gpio_set_dir(GPIO_FIRQ, GPIO_IN)
#define ASSERT_IRQ gpio_set_dir(GPIO_IRQ, GPIO_OUT)
#define DEASSERT_IRQ gpio_set_dir(GPIO_IRQ, GPIO_IN)

enum interrupt {
	INTERRUPT_ILLEGAL,
	INTERRUPT_SWI3,
	INTERRUPT_SWI2,
	INTERRUPT_FIRQ,
	INTERRUPT_IRQ,
	INTERRUPT_SWI,
	INTERRUPT_NMI,
	INTERRUPT_RESET,
	INTERRUPT_NONE,
	NUM_INTERRUPTS
};

#define stringify(a) x_stringify(a)
#define x_stringify(a) #a

#define enum_list(e) e,
#define enum_list_strings(s) #s,

#define foreach_busstate(busstate) \
busstate(BS_RUNNING)       \
busstate(BS_IRQ)           \
busstate(BS_SYNC)          \
busstate(BS_HALT)          \
busstate(BS_RUNNING_RESET) \
busstate(BS_IRQ_RESET)     \
busstate(BS_SYNC_RESET)    \
busstate(BS_HALT_RESET)

enum ba_bs {
	foreach_busstate(enum_list)
	BUS_STATE_COUNT
};

union ba_bs_u {
	uint8_t byte;
	struct {
		uint8_t BS: 1;
		uint8_t BA: 1;
		uint8_t RESET: 1;
		uint8_t : 5;
	} bit;
	enum ba_bs state;
};

#define foreach_runstate(runstate) \
runstate(RS_STARTED) \
runstate(RS_INTERRUPTED) \
runstate(RS_HALTED) \
runstate(RS_SYNCED) \
runstate(RS_STOPPED)

enum run_state {
	foreach_runstate(enum_list)
	RUN_STATE_COUNT
};

extern const char *ba_bs_string[];
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

#ifdef DEBUG
#define TRACE_SIZE 1024u
#define TRACE_READ 0x00000000u
#define TRACE_WRITE 0x10000000u
#endif

class mc6809 {
private:
	volatile ba_bs_u old_bus_state{};
	volatile ba_bs_u bus_state{};
	volatile enum run_state run_state;
	volatile BLA busy_lic_avma{};
	volatile uint8_t task;
	std::atomic<uint16_t> task_stack_ptr{0u};
	volatile bool busy, lic, vma;
#ifdef DEBUG
	static constexpr unsigned trace_size = TRACE_SIZE;
	volatile uint32_t _count_lic;
	volatile uint32_t _count_rti;
	volatile unsigned trace_pos = 0u;
	volatile uint32_t trace[trace_size][4u];
#endif
	float e_freq;
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
#ifdef DEBUG
	void clear_lic_count() { _count_lic = 0u; }
	[[nodiscard]] uint32_t get_lic_count() const { return _count_lic; }
	void count_lic() { if (lic && bus_state.state == BS_RUNNING) _count_lic++; }
	void clear_rti_count() { _count_rti = 0u; }
	[[nodiscard]] uint32_t get_rti_count() const { return _count_rti; }
	void count_rti() { _count_rti++; }
#endif
	void init();
	uint8_t task_change(uint8_t new_task);
	void apply_rti();

	[[nodiscard]] bool assert_stopped() const;
	[[nodiscard]] uint8_t task_get() const;
	void start();
	void start_with_timeout(uint32_t timeout_ms, bool sync_means_stop = true);
	static void stop();
	static void halt();
	static void release();
	static void apply_interrupts(uint32_t interrupt_refcount[NUM_INTERRUPTS]);
	static void uart_task();
	void preserve_state();
	void restore_state();
	void set_e_frequency(float target_mhz, const pio_dma &pio_clock);
	[[nodiscard]] bool is_started() const { return run_state == RS_STARTED; };
	[[nodiscard]] bool is_interrupted() const { return run_state == RS_INTERRUPTED; };
	[[nodiscard]] bool is_synced() const { return run_state == RS_SYNCED; };
	[[nodiscard]] bool is_halted() const { return run_state == RS_HALTED; };
	[[nodiscard]] bool is_stopped() const { return run_state == RS_STOPPED; }
	[[nodiscard]] float get_e_frequency() const;
	friend void gpio_clock_eq_irq_handler();
	friend void dma_bus_read_irq_handler();
	friend void dma_bus_write_irq_handler();
	friend void process_command(char *buf);
};

// Tidy up the singleton access semantics
// auto MC6809() { return &processor::getInstance(); }
// processor *MC6809 = processor::getInstance();
#define MC6809 mc6809::getInstance()
