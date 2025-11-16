/**
* Copyright (c) 2025 Mark R V Murray.
	*
	* SPDX-License-Identifier: BSD-2-Clause
	*/

#include "hardware/gpio.h"
#include "task.h"
#include "mc6809.h"

// Done once at start of run
void
task_initialise(volatile struct processor *guest, volatile uint8_t read_registers[])
{
	guest->task_stack_ptr = 0x00u;
	guest->task = 0u;
	read_registers[SYSTEM_TASK] = 0u;
	init_pins_range(GPIO_TASK_BASE, NUM_TASK_PINS);
	gpio_set_dir_out_masked64(TASK_PINS_MASK << GPIO_TASK_BASE);
	gpio_init(GPIO_TASK_0);
	gpio_set_dir(GPIO_TASK_0, GPIO_OUT);
	gpio_put_masked64(TASK_PINS_MASK << GPIO_TASK_BASE, 0LLu);
	gpio_put(GPIO_TASK_0, true);
}

// Done when the task register is changed by the guest
uint8_t
task_change(volatile struct processor *guest, uint8_t new_task)
{
	new_task &= TASK_PINS_MASK;
	guest->task = new_task;
	gpio_put_masked64(TASK_PINS_MASK << GPIO_TASK_BASE, new_task << GPIO_TASK_BASE);
	gpio_put(GPIO_TASK_0, new_task == 0u);
	return new_task;
}
