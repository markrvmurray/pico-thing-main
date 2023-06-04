/**
* Copyright (c) 2025 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef PICO_EXAMPLES_PIO_MC6809_TASK_H
#define PICO_EXAMPLES_PIO_MC6809_TASK_H

struct processor;

void task_initialise(volatile struct processor *guest);
uint8_t task_change(volatile struct processor *guest, uint8_t new_task);

#endif // PICO_EXAMPLES_PIO_MC6809_TASK_H
