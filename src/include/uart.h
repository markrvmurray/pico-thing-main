/**
* Copyright (c) 2025 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef PICO_EXAMPLES_PIO_MC6809_UART_H
#define PICO_EXAMPLES_PIO_MC6809_UART_H

struct emulated_uart;

struct emulated_uart *uart_initialise(volatile uint8_t *status);
void uart_task(struct emulated_uart *port, volatile uint8_t *config, volatile uint8_t *status);
uint uart_send_level(struct emulated_uart *port);
uint uart_recv_level(struct emulated_uart *port);
bool uart_host_send_avail(struct emulated_uart *port);
bool uart_host_receive_avail(struct emulated_uart *port);
uint8_t uart_host_receive(struct emulated_uart *port);
void uart_host_send(struct emulated_uart *port, const uint8_t ch);
uint8_t uart_guest_receive(struct emulated_uart *port, volatile uint8_t *status);
void uart_guest_send(struct emulated_uart *port, const uint8_t ch, volatile uint8_t *status);
uint8_t uart_guest_control(struct emulated_uart *port, uint8_t written, volatile uint8_t *status);
enum interrupt uart_has_interrupt(const struct emulated_uart *port);

#endif // PICO_EXAMPLES_PIO_MC6809_UART_H
