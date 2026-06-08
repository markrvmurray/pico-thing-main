/**
 * Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum cmd_tag {
	CMD_NONE = 0,
	CMD_SYNTAX_ERROR,

	CMD_POWER,
	CMD_POWER_ON,
	CMD_POWER_OFF,
	CMD_VERBOSE,
	CMD_STATUS,
	CMD_RESET,

	CMD_EXAMINE_DAT,
	CMD_EXAMINE,

	CMD_MODIFY,
	CMD_MODIFY_READ,
	CMD_SNIPPET,

	CMD_LOOP,

	CMD_VECTOR,

	CMD_FILL_ALL,
	CMD_FILL_ALL_VALUE,
	CMD_FILL,
	CMD_FILL_VALUE,

	CMD_TIME_QUERY,
	CMD_TIME_SET,

	CMD_GO,
	CMD_GO_ADDR,

	CMD_TASK_QUERY,
	CMD_TASK_SET,

	CMD_IRQ,

	CMD_TRACE,
	CMD_TRACE_LEN,
	CMD_TRACE_ARM,
	CMD_TRACE_OFF,
	CMD_TRACE_IRQ,

	CMD_RUN_LIST,
	CMD_RUN,
	CMD_RUN_NOWAIT,

	CMD_STOP,
	CMD_STOP_HALT,
	CMD_STOP_RESTART,
	CMD_STOP_STOP,

	CMD_FREQ,

	CMD_USB,

	CMD_BREAK,
	CMD_BREAK_ADDR,
	CMD_BREAK_RETURN,

	CMD_SRECORD,

	CMD_NITROS9,
} cmd_tag_t;

typedef struct parsed_cmd {
	cmd_tag_t tag;
	union {
		/* CMD_EXAMINE */
		struct {
			uint16_t start;
			uint16_t end;
		} examine;

		/* CMD_MODIFY */
		struct {
			uint16_t start;
			uint8_t  data[64];
			uint8_t  count;
		} modify;

		/* CMD_SNIPPET */
		struct {
			uint8_t data[16];
			uint8_t count;
		} snippet;

		/* CMD_LOOP */
		struct {
			uint8_t  opcode;
			uint16_t address;
		} loop;

		/* CMD_VECTOR */
		struct {
			uint16_t vec[8];
		} vector;

		/* CMD_FILL_ALL, CMD_FILL_ALL_VALUE, CMD_FILL, CMD_FILL_VALUE */
		struct {
			uint16_t start;
			uint16_t end;
			uint8_t  value;
		} fill;

		/* CMD_TIME_SET */
		struct {
			int year;
			int month;
			int day;
			int hour;
			int minute;
			int second;
			int count; /* number of fields provided (1-6) */
		} time;

		/* CMD_GO_ADDR */
		struct {
			uint16_t address;
		} go;

		/* CMD_TASK_SET */
		struct {
			uint8_t tasknum;
		} task;

		/* CMD_IRQ */
		struct {
			uint8_t testnum;
		} irq;

		/* CMD_TRACE_LEN / CMD_TRACE_IRQ */
		struct {
			uint32_t length;	/* CMD_TRACE_LEN: dump count */
			uint8_t  intr;		/* CMD_TRACE_IRQ: enum interrupt value */
		} trace;

		/* CMD_RUN, CMD_RUN_NOWAIT */
		struct {
			uint32_t index;
		} run;

		/* CMD_FREQ */
		struct {
			float mhz;
		} freq;

		/* CMD_USB */
		struct {
			char args[128]; /* space-separated remainder of line */
		} usb;

		/* CMD_BREAK_ADDR */
		struct {
			uint16_t address;
		} brk;

		/* CMD_SRECORD */
		struct {
			char line[256]; /* full S-record line */
		} srecord;

		/* CMD_NITROS9 */
		struct {
			uint8_t boot_device; /* 0 = IDE, 1 = DriveWire */
		} nitros9;

		/* CMD_STATUS */
		struct {
			uint32_t period; /* 0 = one-shot, >0 = repeat interval in seconds */
		} status;
	};
} parsed_cmd_t;

/*
 * Parse one command line into *out.
 * Returns false (and prints a message) on syntax error.
 * The input buffer may be modified by the lexer.
 */
bool cmd_parse(char *input, parsed_cmd_t *out);

#ifdef __cplusplus
}
#endif
