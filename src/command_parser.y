/**
 * Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Bison LALR(1) grammar for the pico-thing-main interactive command parser.
 *
 * Design notes:
 *   - All symbols prefixed with cmd_yy via %define api.prefix.
 *   - Result is passed in via %parse-param so the grammar is reentrant
 *     with respect to the result struct (lexer is still global/stateful).
 *   - Number arguments are carried as raw strings (TOK_NUMBER) and
 *     converted in actions via parse_hex() / parse_dec() / parse_float().
 *   - Static accumulator arrays (_bytes, _vecs, _time_*) are used for
 *     variadic argument rules; they are reset at the start of each rule
 *     that uses them.
 *   - Range/validity checks are performed in actions; on failure, the
 *     action calls YYERROR (which triggers yyerror() and returns 1).
 */

/*
 * %code requires is placed into the generated header, ensuring that
 * parsed_cmd_t is visible to both the parser and the lexer (which
 * includes the generated header).
 */
%code requires {
#include "command_parser.h"
}

%{
/*
 * %{ %} is placed before #include "command_parser.tab.h" in the generated .c
 * file, so we explicitly include our project header here to make parsed_cmd_t
 * visible for the yyerror forward declaration below.
 */
#include "command_parser.h"
#include "interrupt.h"   /* enum interrupt — for the trace IRQ trigger */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>   /* LONG_MIN */
#include <stdint.h>

/* Forward declaration of lexer function */
extern int  cmd_yylex(void);
void        cmd_yyerror(parsed_cmd_t *result, const char *msg);

/* -----------------------------------------------------------------------
 * Number conversion helpers
 * ----------------------------------------------------------------------- */

/* Parse a hex string (no 0x prefix) into a uint32_t.
 * Returns UINT32_MAX on invalid input (non-hex character found). */
static uint32_t
parse_hex(const char *s)
{
	char *end;
	uint32_t v = (uint32_t)strtoul(s, &end, 16);
	if (*end != '\0')
		return UINT32_MAX;
	return v;
}

/* Parse a decimal string into a long. Returns LONG_MIN on error. */
static long
parse_dec(const char *s)
{
	char *end;
	long v = strtol(s, &end, 10);
	if (*end != '\0')
		return LONG_MIN;
	return v;
}

/* Map an interrupt mnemonic to its enum interrupt value, or -1 if unknown.
 * Only the non-keyword mnemonics arrive here as TOK_WORD; "irq" comes via
 * TOK_IRQ in the grammar. */
static int
parse_intname(const char *s)
{
	if (!strcasecmp(s, "swi"))  return INTERRUPT_SWI;
	if (!strcasecmp(s, "swi2")) return INTERRUPT_SWI2;
	if (!strcasecmp(s, "swi3")) return INTERRUPT_SWI3;
	if (!strcasecmp(s, "nmi"))  return INTERRUPT_NMI;
	if (!strcasecmp(s, "firq")) return INTERRUPT_FIRQ;
	return -1;
}

/* -----------------------------------------------------------------------
 * Static accumulators for variadic rules
 * ----------------------------------------------------------------------- */
static uint8_t  _bytes[64];
static int      _byte_count;

static uint16_t _vecs[8];
static int      _vec_count;

/* Time fields */
static int _t_year, _t_month, _t_day, _t_hour, _t_minute, _t_second;
static int _t_count;
%}

/* -----------------------------------------------------------------------
 * Bison directives
 * ----------------------------------------------------------------------- */
%define api.prefix {cmd_yy}

/* Pass the result struct pointer into yyparse() and yyerror() */
%parse-param { parsed_cmd_t *result }

/* Verbose error messages */
%define parse.error verbose

%union {
	char  *str;
	float  fval;
}

/* -----------------------------------------------------------------------
 * Token declarations
 * ----------------------------------------------------------------------- */

/* Command keywords */
%token TOK_POWER
%token TOK_VERBOSE
%token TOK_EXAMINE
%token TOK_MODIFY
%token TOK_STATUS
%token TOK_SNIPPET
%token TOK_LOOP
%token TOK_VECTOR
%token TOK_FILL
%token TOK_TIME
%token TOK_GO
%token TOK_TASK
%token TOK_IRQ
%token TOK_TRACE
%token TOK_RUN
%token TOK_STOP
%token TOK_FREQ
%token TOK_RESET
%token TOK_BREAK
%token TOK_USB
%token TOK_NITROS9

/* Subcommand keywords */
%token TOK_DAT
%token TOK_ALL
%token TOK_HALT
%token TOK_RESTART
%token TOK_RETURN
%token TOK_IDE
%token TOK_DRIVEWIRE
%token TOK_ON
%token TOK_OFF

/* Loop access types */
%token TOK_r
%token TOK_R
%token TOK_w
%token TOK_W
%token TOK_RW

/* Value tokens */
%token <str>  TOK_NUMBER
%token <str>  TOK_SRECORD
%token <str>  TOK_USB_ARGS
%token <str>  TOK_WORD
%token <fval> TOK_FLOAT

/* Control tokens */
%token TOK_BANG

%%

/* -----------------------------------------------------------------------
 * Top-level: a command line is a single command
 * ----------------------------------------------------------------------- */
start:
	command
  ;

command:
	  power_cmd
	| verbose_cmd
	| status_cmd
	| reset_cmd
	| examine_cmd
	| modify_cmd
	| snippet_cmd
	| loop_cmd
	| vector_cmd
	| fill_cmd
	| time_cmd
	| go_cmd
	| task_cmd
	| irq_cmd
	| trace_cmd
	| run_cmd
	| stop_cmd
	| freq_cmd
	| break_cmd
	| usb_cmd
	| srecord_cmd
	| nitros9_cmd
	| TOK_WORD        { printf("Unknown command \"%s\"\n", $1); YYERROR; }
	;

/* -----------------------------------------------------------------------
 * Simple no-argument commands
 * ----------------------------------------------------------------------- */
power_cmd:
	TOK_POWER          { result->tag = CMD_POWER; }
	| TOK_POWER TOK_ON  { result->tag = CMD_POWER_ON; }
	| TOK_POWER TOK_OFF { result->tag = CMD_POWER_OFF; }
	;

verbose_cmd:
	TOK_VERBOSE { result->tag = CMD_VERBOSE; }
	;

status_cmd:
	  TOK_STATUS {
		result->tag           = CMD_STATUS;
		result->status.period = 0;
	}
	| TOK_STATUS TOK_NUMBER {
		result->tag           = CMD_STATUS;
		result->status.period = (uint32_t)strtoul($2, NULL, 0);
	}
	;

reset_cmd:
	TOK_RESET   { result->tag = CMD_RESET; }
	;

/* -----------------------------------------------------------------------
 * examine { dat | <start> <end> }
 * ----------------------------------------------------------------------- */
examine_cmd:
	  TOK_EXAMINE TOK_DAT {
		result->tag = CMD_EXAMINE_DAT;
	}
	| TOK_EXAMINE TOK_NUMBER TOK_NUMBER {
		uint32_t s = parse_hex($2);
		uint32_t e = parse_hex($3);
		if (s == UINT32_MAX || s >= 0xFFC0u) {
			printf("Invalid hex start address '%s' (valid is 0 .. FFC0)\n", $2);
			YYERROR;
		}
		if (e == UINT32_MAX || e == 0 || e > 0xFFC0u || e <= s) {
			printf("Invalid hex end address '%s' (must be > start and <= FFC0)\n", $3);
			YYERROR;
		}
		result->tag = CMD_EXAMINE;
		result->examine.start = (uint16_t)s;
		result->examine.end   = (uint16_t)e;
	}
	;

/* -----------------------------------------------------------------------
 * modify <start> <byte> [<byte> ...]   (1..64 bytes)
 * ----------------------------------------------------------------------- */
modify_cmd:
	TOK_MODIFY TOK_NUMBER { _byte_count = 0; } byte_list_ne {
		uint32_t s = parse_hex($2);
		if (s == UINT32_MAX || s > 0xFFFEu) {
			printf("Invalid hex start address '%s'\n", $2);
			YYERROR;
		}
		if (_byte_count > 64) {
			printf("Too many bytes (maximum 64)\n");
			YYERROR;
		}
		result->tag          = CMD_MODIFY;
		result->modify.start = (uint16_t)s;
		result->modify.count = (uint8_t)_byte_count;
		memcpy(result->modify.data, _bytes, (size_t)_byte_count);
	}
	| TOK_MODIFY TOK_BANG TOK_NUMBER { _byte_count = 0; } byte_list_ne {
		uint32_t s = parse_hex($3);
		if (s == UINT32_MAX || s > 0xFFFEu) {
			printf("Invalid hex start address '%s'\n", $3);
			YYERROR;
		}
		if (_byte_count > 64) {
			printf("Too many bytes (maximum 64)\n");
			YYERROR;
		}
		result->tag          = CMD_MODIFY_READ;
		result->modify.start = (uint16_t)s;
		result->modify.count = (uint8_t)_byte_count;
		memcpy(result->modify.data, _bytes, (size_t)_byte_count);
	}
	;

/* -----------------------------------------------------------------------
 * snippet <byte> [<byte> ...]   (1..16 bytes)
 * ----------------------------------------------------------------------- */
snippet_cmd:
	TOK_SNIPPET { _byte_count = 0; } byte_list_ne {
		if (_byte_count > 16) {
			printf("Too many bytes (maximum 16 for snippet)\n");
			YYERROR;
		}
		result->tag           = CMD_SNIPPET;
		result->snippet.count = (uint8_t)_byte_count;
		memcpy(result->snippet.data, _bytes, (size_t)_byte_count);
	}
	;

/* -----------------------------------------------------------------------
 * Byte accumulator rules
 * byte_list_ne  – one or more bytes (resets accumulator on first byte)
 * ----------------------------------------------------------------------- */
byte_list_ne:
	  byte_item
	| byte_list_ne byte_item
	;

byte_item:
	TOK_NUMBER {
		uint32_t v = parse_hex($1);
		if (v == UINT32_MAX || v > 0xFFu) {
			printf("Invalid byte value '%s'\n", $1);
			YYERROR;
		}
		if (_byte_count == 0)
			memset(_bytes, 0, sizeof(_bytes));
		if (_byte_count < 64)
			_bytes[_byte_count] = (uint8_t)v;
		_byte_count++;
	}
	;

/* Reset the byte accumulator before the first byte.
 * Calling byte_list_ne directly resets inside byte_item for the first item;
 * we seed _byte_count = 0 at the start of each command that uses it. */

/* -----------------------------------------------------------------------
 * loop { r | R | w | W | rw } <address>
 * ----------------------------------------------------------------------- */
loop_cmd:
	TOK_LOOP loop_type TOK_NUMBER {
		uint32_t addr = parse_hex($3);
		if (addr == UINT32_MAX || addr > 0xFFFFu) {
			printf("Invalid address '%s'\n", $3);
			YYERROR;
		}
		result->tag          = CMD_LOOP;
		result->loop.address = (uint16_t)addr;
		/* opcode already set by loop_type rule via result->loop.opcode */
	}
	;

loop_type:
	  TOK_r  { result->loop.opcode = 0xB6u; } /* LDA >ADDRESS */
	| TOK_R  { result->loop.opcode = 0xFCu; } /* LDD >ADDRESS */
	| TOK_w  { result->loop.opcode = 0xB7u; } /* STA >ADDRESS */
	| TOK_W  { result->loop.opcode = 0xFDu; } /* STD >ADDRESS */
	| TOK_RW { result->loop.opcode = 0x70u; } /* NEG >ADDRESS */
	;

/* -----------------------------------------------------------------------
 * vector <vec0> <vec1> ... <vec7>   (exactly 8 16-bit addresses)
 * ----------------------------------------------------------------------- */
vector_cmd:
	TOK_VECTOR { _vec_count = 0; } vec8 {
		result->tag = CMD_VECTOR;
		memcpy(result->vector.vec, _vecs, sizeof(_vecs));
	}
	;

vec8:
	  vec_item vec_item vec_item vec_item
	  vec_item vec_item vec_item vec_item
	;

vec_item:
	TOK_NUMBER {
		uint32_t v = parse_hex($1);
		if (v == UINT32_MAX || v > 0xFFFFu) {
			printf("Invalid vector address '%s'\n", $1);
			YYERROR;
		}
		if (_vec_count == 0)
			memset(_vecs, 0, sizeof(_vecs));
		if (_vec_count < 8)
			_vecs[_vec_count] = (uint16_t)v;
		_vec_count++;
	}
	;

/* -----------------------------------------------------------------------
 * fill { all | <start> <end> } [ <value> ]
 * ----------------------------------------------------------------------- */
fill_cmd:
	  TOK_FILL TOK_ALL {
		result->tag        = CMD_FILL_ALL;
		result->fill.start = 0x0000u;
		result->fill.end   = 0xFE00u;
		result->fill.value = 0x00u;
	}
	| TOK_FILL TOK_ALL TOK_NUMBER {
		uint32_t v = parse_hex($3);
		if (v == UINT32_MAX || v > 0xFFu) {
			printf("Invalid fill byte '%s'\n", $3);
			YYERROR;
		}
		result->tag        = CMD_FILL_ALL_VALUE;
		result->fill.start = 0x0000u;
		result->fill.end   = 0xFE00u;
		result->fill.value = (uint8_t)v;
	}
	| TOK_FILL TOK_NUMBER TOK_NUMBER {
		uint32_t s = parse_hex($2);
		uint32_t e = parse_hex($3);
		if (s == UINT32_MAX || s >= 0xFFFFu) {
			printf("Invalid start address '%s'\n", $2);
			YYERROR;
		}
		if (e == UINT32_MAX || e > 0xFFFFu || e <= s) {
			printf("Invalid end address '%s'\n", $3);
			YYERROR;
		}
		result->tag        = CMD_FILL;
		result->fill.start = (uint16_t)s;
		result->fill.end   = (uint16_t)e;
		result->fill.value = 0x00u;
	}
	| TOK_FILL TOK_NUMBER TOK_NUMBER TOK_NUMBER {
		uint32_t s = parse_hex($2);
		uint32_t e = parse_hex($3);
		uint32_t v = parse_hex($4);
		if (s == UINT32_MAX || s >= 0xFFFFu) {
			printf("Invalid start address '%s'\n", $2);
			YYERROR;
		}
		if (e == UINT32_MAX || e > 0xFFFFu || e <= s) {
			printf("Invalid end address '%s'\n", $3);
			YYERROR;
		}
		if (v == UINT32_MAX || v > 0xFFu) {
			printf("Invalid fill byte '%s'\n", $4);
			YYERROR;
		}
		result->tag        = CMD_FILL_VALUE;
		result->fill.start = (uint16_t)s;
		result->fill.end   = (uint16_t)e;
		result->fill.value = (uint8_t)v;
	}
	;

/* -----------------------------------------------------------------------
 * time [ <Year> [ <Mon> [ <Day> [ <Hour> [ <Min> [ <Sec> ] ] ] ] ] ]
 * All arguments are decimal.
 * ----------------------------------------------------------------------- */
time_cmd:
	  TOK_TIME {
		result->tag = CMD_TIME_QUERY;
	}
	| TOK_TIME time_args {
		result->tag         = CMD_TIME_SET;
		result->time.year   = _t_year;
		result->time.month  = _t_month;
		result->time.day    = _t_day;
		result->time.hour   = _t_hour;
		result->time.minute = _t_minute;
		result->time.second = _t_second;
		result->time.count  = _t_count;
	}
	;

time_args:
	  time_year
	| time_year time_month
	| time_year time_month time_day
	| time_year time_month time_day time_hour
	| time_year time_month time_day time_hour time_minute
	| time_year time_month time_day time_hour time_minute time_second
	;

time_year:
	TOK_NUMBER {
		long v = parse_dec($1);
		if (v == LONG_MIN || v < 2025 || v > 2061) {
			printf("Invalid year '%s' (valid: 2025-2061)\n", $1);
			YYERROR;
		}
		_t_year = (int)v; _t_month = 0; _t_day = 1;
		_t_hour = 0; _t_minute = 0; _t_second = 0; _t_count = 1;
	}
	;

time_month:
	TOK_NUMBER {
		long v = parse_dec($1);
		if (v == LONG_MIN || v < 1 || v > 12) {
			printf("Invalid month '%s' (valid: 1-12)\n", $1);
			YYERROR;
		}
		_t_month = (int)(v - 1); /* convert to 0-based for tm_mon */
		_t_count = 2;
	}
	;

time_day:
	TOK_NUMBER {
		long v = parse_dec($1);
		if (v == LONG_MIN || v < 1 || v > 31) {
			printf("Invalid day '%s' (valid: 1-31)\n", $1);
			YYERROR;
		}
		_t_day = (int)v; _t_count = 3;
	}
	;

time_hour:
	TOK_NUMBER {
		long v = parse_dec($1);
		if (v == LONG_MIN || v < 0 || v > 23) {
			printf("Invalid hour '%s' (valid: 0-23)\n", $1);
			YYERROR;
		}
		_t_hour = (int)v; _t_count = 4;
	}
	;

time_minute:
	TOK_NUMBER {
		long v = parse_dec($1);
		if (v == LONG_MIN || v < 0 || v > 59) {
			printf("Invalid minute '%s' (valid: 0-59)\n", $1);
			YYERROR;
		}
		_t_minute = (int)v; _t_count = 5;
	}
	;

time_second:
	TOK_NUMBER {
		long v = parse_dec($1);
		if (v == LONG_MIN || v < 0 || v > 59) {
			printf("Invalid second '%s' (valid: 0-59)\n", $1);
			YYERROR;
		}
		_t_second = (int)v; _t_count = 6;
	}
	;

/* -----------------------------------------------------------------------
 * go [ <start> ]
 * ----------------------------------------------------------------------- */
go_cmd:
	  TOK_GO {
		result->tag = CMD_GO;
	}
	| TOK_GO TOK_NUMBER {
		uint32_t v = parse_hex($2);
		if (v == UINT32_MAX || v > 0xFFFFu) {
			printf("Invalid hex start address '%s'\n", $2);
			YYERROR;
		}
		result->tag        = CMD_GO_ADDR;
		result->go.address = (uint16_t)v;
	}
	;

/* -----------------------------------------------------------------------
 * task [ <tasknum> ]
 * ----------------------------------------------------------------------- */
task_cmd:
	  TOK_TASK {
		result->tag = CMD_TASK_QUERY;
	}
	| TOK_TASK TOK_NUMBER {
		uint32_t v = parse_hex($2);
		if (v == UINT32_MAX || v > 15u) {
			printf("Invalid task number '%s' (valid: 0-F)\n", $2);
			YYERROR;
		}
		result->tag          = CMD_TASK_SET;
		result->task.tasknum = (uint8_t)v;
	}
	;

/* -----------------------------------------------------------------------
 * irq <testnum>   (0-6)
 * ----------------------------------------------------------------------- */
irq_cmd:
	TOK_IRQ TOK_NUMBER {
		uint32_t v = parse_hex($2);
		if (v == UINT32_MAX || v > 6u) {
			printf("Invalid IRQ test ID '%s' (valid: 0-6)\n", $2);
			YYERROR;
		}
		result->tag         = CMD_IRQ;
		result->irq.testnum = (uint8_t)v;
	}
	;

/* -----------------------------------------------------------------------
 * trace [ <len> ]
 * ----------------------------------------------------------------------- */
trace_cmd:
	  TOK_TRACE {
		result->tag = CMD_TRACE;
	}
	| TOK_TRACE TOK_NUMBER {
		uint32_t v = parse_hex($2);
		if (v == UINT32_MAX) {
			printf("Invalid trace length '%s'\n", $2);
			YYERROR;
		}
		result->tag          = CMD_TRACE_LEN;
		result->trace.length = v;
	}
	| TOK_TRACE TOK_ON {
		result->tag = CMD_TRACE_ARM;
	}
	| TOK_TRACE TOK_OFF {
		result->tag = CMD_TRACE_OFF;
	}
	| TOK_TRACE TOK_IRQ {
		result->tag        = CMD_TRACE_IRQ;
		result->trace.intr = (uint8_t)INTERRUPT_IRQ;
	}
	| TOK_TRACE TOK_WORD {
		int code = parse_intname($2);
		if (code < 0) {
			printf("Unknown trace interrupt '%s' "
			       "(use swi, swi2, swi3, nmi, firq, irq)\n", $2);
			YYERROR;
		}
		result->tag        = CMD_TRACE_IRQ;
		result->trace.intr = (uint8_t)code;
	}
	;

/* -----------------------------------------------------------------------
 * run [ <index> [ ! ] ]
 * ----------------------------------------------------------------------- */
run_cmd:
	  TOK_RUN {
		result->tag = CMD_RUN_LIST;
	}
	| TOK_RUN TOK_NUMBER {
		uint32_t v = parse_hex($2);
		if (v == UINT32_MAX) {
			printf("Invalid snippet/chunk index '%s'\n", $2);
			YYERROR;
		}
		result->tag       = CMD_RUN;
		result->run.index = v;
	}
	| TOK_RUN TOK_NUMBER TOK_BANG {
		uint32_t v = parse_hex($2);
		if (v == UINT32_MAX) {
			printf("Invalid snippet/chunk index '%s'\n", $2);
			YYERROR;
		}
		result->tag       = CMD_RUN_NOWAIT;
		result->run.index = v;
	}
	;

/* -----------------------------------------------------------------------
 * stop [ halt | restart | stop ]
 * ----------------------------------------------------------------------- */
stop_cmd:
	  TOK_STOP            { result->tag = CMD_STOP; }
	| TOK_STOP TOK_HALT   { result->tag = CMD_STOP_HALT; }
	| TOK_STOP TOK_RESTART { result->tag = CMD_STOP_RESTART; }
	| TOK_STOP TOK_STOP   { result->tag = CMD_STOP_STOP; }
	;

/* -----------------------------------------------------------------------
 * freq <freqMHz>
 * ----------------------------------------------------------------------- */
freq_cmd:
	  TOK_FREQ TOK_FLOAT {
		if ($2 < 0.8f || $2 > 3.0f) {
			printf("Frequency %.2f out of range [0.8 .. 3.0]\n", (double)$2);
			YYERROR;
		}
		result->tag      = CMD_FREQ;
		result->freq.mhz = $2;
	}
	| TOK_FREQ TOK_NUMBER {
		/* Accept e.g. "freq 2" as 2.0 MHz */
		float f = strtof($2, NULL);
		if (f < 0.8f || f > 3.0f) {
			printf("Frequency '%s' out of range [0.8 .. 3.0]\n", $2);
			YYERROR;
		}
		result->tag      = CMD_FREQ;
		result->freq.mhz = f;
	}
	;

/* -----------------------------------------------------------------------
 * break [ <address> ]
 * ----------------------------------------------------------------------- */
break_cmd:
	  TOK_BREAK {
		result->tag = CMD_BREAK;
	}
	| TOK_BREAK TOK_NUMBER {
		uint32_t v = parse_hex($2);
		if (v == UINT32_MAX || v > 0xFFFFu) {
			printf("Invalid hex address '%s'\n", $2);
			YYERROR;
		}
		result->tag         = CMD_BREAK_ADDR;
		result->brk.address = (uint16_t)v;
	}
	| TOK_BREAK TOK_RETURN {
		result->tag = CMD_BREAK_RETURN;
	}
	| TOK_BREAK TOK_r {
		result->tag = CMD_BREAK_RETURN;
	}
	;

/* -----------------------------------------------------------------------
 * usb <args...>
 * ----------------------------------------------------------------------- */
usb_cmd:
	  TOK_USB TOK_USB_ARGS {
		result->tag = CMD_USB;
		strncpy(result->usb.args, $2, sizeof(result->usb.args) - 1u);
		result->usb.args[sizeof(result->usb.args) - 1u] = '\0';
	}
	| TOK_USB {
		printf("Usage:\n\n> usb <arg> [...]\n\n");
		YYERROR;
	}
	;

/* -----------------------------------------------------------------------
 * S-record line (e.g. S10E0130...)
 * ----------------------------------------------------------------------- */
srecord_cmd:
	TOK_SRECORD {
		result->tag = CMD_SRECORD;
		strncpy(result->srecord.line, $1, sizeof(result->srecord.line) - 1u);
		result->srecord.line[sizeof(result->srecord.line) - 1u] = '\0';
	}
	;

/* -----------------------------------------------------------------------
 * nitros9 [ ide | drivewire ]
 * ----------------------------------------------------------------------- */
nitros9_cmd:
	  TOK_NITROS9 {
		result->tag = CMD_NITROS9;
		result->nitros9.boot_device = 0; /* IDE default */
	}
	| TOK_NITROS9 TOK_IDE {
		result->tag = CMD_NITROS9;
		result->nitros9.boot_device = 0;
	}
	| TOK_NITROS9 TOK_DRIVEWIRE {
		result->tag = CMD_NITROS9;
		result->nitros9.boot_device = 1;
	}
	;

%%

/* -----------------------------------------------------------------------
 * yyerror
 * ----------------------------------------------------------------------- */
void
cmd_yyerror(parsed_cmd_t *r, const char *msg)
{
	printf("Parse error: %s\n", msg);
	if (r)
		r->tag = CMD_SYNTAX_ERROR;
}
