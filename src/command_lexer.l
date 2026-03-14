/**
 * Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Flex lexer for the pico-mc6809 interactive command parser.
 *
 * Design notes:
 *   - All symbols prefixed with cmd_yy to avoid collisions.
 *   - Command keywords are matched case-insensitively via match_command_keyword().
 *     Each command accepts exactly its full name or a fixed 2-letter mnemonic
 *     (e.g. "examine" or "ex").  Arbitrary prefix matching is intentionally
 *     not supported: it caused hex addresses (e.g. "e", "f") to collide with
 *     command prefixes when lexed as TOK_NUMBER.
 *   - Subcommand keywords (dat, all, halt, restart) and loop types (r/R/w/W/rw)
 *     are matched exactly (case-sensitive for loop types).
 *   - Numbers: any run of hex digits is TOK_NUMBER (action decides base).
 *     Floats (digits + '.') are TOK_FLOAT.
 *   - S-records matched before the general hex-digit rule.
 *   - USB arguments captured as a single string via the USB_MODE start condition.
 */

%{
#include <stdio.h>
#include <string.h>
#include <strings.h>  /* strncasecmp */

/* Include the bison-generated token definitions and YYSTYPE.
 * Named command_parser.tab.h to avoid collision with our command_parser.h. */
#include "command_parser.tab.h"

/* -----------------------------------------------------------------------
 * String pool for TOK_NUMBER values.
 *
 * flex's yytext is only valid during a lex action: on the next yylex()
 * call flex restores the "hold character" at yytext[yyleng], making any
 * saved yytext pointer point into the middle of the remaining input.
 * Parser actions that reference $N values from earlier shifts (e.g. $2
 * in "TOK_EXAMINE TOK_NUMBER TOK_NUMBER { use $2; }") would therefore
 * read stale data.
 *
 * Fix: copy each matched number into a ring-buffer slot that remains
 * valid for the duration of the parse.  The pool is reset by
 * cmd_yy_reset_str_pool() which must be called once per cmd_parse().
 *
 * Sizing: modify accepts up to 64 bytes plus an address = 65 tokens;
 * 96 slots with 12 bytes each (max 8 hex digits + NUL + spare) covers
 * all commands with margin.
 * ----------------------------------------------------------------------- */
#define CMD_STR_SLOTS 96
#define CMD_STR_WIDTH 12

static char     _str_pool[CMD_STR_SLOTS][CMD_STR_WIDTH];
static unsigned _str_head;

/* Forward declaration — defined after the second %% where BEGIN() is valid. */
void cmd_yy_reset_str_pool(void);

static char *
pool_tok(void)
{
	char   *d = _str_pool[_str_head++ % CMD_STR_SLOTS];
	size_t  n = (size_t)cmd_yyleng;
	if (n >= CMD_STR_WIDTH)
		n = CMD_STR_WIDTH - 1;
	memcpy(d, cmd_yytext, n);
	d[n] = '\0';
	return d;
}

/* -----------------------------------------------------------------------
 * match_command_keyword()
 *
 * Returns the bison token for the given word if it exactly matches either
 * the full command name or its fixed 2-letter mnemonic (case-insensitive).
 * Returns 0 if not recognised.
 *
 * Mnemonics are chosen so that no mnemonic consists entirely of hex digits
 * (a-f), which would otherwise be lexed as TOK_NUMBER before the word rule
 * has a chance to apply.
 * ----------------------------------------------------------------------- */
static int
match_command_keyword(const char *text)
{
	static const struct { const char *name; const char *alias; int tok; } kw[] = {
		{ "power",   "pw", TOK_POWER   },
		{ "verbose", "vb", TOK_VERBOSE },
		{ "examine", "ex", TOK_EXAMINE },
		{ "modify",  "mo", TOK_MODIFY  },
		{ "status",  "st", TOK_STATUS  },
		{ "snippet", "sn", TOK_SNIPPET },
		{ "loop",    "lo", TOK_LOOP    },
		{ "vector",  "vc", TOK_VECTOR  },
		{ "fill",    "fi", TOK_FILL    },
		{ "time",    "ti", TOK_TIME    },
		{ "go",      NULL, TOK_GO      },
		{ "task",    "ta", TOK_TASK    },
		{ "irq",     "ir", TOK_IRQ     },
		{ "trace",   "tr", TOK_TRACE   },
		{ "run",     "ru", TOK_RUN     },
		{ "stop",    "sp", TOK_STOP    },
		{ "freq",    "fq", TOK_FREQ    },
		{ "reset",   "rs", TOK_RESET   },
		{ "break",   "br", TOK_BREAK   },
		{ "usb",     "ub", TOK_USB     },
		{ "nitros9", "ni", TOK_NITROS9 },
		{ NULL, NULL, 0 }
	};

	for (int i = 0; kw[i].name != NULL; i++) {
		if (strcasecmp(text, kw[i].name) == 0)
			return kw[i].tok;
		if (kw[i].alias && strcasecmp(text, kw[i].alias) == 0)
			return kw[i].tok;
	}
	return 0; /* not recognised */
}
%}

%option prefix="cmd_yy"
%option noyywrap noinput nounput
%option never-interactive

/* Start condition for capturing USB arguments verbatim */
%x USB_MODE

%%

	/* ---- Whitespace: skip ---- */
[ \t\r\n]+          { /* skip */ }

	/* ---- Subcommand keywords (exact, case-insensitive) ---- */
[Dd][Aa][Tt]                        { return TOK_DAT; }
[Aa][Ll][Ll]                        { return TOK_ALL; }
[Hh][Aa][Ll][Tt]                    { return TOK_HALT; }
[Rr][Ee][Ss][Tt][Aa][Rr][Tt]        { return TOK_RESTART; }
[Rr][Ee][Tt][Uu][Rr][Nn]            { return TOK_RETURN; }
[Ii][Dd][Ee]                        { return TOK_IDE; }
[Dd][Rr][Ii][Vv][Ee][Ww][Ii][Rr][Ee] { return TOK_DRIVEWIRE; }

	/* ---- Loop types (exact, CASE-SENSITIVE per original behaviour) ---- */
"rw"|"RW"                           { return TOK_RW; }
"r"                                 { return TOK_r; }
"R"                                 { return TOK_R; }
"w"                                 { return TOK_w; }
"W"                                 { return TOK_W; }

	/* ---- Bang ---- */
"!"                                 { return TOK_BANG; }

	/* ---- S-records: must appear before the hex-digit rule ---- */
S[0159][0-9A-Fa-f]+                 { cmd_yylval.str = cmd_yytext; return TOK_SRECORD; }

	/* ---- Floating-point numbers ---- */
[0-9]+\.[0-9]*                      { cmd_yylval.fval = strtof(cmd_yytext, NULL); return TOK_FLOAT; }
\.[0-9]+                            { cmd_yylval.fval = strtof(cmd_yytext, NULL); return TOK_FLOAT; }

	/* ---- Hex/decimal numbers (including pure decimal) ---- */
	/* pool_tok() copies yytext so the string survives subsequent yylex() calls */
[0-9A-Fa-f]+                        { cmd_yylval.str = pool_tok(); return TOK_NUMBER; }

	/* ---- General alphabetic words: subcommands, keywords, unknowns ---- */
[A-Za-z][A-Za-z0-9_]*  {
	int tok = match_command_keyword(cmd_yytext);
	if (tok == TOK_USB) {
		/* Switch to USB_MODE to capture the rest of the line */
		BEGIN(USB_MODE);
		return TOK_USB;
	}
	if (tok != 0)
		return tok;
	/* Unknown word */
	cmd_yylval.str = cmd_yytext;
	return TOK_WORD;
}

	/* ---- USB_MODE: capture everything remaining on this line ---- */
	/*
	 * Leading whitespace is consumed by [ \t]+ first.  The content rule
	 * requires the first character to be non-whitespace so that flex's
	 * longest-match rule cannot skip the whitespace rule by matching a
	 * longer string that starts with spaces.
	 */
<USB_MODE>[ \t]+                { /* skip leading whitespace */ }
<USB_MODE>[^ \t\r\n][^\r\n]*   {
	cmd_yylval.str = cmd_yytext;
	BEGIN(INITIAL);
	return TOK_USB_ARGS;
}
<USB_MODE>[\r\n]                { BEGIN(INITIAL); }

	/* ---- Anything else: produce a word token ---- */
[^ \t\r\n]+                 { cmd_yylval.str = cmd_yytext; return TOK_WORD; }

%%

/* Reset the string pool AND the flex start condition.
 * Must live after the second %% so that BEGIN() is in scope. */
void
cmd_yy_reset_str_pool(void)
{
	_str_head = 0;
	BEGIN(INITIAL);
}
