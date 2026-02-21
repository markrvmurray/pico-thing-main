/**
 * Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Flex lexer for the pico-mc6809 interactive command parser.
 *
 * Design notes:
 *   - All symbols prefixed with cmd_yy to avoid collisions.
 *   - Command keywords are matched case-insensitively with unambiguous-prefix
 *     support via match_command_keyword() (mirrors the original strncasecmp logic).
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

void
cmd_yy_reset_str_pool(void) { _str_head = 0; }

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
 * Returns the bison token for the given word if it is an unambiguous
 * case-insensitive prefix of a known command keyword.
 * Returns 0 if not found, TOK_AMBIGUOUS if ambiguous (and prints message).
 * ----------------------------------------------------------------------- */
static int
match_command_keyword(const char *text)
{
	static const struct { const char *name; int tok; } kw[] = {
		{ "power",   TOK_POWER   },
		{ "verbose", TOK_VERBOSE },
		{ "examine", TOK_EXAMINE },
		{ "modify",  TOK_MODIFY  },
		{ "status",  TOK_STATUS  },
		{ "snippet", TOK_SNIPPET },
		{ "loop",    TOK_LOOP    },
		{ "vector",  TOK_VECTOR  },
		{ "fill",    TOK_FILL    },
		{ "time",    TOK_TIME    },
		{ "go",      TOK_GO      },
		{ "task",    TOK_TASK    },
		{ "irq",     TOK_IRQ     },
		{ "trace",   TOK_TRACE   },
		{ "run",     TOK_RUN     },
		{ "stop",    TOK_STOP    },
		{ "freq",    TOK_FREQ    },
		{ "reset",   TOK_RESET   },
		{ "usb",     TOK_USB     },
		{ NULL, 0 }
	};

	size_t len = strlen(text);
	int matched_tok = 0;
	int matched_count = 0;
	const char *matched_name = NULL;

	for (int i = 0; kw[i].name != NULL; i++) {
		if (strncasecmp(text, kw[i].name, len) == 0) {
			if (matched_count == 0) {
				matched_tok  = kw[i].tok;
				matched_name = kw[i].name;
			} else {
				/* Second match → ambiguous */
				printf("Ambiguous command \"%s\" — \"%s\" vs \"%s\"",
				       text, matched_name, kw[i].name);
				/* Scan remaining matches to complete the message */
				for (int j = i + 1; kw[j].name != NULL; j++) {
					if (strncasecmp(text, kw[j].name, len) == 0)
						printf(" vs \"%s\"", kw[j].name);
				}
				printf("\n");
				return TOK_AMBIGUOUS;
			}
			matched_count++;
		}
	}
	return matched_tok; /* 0 if nothing matched */
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
