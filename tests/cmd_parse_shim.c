/**
 * Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Host-side shim that provides cmd_parse() for the test build.
 *
 * This replicates what pico_thing.cpp does: forward-declare the flex
 * buffer type (which lives only in the generated command_lexer.c) and
 * call cmd_yy_scan_string / cmd_yyparse / cmd_yy_delete_buffer.
 */

#include <stdbool.h>
#include "command_parser.h"

/* Forward-declare the flex internal buffer type so the typedef is visible
 * without including the generated command_lexer.c header. */
struct yy_buffer_state;
typedef struct yy_buffer_state *YY_BUFFER_STATE;

extern YY_BUFFER_STATE cmd_yy_scan_string(const char *str);
extern void            cmd_yy_delete_buffer(YY_BUFFER_STATE buf);
extern int             cmd_yyparse(parsed_cmd_t *result);
extern void            cmd_yy_reset_str_pool(void);

bool
cmd_parse(char *input, parsed_cmd_t *out)
{
	out->tag = CMD_NONE;
	cmd_yy_reset_str_pool();
	YY_BUFFER_STATE buf = cmd_yy_scan_string(input);
	int rc = cmd_yyparse(out);
	cmd_yy_delete_buffer(buf);
	/* YYERROR in grammar actions does not call yyerror(), so cmd.tag may
	 * still be CMD_NONE even when parsing failed.  Ensure callers always
	 * see a non-NONE tag on error. */
	if (rc != 0 && out->tag == CMD_NONE)
		out->tag = CMD_SYNTAX_ERROR;
	return rc == 0;
}