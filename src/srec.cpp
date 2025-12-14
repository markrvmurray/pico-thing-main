/**
* Copyright (c) 2025 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>

#include "srec.h"

uint
hex(const char *buf, const uint count)
{
	uint val = 0u;
	uint tally = 0u;
	for (;;) {
		val <<= 4;
		if (*buf >= '0' && *buf <= '9')
			val += *buf - '0';
		else if (*buf >= 'A' && *buf <= 'F')
			val += *buf - 'A' + 10;
		else if (*buf >= 'a' && *buf <= 'f')
			val += *buf - 'a' + 10;
		else
			return (uint)-1;
		buf++;
		tally++;
		if (count && tally == count)
			return val;
		if (!count && *buf == '\0')
			return val;
	}
}

bool
process_srecord(const char *buf, uint8_t *count, uint16_t *address, uint8_t *data, uint8_t *checksum)
{
	uint hexnum, pos = 2;

	if ((hexnum = hex(buf + pos, 2)) > 0xFF)
		return false;
	uint8_t index = hexnum;
	*count = index;
	uint16_t sum = index;
	index -= 3;
	pos += 2;
	if ((hexnum = hex(buf + pos, 4)) > 0xFFFF)
		return false;
	const uint16_t addr = hexnum;
	*address = addr;
	sum += (addr >> 8) & 0xFF;
	sum += (addr & 0xFF);
	pos += 4;
	for (uint i = 0; i < index; i++) {
		if ((hexnum = hex(buf + pos, 2)) > 0xFF)
			return false;
		const uint8_t val = hexnum;
		data[i] = val;
		sum += val;
		pos += 2;
	}
	sum = 0x00FF - (uint8_t)sum;
	if ((hexnum = hex(buf + pos, 2)) > 0xFF)
		return false;
	*checksum = hexnum;
	return *checksum == sum;
}