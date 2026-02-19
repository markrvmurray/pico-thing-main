/**
 * Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <cctype>
#include <cstdio>

#include "pico_thing.h"
#include "registers.h"

registers &reg = registers::getInstance();

void
registers::copy_out(uint8_t *dst, uint16_t addr, uint16_t len)
{
	for (uint16_t i = 0; i < len; i++)
		dst[i] = read_registers[addr + i];
}

void
registers::copy_out_write(uint8_t *dst, uint16_t addr, uint16_t len)
{
	for (uint16_t i = 0; i < len; i++)
		dst[i] = write_registers[addr + i];
}

void
registers::copy_in(uint16_t addr, const uint8_t *src, uint16_t len)
{
	for (uint16_t i = 0u; i < len; i++)
		read_registers[addr + i] = src[i];
}

void
registers::dump()
{
	static uint8_t printables[17]; // Include the decorative bar down the middle

	printf("Write block:\n");
	for (uint16_t i = 0u; i < 4u; i++) {
		copy_out_write(printables, i*16u, 16u);
		printf("%04X:", REGISTER_BASE + i*16u);
		for (uint16_t j = 0u; j < 16u; j++) {
			auto ch = static_cast<unsigned char>(printables[j]) & 0x7F;
			printf(" %02X", printables[j]);
			if (j == 7u)
				printf(" ");
			printables[j] = std::isprint(ch) ? static_cast<char>(ch) : '.';
		}
		printf(" |%.8s|%.8s|\n", reinterpret_cast<char *>(printables), reinterpret_cast<char *>(printables + 8));
	}
	printf("Read block:\n");
	for (uint16_t i = 0u; i < 4u; i++) {
		copy_out(printables, i*16u, 16u);
		printf("%04X:", static_cast<uint16_t>(REGISTER_BASE + i*16u));
		for (uint16_t j = 0u; j < 16u; j++) {
			auto ch = static_cast<unsigned char>(printables[j]) & 0x7F;
			printf(" %02X", printables[j]);
			if (j == 7u)
				printf(" ");
			printables[j] = std::isprint(ch) ? static_cast<char>(ch) : '.';
		}
		printf(" |%.8s|%.8s|\n", reinterpret_cast<char *>(printables), reinterpret_cast<char *>(printables + 8));
	}
	printf("--\n");
}
