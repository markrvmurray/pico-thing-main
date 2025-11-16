/**
* Copyright (c) 2025 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef SREC_H
#define SREC_H

uint hex(const char *buf, uint count);
bool process_srecord(const char *buf, uint8_t *count, uint16_t *address, uint8_t *data, uint8_t *checksum);

#endif // SREC_H
