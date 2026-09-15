/**
 * Copyright (c) 2025-2026 Mark R V Murray.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <cstdint>

#include "registers.h"

// System request opcodes
#define	REQUEST_NULL		uint8_t(0x00u)
#define REQUEST_TIME		uint8_t('T')
#define REQUEST_SET_TIME	uint8_t('t')
#define REQUEST_MOUNT		uint8_t('M')
#define REQUEST_UNMOUNT		uint8_t('m')
#define REQUEST_FLOAT		uint8_t('R')

// System status values
#define	RESPONSE_NULL		uint8_t(0x00u)
#define	RESPONSE_DONE		uint8_t(0x01u)
#define	RESPONSE_BUSY		uint8_t(0x7Fu)
#define RESPONSE_UNKNOWN	uint8_t(0xFFu)
#define	RESPONSE_UNAVAILABLE	uint8_t(0xFDu)

// Floating-point requests
#define FLOAT_ADD		uint8_t('+')
#define FLOAT_SUBTRACT		uint8_t('-')
#define FLOAT_MULTIPLY		uint8_t('*')
#define FLOAT_DIVIDE		uint8_t('/')

// SYSTEM_REQUEST_RESPONSE after a floating-point request: the result's sign
// and zeroness, laid out as the N and Z bits of the 6809 condition codes.
#define FLOAT_RESPONSE_NEGATIVE	uint8_t(0b1000u)
#define FLOAT_RESPONSE_ZERO	uint8_t(0b0100u)

// What a floating-point request leaves for SYSTEM_REQUEST_STATUS and
// SYSTEM_REQUEST_RESPONSE.
struct sysreq_float_result {
	uint8_t status;
	uint8_t response;
};

// Carry out floating-point request op on the two 4-byte big-endian floats at
// register offsets buffer and buffer + 4, leaving the result at buffer and the
// second operand untouched.  An operation it does not know leaves the buffer
// alone and answers RESPONSE_UNKNOWN.
//
// The casts matter: a register's float conversion is explicit, so without
// them the implicit uint8_t conversion is taken and each operand is a single
// byte.
inline sysreq_float_result
sysreq_float(const uint8_t op, const uint16_t buffer)
{
	const float a = static_cast<float>(reg[buffer]);
	const float b = static_cast<float>(reg[static_cast<uint16_t>(buffer + 4u)]);
	float c;
	switch (op) {
	case FLOAT_ADD:      c = a + b; break;
	case FLOAT_SUBTRACT: c = a - b; break;
	case FLOAT_MULTIPLY: c = a*b;   break;
	case FLOAT_DIVIDE:   c = a/b;   break;
	default:
		return { RESPONSE_UNKNOWN, RESPONSE_NULL };
	}
	reg[buffer] = c;
	return { RESPONSE_DONE,
		 static_cast<uint8_t>(c < 0.0f ? FLOAT_RESPONSE_NEGATIVE
					       : (c == 0.0f ? FLOAT_RESPONSE_ZERO : 0u)) };
}
