/*
 * Copyright (c) 2025-2026 Mark R V Murray
 *
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * ACIA Flow Control Stress Test (6809 side)
 *
 * Runs on the AUXILIARY ACIA ($FFC5/$FFC6, USB CDC port 1).
 * Console ACIA ($FFC3/$FFC4, USB CDC port 0) is used for status output.
 *
 * Protocol:
 *   - Waits for 'G' (go) from host on aux ACIA (30s timeout)
 *   - Sends 'R' (ready) back so host can synchronise
 *   - TX: sends a framed packet (SOF + LEN + payload + CHK + EOF)
 *   - RX: expects the host to echo the same frame back
 *   - If the echo frame is invalid, the whole frame is discarded
 *   - Uses OUTCH/INCH on the aux ACIA (works with IRQ or polled I/O)
 *   - Reports progress and results on the console
 *
 * Frame format (big-endian):
 *   SOF(0x02) | SEQ | LEN_H | LEN_L | PAYLOAD[LEN] | CHK | EOF(0x03)
 *   CHK = XOR of SEQ, LEN_H, LEN_L, and all payload bytes
 *
 * The host-side Python script (extra/stress_acia.py) buffers and
 * validates each frame before echoing, so partial/corrupt data
 * is discarded rather than echoed.
 */

#include <cmoc.h>
#include "pico.h"

/* Frame markers */
#define SOF     0x02
#define FRAME_EOF 0x03

/* Test parameters */
#define PAYLOAD_LEN 4096u
#define NUM_FRAMES  16u
#define TIMEOUT     3000000UL	/* ~30s at 1 MHz E clock */

/* Buffer for block receive — verified in C after the tight asm loop. */
static uint8_t rx_payload[PAYLOAD_LEN];

/* --- I/O helpers (work with both IRQ and polled io.asm) ---------- */

/* Send one byte via OUTCH (blocks until sent/queued).
 * Returns 0 always. */
static uint8_t
aux_send(uint8_t b)
{
	asm {
		EXTERN OUTCH, AUX_PORT
		lda :b
		pshs u
		ldu #AUX_PORT
		lbsr OUTCH
		puls u
	}
	return 0;
}

/* Receive one byte via INCH (polls with timeout).
 * Returns 0 on success, result stored through *out. */
static uint8_t
aux_recv(uint8_t *out)
{
	uint8_t val;
	uint32_t t = TIMEOUT;
	while (t > 0) {
		uint8_t got = 0;
		asm {
			EXTERN INCH, AUX_PORT
			pshs u
			ldu #AUX_PORT
			lbsr INCH
			puls u
			bcc @no
			sta :val
			inc :got
@no
		}
		if (got) {
			*out = val;
			return 0;
		}
		t--;
	}
	return 1;
}

/* --- Frame TX / RX ----------------------------------------------- */

/* Send one framed packet: SOF SEQ LEN_H LEN_L PAYLOAD CHK EOF.
 * Payload is the incrementing byte pattern starting at *tx_val.
 * CHK covers SEQ + LEN + payload.
 * Returns 0 on success (timeout otherwise). */
static uint8_t
send_frame(uint8_t seq, uint16_t len, uint8_t *tx_val)
{
	uint8_t chk = 0;
	uint8_t b;

	if (aux_send(SOF))       return 1;
	chk ^= seq;
	if (aux_send(seq))       return 1;
	b = len >> 8;  chk ^= b;
	if (aux_send(b))         return 1;
	b = len & 0xFF; chk ^= b;
	if (aux_send(b))         return 1;

	/* Payload — tight asm block, ~47 cycles/byte vs ~120 in C.
	 * OUTCH preserves B,X,Y,U.  A may be destroyed on the O@D
	 * drain path, so we save/restore it around each call.
	 * 6809 has no register-to-register XOR, so we XOR via the stack. */
	{
		uint8_t sv = *tx_val;
		uint8_t payload_chk = 0;
		uint16_t count = len;
		asm {
			EXTERN OUTCH, AUX_PORT
			lda :sv
			ldx :count
			clrb
			pshs u
			ldu #AUX_PORT
@sloop			pshs a
			eorb ,s
			lbsr OUTCH
			puls a
			inca
			leax -1,x
			bne @sloop
			puls u
			sta :sv
			stb :payload_chk
		}
		*tx_val = sv;
		chk ^= payload_chk;
	}

	if (aux_send(chk))       return 1;
	if (aux_send(FRAME_EOF)) return 1;
	return 0;
}

/* Receive one framed echo packet.  Verifies SOF, sequence number,
 * length, payload pattern, checksum, and EOF.
 * Returns 0 on full pass; sets *errors, *got on any mismatch. */
static uint8_t
recv_frame(uint8_t expected_seq, uint16_t expected_len,
	   uint8_t *rx_expected, uint16_t *got, uint16_t *errors)
{
	uint8_t b, chk = 0;
	uint16_t len, i;

	/* SOF */
	if (aux_recv(&b))  return 1;
	if (b != SOF) {
		(*errors)++;
		return 2;
	}

	/* SEQ */
	if (aux_recv(&b))  return 1;
	chk ^= b;
	if (b != expected_seq) {
		if (*errors < 10)
			printf("SEQ err: exp=%02X got=%02X\n",
				(unsigned)expected_seq, (unsigned)b);
		(*errors)++;
	}

	/* LEN (big-endian) */
	if (aux_recv(&b))  return 1;
	chk ^= b;
	len = (uint16_t)b << 8;
	if (aux_recv(&b))  return 1;
	chk ^= b;
	len |= b;
	if (len != expected_len) {
		(*errors)++;
		return 3;
	}

	/* PAYLOAD — tight asm receive into rx_payload[], then verify in C.
	 * INCH preserves B,X,Y,U; returns byte in A, carry set = got byte.
	 * No per-byte timeout: spins until each byte arrives.  The Python
	 * host script has its own 30s timeout if something goes wrong.
	 * Checksum is computed in the C verification loop to keep the
	 * asm receive loop as tight as possible (~36 cycles/byte). */
	{
		uint16_t count = len;
		uint8_t *buf = rx_payload;
		asm {
			EXTERN INCH, AUX_PORT
			ldx :buf
			ldy :count
			pshs u
			ldu #AUX_PORT
@rloop			lbsr INCH
			bcc @rloop
			sta ,x+
			leay -1,y
			bne @rloop
			puls u
		}
	}
	/* Verify pattern and accumulate checksum (in-memory, no I/O). */
	{
		uint8_t exp_val = *rx_expected;
		uint16_t local_got = *got;
		uint16_t local_errors = *errors;
		for (i = 0; i < len; i++) {
			chk ^= rx_payload[i];
			if (rx_payload[i] != exp_val) {
				if (local_errors < 10)
					printf("ERR @%u: exp=%02X got=%02X\n",
						local_got, (unsigned)exp_val,
						(unsigned)rx_payload[i]);
				local_errors++;
				exp_val = rx_payload[i];
			}
			exp_val++;
			local_got++;
		}
		*rx_expected = exp_val;
		*got = local_got;
		*errors = local_errors;
	}

	/* CHK */
	if (aux_recv(&b))  return 1;
	if (b != chk) {
		(*errors)++;
		return 4;
	}

	/* EOF */
	if (aux_recv(&b))  return 1;
	if (b != FRAME_EOF) {
		(*errors)++;
		return 5;
	}

	return 0;
}

/* ----------------------------------------------------------------- */

int
main(void)
{
	uint8_t tx_val, rx_expected, ch, seq;
	uint16_t rx_got, rx_errors;
	uint16_t frame_ok, frame_fail;
	uint8_t send_rc, recv_rc;

	/* Flush stale data from aux ACIA BEFORE enabling IRQs.
	 * Repeatedly reset the ACIA (clears the 256-byte SpscQueue on
	 * the Pico side), drain the data register, and wait for
	 * uart_port_pump() to refill from USB.  Loop until no new data
	 * arrives, meaning USB buffers are exhausted.
	 * IRQs are still disabled here (startup default), so no ISR runs. */
	asm {
		pshs a,x
@flush		lda #$03
		sta $FFC5	master reset: clears SpscQueue
		ldx #2000	wait ~6ms for uart_port_pump to refill
@wait		leax -1,x
		bne @wait
@drn		lda $FFC5	drain staged byte
		bita #$01	RDRF?
		beq @chk
		lda $FFC6	discard
		bra @drn
@chk		ldx #500	wait ~1.5ms to see if more data arrives
@chk2		lda $FFC5
		bita #$01
		bne @flush	new data! reset and drain again
		leax -1,x
		bne @chk2
		puls a,x	no more stale data
	}

	/* Now initialise both ACIAs (console + aux).
	 * INIT_AUX does its own C_RESET then enables RX IRQ. */
	initialiseFastSerial();
	setConsoleOutHook(newOutputRoutine);

	printf("\nACIA Flow Control Stress Test\n");
	printf("============================\n");
	printf("Aux ACIA ($FFC5/$FFC6), %u x %u byte frames\n\n",
		NUM_FRAMES, PAYLOAD_LEN);

	/* Wait for 'G' (go) from host script, ~30s timeout. */
	printf("Waiting for host (aux ACIA)...\n");
	{
		uint32_t timeout = TIMEOUT;
		while (timeout > 0) {
			uint8_t have = 0;
			asm {
				EXTERN INCH, AUX_PORT
				pshs u
				ldu #AUX_PORT
				lbsr INCH
				puls u
				bcc @nope
				sta :ch
				inc :have
@nope
			}
			if (have && ch == 'G')
				break;
			timeout--;
		}
		if (timeout == 0) {
			printf("TIMEOUT waiting for host\n");
			SYNC;
			return 1;
		}
	}

	/* Drain any trailing stale data after finding 'G'.
	 * The host won't send anything more until it sees 'R',
	 * so anything arriving now is definitely stale. */
	{
		uint16_t quiet = 0;
		while (quiet < 10000) {
			uint8_t have = 0;
			asm {
				EXTERN INCH, AUX_PORT
				pshs u
				ldu #AUX_PORT
				lbsr INCH
				puls u
				bcc @stl
				inc :have
@stl
			}
			if (have)
				quiet = 0;
			else
				quiet++;
		}
	}

	/* Send 'R' (ready) back to host so it can synchronise. */
	aux_send('R');

	printf("Host connected\n");

	/* --- Send frames, receive echo for each --- */
	tx_val = 0;
	rx_expected = 0;
	rx_got = 0;
	rx_errors = 0;
	frame_ok = 0;
	frame_fail = 0;

	for (seq = 0; seq < NUM_FRAMES; seq++) {
		printf("[%u] ", (unsigned)seq);

		send_rc = send_frame(seq, PAYLOAD_LEN, &tx_val);
		if (send_rc) {
			printf("TX timeout\n");
			frame_fail++;
			break;
		}

		recv_rc = recv_frame(seq, PAYLOAD_LEN, &rx_expected,
				     &rx_got, &rx_errors);
		if (recv_rc) {
			printf("RX err rc=%u\n", (unsigned)recv_rc);
			frame_fail++;
			break;
		}

		printf("OK\n");
		frame_ok++;
	}

	/* Final report */
	printf("\n--- Results ---\n");
	printf("Frames OK:  %u / %u\n", frame_ok, NUM_FRAMES);
	printf("Frames bad: %u\n", frame_fail);
	printf("RX total:   %u bytes\n", rx_got);
	printf("RX errors:  %u\n", rx_errors);

	if (frame_ok == NUM_FRAMES && rx_errors == 0)
		printf("\nPASS\n");
	else
		printf("\n*** FAIL ***\n");

	/* Halt */
	SYNC;
	return 0;
}
