// Copyright (C) 2026 Mark Murray
//
// THE Q-RISING HANDLER'S ONE-BYTE ARMED TEST.
//
// The handler runs on every E cycle and ends early unless the bus state
// changed or one of a handful of features is armed: the trace's per-cycle
// column, the single-step gate, the E-cycle countdown, the delayed NMI block
// after a DAT fault.  Testing those meant six volatile loads from four
// objects on every entry - about 10 clk_sys cycles of a 128-cycle E cycle at
// 2.5 MHz - to get the answer "no" almost every time.  This byte is that
// answer, kept by the writers instead of recomputed by the reader.
//
// THE RULE: anything that makes one of the handler's armed sources non-zero
// calls q_rising_arm() AFTER writing the source.  The slow path re-derives
// the byte from the sources when it runs, which is how it is cleared: it
// stores 0, fences, then re-reads the sources and stores 1 if any is live.
// With the writer's fence between its source and its store of 1, every
// interleaving ends with the byte set when a source is live: a source
// written before the slow path's re-read is seen by it, and one written
// after it has its store of 1 land after the slow path's store of 0.
//
// The sources themselves stay authoritative.  A DIAG build audits the byte
// against them from Core 1's loop and st reports every pass that found a
// live source with the byte clear, which is what catches a writer that was
// missed - the failure mode here is silent, and two conditions were missed
// on the way into the gated handler the first time.
#pragma once

#include <atomic>
#include <cstdint>

inline volatile uint8_t q_rising_armed = 0u;

static inline void
q_rising_arm()
{
	// The source is written before this is called; keep it that way in
	// the other core's view too.
	std::atomic_thread_fence(std::memory_order_release);
	q_rising_armed = 1u;
}
