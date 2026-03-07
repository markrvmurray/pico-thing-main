#pragma once
#include <cstdint>
// Stub for host-side tests — no real interrupt masking needed.
static inline uint32_t save_and_disable_interrupts() { return 0; }
static inline void restore_interrupts(uint32_t) {}
