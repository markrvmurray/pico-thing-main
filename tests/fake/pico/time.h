#pragma once
// Fake pico/time.h for host-side testing.
// Provides the repeating_timer struct and a stub add_repeating_timer_us().

#include <cstdint>

typedef bool (*repeating_timer_callback_t)(struct repeating_timer *);

struct repeating_timer {
	int64_t delay_us;
	repeating_timer_callback_t callback;
	void *user_data;
};

// Stub: records the callback and user_data but does not actually schedule anything.
// Tests call the callback directly to simulate timer fires.
static inline bool add_repeating_timer_us(int64_t delay_us,
                                          repeating_timer_callback_t callback,
                                          void *user_data,
                                          struct repeating_timer *out)
{
	out->delay_us = delay_us;
	out->callback = callback;
	out->user_data = user_data;
	return true;
}

// Stub: in tests, nothing to cancel.
static inline bool cancel_repeating_timer(struct repeating_timer *timer)
{
	(void)timer;
	return true;
}
