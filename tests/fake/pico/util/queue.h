#pragma once
// Host-side fake for pico/util/queue.h.
// Single-producer / single-consumer byte ring buffer.
// Only element_size==1 is used by mc6850.

#include <cstddef>
#include <cstdint>
#include <cstring>

struct queue_t {
	uint8_t  *buf  = nullptr;
	uint32_t  head = 0;
	uint32_t  tail = 0;
	uint32_t  size = 0;	// capacity in elements
};

static inline void
queue_init(queue_t *q, size_t /*element_size*/, size_t count)
{
	q->buf  = new uint8_t[count]();
	q->head = 0;
	q->tail = 0;
	q->size = static_cast<uint32_t>(count);
}

static inline bool queue_is_empty(queue_t *q) { return q->head == q->tail; }
static inline bool queue_is_full(queue_t *q)  { return ((q->head + 1) % q->size) == q->tail; }

static inline uint16_t
queue_get_level(queue_t *q)
{
	return static_cast<uint16_t>((q->head + q->size - q->tail) % q->size);
}

static inline bool
queue_try_add(queue_t *q, const void *data)
{
	if (queue_is_full(q)) return false;
	q->buf[q->head] = *static_cast<const uint8_t *>(data);
	q->head = (q->head + 1) % q->size;
	return true;
}

static inline bool
queue_try_remove(queue_t *q, void *data)
{
	if (queue_is_empty(q)) return false;
	*static_cast<uint8_t *>(data) = q->buf[q->tail];
	q->tail = (q->tail + 1) % q->size;
	return true;
}
