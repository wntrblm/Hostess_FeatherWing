#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum wtr_queue_overflow_behavior {
    WTR_QUEUE_OVF_DROP_NEWEST,
    WTR_QUEUE_OVF_DROP_OLDEST,
};

struct wtr_queue {
    // The underlying data
    uint8_t *data;
    // The size of one item.
    size_t item_size;
    // Total capacity. Should be in number of *items*, not number of *bytes*.
    size_t capacity;
    // The behavior when an overflow condition occurs.
    enum wtr_queue_overflow_behavior overflow_behavior;
    // The count of stored items.
    size_t _count;
    // Internal data.
    size_t _read_offset;
    size_t _write_offset;
};

void wtr_queue_init(struct wtr_queue *q);
void wtr_queue_push(volatile struct wtr_queue *q, uint8_t *i);
void wtr_queue_pop(volatile struct wtr_queue *q, uint8_t *i);
uint8_t * wtr_queue_pop_zerocopy(volatile struct wtr_queue *q);
void wtr_queue_empty(volatile struct wtr_queue *q);

inline bool wtr_queue_is_empty(volatile struct wtr_queue *q) { return q->_count == 0; }

inline bool wtr_queue_is_full(volatile struct wtr_queue *q) { return q->_count == q->capacity; }