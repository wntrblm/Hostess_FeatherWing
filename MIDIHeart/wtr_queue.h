#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

struct wtr_queue {
    // The underlying data
    uint8_t *data;
    // The size of one item.
    size_t item_size;
    // Total capacity. Should be in number of *items*, not number of *bytes*.
    size_t capacity;
    // The count of stored items.
    size_t _count;
    // Internal data.
    size_t _read_offset;
    size_t _write_offset;
};


void wtr_queue_init(struct wtr_queue *q);
void wtr_queue_push(struct wtr_queue *q, uint8_t *i);
void wtr_queue_pop(struct wtr_queue *q, uint8_t *i);
void wtr_queue_empty(struct wtr_queue *q);

inline bool wtr_queue_is_empty(struct wtr_queue *q) {
    return q->_count == 0;
}

inline bool wtr_queue_is_full(struct wtr_queue *q) {
    return q->_count == q->capacity;
}