#include "wtr_queue.h"
#include <string.h>
#include <utils_assert.h>

void wtr_queue_init(struct wtr_queue *q) {
    ASSERT(q->data != NULL);
    q->_read_offset = 0;
    q->_write_offset = 0;
    q->_count = 0;
}

inline static void _check_offsets(volatile struct wtr_queue *q) {
#ifdef DEBUG
    // queue is empty/full.
    if (q->_count == q->capacity && q->_read_offset == q->_write_offset)
        return;
    size_t tmp = q->_write_offset;
    // deal with wrap-around
    if (tmp < q->_read_offset)
        tmp += q->capacity;
    ASSERT(tmp - q->_read_offset == q->_count);
#endif
}

void wtr_queue_push(volatile struct wtr_queue *q, uint8_t *i) {
    bool overflow = q->_count == q->capacity;

    if (overflow && q->overflow_behavior == WTR_QUEUE_OVF_DROP_NEWEST)
        return;

    if (overflow && q->overflow_behavior == WTR_QUEUE_OVF_DROP_OLDEST) {
        // Advance the read head one to drop the oldest item.
        q->_read_offset = (q->_read_offset + 1) % q->capacity;
        // This will be incremented again below, keeping the count the same.
        q->_count--;
    }

    uint8_t *write_ptr = q->data + (q->item_size * q->_write_offset);

    memcpy(write_ptr, i, q->item_size);

    q->_write_offset = (q->_write_offset + 1) % q->capacity;
    q->_count++;
    _check_offsets(q);
}

void wtr_queue_pop(volatile struct wtr_queue *q, uint8_t *i) {
    uint8_t *read_ptr = q->data + (q->item_size * q->_read_offset);

    memcpy(i, read_ptr, q->item_size);

    q->_read_offset = (q->_read_offset + 1) % q->capacity;
    q->_count--;
    _check_offsets(q);
}

void wtr_queue_empty(volatile struct wtr_queue *q) {
    q->_write_offset = 0;
    q->_read_offset = 0;
    q->_count = 0;
}