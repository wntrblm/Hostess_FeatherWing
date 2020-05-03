#include <string.h>
#include <utils_assert.h>
#include "wtr_queue.h"


void wtr_queue_init(struct wtr_queue *q) {
    // Make sure the capacity is evenly divisible by the size.
    ASSERT(q->capacity % q->item_size == 0);
    ASSERT(q->data != NULL);
    q->_read_offset = 0;
    q->_write_offset = 0;
	q->_count = 0;
}


inline void _check_offsets(struct wtr_queue *q) {
	#ifdef DEBUG
		size_t tmp = q->_write_offset;
		// deal with wrap-around
		if (tmp < q->_read_offset) tmp += q->capacity;
		ASSERT(tmp - q->_read_offset == q->_count);
	#endif
}


void wtr_queue_push(struct wtr_queue *q, uint8_t *i) {
    uint8_t *write_ptr =  q->data + (q->item_size * q->_write_offset);

    memcpy(write_ptr, i, q->item_size);

    q->_write_offset = (q->_write_offset + 1) % q->capacity;
	q->_count++;
	_check_offsets(q);
}


void wtr_queue_pop(struct wtr_queue *q, uint8_t *i) {
    uint8_t *read_ptr =  q->data + (q->item_size * q->_read_offset);

    memcpy(i, read_ptr, q->item_size);

    q->_read_offset = (q->_read_offset + 1) % q->capacity;
	q->_count--;
	_check_offsets(q);
}
