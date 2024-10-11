/*
 * ring_buffer.c
 *
 *  Created on: Aug 15, 2024
 *      Author: 57316
 */

#include "ring_buffer.h"

/**
 * @brief Initializes a ring buffer with the given parameters.
 *
 * This function initializes a ring buffer with the provided buffer and capacity.
 * The head and tail pointers are set to the beginning of the buffer, and the is_full flag is set to 0.
 *
 * @param rb A pointer to the RingBuffer structure to be initialized.
 * @param buffer A pointer to the buffer array.
 * @param capacity The capacity of the buffer.
 */
void ring_buffer_init(RingBuffer *rb, uint8_t *buffer, uint16_t capacity) {
    rb->buffer = buffer;
    rb->capacity = capacity;
    rb->head = 0;
    rb->tail = 0;
    rb->is_full = 0;
}

/**
 * @brief Puts data into the ring buffer.
 *
 * This function puts the specified data into the ring buffer.
 *
 * @param rb Pointer to the RingBuffer structure.
 * @param data The data to be put into the ring buffer.
 */
void ring_buffer_put(RingBuffer *rb, uint8_t data) {
    rb->buffer[rb->head] = data;
    rb->head = (rb->head + 1) % rb->capacity;
    if (rb->is_full) {
        rb->tail = (rb->tail + 1) % rb->capacity;
    }
    if (rb-> head == rb->tail) {
        rb->is_full = 1;
    }
}

/**
 * @brief Retrieves and removes an element from the ring buffer.
 *
 * This function retrieves and removes an element from the ring buffer pointed to by `rb`.
 * If the buffer is empty, it returns 0.
 *
 * @param rb A pointer to the RingBuffer structure.
 * @return The retrieved element from the buffer, or 0 if the buffer is empty.
 */
uint8_t ring_buffer_get(RingBuffer *rb, uint8_t *data) {
    if ((rb ->is_full !=0) || (rb->head != rb->tail)) {
        *data = rb->buffer[rb->tail];
        rb->tail = (rb->tail + 1) % rb->capacity;
        rb->is_full = 0;
        return 1;
    }
    return 0; // buffer is empty
}

void ring_buffer_reset(RingBuffer *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->is_full = 0;
}

uint16_t ring_buffer_size(RingBuffer *rb) {
    if (rb->is_full) {
        return rb->capacity;
    }
    if (rb->head >= rb->tail) {
        return rb->head - rb->tail;
    }
    return rb->capacity + rb->head - rb->tail;
}

uint8_t ring_buffer_is_empty(RingBuffer *rb) {
    return (rb->head == rb->tail && !rb->is_full);
}

uint8_t ring_buffer_is_full(RingBuffer *rb) {
    return rb->is_full;
}


