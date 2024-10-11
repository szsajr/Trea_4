/*
 * ring_buffer.h
 *
 *  Created on: Aug 15, 2024
 *      Author: 57316
 */

#ifndef INC_RING_BUFFER_H_
#define INC_RING_BUFFER_H_

#include <stdint.h>

// Create a ring buffer structure
typedef struct {
    uint8_t *buffer; // Pointer to the data buffer
    uint16_t head; // Index of the head
    uint16_t tail; // Index of the tail
    uint16_t capacity; // Capacity of the buffer
    uint8_t is_full; // Flag to indicate if the buffer is full
} RingBuffer;

// Initialize the ring buffer
void ring_buffer_init(RingBuffer *rb, uint8_t *buffer, uint16_t capacity);
// Check if the ring buffer is full
void ring_buffer_put(RingBuffer *rb, uint8_t data);
// Put a byte into the ring buffer
uint8_t ring_buffer_get(RingBuffer *rb, uint8_t *data);
// reset the ring buffer
void ring_buffer_reset(RingBuffer *rb);
// Check size of the ring buffer
uint16_t ring_buffer_size(RingBuffer *rb);
// check if the ring buffer is empty
uint8_t ring_buffer_is_empty(RingBuffer *rb);
// check if the ring buffer is full
uint8_t ring_buffer_is_full(RingBuffer *rb);


#endif /* SRC_RING_BUFFER_H_ */
