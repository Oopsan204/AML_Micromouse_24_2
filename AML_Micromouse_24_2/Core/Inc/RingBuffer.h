#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <stdint.h>

#define ADC_CHANNELS 5
#define BUFFER_SIZE  100  // Adjust the buffer size as needed

typedef struct {
    uint32_t data[ADC_CHANNELS][BUFFER_SIZE];
    uint32_t head;
    uint32_t tail;
    uint32_t size;
} RingBuffer;

void RingBuffer_Init(RingBuffer *buffer);
void RingBuffer_Put(RingBuffer *buffer, uint32_t *data);
int RingBuffer_Get(RingBuffer *buffer, uint32_t *data);
int RingBuffer_IsEmpty(RingBuffer *buffer);
int RingBuffer_IsFull(RingBuffer *buffer);

#endif // RINGBUFFER_H