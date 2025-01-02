#include "RingBuffer.h"

void RingBuffer_Init(RingBuffer *buffer)
{
    buffer->head = 0;
    buffer->tail = 0;
    buffer->size = BUFFER_SIZE;
}

void RingBuffer_Put(RingBuffer *buffer, uint32_t *data)
{
    for (int i = 0; i < ADC_CHANNELS; i++) {
        buffer->data[i][buffer->head] = data[i];
    }
    buffer->head = (buffer->head + 1) % buffer->size;

    // Overwrite the oldest data if the buffer is full
    if (buffer->head == buffer->tail) {
        buffer->tail = (buffer->tail + 1) % buffer->size;
    }
}

int RingBuffer_Get(RingBuffer *buffer, uint32_t *data)
{
    if (buffer->head == buffer->tail) {
        // Buffer is empty
        return -1;
    }

    for (int i = 0; i < ADC_CHANNELS; i++) {
        data[i] = buffer->data[i][buffer->tail];
    }
    buffer->tail = (buffer->tail + 1) % buffer->size;
    return 0;
}

int RingBuffer_IsEmpty(RingBuffer *buffer)
{
    return buffer->head == buffer->tail;
}

int RingBuffer_IsFull(RingBuffer *buffer)
{
    return ((buffer->head + 1) % buffer->size) == buffer->tail;
}
