#ifndef QUEUE_H
#define QUEUE_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define QUEUE_MAX_SIZE 256

typedef struct _queue *queue;
void queue_reset();
void queue_push(int32_t elem);
int32_t queue_pop();
int32_t queue_first();
int32_t queue_is_empty();
int32_t queue_size();
void queue_clear();

struct _queue
{
    int32_t data[QUEUE_MAX_SIZE];
    int32_t head;
    int32_t tail;
    int32_t size;
};

#endif