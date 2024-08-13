#ifndef QUEUE_H
#define QUEUE_H

#include <stdio.h>
#include <stdlib.h>

typedef struct _queue* queue;
queue queue_create();
void queue_destroy(queue q);
void queue_push(queue q, int elem);
int queue_pop(queue q);
int queue_first(queue q);
int queue_is_empty(queue q);
int queue_size(queue q);
void queue_clear(queue q);

struct node {
    int data;
    struct node* next;
};

struct _queue {
    struct node* head;
    struct node* tail;
    int size;
};

#endif