#include "queue.h"

struct _queue squares_queue;

void queue_reset()
{
    for (int i = 0; i < 256; i++)
    {
        squares_queue.data[i] = 0;
    }

    squares_queue.head = 0;
    squares_queue.tail = 0;
    squares_queue.size = 0;
}

void queue_push(int32_t elem)
{
    if (squares_queue.size == QUEUE_MAX_SIZE)
    {
        // Hàng đợi đầy
        return;
    }
    squares_queue.data[squares_queue.tail] = elem;
    squares_queue.tail = (squares_queue.tail + 1) % QUEUE_MAX_SIZE;
    squares_queue.size++;
}

int32_t queue_pop()
{
    if (squares_queue.size == 0)
    {
        // Hàng đợi rỗng
        return -1; // Hoặc giá trị đặc biệt để biểu thị lỗi
    }
    int32_t elem = squares_queue.data[squares_queue.head];
    squares_queue.head = (squares_queue.head + 1) % QUEUE_MAX_SIZE;
    squares_queue.size--;
    return elem;
}

int32_t queue_first()
{
    if (squares_queue.size == 0)
    {
        // Hàng đợi rỗng
        return -1; // Hoặc giá trị đặc biệt để biểu thị lỗi
    }
    return squares_queue.data[squares_queue.head];
}

int32_t queue_is_empty()
{
    return squares_queue.size == 0;
}

int32_t queue_size()
{
    return squares_queue.size;
}

void queue_clear()
{
    squares_queue.head = 0;
    squares_queue.tail = 0;
    squares_queue.size = 0;
}