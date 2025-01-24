#ifndef MEDIANFILTER_H
#define MEDIANFILTER_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdlib.h>

typedef struct {
    int medFilterWin;
    int medDataPointer;
    int *data;
    uint8_t *sizeMap;
    uint8_t *locationMap;
    int oldestDataPoint;
    int totalSum;
} MedianFilter;

void MedianFilter_Init(MedianFilter *filter, int size, int seed);
int MedianFilter_AddValue(MedianFilter *filter, int value);
int MedianFilter_GetMedian(MedianFilter *filter);

#endif // MEDIANFILTER_H