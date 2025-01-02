#include "MedianFilter.h"

static int constrain(int value, int min, int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void MedianFilter_Init(MedianFilter *filter, int size, int seed) {
    filter->medFilterWin = constrain(size, 3, 255); // number of samples in sliding median filter window - usually odd #
    filter->medDataPointer = size >> 1;             // mid point of window
    filter->data = (int*) calloc(size, sizeof(int)); // array for data
    filter->sizeMap = (uint8_t*) calloc(size, sizeof(uint8_t)); // array for locations of data in sorted list
    filter->locationMap = (uint8_t*) calloc(size, sizeof(uint8_t)); // array for locations of history data in map list
    filter->oldestDataPoint = filter->medDataPointer; // oldest data point location in data array
    filter->totalSum = size * seed;                   // total of all values

    for (uint8_t i = 0; i < filter->medFilterWin; i++) { // initialize the arrays
        filter->sizeMap[i] = i;      // start map with straight run
        filter->locationMap[i] = i;  // start map with straight run
        filter->data[i] = seed;      // populate with seed value
    }
}

int MedianFilter_AddValue(MedianFilter *filter, int value) {
    // Add new value to the data array
    filter->data[filter->oldestDataPoint] = value;
    filter->oldestDataPoint = (filter->oldestDataPoint + 1) % filter->medFilterWin;
    // Sort the data array to find the median
    // This is a simple insertion sort for demonstration purposes
    for (int i = 0; i < filter->medFilterWin - 1; i++) {
        for (int j = i + 1; j < filter->medFilterWin; j++) {
            if (filter->data[i] > filter->data[j]) {
                int temp = filter->data[i];
                filter->data[i] = filter->data[j];
                filter->data[j] = temp;
            }
        }
    }
    return filter->data[filter->medDataPointer];
}

int MedianFilter_GetMedian(MedianFilter *filter) {
    return filter->data[filter->medDataPointer];
}