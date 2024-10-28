#ifndef ANL_SENSOR_H
#define ANL_SENSOR_H
#include "stm32h7xx.h"
#include "main.h"
#include "math.h"
typedef enum{
    SENSOR_FF,
    SENSOR_FR,
    SENSOR_FL,
    SENSOR_R,
    SENSOR_L,
    SENSOR_BL,
    SENSOR_BR
}SENSOR_NAME;


void SENSOR_INIT(void);
void SENSOR_READ_ALL(void);
uint32_t SENSOR_READ_ONE(int num);

#endif
