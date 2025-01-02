#ifndef SenSor_H
#define SenSor_H

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <AML_GlobalVariable.h>
#include <HC_SR04.h>
#include <RingBuffer.h>
#include <MedianFilter.h>
#include "math.h"


void AML_IRSensor_Setup(void);
double AML_IRSensor_GetDistance(uint8_t sensor);
bool AML_IRSensor_IsFrontWall(void);
bool AML_IRSensor_IsLeftWall(void);
bool AML_IRSensor_IsRightWall(void);

bool AML_IRSensor_IsNoFrontWall(void);
bool AML_IRSensor_IsNoLeftWall(void);
bool AML_IRSensor_IsNoRightWall(void);
#endif
