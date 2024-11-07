#ifndef SenSor_H
#define SenSor_H

#include "stm32h7xx_hal.h"
#include "main.h"

void AML_IRSensor_Setup(void);
void AML_ReadAll_IRSensor(void);
void AML_Read_IRSensor(uint8_t sensor);
uint32_t AML_Get_IRSensor(uint8_t sensor);

#endif
