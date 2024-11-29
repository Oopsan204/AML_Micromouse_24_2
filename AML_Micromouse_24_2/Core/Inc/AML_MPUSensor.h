#ifndef AML_MPUSensor_H
#define AML_MPUSensor_H

#include "stm32h7xx_hal.h"
#include <stm32h743xx.h>
// #include "stm32f4xx_hal_uart.h"

void AML_MPUSensor_Setup(void);
void AML_MPUSensor_ResetAngle(void);
double AML_MPUSensor_GetAngle(void);

#endif /* test_h */
