#ifndef AML_LEDDEBUG_H
#define AML_LEDDEBUG_H

#include "stm32h7xx_hal.h"
#include "main.h"

// Led number
// #define LEDPORT GPIOC

typedef enum
{
    N_R,
    N_FR,
    N_FF,
    N_FL,
    N_L,
} Name_SS;

void AML_LedDebug_TurnOnLED(Name_SS name);
void AML_LedDebug_TurnOffLED(Name_SS name);
void AML_LedDebug_ToggleLED(Name_SS name);
void AML_LedDebug_SetLED(Name_SS name, GPIO_PinState state);
void AML_LedDebug_SetAllLED(GPIO_PinState state);
void AML_LedDebug_ToggleAllLED(void);


#endif // AML_DEBUGDEVICE_H
