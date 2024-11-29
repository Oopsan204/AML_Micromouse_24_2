#ifndef AML_LEDDEBUG_H
#define AML_LEDDEBUG_H

#include "stm32h7xx_hal.h"
#include "main.h"

// Led number
// #define LEDPORT GPIOC

typedef enum
{
    YELLOW,
    GREEN,
    BLUE,
    RED,
    PURPLE
} COLOR;

void AML_LedDebug_TurnOnLED(COLOR color);
void AML_LedDebug_TurnOffLED(COLOR color);
void AML_LedDebug_ToggleLED(COLOR color);
void AML_LedDebug_SetLED(COLOR color, GPIO_PinState state);
void AML_LedDebug_SetAllLED(GPIO_PinState state);
void AML_LedDebug_ToggleAllLED(void);

#endif // AML_DEBUGDEVICE_H
