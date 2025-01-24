#include "AML_LedDebug.h"

// extern TIM_HandleTypeDef htim10;

GPIO_TypeDef *LedPort = GPIOC;
GPIO_TypeDef *LedPort1 = GPIOD;
uint16_t Led[5] = {GPIO_PIN_15, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9};

uint8_t LedIndex = 0;
uint8_t LedIndexFlag = 1;
uint8_t MidIndex = 0;

//-------------------------------------------------------------------------------------------------------//

void AML_LedDebug_TurnOnLED(Name_SS name);
void AML_LedDebug_TurnOffLED(Name_SS name);
void AML_LedDebug_ToggleLED(Name_SS name);
void AML_LedDebug_SetLED(Name_SS name, GPIO_PinState state);
void AML_LedDebug_SetAllLED(GPIO_PinState state);
void AML_LedDebug_ToggleAllLED(void);



//-------------------------------------------------------------------------------------------------------//

void AML_LedDebug_TurnOnLED(Name_SS name)
{
    // HAL_GPIO_WritePin(GPIOD, Led[name], GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, Led[name], GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, Led[name], GPIO_PIN_SET);
}

void AML_LedDebug_TurnOffLED(Name_SS name)
{
    // HAL_GPIO_WritePin(GPIOD, Led[name], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, Led[name], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, Led[name], GPIO_PIN_RESET);
}

void AML_LedDebug_ToggleLED(Name_SS name)
{
    // HAL_GPIO_TogglePin(GPIOD, Led[name]);
    HAL_GPIO_TogglePin(GPIOC, Led[name]);
    HAL_GPIO_TogglePin(GPIOD, Led[name]);
}

void AML_LedDebug_SetLED(Name_SS name, GPIO_PinState state)
{
    // HAL_GPIO_WritePin(GPIOD, Led[name], state);
    HAL_GPIO_WritePin(GPIOC, Led[name], state);
    HAL_GPIO_WritePin(GPIOD, Led[name], state);
}

void AML_LedDebug_SetAllLED(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOC, Led[4], state);
    HAL_GPIO_WritePin(GPIOC, Led[3], state);
    HAL_GPIO_WritePin(GPIOC, Led[2], state);
    HAL_GPIO_WritePin(GPIOC, Led[1], state);
    HAL_GPIO_WritePin(GPIOD, Led[0], state);
}

void AML_LedDebug_ToggleAllLED()
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
}

