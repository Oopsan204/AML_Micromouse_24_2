#include "AML_LedDebug.h"

// extern TIM_HandleTypeDef htim10;

GPIO_TypeDef *LedPort = GPIOC;
// GPIO_TypeDef *LedPort1 = GPIOD;
uint16_t Led[5] = {GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_15};

uint8_t LedIndex = 0;
uint8_t LedIndexFlag = 1;
uint8_t MidIndex = 0;

//-------------------------------------------------------------------------------------------------------//

void AML_LedDebug_TurnOnLED(COLOR color);
void AML_LedDebug_TurnOffLED(COLOR color);
void AML_LedDebug_ToggleLED(COLOR color);
void AML_LedDebug_SetLED(COLOR color, GPIO_PinState state);
void AML_LedDebug_SetAllLED(GPIO_PinState state);
void AML_LedDebug_ToggleAllLED(void);

//-------------------------------------------------------------------------------------------------------//

void AML_LedDebug_TurnOnLED(COLOR color)
{
    // HAL_GPIO_WritePin(GPIOD, Led[color], GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, Led[color], GPIO_PIN_SET);
}

void AML_LedDebug_TurnOffLED(COLOR color)
{
    // HAL_GPIO_WritePin(GPIOD, Led[color], GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, Led[color], GPIO_PIN_RESET);
}

void AML_LedDebug_ToggleLED(COLOR color)
{
    // HAL_GPIO_TogglePin(GPIOD, Led[color]);
    HAL_GPIO_TogglePin(GPIOC, Led[color]);
}

void AML_LedDebug_SetLED(COLOR color, GPIO_PinState state)
{
    // HAL_GPIO_WritePin(GPIOD, Led[color], state);
    HAL_GPIO_WritePin(GPIOC, Led[color], state);
}

void AML_LedDebug_SetAllLED(GPIO_PinState state)
{
    HAL_GPIO_WritePin(GPIOC, Led[0], state);
    HAL_GPIO_WritePin(GPIOC, Led[1], state);
    HAL_GPIO_WritePin(GPIOC, Led[2], state);
    HAL_GPIO_WritePin(GPIOC, Led[3], state);
    // HAL_GPIO_WritePin(GPIOD, Led[15], state);
}

void AML_LedDebug_ToggleAllLED()
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
}

