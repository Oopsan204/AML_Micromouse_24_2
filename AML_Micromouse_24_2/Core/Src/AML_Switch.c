#include "AML_Switch.h"

GPIO_TypeDef *BitSwitchPort = GPIOE;
uint16_t BitSwitchPinArray[5] = {BIT_SW_0_Pin, BIT_SW_1_Pin, BIT_SW_2_Pin, BIT_SW_3_Pin, BIT_SW_4_Pin};
uint8_t BitSwitchState[5] = {0, 0, 0, 0, 0};

GPIO_TypeDef *ButtonPort = GPIOA;
uint16_t ButtonPinArray[2] = {BUTTON_0_Pin, BUTTON_1_Pin};
uint8_t ButtonState[2] = {0, 0};

//-------------------------------------------------------------------------------------------------------//

void AML_ReadAll_BitSwitch(void);
uint8_t AML_Read_BitSwitch(uint8_t index);

void AML_ReadAll_Button(void);
uint8_t AML_Read_Button(uint8_t index);

//-------------------------------------------------------------------------------------------------------//
// Function of Bit Switch

void AML_ReadAll_BitSwitch(void)
{
    for (int i = 0; i < 5; i++)
    {
        BitSwitchState[i] = HAL_GPIO_ReadPin(BitSwitchPort, BitSwitchPinArray[i]);
    }
}

uint8_t AML_Read_BitSwitch(uint8_t index)
{
    return HAL_GPIO_ReadPin(BitSwitchPort, BitSwitchPinArray[index]);
}

//-------------------------------------------------------------------------------------------------------//
// Function of Button

void AML_ReadAll_Button(void)
{
    for (int i = 0; i < 2; i++)
    {
        ButtonState[i] = !HAL_GPIO_ReadPin(ButtonPort, ButtonPinArray[i]);
    }
}

uint8_t AML_Read_Button(uint8_t index)
{
    return !HAL_GPIO_ReadPin(ButtonPort, ButtonPinArray[index]);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BUTTON_0_Pin)
    {
        ButtonState[0] = 1;
    }
    else if (GPIO_Pin == BUTTON_1_Pin)
    {
        ButtonState[1] = 1;
    }
}

//-------------------------------------------------------------------------------------------------------//
