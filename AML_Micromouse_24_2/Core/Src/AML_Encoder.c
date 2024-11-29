#include "AML_Encoder.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;

void AML_Encoder_Setup(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); // left encoder
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL); // right encoder
}

int32_t AML_Encoder_GetLeftValue(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2);
}

void AML_Encoder_ResetLeftValue(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
}

int32_t AML_Encoder_GetRightValue(void)
{
    return __HAL_TIM_GET_COUNTER(&htim5);
}

void AML_Encoder_ResetRightValue(void)
{
    __HAL_TIM_SET_COUNTER(&htim5, 0);
}