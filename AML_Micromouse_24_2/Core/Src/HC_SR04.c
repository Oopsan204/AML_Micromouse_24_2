/*
 * HC_SR04.c
 *
 *  Created on: Mar 23, 2022
 *      Author: Mr Ruler
 */

// #include "stm32h7xx_hal.h"
#include "HC_SR04.h"

extern TIM_HandleTypeDef htim8;

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;

#define TRIG_PIN GPIO_PIN_6
#define TRIG_PORT GPIOA


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}
			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}
			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false
			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim8, TIM_IT_CC4);
		}
	}
}
void HCSR04_Init (void)
{
	HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_4);
}
void delay(uint16_t time){
	__HAL_TIM_SET_COUNTER(&htim8, 0);
	while(__HAL_TIM_GET_COUNTER(&htim8) < time);
}

uint8_t HCSR04_GetDis (void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_CC4);
	return Distance;
}



