#include "AML_Buzzer.h"

// Bộ định thời cho PWM
extern TIM_HandleTypeDef htim4;

uint16_t ClockAfterPrescal = 60000;

const float frequencies[] = {
    392.00, 392.00, 587.33, 392.00, 392.00, 493.88, 392.00, 392.00,         // Dòng 1
    392.00, 440.00, 440.00, 440.00, 493.88, 440.00, 392.00, 587.33, 523.25, // Dòng 2
    523.25, 440.00, 392.00, 493.88, 440.00, 587.33, 392.00, 440.00, 587.33, // Dòng 3
    392.00, 523.25, 440.00, 392.00, 392.00, 440.00, 493.88, 587.33, 523.25  // Dòng 4
};

const uint16_t durations[] = {
    500, 500, 500, 500, 250, 250, 500, 500,      // Dòng 1
    500, 250, 250, 250, 250, 250, 500, 500, 500, // Dòng 2
    500, 250, 250, 500, 500, 500, 250, 250, 500, // Dòng 3
    500, 500, 250, 250, 250, 250, 500, 500, 500  // Dòng 4
};

//-------------------------------------------------------------------------------------------------//
void AML_Buzzer_TurnOn(void);
void AML_Buzzer_TurnOff(void);
int AML_Buzzer_PlaySong(void);
void AML_Buzzer_PlayNote(float frequency, uint16_t duration);
void AML_Buzzer_PlayNoteInSong(float frequency, uint16_t duration);
void AML_Buzzer_Beep(void);

//-------------------------------------------------------------------------------------------------//

void AML_Buzzer_TurnOn(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

void AML_Buzzer_TurnOff(void)
{
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
}

int AML_Buzzer_PlaySong(void)
{
    AML_Buzzer_TurnOn();

    for (uint16_t i = 0; i < sizeof(frequencies) / sizeof(frequencies[0]); i++)
    {
        AML_Buzzer_PlayNoteInSong(frequencies[i], durations[i]);
    }

    AML_Buzzer_TurnOff();
}

void AML_Buzzer_PlayNote(float frequency, uint16_t duration)
{
    if (frequency > 10000)
    {
        frequency = 10000;
    }
    else if (frequency <  30)
    {
        frequency = 30;
    }

    // Compute the value of ARR register
    uint32_t ARR_value = (uint32_t)(ClockAfterPrescal / frequency) - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim4, ARR_value);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (ARR_value / 2)); // 50% duty cycle

    // Start PWM
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

    // Wait for the note to play
    HAL_Delay(duration);

    // Turn off buzzer
    // DO NOT REMOVE THIS LINE (IMPORTANT)
    // DO NOT REMOVE THIS LINE (IMPORTANT)
    // DO NOT REMOVE THIS LINE (IMPORTANT)
    // DO NOT REMOVE THIS LINE (IMPORTANT)
    // DO NOT REMOVE THIS LINE (IMPORTANT)
    AML_Buzzer_TurnOff(); // DO NOT REMOVE THIS LINE (IMPORTANT)
    // DO NOT REMOVE THIS LINE (IMPORTANT)
    // DO NOT REMOVE THIS LINE (IMPORTANT)
    // DO NOT REMOVE THIS LINE (IMPORTANT)
    // DO NOT REMOVE THIS LINE (IMPORTANT)
    // DO NOT REMOVE THIS LINE (IMPORTANT)
}

void AML_Buzzer_PlayNoteInSong(float frequency, uint16_t duration)
{
    if (frequency > 10000)
    {
        frequency = 10000;
    }
    else if (frequency < 30)
    {
        frequency = 30;
    }

    // Compute the value of ARR register
    uint32_t ARR_value = (uint32_t)(ClockAfterPrescal / frequency) - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim4, ARR_value);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (ARR_value / 2)); // 50% duty cycle

    // Wait for the note to play
    HAL_Delay(duration);
}

void AML_Buzzer_Beep()
{
    AML_Buzzer_PlayNote(3000, 100);
}

