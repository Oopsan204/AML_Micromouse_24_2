#include "AML_Buzzer.h"

// Timer for PWM
extern TIM_HandleTypeDef htim4;

uint16_t ClockAfterPrescal = 60000;

// Define the frequencies and durations for "Journey to the West"
const float journey_frequencies[] = {
    392.00, 392.00, 587.33, 392.00, 392.00, 493.88, 392.00, 392.00,         // Line 1
    392.00, 440.00, 440.00, 440.00, 493.88, 440.00, 392.00, 587.33, 523.25, // Line 2
    523.25, 440.00, 392.00, 493.88, 440.00, 587.33, 392.00, 440.00, 587.33, // Line 3
    392.00, 523.25, 440.00, 392.00, 392.00, 440.00, 493.88, 587.33, 523.25  // Line 4
};

const uint16_t journey_durations[] = {
    500, 500, 500, 500, 250, 250, 500, 500,      // Line 1
    500, 250, 250, 250, 250, 250, 500, 500, 500, // Line 2
    500, 250, 250, 500, 500, 500, 250, 250, 500, // Line 3
    500, 500, 250, 250, 250, 250, 500, 500, 500  // Line 4
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

    // Start PWM
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

    // Wait for the note to play
    HAL_Delay(duration);

    // Turn off buzzer
    AML_Buzzer_TurnOff();
}

int AML_Buzzer_PlaySong(void)
{
    AML_Buzzer_TurnOn();

    for (uint16_t i = 0; i < sizeof(journey_frequencies) / sizeof(journey_frequencies[0]); i++)
    {
        AML_Buzzer_PlayNoteInSong(journey_frequencies[i], journey_durations[i]);
    }

    AML_Buzzer_TurnOff();
    return 0;
}

void AML_Buzzer_PlayNote(float frequency, uint16_t duration)
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

    // Start PWM
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

    // Wait for the note to play
    HAL_Delay(duration);

    // Turn off buzzer
    AML_Buzzer_TurnOff();
}

void AML_Buzzer_Beep()
{
    AML_Buzzer_PlayNote(3000, 100);
}

