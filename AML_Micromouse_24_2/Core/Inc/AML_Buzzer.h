#ifndef AML_BUZZER_H
#define AML_BUZZER_H


#include "stm32h7xx_hal.h"
#include "main.h"

void AML_Buzzer_TurnOn(void);
void AML_Buzzer_TurnOff(void);
int AML_Buzzer_PlaySong(void);
void AML_Buzzer_PlayNote(float frequency, uint16_t duration);
void AML_Buzzer_Beep(void);

#endif
