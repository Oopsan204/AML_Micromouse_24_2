#ifndef AML_ENCODER_H
#define AML_ENCODER_H

#include "stm32h743xx.h"
#include "main.h"

#include "AML_GlobalVariable.h"


void AML_Encoder_Setup(void);
int32_t AML_Encoder_GetLeftValue(void);
int32_t AML_Encoder_GetRightValue(void);
void AML_Encoder_ResetLeftValue(void);
void AML_Encoder_ResetRightValue(void);

#endif // AML_ENCODER_H
