#ifndef AML_MOTORCONTROL_H
#define AML_MOTORCONTROL_H

#include "stm32h7xx_hal.h"
#include "main.h"
#include "AML_PID.h"

#include "AML_GlobalVariable.h"
#include "AML_LedDebug.h"
#include "AML_Buzzer.h"
#include "AML_Switch.h"
#include "AML_MPUSensor.h"

void AML_MotorControl_AMLPIDSetup(void);
void AML_MotorControl_Setup(void);
void AML_MotorControl_LeftPWM(int32_t DutyCycle);
void AML_MotorControl_RightPWM(int32_t DutyCycle);
void AML_MotorControl_Move(int32_t LeftDutyCycle, int32_t RightDutyCycle);
void AML_MotorControl_Stop(void);

#endif // AML_MOTORCONTROL_H
