#ifndef AML_MOTORCONTROL_H
#define AML_MOTORCONTROL_H

#include "stm32h7xx_hal.h"
#include "main.h"
#include "AML_PID.h"
#include "AML_GlobalVariable.h"
#include "SenSor.h"
#include "AML_LedDebug.h"
#include "AML_Buzzer.h"
#include "AML_Switch.h"
#include "AML_MPUSensor.h"
#include "AML_Encoder.h"

void AML_MotorControl_AMLPIDSetup(void);
void AML_MotorControl_Setup(void);
void AML_MotorControl_LeftPWM(int32_t DutyCycle);
void AML_MotorControl_RightPWM(int32_t DutyCycle);
void AML_MotorControl_Move(int32_t LeftDutyCycle, int32_t RightDutyCycle);
void AML_MotorSpeed_Left(int32_t rps);
void AML_MotorSpeed_Right(int32_t rps);
void AML_MotorControl_GoStraghtWithMPU(double setpoint);
void AML_MotorControl_TurnOnWallFollow(void);
void AML_MotorControl_TurnOffWallFollow(void);

void AML_MotorControl_Stop(void);

#endif // AML_MOTORCONTROL_H
