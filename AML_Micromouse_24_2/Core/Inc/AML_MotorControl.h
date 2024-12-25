#ifndef AML_MOTORCONTROL_H
#define AML_MOTORCONTROL_H

#include "stm32h7xx_hal.h"
#include "main.h"
#include "AML_PID.h"
#include "AML_GlobalVariable.h"
#include "SenSor.h"
#include "AML_Buzzer.h"
#include "AML_MPUSensor.h"
#include "AML_Encoder.h"

void AML_MotorControl_AMLPIDSetup(void);
void AML_MotorControl_Setup(void);
void AML_MotorControl_LeftPWM(int32_t DutyCycle);
void AML_MotorControl_RightPWM(int32_t DutyCycle);
void AML_MotorControl_Move(int32_t LeftDutyCycle, int32_t RightDutyCycle);
void AML_MotorControl_Stop(void);

void AML_MotorControl_LeftMotorSpeed(int32_t rpm);

void AML_MotorControl_TurnOnWallFollow(void);
void AML_MotorControl_TurnOffWallFollow(void);
void AML_MotorControl_GoStraghtWithMPU(double setpoint);
void AML_MotorControl_GoStraight(void);

void AML_MotorControl_TurnLeft(void);
void AML_MotorControl_TurnRight(void);

void AML_MotorControl_MoveForwardOneCell(void);
void AML_MotorControl_MoveForwardDistance(int32_t distance);

#endif // AML_MOTORCONTROL_H
