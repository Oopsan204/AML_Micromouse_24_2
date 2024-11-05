#ifndef AML_GLOBAL_H
#define AML_GLOBAL_H

#include "stm32h7xx_hal.h"

// DEFINE FOR PINOUT-------------------------------------------------------------------------------------------------------//

#define SW_0 0
#define SW_1 1

#define BIT_SW_0 0
#define BIT_SW_1 1
#define BIT_SW_2 2
#define BIT_SW_3 3
#define BIT_SW_4 4

#define LED_0 0
#define LED_1 1
#define LED_2 2
#define LED_3 3

// DEFINE FOR MOTOR CONTROL-------------------------------------------------------------------------------------------------------//
#define MotorDirection 0 // use for change direction of motor
#define LeftMotorDirection MotorDirection // use for change direction of left motor
#define RightMotorDirection !MotorDirection // use for change direction of right motor

// define parameter for transmission function
#define Pi 3.14159265359  // Pi number
#define WheelDiameter 21  // mm
#define TransmissionRatio 1     // ratio between wheel and encoder
#define EncoderPulsePerRound 720 // 720 pulse per round encoder
#define MouseSpeed 30 // % of duty cycle

// define parameter for PID control

#define SampleTime 20 // time per second
#define PIDMode 1 // automatic mode
#define PIDOutputMin -MouseSpeed // minimum output of PID (duty cycle)
#define PIDOutputMax MouseSpeed // maximum output of PID (duty cycle)

// DEFINE FOR -------------------------------------------------------------------------------------------------------//



//-------------------------------------------------------------------------------------------------------//

#endif
