#ifndef AML_PID_H
#define AML_PID_H

#include "stm32h7xx.h"
#include "main.h"

typedef struct
{
    uint32_t *MyInput;
    uint32_t *MyOutput;
    uint32_t *MySetpoint;

    double Kp;
    double Ki;
    double Kd;

    double tau;

    double limMin;
    double limMax;

    double linMinInt;
    double linMaxInt;

    uint32_t sampleTime;
    uint32_t lastTime;

    double integratol;
    double prevError;
    double differentiator;
    double prevMeasurement;

    double out;
} AML_PID_Struct;

void AML_PID_Init(AML_PID_Struct *pid, uint32_t *input, uint32_t *output, uint32_t *setpoint, double kp, double ki, double kd, double tau, double limMin, double limMax, double linMinInt, double linMaxInt, uint32_t sampleTime);
double AML_PID_Compute(AML_PID_Struct *pid);

#endif // AML_PID_H
