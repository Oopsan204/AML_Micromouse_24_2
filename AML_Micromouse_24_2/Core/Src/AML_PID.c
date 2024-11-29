#include "AML_PID.h"

//-------------------------------------------------------------------------------------------------------//

void AML_PID_Init(AML_PID_Struct *pid, double *input, double *output, double *setpoint, double kp, double ki, double kd, double tau, double limMin, double limMax, double linMinInt, double linMaxInt, uint32_t sampleTime);

double AML_PID_Compute(AML_PID_Struct *pid);

//-------------------------------------------------------------------------------------------------------//

void AML_PID_Init(AML_PID_Struct *pid, double *input, double *output, double *setpoint, double kp, double ki, double kd, double tau, double limMin, double limMax, double linMinInt, double linMaxInt, uint32_t sampleTime)
{
    pid->MyInput = input;
    pid->MyOutput = output;
    pid->MySetpoint = setpoint;

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->tau = tau;

    pid->limMin = limMin;
    pid->limMax = limMax;

    pid->sampleTime = sampleTime;

    pid->integratol = 0;
    pid->prevError = 0;

    pid->differentiator = 0;
    pid->prevMeasurement = 0;
}

double AML_PID_Compute(AML_PID_Struct *pid)
{
    uint32_t now = HAL_GetTick();
    uint32_t timeChange = (now - pid->lastTime);

    if (timeChange >= pid->sampleTime)
    {
        // Compute PID output value for given reference input and feedback

        double error = *pid->MySetpoint - *pid->MyInput;

        double pTerm = pid->Kp * error;

        pid->integratol += error * pid->sampleTime;

        pid->integratol += 0.5f * pid->Ki * pid->sampleTime * (error + pid->prevError);

        if (pid->integratol > pid->linMaxInt)
        {
            pid->integratol = pid->linMaxInt;
        }
        else if (pid->integratol < pid->linMinInt)
        {
            pid->integratol = pid->linMinInt;
        }

        double iTerm = pid->Ki * pid->integratol;

        pid->differentiator = -(2.0f * pid->Kd * (*pid->MyInput - pid->prevMeasurement) + (2.0f * pid->tau - pid->sampleTime) * pid->differentiator) / (2.0f * pid->tau + pid->sampleTime);

        double dTerm = pid->Kd * pid->differentiator;

        *pid->MyOutput = pTerm + iTerm + dTerm;

        if (*pid->MyOutput > pid->limMax)
        {
            *pid->MyOutput = pid->limMax;
        }
        else if (*pid->MyOutput < pid->limMin)
        {
            *pid->MyOutput = pid->limMin;
        }

        pid->prevMeasurement = *pid->MyInput;
        pid->prevError = error;

        // Remember last time for next calculation
        pid->lastTime = now;
    }

    pid->out = *pid->MyOutput;
    return *pid->MyOutput;
}

void AML_PID_SetOutputLimits(AML_PID_Struct *pid, double min, double max)
{
    if (min >= max)
    {
        return;
    }

    pid->limMin = min;
    pid->limMax = max;
}

void AML_PID_SetSampleTime(AML_PID_Struct *pid, uint32_t newSampleTime)
{
    if (newSampleTime > 0)
    {
        pid->sampleTime = newSampleTime;
    }
}

void AML_PID_SetTunings(AML_PID_Struct *pid, double kp, double ki, double kd, double tau)
{
    if (kp < 0 || ki < 0 || kd < 0 || tau < 0)
    {
        return;
    }

    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->tau = tau;
}
