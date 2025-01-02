#include "AML_PID.h"

//-------------------------------------------------------------------------------------------------------//

double AML_PID_Compute(AML_PID_Struct *pid);

//-------------------------------------------------------------------------------------------------------//

double AML_PID_Compute(AML_PID_Struct *pid)
{
    uint32_t now = HAL_GetTick();
    uint32_t timeChange = (now - pid->lastTime);

    if (timeChange >= pid->sampleTime)
    {
        // Compute PID output value for given reference input and feedback

        double error = pid->Setpoint - pid->Input;

        double pTerm = pid->Kp * error;

        pid->integratol += error * timeChange;

        pid->integratol += 0.5f * pid->Ki * timeChange * (error + pid->prevError);

        if (pid->integratol > pid->linMaxInt)
        {
            pid->integratol = pid->linMaxInt;
        }
        else if (pid->integratol < pid->linMinInt)
        {
            pid->integratol = pid->linMinInt;
        }

        double iTerm = pid->Ki * pid->integratol;

        pid->differentiator = -(2.0f * pid->Kd * (pid->Input - pid->prevMeasurement) + (2.0f * pid->tau - timeChange) * pid->differentiator) / (2.0f * pid->tau + timeChange);

        double dTerm = pid->Kd * pid->differentiator;

        pid->Output = pTerm + iTerm + dTerm;

        if (pid->Output > pid->limMax)
        {
            pid->Output = pid->limMax;
        }
        else if (pid->Output < pid->limMin)
        {
            pid->Output = pid->limMin;
        }

        pid->prevMeasurement = pid->Input;
        pid->prevError = error;

        // Remember last time for next calculation
        pid->lastTime = now;
    }

    return pid->Output;
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
