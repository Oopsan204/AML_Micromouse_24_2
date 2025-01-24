#include <AML_MotorControl.h>

extern TIM_HandleTypeDef htim1; // timer for pwm
extern TIM_HandleTypeDef htim7; // timer for interrupt

GPIO_TypeDef *MotorDirectionAPort = GPIOB;
GPIO_TypeDef *MotorDirectionBPort = GPIOD;

extern debug[100];
// TimerClock is 240MHz, Prescaler is 12000, AutoReload is 1, so the frequency is 10kHz
#define TO_CCR(x) (uint16_t)((x) * 10)
#define PULSE_TO_RPM(x) (double)(x / EncoderPulsePerRound) * ((60 * 1000) / SampleTime) // convert pulse to rpm

#define DISTANCE_TO_PULSE(x) (int32_t)(((x) * EncoderPulsePerRound) / (WheelDiameter * Pi)) // convert distance to pulse
// low pass fillter for PID input
#define LOW_PASS_FILTER_ALPHA 0.1
#define LOW_PASS_FILTER(x, y) ((x) * LOW_PASS_FILTER_ALPHA + (y) * (1 - LOW_PASS_FILTER_ALPHA))

// PID struct-------------------------------------------------------------------------------------------------------//

double TempSetPoint = 0;
int32_t PreviousLeftEncoder = 0;
int32_t PreviousLeftFilter = 0;

int32_t PreviousRightEncoder = 0;
int32_t PreviousRightFilter = 0;

AML_PID_Struct PID_LeftMotor =
    {
        .Kp = 1,
        .Ki = 0,
        .Kd = 0,
        .tau = 0,
        .limMin = 8,
        .limMax = 50,
        .linMinInt = 0,
        .linMaxInt = 15,
        .sampleTime = 5,
        .lastTime = 0,
        .integratol = 0,
        .prevError = 0,
        .differentiator = 0,
        .prevMeasurement = 0,
        .Input = 0,
        .Output = 0,
        .Setpoint = 0,
};

AML_PID_Struct PID_RightMotor =
    {
        .Kp = 1,
        .Ki = 0,
        .Kd = 0,
        .tau = 0,
        .limMin = 0,
        .limMax = 50,
        .linMinInt = 0,
        .linMaxInt = 15,
        .sampleTime = 5,
        .lastTime = 0,
        .integratol = 0,
        .prevError = 0,
        .differentiator = 0,
        .prevMeasurement = 0,
        .Input = 0,
        .Output = 0,
        .Setpoint = 0,
};

AML_PID_Struct PID_TurnLeft =
    {
        .Kp = 1.15,
        .Ki = 0.8,
        .Kd = 0.5,
        .tau = 0,
        .limMin = -MouseTurnSpeed,
        .limMax = MouseTurnSpeed,
        .linMinInt = -20,
        .linMaxInt = 20,
        .sampleTime = SampleTime,
        .lastTime = 0,
        .integratol = 0,
        .prevError = 0,
        .differentiator = 0,
        .prevMeasurement = 0,
        .Input = 0,
        .Output = 0,
        .Setpoint = 0,
};

AML_PID_Struct PID_TurnRight =
    {
        .Kp = 1.15,
        .Ki = 0.8,
        .Kd = 0.5,
        .tau = 0,
        .limMin = -MouseTurnSpeed,
        .limMax = MouseTurnSpeed,
        .linMinInt = -20,
        .linMaxInt = 20,
        .sampleTime = SampleTime,
        .lastTime = 0,
        .integratol = 0,
        .prevError = 0,
        .differentiator = 0,
        .prevMeasurement = 0,
        .Input = 0,
        .Output = 0,
        .Setpoint = 0,
};

AML_PID_Struct PID_MPUFollow =
    {
        .Kp = 2,
        .Ki = 1,
        .Kd = 0.01,
        .tau = 0,
        .limMin = -MouseSpeed,
        .limMax = MouseSpeed,
        .linMinInt = -15,
        .linMaxInt = 15,
        .sampleTime = SampleTime,
        .lastTime = 0,
        .integratol = 0,
        .prevError = 0,
        .differentiator = 0,
        .prevMeasurement = 0,
        .Input = 0,
        .Output = 0,
        .Setpoint = 0,
};

AML_PID_Struct PID_LeftWallFollow =
    {
        .Kp = 0.3,
        .Ki = 0.1,
        .Kd = 0.1,
        .tau = 0,
        .limMin = -MouseSpeed,
        .limMax = MouseSpeed,
        .linMinInt = -15,
        .linMaxInt = 15,
        .sampleTime = SampleTime,
        .lastTime = 0,
        .integratol = 0,
        .prevError = 0,
        .differentiator = 0,
        .prevMeasurement = 0,
        .Input = 0,
        .Output = 0,
        .Setpoint = 0,
};

AML_PID_Struct PID_RightWallFollow =
    {
        .Kp = 0.4,
        .Ki = 0.04,
        .Kd = 0.08,
        .tau = 0,
        .limMin = -MouseSpeed,
        .limMax = MouseSpeed,
        .linMinInt = -15,
        .linMaxInt = 15,
        .sampleTime = SampleTime,
        .lastTime = 0,
        .integratol = 0,
        .prevError = 0,
        .differentiator = 0,
        .prevMeasurement = 0,
        .Input = 0,
        .Output = 0,
        .Setpoint = 0,
};

// FUNCTION-----------------------------------------------------------------------------------------------------------------//
void AML_MotorControl_AMLPIDSetup(void);
void AML_MotorControl_Setup(void);
void AML_MotorControl_LeftPWM(int32_t DutyCycle);
void AML_MotorControl_RightPWM(int32_t DutyCycle);
void AML_MotorControl_Move(int32_t LeftDutyCycle, int32_t RightDutyCycle);
void AML_MotorControl_Stop(void);

void AML_MotorControl_GoStraghtWithMPU(double setpoint);
void AML_MotorControl_GoStraight(void);
void AML_MotorControl_TurnOnWallFollow(void);
void AML_MotorControl_TurnOffWallFollow(void);

void AML_MotorControl_LeftWallFollow(void);
void AML_MotorControl_RightWallFollow(void);

void AML_MotorSpeed_Right(int32_t rps);
void AML_MotorSpeed_Left(int32_t rps);

void AML_MotorControl_MoveForwardOneCell(void);
void AML_MotorControl_MoveForwardDistance(int32_t distance);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    UNUSED(htim);
    if (htim->Instance == htim7.Instance) // timer for wall follow
    {
        // AML_MotorControl_GoStraghtWithMPU(0);
        AML_MotorControl_RightWallFollow();
        AML_MotorControl_GoStraghtWithMPU(TempSetPoint+PID_RightWallFollow.Output);
        // AML_MotorControl_LeftMotorSpeed(10);
    }
}

void AML_MotorControl_TurnOnWallFollow(void)
{
    AML_MPUSensor_ResetAngle();
    HAL_TIM_Base_Start_IT(&htim7);
}

void AML_MotorControl_TurnOffWallFollow(void)
{
    HAL_TIM_Base_Stop_IT(&htim7);
}
// PID setup function-------------------------------------------------------------------------------------------------------//
void AML_MotorControl_AMLPIDSetup(void)
{
}

// Motor control function-------------------------------------------------------------------------------------------------------//

// init motor control
void AML_MotorControl_Setup(void)
{
    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    AML_MotorControl_AMLPIDSetup();
}

void AML_MotorControl_UpdateTempAngle(double angle)
{
    TempSetPoint =+ angle;
}
void AML_MotorControl_LeftPWM(int32_t DutyCycle)
{
    if (DutyCycle > 100)
    {
        DutyCycle = 100;
    }
    else if (DutyCycle < -100)
    {
        DutyCycle = -100;
    }

    if (DutyCycle > 0)
    {
        HAL_GPIO_WritePin(MotorDirectionAPort, AIN1_Pin, !LeftMotorDirection);
        HAL_GPIO_WritePin(MotorDirectionAPort, AIN2_Pin, LeftMotorDirection);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TO_CCR(DutyCycle));
    }
    else if (DutyCycle < 0)
    {
        HAL_GPIO_WritePin(MotorDirectionAPort, AIN1_Pin, LeftMotorDirection);
        HAL_GPIO_WritePin(MotorDirectionAPort, AIN2_Pin, !LeftMotorDirection);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TO_CCR(-DutyCycle));
    }
    else if (DutyCycle == 0)
    {
        HAL_GPIO_WritePin(MotorDirectionAPort, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MotorDirectionAPort, AIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    }
}

void AML_MotorControl_RightPWM(int32_t DutyCycle)
{
    DutyCycle = (int32_t)(DutyCycle * 1);

    if (DutyCycle > 100)
    {
        DutyCycle = 100;
    }
    else if (DutyCycle < -100)
    {
        DutyCycle = -100;
    }

    if (DutyCycle > 0)
    {
        HAL_GPIO_WritePin(MotorDirectionBPort, BIN1_Pin, !RightMotorDirection);
        HAL_GPIO_WritePin(MotorDirectionBPort, BIN2_Pin, RightMotorDirection);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, TO_CCR(DutyCycle));
    }
    else if (DutyCycle < 0)
    {
        HAL_GPIO_WritePin(MotorDirectionBPort, BIN1_Pin, RightMotorDirection);
        HAL_GPIO_WritePin(MotorDirectionBPort, BIN2_Pin, !RightMotorDirection);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, TO_CCR(-DutyCycle));
    }
    else if (DutyCycle == 0)
    {
        HAL_GPIO_WritePin(MotorDirectionBPort, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MotorDirectionBPort, BIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
}

void AML_MotorControl_Move(int32_t LeftDutyCycle, int32_t RightDutyCycle)
{
    AML_MotorControl_LeftPWM(LeftDutyCycle);
    AML_MotorControl_RightPWM(RightDutyCycle);
}

void AML_MotorControl_Stop(void)
{
    AML_MotorControl_Move(-10,-10);
    HAL_Delay(50);
    AML_MotorControl_Move(0, 0);
}

void AM_MotorSpeed_Left(int32_t rps)
{
    PID_LeftMotor.Setpoint = (double)rps;
    PID_LeftMotor.Input = (double)AML_Encoder_GetLeftValue() / EncoderPulsePerRound * 50; //
    AML_Encoder_ResetLeftValue();
    AML_PID_Compute(&PID_LeftMotor);
    AML_MotorControl_RightPWM((int32_t)PID_RightMotor.Output);
}
void AML_Motorspeed_Right(int32_t rps)
{
    PID_RightMotor.Setpoint = (double)rps;
    PID_RightMotor.Input = (double)AML_Encoder_GetRightValue() / EncoderPulsePerRound * 50; //
    AML_Encoder_ResetRightValue();
    AML_PID_Compute(&PID_RightMotor);
    AML_MotorControl_LeftPWM((int32_t)PID_LeftMotor.Output);
}
void AML_MotorControl_GoStraghtWithMPU(double setpoint)
{
    PID_MPUFollow.Input = AML_MPUSensor_GetAngle();
    PID_MPUFollow.Setpoint = setpoint;
    AML_PID_Compute(&PID_MPUFollow);
    AML_MotorControl_Move(MouseSpeed + (int32_t)PID_MPUFollow.Output, MouseSpeed - (int32_t)PID_MPUFollow.Output);
}
void AML_MotorControl_LeftWallFollow(void)
{
    PID_LeftWallFollow.Input = AML_IRSensor_GetDistance(IR_SENSOR_L);
    PID_LeftWallFollow.Setpoint = WALL_IN_LEFT;

    AML_PID_Compute(&PID_LeftWallFollow);
}

void AML_MotorControl_RightWallFollow(void)
{
    PID_RightWallFollow.Input = AML_IRSensor_GetDistance(IR_SENSOR_R);
    PID_RightWallFollow.Setpoint = WALL_IN_RIGHT;

    AML_PID_Compute(&PID_RightWallFollow);
}

void AML_MotorControl_GoStraight(void)
{
    if (AML_IRSensor_IsLeftWall())
    {
        AML_MotorControl_LeftWallFollow();

        // TempSetPoint = -PID_LeftWallFollow.Output;

        AML_MotorControl_GoStraghtWithMPU(TempSetPoint - PID_LeftWallFollow.Output);
    }
    else if (AML_IRSensor_IsRightWall())
    {
        AML_MotorControl_RightWallFollow();

        // TempSetPoint = PID_RightWallFollow.Output;

        AML_MotorControl_GoStraghtWithMPU(TempSetPoint + PID_RightWallFollow.Output);
    }
    else
    {
        AML_MotorControl_GoStraghtWithMPU(TempSetPoint);
    }
}

//--------------------------------------------------------------------------------------------------------//

void AML_MotorControl_TurnLeft(void)
{
    uint16_t WaitingTime = 1500;

    PID_TurnLeft.Setpoint = AML_MPUSensor_GetAngle() + TuneLeft90Angle;

    uint32_t InitTime = HAL_GetTick();
    uint32_t CurrentTime = HAL_GetTick();
    uint32_t PreviousTime = CurrentTime;

    while ((CurrentTime - PreviousTime) < 500 && (HAL_GetTick() - InitTime < WaitingTime))
    {
        PID_TurnLeft.Input = AML_MPUSensor_GetAngle();

        AML_PID_Compute(&PID_TurnLeft);

        AML_MotorControl_LeftPWM((int32_t)PID_TurnLeft.Output * 1.07);
        AML_MotorControl_RightPWM(-(int32_t)PID_TurnLeft.Output);

        if (ABS(PID_TurnLeft.Input - PID_TurnLeft.Setpoint) < ErrorAngle)
        {
            CurrentTime = HAL_GetTick();
        }
        else
        {
            CurrentTime = HAL_GetTick();
            PreviousTime = CurrentTime;
        }
    }

    AML_MotorControl_Stop();
    AML_MotorControl_UpdateTempAngle(TuneLeft90Angle);
}

void AML_MotorControl_TurnRight(void)
{
    uint16_t WaitingTime = 1500;

    PID_TurnRight.Setpoint = AML_MPUSensor_GetAngle() - TuneRight90Angle;

    uint32_t InitTime = HAL_GetTick();
    uint32_t CurrentTime = HAL_GetTick();
    uint32_t PreviousTime = CurrentTime;

    while ((CurrentTime - PreviousTime) < 500 && (HAL_GetTick() - InitTime < WaitingTime))
    {
        PID_TurnRight.Input = AML_MPUSensor_GetAngle();

        AML_PID_Compute(&PID_TurnRight);

        AML_MotorControl_LeftPWM((int32_t)PID_TurnRight.Output);
        AML_MotorControl_RightPWM(-(int32_t)PID_TurnRight.Output * 1.07);

        if (ABS(PID_TurnRight.Input - PID_TurnRight.Setpoint) < ErrorAngle)
        {
            CurrentTime = HAL_GetTick();
        }
        else
        {
            CurrentTime = HAL_GetTick();
            PreviousTime = CurrentTime;
        }
    }

    AML_MotorControl_Stop();
    AML_MotorControl_UpdateTempAngle(-TuneRight90Angle);
}

void AML_MotorControl_MoveForwardOneCell(void)
{
    int32_t CurrentLeftEncoder = AML_Encoder_GetLeftValue();

    AML_MotorControl_TurnOnWallFollow();

    while (AML_Encoder_GetLeftValue() - CurrentLeftEncoder < TICKS_ONE_CELL)
    {
        debug[3]= AML_Encoder_GetLeftValue();
        debug[4]= AML_Encoder_GetRightValue();
    }
    AML_MotorControl_TurnOffWallFollow();
    AML_Buzzer_Beep();
    AML_MotorControl_Stop();
}

void AML_MotorControl_MoveForwardDistance(int32_t distance)
{
    int32_t CurrentLeftEncoder = AML_Encoder_GetLeftValue();

    AML_MotorControl_TurnOnWallFollow();

    while (AML_Encoder_GetLeftValue() - CurrentLeftEncoder < DISTANCE_TO_PULSE(distance))
    {
    }

    AML_MotorControl_TurnOffWallFollow();
    AML_MotorControl_Stop();
}
