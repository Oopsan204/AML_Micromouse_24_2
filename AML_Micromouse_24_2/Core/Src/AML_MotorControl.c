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
#define LOW_PASS_FILTER_ALPHA 0.025
#define LOW_PASS_FILTER(x, y) ((x) * LOW_PASS_FILTER_ALPHA + (y) * (1 - LOW_PASS_FILTER_ALPHA))

// currentfishin

bool CalibFlag = true;
double TICKS_ONE_CELL = 380;

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
        .Kp = 2.5,
        .Ki = 0.5,
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
        .Kp = 2.5,
        .Ki = 0.5,
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
        .Kp = 2.5,
        .Ki = 0.5,
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

AML_PID_Struct PID_LeftWallFollow =
    {
        .Kp = 0.7,
        .Ki = 0.15,
        .Kd = 0.2,
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
        .Kp = 0.7,
        .Ki = 0.15,
        .Kd = 0.2,
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
AML_PID_Struct PID_GoStraight =
    {
        .Kp = 1,
        .Ki = 0,
        .Kd = 0,
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
void AML_MotorControl_GoStrastWithIR(void);
void AML_MotorControl_TurnOnWallFollow(void);
void AML_MotorControl_TurnOffWallFollow(void);

void AML_MotorControl_UpdateTempAngle(double angle);
void AML_MotorControl_TurnLeft(void);
void AML_MotorControl_TurnRight(void);

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
        // AML_MotorControl_RightWallFollow();
        // AML_MotorControl_GoStraghtWithMPU(AML_MPUSensor_GetAngle() + PID_RightWallFollow.Output);
        AML_MotorControl_GoStraight();
        // AML_MotorControl_LeftMotorSpeed(10);
        // AML_MotorControl_FontWallFollow();
    }
}

void AML_MotorControl_TurnOnWallFollow(void)
{
    // AML_MPUSensor_ResetAngle();
    HAL_TIM_Base_Start_IT(&htim7);
}

void AML_MotorControl_TurnOffWallFollow(void)
{
    HAL_TIM_Base_Stop_IT(&htim7);
    AML_LedDebug_SetAllLED(GPIO_PIN_RESET);
    AML_LedDebug_TurnOffLED(N_L);
    AML_LedDebug_TurnOffLED(N_R);
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
    TempSetPoint += angle;
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
        HAL_GPIO_WritePin(MotorDirectionBPort, BIN1_Pin, LeftMotorDirection);
        HAL_GPIO_WritePin(MotorDirectionBPort, BIN2_Pin, !LeftMotorDirection);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, TO_CCR(DutyCycle));
    }
    else if (DutyCycle < 0)
    {
        HAL_GPIO_WritePin(MotorDirectionBPort, BIN1_Pin, !LeftMotorDirection);
        HAL_GPIO_WritePin(MotorDirectionBPort, BIN2_Pin, LeftMotorDirection);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, TO_CCR(-DutyCycle));
    }
    else if (DutyCycle == 0)
    {
        HAL_GPIO_WritePin(MotorDirectionBPort, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MotorDirectionBPort, BIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
}

void AML_MotorControl_RightPWM(int32_t DutyCycle)
{
    // DutyCycle = (int32_t)(DutyCycle * 1);

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
        HAL_GPIO_WritePin(MotorDirectionAPort, AIN1_Pin, RightMotorDirection);
        HAL_GPIO_WritePin(MotorDirectionAPort, AIN2_Pin, !RightMotorDirection);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TO_CCR(DutyCycle));
    }
    else if (DutyCycle < 0)
    {
        HAL_GPIO_WritePin(MotorDirectionAPort, AIN1_Pin, !RightMotorDirection);
        HAL_GPIO_WritePin(MotorDirectionAPort, AIN2_Pin, RightMotorDirection);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TO_CCR(-DutyCycle));
    }
    else if (DutyCycle == 0)
    {
        HAL_GPIO_WritePin(MotorDirectionAPort, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MotorDirectionAPort, AIN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    }
}

void AML_MotorControl_Move(int32_t LeftDutyCycle, int32_t RightDutyCycle)
{
    AML_MotorControl_LeftPWM(LeftDutyCycle);
    AML_MotorControl_RightPWM(RightDutyCycle);
}

void AML_MotorControl_Stop(void)
{
    AML_MotorControl_Move(-25, -25);
    HAL_Delay(50);
    AML_MotorControl_Move(0, 0);
}

void AML_MotorControl_LeftMotorSpeed(int32_t rpm)
{
    PID_LeftMotor.Setpoint = (double)rpm;

    int32_t CurrentLeftEncoder = AML_Encoder_GetLeftValue();
    PID_LeftMotor.Input = PULSE_TO_RPM(CurrentLeftEncoder);
    // PreviousLeftEncoder = CurrentLeftEncoder;
    AML_Encoder_ResetLeftValue();

    AML_PID_Compute(&PID_LeftMotor);

    AML_MotorControl_LeftPWM((int32_t)PID_LeftMotor.Output);
}

void AML_MotorControl_RightMotorSpeed(int32_t rpm)
{
    PID_RightMotor.Setpoint = (double)rpm;

    int32_t CurrentRightEncoder = AML_Encoder_GetRightValue();

    PreviousRightEncoder = CurrentRightEncoder;

    PID_RightMotor.Input = PULSE_TO_RPM(CurrentRightEncoder);

    AML_PID_Compute(&PID_RightMotor);

    AML_MotorControl_RightPWM((int32_t)PID_RightMotor.Output);
}
void AML_MotorControl_GoStraghtWithMPU(double setpoint)
{
    PID_MPUFollow.Input = AML_MPUSensor_GetAngle();
    PID_MPUFollow.Setpoint = setpoint;
    AML_PID_Compute(&PID_MPUFollow);
    AML_MotorControl_Move(MouseSpeed - (int32_t)PID_MPUFollow.Output, MouseSpeed + (int32_t)PID_MPUFollow.Output);
}
void AML_MotorControl_LeftWallFollow(void)
{
    PID_LeftWallFollow.Input = AML_IRSensor_GetDistance(IR_SENSOR_FL);
    PID_LeftWallFollow.Setpoint = 85;

    AML_PID_Compute(&PID_LeftWallFollow);

    int32_t leftSpeed = MouseSpeed + (int32_t)PID_LeftWallFollow.Output;
    int32_t rightSpeed = MouseSpeed - (int32_t)PID_LeftWallFollow.Output;
    AML_MotorControl_Move(leftSpeed, rightSpeed);
}

void AML_MotorControl_RightWallFollow(void)
{
    PID_RightWallFollow.Input = AML_IRSensor_GetDistance(IR_SENSOR_FR);
    PID_RightWallFollow.Setpoint = 85;

    AML_PID_Compute(&PID_RightWallFollow);
    int32_t leftSpeed = MouseSpeed - (int32_t)PID_RightWallFollow.Output;
    int32_t rightSpeed = MouseSpeed + (int32_t)PID_RightWallFollow.Output;
    AML_MotorControl_Move(leftSpeed, rightSpeed);
}

void AML_MotorControl_GoStraight(void)
{
    if (AML_IRSensor_GetDistance(IR_SENSOR_FL) <= 100)
    {
        AML_MotorControl_LeftWallFollow();
        // AML_MotorControl_GoStrastWithIR();
        AML_LedDebug_TurnOnLED(N_L);

        // TempSetPoint = -PID_LeftWallFollow.Output;

        // AML_MotorControl_GoStraghtWithMPU(AML_MPUSensor_GetAngle() - PID_LeftWallFollow.Output);
    }
    else if (AML_IRSensor_GetDistance(IR_SENSOR_FR) <= 100)
    {
        AML_MotorControl_RightWallFollow();
        AML_LedDebug_TurnOnLED(N_R);

        // TempSetPoint = PID_RightWallFollow.Output;

        // AML_MotorControl_GoStraghtWithMPU(AML_MPUSensor_GetAngle() + PID_RightWallFollow.Output);
    }
    else
    {
        AML_LedDebug_SetAllLED(GPIO_PIN_SET);
        AML_MotorControl_GoStraghtWithMPU(AML_MPUSensor_GetAngle());
    }
}

//--------------------------------------------------------------------------------------------------------//

void AML_MotorControl_TurnLeft(void)
{

    // Perform calibration
    bool WallCalibFlag = false;
    if (AML_IRSensor_IsRightWall())
    {
        WallCalibFlag = true;
        TICKS_ONE_CELL = 380;
    }

    uint16_t WaitingTime = 1500;

    PID_TurnLeft.Setpoint = TempSetPoint + TuneLeft90Angle;

    uint32_t InitTime = HAL_GetTick();
    uint32_t CurrentTime = HAL_GetTick();
    uint32_t PreviousTime = CurrentTime;

    while ((CurrentTime - PreviousTime) < 350 && (HAL_GetTick() - InitTime < WaitingTime))
    {
        PID_TurnLeft.Input = AML_MPUSensor_GetAngle();

        AML_PID_Compute(&PID_TurnLeft);
        AML_MotorControl_LeftPWM(-(int32_t)PID_TurnLeft.Output);
        AML_MotorControl_RightPWM((int32_t)PID_TurnLeft.Output);

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

    AML_MotorControl_UpdateTempAngle(+TuneLeft90Angle);

    if (WallCalibFlag == true)
    {
        AML_MotorControl_Move(0, 0);
        HAL_Delay(10);
        AML_MotorControl_Move(-50, -50);
        uint32_t startTime = HAL_GetTick();
        while (HAL_GetTick() - startTime < 1000)
        {
            // Do nothing, just wait
        }
        AML_MPUSensor_ResetAngle();
        TempSetPoint = 0;
        AML_MotorControl_Move(0, 0);
        WallCalibFlag = false;
    }
    else
    {
        AML_MotorControl_Move(0, 0);
        TICKS_ONE_CELL = 340;
    }
}

void AML_MotorControl_TurnRight(void)
{

    // Perform calibration
    bool WallCalibFlag = false;
    if (AML_IRSensor_IsLeftWall())
    {
        WallCalibFlag = true;
        TICKS_ONE_CELL = 380;
    }
    uint16_t WaitingTime = 1500;

    PID_TurnRight.Setpoint = TempSetPoint - TuneRight90Angle;

    uint32_t InitTime = HAL_GetTick();
    uint32_t CurrentTime = HAL_GetTick();
    uint32_t PreviousTime = CurrentTime;

    while ((CurrentTime - PreviousTime) < 350 && (HAL_GetTick() - InitTime < WaitingTime))
    {
        PID_TurnRight.Input = AML_MPUSensor_GetAngle();

        AML_PID_Compute(&PID_TurnRight);

        AML_MotorControl_LeftPWM(-(int32_t)PID_TurnRight.Output);
        AML_MotorControl_RightPWM((int32_t)PID_TurnRight.Output);

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

    AML_MotorControl_UpdateTempAngle(-TuneRight90Angle);

    if (WallCalibFlag == true)
    {
        AML_MotorControl_Move(0, 0);
        HAL_Delay(10);
        AML_MotorControl_Move(-50, -50);
        uint32_t startTime = HAL_GetTick();
        while (HAL_GetTick() - startTime < 1000)
        {
            // Do nothing, just wait
        }
        AML_MPUSensor_ResetAngle();
        TempSetPoint = 0;
        AML_MotorControl_Move(0, 0);
        WallCalibFlag = false;
    }
    else
    {
        TICKS_ONE_CELL = 340;
        AML_MotorControl_Move(0, 0);
    }
}

void AML_MotorControl_GoStrastWithIR(void)
{
    uint16_t WaitingTime = 700;

    uint32_t InitTime = HAL_GetTick();
    uint32_t CurrentTime = HAL_GetTick();
    uint32_t PreviousTime = CurrentTime;

    while ((CurrentTime - PreviousTime) < 450 && (HAL_GetTick() - InitTime < WaitingTime))
    {
        PID_GoStraight.Input = AML_IRSensor_GetDistance(IR_SENSOR_R) - AML_IRSensor_GetDistance(IR_SENSOR_L);

        AML_PID_Compute(&PID_GoStraight);

        AML_MotorControl_Move(-(int32_t)PID_GoStraight.Output, (int32_t)PID_GoStraight.Output);

        if (ABS(PID_GoStraight.Input) < 3)
        {
            CurrentTime = HAL_GetTick();
        }
        else
        {
            CurrentTime = HAL_GetTick();
            PreviousTime = CurrentTime;
        }
    }
}

void AML_MotorControl_MoveForwardOneCell(void)
{
    int32_t CurrentLeftEncoder = AML_Encoder_GetLeftValue();

    AML_MotorControl_TurnOnWallFollow();

    while (AML_Encoder_GetLeftValue() - CurrentLeftEncoder < TICKS_ONE_CELL)
    {
        debug[3] = AML_Encoder_GetLeftValue();
        debug[4] = AML_Encoder_GetRightValue();
    }
    AML_MotorControl_TurnOffWallFollow();
    AML_MotorControl_Stop();
    AML_Buzzer_Beep();
    TICKS_ONE_CELL = 340;
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
