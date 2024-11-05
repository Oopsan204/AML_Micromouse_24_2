#include <AML_MotorControl.h>

extern TIM_HandleTypeDef htim1; // timer for pwm
GPIO_TypeDef *MotorDirectionPort = GPIOE;

// TimerClock is 240MHz, Prescaler is 12000, AutoReload is 1, so the frequency is 10kHz
#define TO_CCR(x) (uint16_t)((x) * 10)

// PID struct-------------------------------------------------------------------------------------------------------//
AML_PID_Struct PID_TurnLeft;
double PID_TurnLeft_Kp = 1;
double PID_TurnLeft_Ki = 0;
double PID_TurnLeft_Kd = 0;
double PID_TurnLeft_Tau = 0;
double PID_TurnLeft_Input, PID_TurnLeft_Output, PID_TurnLeft_Setpoint = 0;

AML_PID_Struct PID_TurnRight;
double PID_TurnRight_Kp = 1;
double PID_TurnRight_Ki = 0;
double PID_TurnRight_Kd = 0;
double PID_TurnRight_Tau = 0;
double PID_TurnRight_Input, PID_TurnRight_Output, PID_TurnRight_Setpoint = 0;

AML_PID_Struct PID_MPUFollow;
double PID_MPUFollow_Kp = 1;
double PID_MPUFollow_Ki = 0;
double PID_MPUFollow_Kd = 0;
double PID_MPUFollow_Tau = 0;
double PID_MPUFollow_Input, PID_MPUFollow_Output, PID_MPUFollow_Setpoint = 0;

AML_PID_Struct PID_LeftWallFollow;
double PID_LeftWallFollow_Kp = 1;
double PID_LeftWallFollow_Ki = 0;
double PID_LeftWallFollow_Kd = 0;
double PID_LeftWallFollow_Tau = 0;
double PID_LeftWallFollow_Input, PID_LeftWallFollow_Output, PID_LeftWallFollow_Setpoint = 0;

AML_PID_Struct PID_RightWallFollow;
double PID_RightWallFollow_Kp = 1;
double PID_RightWallFollow_Ki = 0;
double PID_RightWallFollow_Kd = 0;
double PID_RightWallFollow_Tau = 0;
double PID_RightWallFollow_Input, PID_RightWallFollow_Output, PID_RightWallFollow_Setpoint = 0;

// FUNCTION-----------------------------------------------------------------------------------------------------------------//
void AML_MotorControl_AMLPIDSetup(void);
void AML_MotorControl_Setup(void);
void AML_MotorControl_LeftPWM(int32_t DutyCycle);
void AML_MotorControl_RightPWM(int32_t DutyCycle);
void AML_MotorControl_Move(int32_t LeftDutyCycle, int32_t RightDutyCycle);
void AML_MotorControl_Stop(void);

// PID setup function-------------------------------------------------------------------------------------------------------//
void AML_MotorControl_AMLPIDSetup(void)
{
    AML_PID_Init(&PID_TurnLeft, &PID_TurnLeft_Input, &PID_TurnLeft_Output, &PID_TurnLeft_Setpoint, PID_TurnLeft_Kp, PID_TurnLeft_Ki, PID_TurnLeft_Kd, PID_TurnLeft_Tau, PIDOutputMin, PIDOutputMax, -PIDOutputMax, PIDOutputMax, SampleTime);
    AML_PID_Init(&PID_TurnRight, &PID_TurnRight_Input, &PID_TurnRight_Output, &PID_TurnRight_Setpoint, PID_TurnRight_Kp, PID_TurnRight_Ki, PID_TurnRight_Kd, PID_TurnRight_Tau, PIDOutputMin, PIDOutputMax, -PIDOutputMax, PIDOutputMax, SampleTime);

    AML_PID_Init(&PID_MPUFollow, &PID_MPUFollow_Input, &PID_MPUFollow_Output, &PID_MPUFollow_Setpoint, PID_MPUFollow_Kp, PID_MPUFollow_Ki, PID_MPUFollow_Kd, PID_MPUFollow_Tau, PIDOutputMin, PIDOutputMax, -PIDOutputMax, PIDOutputMax, SampleTime);
    AML_PID_Init(&PID_LeftWallFollow, &PID_LeftWallFollow_Input, &PID_LeftWallFollow_Output, &PID_LeftWallFollow_Setpoint, PID_LeftWallFollow_Kp, PID_LeftWallFollow_Ki, PID_LeftWallFollow_Kd, PID_LeftWallFollow_Tau, PIDOutputMin, PIDOutputMax, -PIDOutputMax, PIDOutputMax, SampleTime);
    AML_PID_Init(&PID_RightWallFollow, &PID_RightWallFollow_Input, &PID_RightWallFollow_Output, &PID_RightWallFollow_Setpoint, PID_RightWallFollow_Kp, PID_RightWallFollow_Ki, PID_RightWallFollow_Kd, PID_RightWallFollow_Tau, PIDOutputMin, PIDOutputMax, -PIDOutputMax, PIDOutputMax, SampleTime);
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
        HAL_GPIO_WritePin(MotorDirectionPort, LEFT_MOTOR_IN1_Pin, LeftMotorDirection);
        HAL_GPIO_WritePin(MotorDirectionPort, LEFT_MOTOR_IN2_Pin, !LeftMotorDirection);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TO_CCR(DutyCycle));
    }
    else if (DutyCycle < 0)
    {
        HAL_GPIO_WritePin(MotorDirectionPort, LEFT_MOTOR_IN1_Pin, !LeftMotorDirection);
        HAL_GPIO_WritePin(MotorDirectionPort, LEFT_MOTOR_IN2_Pin, LeftMotorDirection);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TO_CCR(-DutyCycle));
    }
    else if (DutyCycle == 0)
    {
        HAL_GPIO_WritePin(MotorDirectionPort, LEFT_MOTOR_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MotorDirectionPort, LEFT_MOTOR_IN2_Pin, GPIO_PIN_RESET);
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
        HAL_GPIO_WritePin(MotorDirectionPort, RIGHT_MOTOR_IN1_Pin, RightMotorDirection);
        HAL_GPIO_WritePin(MotorDirectionPort, RIGHT_MOTOR_IN2_Pin, !RightMotorDirection);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, TO_CCR(DutyCycle));
    }
    else if (DutyCycle < 0)
    {
        HAL_GPIO_WritePin(MotorDirectionPort, RIGHT_MOTOR_IN1_Pin, !RightMotorDirection);
        HAL_GPIO_WritePin(MotorDirectionPort, RIGHT_MOTOR_IN2_Pin, RightMotorDirection);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, TO_CCR(-DutyCycle));
    }
    else if (DutyCycle == 0)
    {
        HAL_GPIO_WritePin(MotorDirectionPort, RIGHT_MOTOR_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MotorDirectionPort, RIGHT_MOTOR_IN2_Pin, GPIO_PIN_RESET);
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
    AML_MotorControl_Move(0, 0);
}


