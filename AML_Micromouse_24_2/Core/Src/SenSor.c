#include "SenSor.h"


/*
    distance low range: y = 33,9 + -69,5x + 62,3x^2 + -25,4x^3 + 3,83x^4
    distance high range: y = 12.08 * x^(-1.058)
*/

#define ADC_RESOLUTION_BIT 16
#define ADC_MAX (1 << ADC_RESOLUTION_BIT) // 2^14 = 16384
#define ADC_VREF 3.3

#define GET_VOLTAGE(adcValue) ((adcValue * ADC_VREF) / ADC_MAX)
#define GET_DISTANCE_2_15(voltage) (33.9 + (-69.5 * voltage) + (62.3 * pow(voltage, 2)) + (-25.4 * pow(voltage, 3)) + (3.83 * pow(voltage, 4))) * 10 // mm
#define GET_DISTANCE_4_30(voltage) (12.08 * pow(voltage, -1.058)) * 10                                                                               // mm

#define GET_DISTANCE(voltage, index) (index > 0) ? GET_DISTANCE_2_15(voltage) : GET_DISTANCE_4_30(voltage)
// low pass fillter for ADC input
#define LOW_PASS_FILTER_ALPHA 0.01
#define LOW_PASS_FILTER(x, y) ((x)*LOW_PASS_FILTER_ALPHA + (y) * (1 - LOW_PASS_FILTER_ALPHA))


extern debug[100];

extern ADC_HandleTypeDef hadc1;

double IRSensorADCValue[5];
double IRSensorDistanceValue[5];
double IRSensorDistanceValueFillter[5];
double IRSensorDistanceValueFillterPre[5];
int8_t ADCIndex = 0;


//-------------------------------------------------------------------------------------------------------//
void AML_IRSensor_Setup(void);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
double AML_IRSensor_GetDistance(uint8_t sensor);

bool AML_IRSensor_IsFrontWall(void);
bool AML_IRSensor_IsLeftWall(void);
bool AML_IRSensor_IsRightWall(void);

bool AML_IRSensor_IsNoFrontWall(void);
bool AML_IRSensor_IsNoLeftWall(void);
bool AML_IRSensor_IsNoRightWall(void);

void LedDebug(void);

//-------------------------------------------------------------------------------------------------------//

void AML_IRSensor_Setup(void)
{
    memset(IRSensorADCValue, 0, sizeof(IRSensorADCValue));
    memset(IRSensorDistanceValue, 0, sizeof(IRSensorDistanceValue));


    // HAL_ADC_Start_DMA(&hadc2, (uint32_t *)IRSensorADCValue, 7);
    HAL_ADC_Start_IT(&hadc1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    UNUSED(hadc);

    if (hadc->Instance == ADC1)
    {
        // IRSensorDistanceValue[ADCIndex] = GET_DISTANCE(GET_VOLTAGE(filteredValue), ADCIndex);
        IRSensorADCValue[ADCIndex] = HAL_ADC_GetValue(hadc);
        IRSensorDistanceValue[ADCIndex] = GET_DISTANCE(GET_VOLTAGE(IRSensorADCValue[ADCIndex]), ADCIndex);
        IRSensorDistanceValueFillter[ADCIndex] = LOW_PASS_FILTER(IRSensorDistanceValue[ADCIndex], IRSensorDistanceValueFillterPre[ADCIndex]);
        IRSensorDistanceValueFillterPre[ADCIndex] = IRSensorDistanceValueFillter[ADCIndex];

        ADCIndex++;

        if (ADCIndex == 5)
        {
            ADCIndex = 0;
            // HAL_ADC_Start_IT(&hadc2);
        }
    }
}

//-------------------------------------------------------------------------------------------------------//

double AML_IRSensor_GetDistance(uint8_t sensor)
{
    return IRSensorDistanceValue[sensor];
}

//-------------------------------------------------------------------------------------------------------//

bool AML_IRSensor_IsFrontWall(void)
{
    return (IRSensorDistanceValueFillter[IR_SENSOR_FF] < WALL_IN_FRONT) ? 1 : 0;
}

bool AML_IRSensor_IsLeftWall(void)
{
    return (IRSensorDistanceValueFillter[IR_SENSOR_L] < WALL_IN_LEFT) ? 1 : 0;
}

bool AML_IRSensor_IsRightWall(void)
{
    return (IRSensorDistanceValueFillter[IR_SENSOR_R] < WALL_IN_RIGHT) ? 1 : 0;
}

bool AML_IRSensor_IsNoFrontWall(void)
{
    return (IRSensorDistanceValue[IR_SENSOR_FF] > WALL_NOT_IN_FRONT) ? 1 : 0;
}

bool AML_IRSensor_IsNoLeftWall(void)
{
    return (IRSensorDistanceValue[IR_SENSOR_L] > WALL_NOT_IN_LEFT) ? 1 : 0;
}

bool AML_IRSensor_IsNoRightWall(void)
{
    return (IRSensorDistanceValue[IR_SENSOR_R] > WALL_NOT_IN_RIGHT) ? 1 : 0;
}
//-------------------------------------------------------------------------------------------------------//
void LedDebug(void)
{
    if (AML_IRSensor_IsFrontWall())
    {
        AML_LedDebug_TurnOnLED(N_FF);
    }
    else 
    {
        AML_LedDebug_TurnOffLED(N_FF);
    }
    if (AML_IRSensor_IsLeftWall())
    {
        AML_LedDebug_TurnOnLED(N_L);
    }
    else 
    {
        AML_LedDebug_TurnOffLED(N_L);
    }
    if (AML_IRSensor_IsRightWall())
    {
        AML_LedDebug_TurnOnLED(N_R);
    }
    else 
    {
        AML_LedDebug_TurnOffLED(N_R);
    }
   
}
 