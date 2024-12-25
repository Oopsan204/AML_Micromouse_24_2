#include "SenSor.h"
/*
    distance low range: y = 33,9 + -69,5x + 62,3x^2 + -25,4x^3 + 3,83x^4
    distance high range: y = 55.38 - 77.99x + 54.73x^2 - 18.33x^3 + 2.29x^4
*/

#define ADC_RESOLUTION_BIT 16
#define ADC_MAX (1 << ADC_RESOLUTION_BIT) // 1x2^16
#define ADC_VREF 3.3

#define GET_VOLTAGE(adcValue) ((adcValue * ADC_VREF) / ADC_MAX)
#define GET_DISTANCE_2_15(voltage) (33.9 + (-69.5 * voltage) + (62.3 * pow(voltage, 2)) + (-25.4 * pow(voltage, 3)) + (3.83 * pow(voltage, 4))) // mm
#define GET_DISTANCE_4_30(voltage) (120.8 * pow(voltage, -1.058))
// mm
#define GET_DISTANCE(voltage, index) (index > 0) ? GET_DISTANCE_2_15(voltage) : GET_DISTANCE_4_30(voltage)

extern debug[100];

extern ADC_HandleTypeDef hadc1;

uint16_t IRSensorADCValue[5];
double IRSensorVoltageValue[5];
double IRSensorDistanceValue[5];

uint8_t ADCIndex = 0;

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

//-------------------------------------------------------------------------------------------------------//

void AML_IRSensor_Setup(void)
{
    memset(IRSensorADCValue, 0, sizeof(IRSensorADCValue));
    // HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED);

    // HAL_ADC_Start_DMA(&hadc2, IRSensorValue, 7);
    HAL_ADC_Start_IT(&hadc1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    UNUSED(hadc);
    if (hadc->Instance == ADC1)
    {
        IRSensorADCValue[ADCIndex] = HAL_ADC_GetValue(hadc);
        IRSensorVoltageValue[ADCIndex] = GET_VOLTAGE(IRSensorADCValue[ADCIndex]);
        IRSensorDistanceValue[ADCIndex] = GET_DISTANCE(IRSensorVoltageValue[ADCIndex], ADCIndex);
        // debug[20] = HAL_ADCEx_Calibration_GetValue(&hadc1, ADC_SINGLE_ENDED);
        ADCIndex++;

        if (ADCIndex == 5)
        {
            ADCIndex = 0;
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
    return (IRSensorDistanceValue[IR_SENSOR_FF] < WALL_IN_FRONT) ? 1 : 0;
}

bool AML_IRSensor_IsLeftWall(void)
{
    return (IRSensorDistanceValue[IR_SENSOR_L] < WALL_IN_LEFT) ? 1 : 0;
}

bool AML_IRSensor_IsRightWall(void)
{
    return (IRSensorDistanceValue[IR_SENSOR_R] < WALL_IN_RIGHT) ? 1 : 0;
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
