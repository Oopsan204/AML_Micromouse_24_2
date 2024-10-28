#include "AML_SenSor.h"
uint32_t SENSOR_VALUE[5] = {-1};
uint32_t SENSOR_BUFFER[5];
const int AVERAGE_OF = 50;
const float MCU_VOLTAGE = 3.3;
const uint8_t RESOLUTION = 65536; // 2^16;
extern ADC_HandleTypeDef hadc1;

uint32_t READ_DISTACE_2_15(uint32_t sensorValue)
{

    float voltage = 0;
    float distance;
    float voltage_temp_average = 0;
    voltage = (sensorValue * (MCU_VOLTAGE / RESOLUTION));
    for (int i = 0; i < AVERAGE_OF; i++)
    {
        voltage_temp_average += (sensorValue * (MCU_VOLTAGE / RESOLUTION));
    }
    voltage_temp_average /= AVERAGE_OF;

    // eqution of the fitting curvey
    // y = -9.5122x5 + 57.406x4 - 139.06x3 + 173.84x2 - 119.18x + 41.252

    distance = -9.5122 * pow(voltage_temp_average, 5) + 57.406 * pow(voltage_temp_average, 4) - 139.06 * pow(voltage_temp_average, 3) + 173.84 * pow(voltage_temp_average, 2) - 119.18 * voltage_temp_average + 41.252;
    return (uint32_t)distance;
}
uint32_t READ_DISTACE_4_30(uint32_t sensorValue)
{

    float voltage = 0;
    float distance;
    float voltage_temp_average = 0;
    voltage = (sensorValue * (MCU_VOLTAGE / RESOLUTION));
    for (int i = 0; i < AVERAGE_OF; i++)
    {
        voltage_temp_average += (sensorValue * (MCU_VOLTAGE / RESOLUTION));
    }
    voltage_temp_average /= AVERAGE_OF;

    // eqution of the fitting curvey
    // // y = -9.5122x5 + 57.406x4 - 139.06x3 + 173.84x2 - 119.18x + 41.252

    // distance = -9.5122 * pow(voltage_temp_average, 5) + 57.406 * pow(voltage_temp_average, 4) - 139.06 * pow(voltage_temp_average, 3)+173.84 * pow(voltage_temp_average, 2) - 119.18 * voltage_temp_average + 41.252;
    return (uint32_t)distance;
}
void SENSOR_INIT()
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)SENSOR_BUFFER, 5);
    SENSOR_BUFFER[0] = SENSOR_FF;
    SENSOR_BUFFER[1] = SENSOR_FL;
    SENSOR_BUFFER[2] = SENSOR_L;
    SENSOR_BUFFER[3] = SENSOR_FR;
    SENSOR_BUFFER[4] = SENSOR_R;
}

void SENSOR_READ_ALL()
{
    SENSOR_VALUE[0] = READ_DISTACE_4_30(SENSOR_FF);
    SENSOR_VALUE[1] = READ_DISTACE_2_15(SENSOR_FR);
    SENSOR_VALUE[2] = READ_DISTACE_2_15(SENSOR_FL);
    SENSOR_VALUE[3] = READ_DISTACE_2_15(SENSOR_R);
    SENSOR_VALUE[4] = READ_DISTACE_2_15(SENSOR_L);
}
uint32_t SENSOR_READ_ONE(int num)
{
    if (num == 0)
    {
        SENSOR_VALUE[0] = READ_DISTACE_4_30(0);
        return SENSOR_VALUE[0];
    }
    else
    {
        SENSOR_VALUE[num] = READ_DISTACE_2_15(SENSOR_BUFFER[num]);
        return SENSOR_VALUE[num];
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    UNUSED(hadc);
}
