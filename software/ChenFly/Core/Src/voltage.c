#include "voltage.h"
#include "adc.h"
#include "usart.h"

uint16_t adcData;
float voltage;
float voltage_ratio = 1.025;
uint8_t voltage_10;

void voltage_read(void)
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	adcData = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	voltage = adcData*(3.3/4096)*11*voltage_ratio;
	voltage_10 = (uint8_t)(voltage*10);
}
