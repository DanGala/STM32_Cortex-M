#include "ADC.h"
#include "stm32l4xx_hal.h"

static GPIO_InitTypeDef GPIO_InitStruct;
static ADC_HandleTypeDef hadc1;
volatile uint32_t adcValue[1];

/**
 * \brief Initializes the ADC peripheral
 */
void ADConverter::InitializeHardware()
{
	ADC_ChannelConfTypeDef sConfig = {0};

	/* Enable clocks */
	__HAL_RCC_ADC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* Enable GPIOs */
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pin = GPIO_PIN_0; //ADC channel ref input pin
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Configure the ADC peripheral */
	hadc1.Instance = ADC1;
	HAL_ADC_DeInit(&hadc1);

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	HAL_ADC_Init(&hadc1);

	/* Configure ADC regular channel */
	sConfig.Channel = ADC_CHANNEL_5;				  /* Sampled channel number */
	sConfig.Rank = 1;								  /* Rank of sampled channel number ADCx_CHANNEL */
	sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5; /* Sampling time (number of clock cycles unit) */
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.Offset = 0; /* Parameter discarded because offset correction is disabled */
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

	/* Enable interrupts */
	HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC1_2_IRQn);

	/* Start ADC */
	HAL_ADC_Start_IT(&hadc1);
}

/**
 * \brief Calibrates the ADC peripheral to get better accuracy
 */
void ADConverter::Calibrate()
{
	uint32_t calibrationFactor;

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	calibrationFactor = HAL_ADCEx_Calibration_GetValue(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_SetValue(&hadc1, ADC_SINGLE_ENDED, calibrationFactor);
}

/**
 * \brief Returns the raw number of ADC counts for an ADC channel conversion
 * \return Raw number of counts
 */
uint32_t ADCChannel::GetRawValue()
{
	return adcValue[channel];
}

/**
 * \brief Returns the scaled converted value for an ADC channel
 * \return Scaled converted value
 */
float ADCChannel::GetScaledValue()
{
	float scaled = (adcValue[channel] - offset) * scale;
	return scaled;
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	adcValue[0] = HAL_ADC_GetValue(hadc);
}

extern "C" void ADC1_2_IRQHandler()
{
	HAL_ADC_IRQHandler(&hadc1);
}