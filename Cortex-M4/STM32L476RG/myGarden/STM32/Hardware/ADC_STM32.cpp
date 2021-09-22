#include "SystemHeaders.h"

static GPIO_InitTypeDef GPIO_InitStruct;
static ADC_HandleTypeDef hadc1;
volatile uint32_t convertedValues[ADC_CHANNEL_COUNT];
volatile uint16_t lastConvertedIndex = 0;

const uint32_t ADC_GPIO_PINS[ADC_CHANNEL_COUNT] =
{
	GPIO_PIN_0, //MOISTURE_SENSE_1
	GPIO_PIN_1, //MOISTURE_SENSE_2
	GPIO_PIN_4  //MOISTURE_SENSE_3
};

GPIO_TypeDef * ADC_GPIO_PORTS[ADC_CHANNEL_COUNT] =
{
	GPIOA, //MOISTURE_SENSE_1
	GPIOA, //MOISTURE_SENSE_2
	GPIOA  //MOISTURE_SENSE_3
};

const uint32_t ADC_CHANNEL_INPUTS[ADC_CHANNEL_COUNT] =
{
	ADC_CHANNEL_5, //MOISTURE_SENSE_1
	ADC_CHANNEL_6, //MOISTURE_SENSE_2
	ADC_CHANNEL_9  //MOISTURE_SENSE_3
};

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
	for(uint16_t i = 0; i < ADC_CHANNEL_COUNT; i++)
	{
		GPIO_InitStruct.Pin = ADC_GPIO_PINS[i];
		HAL_GPIO_Init(ADC_GPIO_PORTS[i], &GPIO_InitStruct);
	}
	
	/* Configure the ADC peripheral */
	hadc1.Instance = ADC1;
	HAL_ADC_DeInit(&hadc1);

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = ADC_CHANNEL_COUNT;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	HAL_ADC_Init(&hadc1);

	/* Configure ADC regular channels */
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.Offset = 0;
	for(uint16_t i = 0; i < ADC_CHANNEL_COUNT; i++)
	{
		sConfig.Channel = ADC_CHANNEL_INPUTS[i];
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	}

	/* Enable interrupts */
	HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

/**
 * \brief Starts the ADC conversion sequence
 */
void ADConverter::Start()
{
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
uint32_t ADCChannel::GetRaw()
{
	return convertedValues[index];
}

/**
 * \brief Returns the scaled converted value for an ADC channel
 * \return Scaled converted value
 */
float ADCChannel::GetScaled()
{
	float scaled = (convertedValues[index] - offset) * scale;
	return scaled;
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	convertedValues[lastConvertedIndex] = HAL_ADC_GetValue(hadc);
	lastConvertedIndex = (lastConvertedIndex + 1) % ADC_CHANNEL_COUNT;
}

extern "C" void ADC1_2_IRQHandler()
{
	HAL_ADC_IRQHandler(&hadc1);
}