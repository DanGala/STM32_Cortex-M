/**
 * @file ADC_STM32.cpp
 * @author DGM
 * \brief Low-level driver for the ADC peripheral for STM32L4xx
 * @version 0.1
 * @date 2021-10-16
 * @note ADC is configured to work in single mode as a sequencer. Conversions are triggered by the TRGO event
 * 		 of an internal timer running at 10kHz. Converted data for all channels is copied to RAM by DMA in circular mode.
 * 		 A succesful triggered conversion requires the scan to be completed before the next TRGO event, in under 100us.
 * 		 If changes are made, the following inequality must hold:
 * 			100us >= ADC_CHANNEL_COUNT * ((12.5 + Sampling time) / ADC_CLK_FRQ)
 * 		 For the current setup:
 * 			3 * (260 / (64MHz / 8)) = 97.5us < 100us => OK
 */

#include "SystemHeaders.h"

static GPIO_InitTypeDef GPIO_InitStruct;
static ADC_HandleTypeDef hadc1;
static TIM_HandleTypeDef htim;
static DMA_HandleTypeDef dmaHandle;
static uint16_t doubleBuffer[2][ADC_CHANNEL_COUNT] = {0};
static uint16_t convertedValues[ADC_CHANNEL_COUNT] = {0};
static uint8_t doneBufferIndex = 0;

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
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_ADC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_ADC_CONFIG(RCC_ADCCLKSOURCE_SYSCLK);
	__HAL_DBGMCU_FREEZE_TIM2();

	/* Configure the timer peripheral */
	htim.Instance = TIM2;
	HAL_TIM_Base_DeInit(&htim);

	htim.Init.Prescaler = 0;
	htim.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim.Init.Period = (SystemCoreClock / 10000) - 1; //10kHz sample rate
	htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim.Init.RepetitionCounter = 0;
	HAL_TIM_Base_Init(&htim);

	/* TRGO selection */
	TIM_MasterConfigTypeDef sMasterConfig;
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);

	/* Start */
	HAL_TIM_Base_Start(&htim);

	/* Enable debug pin D0 */
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* Enable GPIOs */
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	for(uint16_t i = 0; i < ADC_CHANNEL_COUNT; i++)
	{
		GPIO_InitStruct.Pin = ADC_GPIO_PINS[i];
		HAL_GPIO_Init(ADC_GPIO_PORTS[i], &GPIO_InitStruct);
	}
	
	/* Deinitialize  & Initialize the DMA for new transfer */
	dmaHandle.Instance                 = DMA1_Channel1;
	HAL_DMA_DeInit(&dmaHandle);

	dmaHandle.Init.Request             = DMA_REQUEST_0;
	dmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
	dmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
	dmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
	dmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	dmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
	dmaHandle.Init.Mode                = DMA_CIRCULAR;
	dmaHandle.Init.Priority            = DMA_PRIORITY_HIGH;
	
	HAL_DMA_Init(&dmaHandle);

	/* Associate the DMA handle */
	__HAL_LINKDMA(&hadc1, DMA_Handle, dmaHandle);

	/* Configure the ADC peripheral */
	hadc1.Instance = ADC1;
	HAL_ADC_DeInit(&hadc1);

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = ADC_CHANNEL_COUNT;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.NbrOfDiscConversion = 0;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	hadc1.Init.OversamplingMode = DISABLE;
	HAL_ADC_Init(&hadc1);

	/* Configure ADC regular channels */
	uint32_t chRank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	for(uint16_t idx = 0; idx < ADC_CHANNEL_COUNT; idx++)
	{
		sConfig.Channel = ADC_CHANNEL_INPUTS[idx];
		sConfig.Rank = chRank++;
		HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	}

	/* Enable interrupts */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	/* Launch calibration after exiting Deep-power-down mode */
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)doubleBuffer, 2*ADC_CHANNEL_COUNT);
}

/**
 * \brief Calibrates the ADC peripheral to get better accuracy
 */
void ADConverter::Calibrate()
{
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
}

/**
 * \brief Returns the raw number of ADC counts for an ADC channel conversion
 * \return Raw number of counts
 */
uint16_t ADCChannel::GetRaw()
{
	return convertedValues[index];
}

static constexpr float countsToVolts = (3.3 / 4096);

/**
 * \brief Returns the scaled converted value for an ADC channel
 * \return Scaled converted value
 */
float ADCChannel::GetScaled()
{
	float volts = countsToVolts * convertedValues[index];
	float scaled = (volts - offset) * scale;
	return scaled;
}

/**
 * \brief Copies the latest results to the output buffer
 * 
 * \param _doneBufferIndex index of the double buffer that will contain the latest data
 */
void ADConverter::BufferDoneInterrupt(uint8_t _doneBufferIndex)
{
	doneBufferIndex = _doneBufferIndex;

	for(uint8_t idx = 0; idx < ADC_CHANNEL_COUNT; idx++)
	{
		convertedValues[idx] = doubleBuffer[doneBufferIndex][idx];
	}
}

extern "C" void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	ADConverter::BufferDoneInterrupt(1); //doubleBuffer[1][] contains latest data
}

extern "C" void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
	ADConverter::BufferDoneInterrupt(0); //doubleBuffer[0][] contains latest data
}

extern "C" void DMA1_Channel1_IRQHandler()
{
	HAL_DMA_IRQHandler(hadc1.DMA_Handle);
}