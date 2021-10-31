/**
 * @file IncrementalEncoder_STM32.cpp
 * @author DGM
 * @version 0.1
 * @date 2021-10-21
 * @copyright Copyright (c) 2021
 */

#include "SystemHeaders.h"

#ifdef USE_INCREMENTAL_ENCODER

static TIM_HandleTypeDef htim;
static volatile uint8_t rotatoryState = 0;
static volatile uint8_t rotatoryTransition = 0;
static constexpr int8_t encoderState[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; /**< Encoder state transition result */
static volatile uint32_t counts = 0;

/**
 * \brief Initializes the GPIOs and timer peripheral in Encoder mode
 */
void IncrementalEncoder::InitializeHardware()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_Encoder_InitTypeDef encoderConfig;

	/* Configure GPIO inputs for rotatory encoder interface */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* Configure timer inputs as encoder interface */
	__HAL_RCC_TIM3_CLK_ENABLE();
	htim.Instance               = TIM3;
	htim.Init.Period            = 0xFFFF;
	htim.Init.Prescaler         = 0;
	htim.Init.ClockDivision     = 0;
	htim.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim.Init.RepetitionCounter = 0;

	encoderConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
	encoderConfig.IC1Polarity  = TIM_ICPOLARITY_BOTHEDGE;
	encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	encoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	encoderConfig.IC1Filter    = 0;
	encoderConfig.IC2Polarity  = TIM_ICPOLARITY_BOTHEDGE;
	encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	encoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	encoderConfig.IC2Filter    = 0;
	HAL_TIM_Encoder_Init(&htim, &encoderConfig);

	/* Start the encoder interface */
	HAL_TIM_Encoder_Start_IT(&htim, TIM_CHANNEL_ALL);

	/* Enable the interrupt for TIM3 */
	HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);

	/* Encoder initial state */
	rotatoryState = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) << 1) | HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
	rotatoryTransition = (rotatoryTransition << 2) | rotatoryState;
}

/**
 * \brief Returns the current angular position
 * \return Angular position in tenths of degrees
 */
uint16_t IncrementalEncoder::GetPosition()
{
	static constexpr uint16_t countsPerRev = ENCODER_CPR * GEAR_RATIO;
	static constexpr uint16_t angleUnitsPerRev = 3600;
	static constexpr float countsToAngle = 1.0 * angleUnitsPerRev / countsPerRev;
	return static_cast<uint16_t>((counts % angleUnitsPerRev) * countsToAngle);
}

extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	rotatoryState = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) << 1) | HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
	rotatoryTransition = (rotatoryTransition << 2) | rotatoryState;
	counts += encoderState[rotatoryTransition & 0x0F];
}

extern "C" void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim);
}

#endif //#ifdef USE_INCREMENTAL_ENCODER