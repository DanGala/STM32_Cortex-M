/**
 * @file BDC_Motor_STM32.cpp
 * @author DGM
 * @version 0.1
 * @date 2021-10-30
 * @copyright Copyright (c) 2021
 */

#include "SystemHeaders.h"

#ifdef USE_BDC_MOTOR

/* Timer handles */
static TIM_HandleTypeDef htim_PWM;

/* GPIOs and timer channels configuration */
static constexpr uint32_t timerChannels[POT_COUNT] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};
static constexpr uint32_t phasePins[POT_COUNT] = {GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8};
static constexpr GPIO_TypeDef * phasePorts[POT_COUNT] = {GPIOC, GPIOC, GPIOC};

/**
 * \brief Initializes the GPIOs and timer peripherals in PWM mode and Hall mode
 */
void BDC_Motor::InitializeHardware()
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Enable GPIO clocks */
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* Timer channel outputs */
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
	for(uint8_t phase = 0; phase < POT_COUNT; phase++)
	{
		GPIO_InitStruct.Pin = phasePins[phase];
		HAL_GPIO_Init(phasePorts[phase], &GPIO_InitStruct);
	}
	

	/* PWM generation setup */
	constexpr uint32_t TIM_CLK_FREQ = 20000000; //20Mhz
	constexpr uint32_t PWM_FREQ = 50000; //50kHz
	constexpr uint32_t PWM_PERIOD = TIM_CLK_FREQ / PWM_FREQ; //400

	__HAL_RCC_TIM8_CLK_ENABLE();
	htim_PWM.Instance               = TIM8;
	htim_PWM.Init.Prescaler         = (SystemCoreClock / TIM_CLK_FREQ) - 1;
	htim_PWM.Init.Period            = PWM_PERIOD - 1;
	htim_PWM.Init.ClockDivision     = 0;
	htim_PWM.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim_PWM.Init.RepetitionCounter = 0;
	HAL_TIM_PWM_Init(&htim_PWM);

	/* Configure the PWM channels */
	TIM_OC_InitTypeDef sPWMConfig;
	sPWMConfig.OCMode       = TIM_OCMODE_PWM1;
	sPWMConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
	sPWMConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
	sPWMConfig.OCFastMode   = TIM_OCFAST_DISABLE;
	sPWMConfig.Pulse        = 0;
	for(uint8_t phase = 0; phase < POT_COUNT; phase++)
	{
		/* Configure channel as PWM output */
		HAL_TIM_PWM_ConfigChannel(&htim_PWM, &sPWMConfig, timerChannels[phase]);
		/* Start PWM generation */
		HAL_TIM_PWM_Start(&htim_PWM, timerChannels[phase]);
	}
}

/**
 * \brief Updates the duty cycle of the generated PWM output for all phases
 * \param duty New duty cycle within the range [0.0 , 1.0]
 */
void BDC_Motor::SetDutyCycle(float duty)
{
	assert((duty >= 0) && (duty <= 1.0)); //range check
	uint32_t pulseValue = static_cast<uint32_t>((htim_PWM.Init.Period + 1) * duty);
	switch (index)
	{
		case 0:
			htim_PWM.Instance->CCR1 = pulseValue;
			break;
		case 1:
			htim_PWM.Instance->CCR2 = pulseValue;
			break;
		case 2:
			htim_PWM.Instance->CCR3 = pulseValue;
			break;
		case 3:
			htim_PWM.Instance->CCR4 = pulseValue;
			break;
		default:
			assert(0); //out-of-bounds
	}
}

#endif //#ifdef USE_BDC_MOTOR