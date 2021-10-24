/**
 * @file BLDC_Motor_STM32.cpp
 * @author DGM
 * @version 0.1
 * @date 2021-10-24
 * @copyright Copyright (c) 2021
 */

#include "SystemHeaders.h"

#ifdef USE_BLDC_MOTOR

/* GPIOs and timer channels configuration */
static constexpr uint32_t timerChannels[BLDC_Motor::PHASE_COUNT] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3};
static constexpr uint32_t phasePins[BLDC_Motor::PHASE_COUNT] = {GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10};
static constexpr GPIO_TypeDef * phasePorts[BLDC_Motor::PHASE_COUNT] = {GPIOA, GPIOA, GPIOA};
static constexpr uint32_t phaseEnablePins[BLDC_Motor::PHASE_COUNT] = {GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12};
static constexpr GPIO_TypeDef * phaseEnablePorts[BLDC_Motor::PHASE_COUNT] = {GPIOC, GPIOC, GPIOC};

/* Hall sensor GPIO configuration */
static constexpr uint8_t HALL_A = 0;
static constexpr uint8_t HALL_B = 1;
static constexpr uint8_t HALL_C = 2;
static constexpr uint8_t HALL_COUNT = 3;
static constexpr uint32_t hallPins[HALL_COUNT] = {GPIO_PIN_15, GPIO_PIN_3, GPIO_PIN_10};
static constexpr GPIO_TypeDef * hallPorts[HALL_COUNT] = {GPIOA, GPIOB, GPIOB};

/* Timer handles */
static TIM_HandleTypeDef htim_PWM;
static TIM_HandleTypeDef htim_Hall;

/* Helper function prototype declarations */
void HallSensorConfig();
void PWMConfig();

/**
 * \brief Initializes the GPIOs and timer peripherals in PWM mode and Hall mode
 */
void BLDC_Motor::InitializeHardware()
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Enable GPIO clocks */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* Phase enable pins */
	GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull  = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	for(uint8_t phase = 0; phase < PHASE_COUNT; phase++)
	{
		GPIO_InitStruct.Pin = phaseEnablePins[phase];
		HAL_GPIO_Init(phaseEnablePorts[phase], &GPIO_InitStruct);
	}

	/* Phases */
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	for(uint8_t phase = 0; phase < PHASE_COUNT; phase++)
	{
		GPIO_InitStruct.Pin = phasePins[phase];
		HAL_GPIO_Init(phasePorts[phase], &GPIO_InitStruct);
	}

	/* Hall sensor inputs */
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	for(uint8_t sensor = 0; sensor < HALL_COUNT; sensor++)
	{
		GPIO_InitStruct.Pin = hallPins[sensor];
		HAL_GPIO_Init(hallPorts[sensor], &GPIO_InitStruct);
	}

	/* Configure a timer peripheral in Hall sensor interface mode */
	HallSensorConfig();

	/* PWM generation setup */
	PWMConfig();
}

/* These flags are used to keep track of complementary PWM signal activation/deactivation */
static bool negativePolarityOn[BLDC_Motor::PHASE_COUNT] = {false, false, false};
static bool positivePolarityOn[BLDC_Motor::PHASE_COUNT] = {false, false, false};

/**
 * \brief Energizes a coil phase by generating PWM outputs on that channel
 * \param phase BLDC motor's coil phase to energize
 * \param polarity When false, use complementary PWM outputs
 */
void BLDC_Motor::EnergizeCoil(uint8_t phase, bool polarity)
{
	/* Enable the phase output */
	HAL_GPIO_WritePin(phaseEnablePorts[phase], phaseEnablePins[phase], GPIO_PIN_SET);

	/* Start the PWM generation for its timer channel */
	if(polarity && !positivePolarityOn[phase])
	{
		if(negativePolarityOn[phase])
		{
			negativePolarityOn[phase] = (HAL_TIMEx_PWMN_Stop(&htim_PWM, timerChannels[phase]) == HAL_OK);
		}
		positivePolarityOn[phase] = (HAL_TIM_PWM_Start(&htim_PWM, timerChannels[phase]) == HAL_OK);
	}
	else if (!polarity && !negativePolarityOn[phase])
	{
		if(positivePolarityOn[phase])
		{
			positivePolarityOn[phase] = (HAL_TIM_PWM_Stop(&htim_PWM, timerChannels[phase]) == HAL_OK);
		}
		negativePolarityOn[phase] = (HAL_TIMEx_PWMN_Start(&htim_PWM, timerChannels[phase]) == HAL_OK);
	}
	else
	{
		/* Nothing to be done for this phase */
	}

	/* Ensure we are not driving complementary signals into a single channel, and that at least one signal is being driven */
	assert(negativePolarityOn[phase] != positivePolarityOn[phase]);
}

/**
 * \brief De-energizes a coil phase by stopping PWM outputs on that channel
 * \param phase BLDC motor's coil phase to de-energize
 */
void BLDC_Motor::DeEnergizeCoil(uint8_t phase)
{
	/* Disable the phase output */
	HAL_GPIO_WritePin(phaseEnablePorts[phase], phaseEnablePins[phase], GPIO_PIN_RESET);

	/* Stop the PWM generation for its timer channel */
	if(negativePolarityOn[phase] && (HAL_TIMEx_PWMN_Stop(&htim_PWM, timerChannels[phase]) == HAL_OK))
	{
		negativePolarityOn[phase] = false;
	}
	else if(positivePolarityOn[phase] && (HAL_TIM_PWM_Stop(&htim_PWM, timerChannels[phase]) == HAL_OK))
	{
		positivePolarityOn[phase] = false;
	}
	else
	{
		/* Nothing to be done for this phase */
	}

	/* Ensure all phases are off by now */
	assert(!negativePolarityOn[phase] && !positivePolarityOn[phase]);
}

/**
 * \brief Updates the duty cycle of the generated PWM output for all phases
 * \param duty New duty cycle within the range [0.0 , 1.0]
 */
void BLDC_Motor::SetDutyCycle(float duty)
{
	assert((duty >= 0) && (duty <= 1.0)); //range check
	uint32_t pulseValue = static_cast<uint32_t>(htim_PWM.Init.Period * duty);
    htim_PWM.Instance->CCR1 = pulseValue;
}

/**
 * \brief Detects the current 6-Step stage based on the Hall sensor readings
 * \return The current position of the BLDC motor with respect to the 6-Step Mode sequence
 */
BLDC_Motor::SixStepStage BLDC_Motor::DetectCurrentStage()
{
	/* Hall sensor based driving stage */
	static constexpr BLDC_Motor::SixStepStage hallState[6] = {
		BLDC_Motor::SixStepStage::B_A,
		BLDC_Motor::SixStepStage::A_C,
		BLDC_Motor::SixStepStage::B_C,
		BLDC_Motor::SixStepStage::C_B,
		BLDC_Motor::SixStepStage::C_A,
		BLDC_Motor::SixStepStage::A_B
	};

	uint8_t detectionFlags = 0;
	for(uint8_t sensor = 0; sensor < HALL_COUNT; sensor++)
	{
		detectionFlags |= HAL_GPIO_ReadPin(hallPorts[sensor], hallPins[sensor]) << sensor;
	}

	assert(detectionFlags < 0x8);
	assert(detectionFlags != 0x7);
	assert(detectionFlags != 0);

    SixStepStage stage = hallState[detectionFlags & 0x07];
	return stage;
}

/**
 * \brief Configures TIM4 in non-blocking Hall sensor interface mode
 */
void HallSensorConfig()
{
	__HAL_RCC_TIM4_CLK_ENABLE();
	htim_Hall.Instance               = TIM4;
	htim_Hall.Init.Prescaler         = 0;
	htim_Hall.Init.Period            = 0xFFFF;
	htim_Hall.Init.ClockDivision     = 0;
	htim_Hall.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim_Hall.Init.RepetitionCounter = 0;
	HAL_TIM_IC_Init(&htim_Hall);

	/* Configure the Input Capture of channel 1 */
	TIM_HallSensor_InitTypeDef sHallConfig;
	sHallConfig.IC1Polarity  = TIM_ICPOLARITY_BOTHEDGE;
	sHallConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sHallConfig.IC1Filter    = 0;
	sHallConfig.Commutation_Delay = 0;
	HAL_TIMEx_HallSensor_Init(&htim_Hall, &sHallConfig);

	/* Start the Hall sensor interface mode */
	HAL_TIMEx_HallSensor_Start_IT(&htim_Hall);

	/* Enable the interrupt for TIM4 */
	HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/**
 * \brief Configures TIM1 to generate complementary PWM outputs on three channels
 */
void PWMConfig()
{
	constexpr uint32_t TIM_CLK_FREQ = 20000000; //20Mhz
	constexpr uint32_t PWM_FREQ = 50000; //50kHz

	__HAL_RCC_TIM1_CLK_ENABLE();
	htim_PWM.Instance               = TIM1;
	htim_PWM.Init.Prescaler         = (SystemCoreClock / TIM_CLK_FREQ) - 1;
	htim_PWM.Init.Period            = (TIM_CLK_FREQ / PWM_FREQ) - 1;
	htim_PWM.Init.ClockDivision     = 0;
	htim_PWM.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim_PWM.Init.RepetitionCounter = 0;
	HAL_TIM_PWM_Init(&htim_PWM);

	/* Configure the PWM channels */
	TIM_OC_InitTypeDef sPWMConfig;
	sPWMConfig.OCMode       = TIM_OCMODE_TIMING;
	sPWMConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
	sPWMConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	sPWMConfig.OCIdleState  = TIM_OCIDLESTATE_SET;
	sPWMConfig.OCNIdleState = TIM_OCNIDLESTATE_SET;
	sPWMConfig.OCFastMode   = TIM_OCFAST_DISABLE;
	sPWMConfig.Pulse        = 0;
	for(uint8_t phase = 0; phase < BLDC_Motor::PHASE_COUNT; phase++)
	{
		HAL_TIM_OC_ConfigChannel(&htim_PWM, &sPWMConfig, timerChannels[phase]);
	}

	/* Configure the break stage */
	TIM_BreakDeadTimeConfigTypeDef sBreakConfig;
	sBreakConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
	sBreakConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
	sBreakConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
	sBreakConfig.BreakState       = TIM_BREAK_DISABLE;
	sBreakConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
	sBreakConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_ENABLE;
	sBreakConfig.DeadTime         = 1;
	HAL_TIMEx_ConfigBreakDeadTime(&htim_PWM, &sBreakConfig);

	/* Configure the commutation event */
	HAL_TIMEx_ConfigCommutationEvent_IT(&htim_PWM, TIM_TS_NONE, TIM_COMMUTATION_SOFTWARE);

	/* Enable the TIM1 global Interrupt */
	HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);

	/* Start PWM generation */
	for(uint8_t phase = 0; phase < BLDC_Motor::PHASE_COUNT; phase++)
	{
		HAL_TIM_OC_Start(&htim_PWM, timerChannels[phase]);
		HAL_TIMEx_OCN_Start(&htim_PWM, timerChannels[phase]);
	}
}

extern "C" void TIM1_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim_PWM);
}

/**
 * \brief TIM4 IRQHandler
 * \note HAL_TIM_IC_CaptureCallback() is not used here to avoid name clashes with other bits of the code
 * 		 that already provide a definition for it.
 */
extern "C" void TIM4_IRQHandler(void)
{
	__HAL_TIM_CLEAR_IT(&htim_Hall, TIM_IT_CC1);
	BLDC_Motor::HallSensorCallback();
}

#endif //#ifdef USE_BLDC_MOTOR