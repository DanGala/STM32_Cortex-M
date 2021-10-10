#include "SystemHeaders.h"

static GPIO_InitTypeDef GPIO_InitStruct;

const uint32_t PUMP_GPIO_PINS[ADC_CHANNEL_COUNT] =
{
	GPIO_PIN_10, //PUMP_1 - D2
	GPIO_PIN_5, //PUMP_2 - D4
	GPIO_PIN_8  //PUMP_3 - D7
};

GPIO_TypeDef * PUMP_GPIO_PORTS[ADC_CHANNEL_COUNT] =
{
	GPIOA, //PUMP_1
	GPIOB, //PUMP_2
	GPIOA  //PUMP_3
};

/**
 * \brief Initializes the ADC peripheral
 */
void Pump::InitializeHardware()
{
	/* Enable clocks */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* Enable GPIOs */
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	for(uint16_t i = 0; i < ADC_CHANNEL_COUNT; i++)
	{
		GPIO_InitStruct.Pin = PUMP_GPIO_PINS[i];
		HAL_GPIO_Init(PUMP_GPIO_PORTS[i], &GPIO_InitStruct);
	}
}

/**
 * \brief Pumps fluid continuosly
 */
void Pump::StartPumping()
{
	HAL_GPIO_WritePin(PUMP_GPIO_PORTS[index], PUMP_GPIO_PINS[index], GPIO_PinState::GPIO_PIN_RESET);
}

/**
 * \brief Stops any ongoing pumping
 */
void Pump::StopPumping()
{
	if(pumping)
	{
		HAL_GPIO_WritePin(PUMP_GPIO_PORTS[index], PUMP_GPIO_PINS[index], GPIO_PinState::GPIO_PIN_SET);
	}
	else
	{
		/* Do nothing */
	}
}