/* Includes ------------------------------------------------------------------*/
#include "ADC.h"
/* Scheduler includes */
#include "task.h"

TaskHandle_t ADConverter::analogTask_Handle = nullptr;
/* ADC channels definition */
ADCChannel ADConverter::channels[1] = {ADCChannel(0, 1.0, 0.0)}; // test

/**
 * \brief Empty constructor
 */
ADConverter::ADConverter()
{
}

/**
 * \brief Initializes the ADC
 */
void ADConverter::Initialize()
{
	InitializeHardware();
	Calibrate();
	xTaskCreate(AnalogTask, "ADCTask", 70, NULL, 0, &analogTask_Handle);
}

/**
 * \brief Construct a new ADCChannel::ADCChannel object
 * \param channel channel number
 */
ADCChannel::ADCChannel(uint16_t channel, float scale, float offset) :
	channel(channel),
	scale(scale),
	offset(offset)
{
}

/**
 * \brief FreeRTOS task responsible for processing values converted by the ADC peripheral
 * \param pvParams 
 */
void ADConverter::AnalogTask(void * pvParams)
{
	TickType_t xLastWakeTime;
	uint32_t value = 0;
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		/* Read converted value */
		value = channels[0].GetScaledValue();
		/* Do something useful with it */

		/* Block for 50ms */
		vTaskDelayUntil(&xLastWakeTime, (50 / portTICK_PERIOD_MS));
	}
}