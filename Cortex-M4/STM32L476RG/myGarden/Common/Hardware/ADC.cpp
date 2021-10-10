/**
 * @file ADC.cpp
 * @author DGM
 * @version 0.1
 * @date 2021-09-22
 * @copyright Copyright (c) 2021
 */

#include "SystemHeaders.h"

uint16_t ADCChannel::ctorCount = 0;
TaskHandle_t ADConverter::analogTask_Handle = nullptr;
ADCChannel ADConverter::channels[ADC_CHANNEL_COUNT] = ADC_CHANNELS;

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

float ADConverter::AnalogReadScaled(uint16_t index)
{
	return channels[index].GetScaled();
}

uint32_t ADConverter::AnalogReadRaw(uint16_t index)
{
	return channels[index].GetRaw();
}

/**
 * \brief Construct a new ADCChannel::ADCChannel object
 * \param channel channel number
 */
ADCChannel::ADCChannel(const char * name, float scale, float offset) :
	index(ctorCount++),
	name(name),
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
	xLastWakeTime = xTaskGetTickCount();

	while(1)
	{
		/* Trigger a new ADC scan */
		Start();
		/* Block for 10 ms */
		vTaskDelayUntil(&xLastWakeTime, (10 / portTICK_PERIOD_MS));
	}
}