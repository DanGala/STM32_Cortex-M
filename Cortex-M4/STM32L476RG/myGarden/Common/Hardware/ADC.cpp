/**
 * @file ADC.cpp
 * @author DGM
 * @version 0.1
 * @date 2021-09-22
 * @copyright Copyright (c) 2021
 */

#include "SystemHeaders.h"

uint16_t ADCChannel::ctorCount = 0;
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
}

float ADConverter::AnalogReadScaled(uint16_t index)
{
	return channels[index].GetScaled();
}

uint16_t ADConverter::AnalogReadRaw(uint16_t index)
{
	return channels[index].GetRaw();
}

/**
 * \brief Construct a new ADCChannel::ADCChannel object
 * \param channel channel number
 */
ADCChannel::ADCChannel(float scale, float offset) :
	index(ctorCount++),
	scale(scale),
	offset(offset)
{
}