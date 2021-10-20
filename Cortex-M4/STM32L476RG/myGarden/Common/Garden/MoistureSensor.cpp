/**
 * @file MoistureSensor.cpp
 * @author DGM
 * @version 0.1
 * @date 2021-09-16
 * @copyright Copyright (c) 2021
 */

#include "SystemHeaders.h"
#include "MoistureSensor.h"

/**
 * \brief Constructor
 * \param adcChannel ADC channel associated with this sensor
 */
MoistureSensor::MoistureSensor(uint16_t adcChannelIndex) :
	adcChannelIndex(adcChannelIndex),
	overrideMoisture(0.0),
	overrideEnabled(false)
{
}

/**
 * \brief Returns the latest moisture level recorded
 * \return Moisture level in %
 */
float MoistureSensor::GetMoistureLevel()
{
	if(overrideEnabled)
	{
		return overrideMoisture;
	}
	else
	{
		return ADConverter::AnalogReadScaled(adcChannelIndex);
	}
}

/**
 * \brief Sets the moisture level override
 * \param moisture Moisture override value
 * \param enabled When true, moisture override is enabled
 */
void MoistureSensor::SetOverrideMoisture(float moisture, bool enabled)
{
	overrideMoisture = moisture;
	overrideEnabled = enabled;
}