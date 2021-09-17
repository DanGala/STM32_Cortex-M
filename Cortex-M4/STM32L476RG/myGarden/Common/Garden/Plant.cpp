/**
 * @file Plant.cpp
 * @author DGM
 * @version 0.1
 * @date 2021-09-16
 * @copyright Copyright (c) 2021
 */

#include "SystemHeaders.h"
#include "Plant.h"

/**
 * \brief Constructor
 * \param moistureLimitLow Minimum soil moisture level required by the plant in cm^3/cm^3
 */
Plant::Plant(const char * name, float moistureLimitLow) :
	Plant(name, 100, moistureLimitLow, false)
{
}

/**
 * \brief Specialized constructor
 * \param health Initial health of the plant in %
 * \param moistureLimitLow Minimum soil moisture level required by the plant in cm^3/cm^3
 */
Plant::Plant(const char * name, uint8_t health, float moistureLimitLow, bool waterNeeded) :
	name(name),
	health(health),
	moistureLimitLow(moistureLimitLow),
	waterNeeded(waterNeeded)
{
}

/**
 * \brief Updates the plant's health bar and sets/resets the waterNeeded flag
 * \param moisture Current soil moisture
 */
void Plant::UpdateHealth(float moisture)
{
	if(moisture < moistureLimitLow)
	{
		if(health >= MOISTURE_BELOW_LIMIT_PENALTY)
		{
			health -= MOISTURE_BELOW_LIMIT_PENALTY;
		}
		else
		{
			health = 0;
		}
	}
	else
	{
		if((health + MOISTURE_OK_RECOVERY) > FULL_HEALTH)
		{
			health = FULL_HEALTH;
		}
		else
		{
			health += MOISTURE_OK_RECOVERY;
		}
	}

	if(health <= WATERING_THRESHOLD)
	{
		waterNeeded = true;
	}
	else
	{
		waterNeeded = false;
	}
}

/**
 * \brief Checks if the plant needs to be watered
 * \return true if water is needed, false otherwise
 */
bool Plant::NeedsWatered()
{
	return waterNeeded;
}