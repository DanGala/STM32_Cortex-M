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
 * \param name String used to identify this plant
 * \param minMoisture Minimum soil moisture level required by the plant in %
 */
Plant::Plant(const char * name, float minMoisture) :
	Plant(name, 100, minMoisture, false)
{
}

/**
 * \brief Specialized constructor
 * \param name String used to identify this plant
 * \param health Initial health of the plant in %
 * \param minMoisture Minimum soil moisture level required by the plant in cm^3/cm^3
 * \param waterNeeded Flag to indicate whether watering is required
 */
Plant::Plant(const char * name, uint8_t health, float minMoisture, bool waterNeeded) :
	name(name),
	health(health),
	minMoisture(minMoisture),
	waterNeeded(waterNeeded)
{
}

/**
 * \brief Updates the plant's health bar and sets/resets the waterNeeded flag
 * \param moisture Current soil moisture
 */
void Plant::UpdateHealth(float moisture)
{
	/* Update health bar */
	if(moisture < minMoisture)
	{
		/* Soil was too dry - decrease health bar */
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
		/* Soil humidity was OK - increase health bar */
		if((health + MOISTURE_OK_RECOVERY) > FULL_HEALTH)
		{
			health = FULL_HEALTH;
		}
		else
		{
			health += MOISTURE_OK_RECOVERY;
		}
	}

	/* Update flag */
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