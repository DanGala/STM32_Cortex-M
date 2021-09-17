/**
 * @file Pot.cpp
 * @author DGM
 * @version 0.1
 * @date 2021-09-16
 * @copyright Copyright (c) 2021
 */

#include "SystemHeaders.h"
#include "Pot.h"

Pot * Pot::pots = nullptr;

/**
 * \brief Constructor
 * \param plant Plant planted on this pot
 * \param plate RotatingPlate this pot is on
 * \param waterPump Pump used for irrigating this pot
 * \param moistureSense MoistureSensor used to measure soil moisture in this pot
 */
Pot::Pot(Plant plant, RotatingPlate plate, Pump waterPump, MoistureSensor moistureSense) :
	plant(plant),
	plate(plate),
	waterPump(waterPump),
	moistureSense(moistureSense),
	nextPot(nullptr)
{
	if(!pots)
	{
		//First pot becomes head of the linked list
		pots =  this;
	}
	else
	{
		//Successive pots are sent to the back of the linked list
		Pot * currentPot = pots;
		while(currentPot->nextPot)
		{
			currentPot = currentPot->nextPot;
		}
		currentPot->nextPot = this;
	}
}

/**
 * \brief Pumps water into the pot until the plant no longer needs to be watered
 */
void Pot::Water()
{
	if(plant.NeedsWatered())
	{
		waterPump.StartPumping();
	}
	else
	{
		waterPump.StopPumping();
	}
}

/**
 * \brief Rotates the pot clockwise by the angle specified
 * \param angle Angular rotation desired in tens of degrees
 */
void Pot::Rotate(uint16_t angle)
{
	plate.Rotate(angle);
}

/**
 * \brief Pumps water into all pots that need to be watered
 */
void Pot::WaterAll()
{
	Pot * currentPot = pots;
	while(currentPot)
	{
		currentPot->Water();
		currentPot = currentPot->nextPot;
	}
}

/**
 * \brief Rotates all pots clockwise by the same angle
 * \param angle Angular rotation desired in tens of degrees
 */
void Pot::RotateAll(uint16_t angle)
{
	Pot * currentPot = pots;
	while(currentPot)
	{
		currentPot->Rotate(angle);
		currentPot = currentPot->nextPot;
	}
}