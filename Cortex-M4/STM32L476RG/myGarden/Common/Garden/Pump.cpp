/**
 * @file Pump.cpp
 * @author DGM
 * @version 0.1
 * @date 2021-09-16
 * @copyright Copyright (c) 2021
 */

#include "SystemHeaders.h"
#include "Pump.h"

/**
 * \brief Constructor
 */
Pump::Pump()
{
}

/**
 * \brief Pumps fluid continuosly
 */
void Pump::StartPumping()
{
	///TODO: Switch on the power to the pump driver
}

/**
 * \brief Stops any ongoing pumping
 */
void Pump::StopPumping()
{
	if(pumping)
	{
		///TODO: Switch off the power to the pump driver
	}
	else
	{
		/* Do nothing */
	}
}