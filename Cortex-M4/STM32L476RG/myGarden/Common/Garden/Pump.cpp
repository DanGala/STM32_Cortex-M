/**
 * @file Pump.cpp
 * @author DGM
 * @version 0.1
 * @date 2021-09-16
 * @copyright Copyright (c) 2021
 */

#include "SystemHeaders.h"
#include "Pump.h"

uint16_t Pump::ctorCount = 0;

/**
 * \brief Constructor
 */
Pump::Pump() :
	index(ctorCount++)
{
}

/**
 * \brief Initializes the pump hardware
 */
void Pump::Initialize()
{
	InitializeHardware();
}