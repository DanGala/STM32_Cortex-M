/**
 * @file BDC_Motor.cpp
 * @author DGM
 * @version 0.1
 * @date 2021-10-30
 * @copyright Copyright (c) 2021
 */

#include "SystemHeaders.h"

#ifdef USE_BDC_MOTOR

uint16_t BDC_Motor::ctorCount = 0;

/**
 * \brief Default c'tor
 */
BDC_Motor::BDC_Motor() :
	forwards(true),
	running(false),
	index(ctorCount++)
{
}

/**
 * \brief Initializer
 */
void BDC_Motor::Initialize()
{
	InitializeHardware();
}

/**
 * \brief Drives the BDC motor according to the excitation signal provided
 * \param excitation Excitation voltage to drive the BDC motor
 */
void BDC_Motor::Drive(float excitation)
{
	float excitationAbs;

	/* Excitation signal's sign indicates direction */
	if(excitation == 0)
	{
		running = false;
		excitationAbs = excitation;
	}
	else if(excitation > 0){
		running = true;
		forwards = true;
		excitationAbs = excitation;	
	}
	else
	{
		assert(0); //not currently supported by hardware
	}

	/* Update PWM duty cycle based on excitation */
	SetDutyCycle(excitationAbs);
}

#endif //#ifdef USE_BDC_MOTOR