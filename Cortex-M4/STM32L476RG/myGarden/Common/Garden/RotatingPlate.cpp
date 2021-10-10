/**
 * @file RotatingPlate.cpp
 * @author DGM
 * @version 0.1
 * @date 2021-09-16
 * @copyright Copyright (c) 2021
 */

#include "SystemHeaders.h"
#include "RotatingPlate.h"

/**
 * \brief Constructor
 * \param Kp Proportional gain of the PD loop
 * \param Kd Differential gain of the PD loop
 */
RotatingPlate::RotatingPlate(float Kp, float Kd) :
	position(0),
	target(0),
	propGain(Kp),
	diffGain(Kd)
{
}

/**
 * \brief Sets the reference position of the PD loop
 * \param angle Angular position in tens of degrees
 */
void RotatingPlate::Rotate(uint16_t angle)
{
	target = (position + angle) % ANGLE_WRAP_AROUND;
}

/**
 * \brief PD control loop update routine
 */
void RotatingPlate::LoopUpdate()
{
	///TODO: position to be updated by timer interrupt in encoder interface mode
	uint16_t actualPos = position;
	uint16_t targetPos = target;
	uint16_t error;

	if(targetPos < actualPos)
	{
		error = targetPos + ANGLE_WRAP_AROUND - actualPos;
	}
	else
	{
		error = targetPos - actualPos;
	}

	float pPos = propGain * error;
	uint16_t deltaE;
	if(error < prevError)
	{
		deltaE = prevError - error;
	}
	else
	{
		deltaE = prevError + ANGLE_WRAP_AROUND - error;
	}
	prevError = error;

	float diffPos = diffGain * deltaE * UPDATE_FREQ;
	float controlVar = pPos + diffPos;

	Motor(controlVar);
}

void RotatingPlate::Motor(float control)
{
	///TODO: implement motor control
}