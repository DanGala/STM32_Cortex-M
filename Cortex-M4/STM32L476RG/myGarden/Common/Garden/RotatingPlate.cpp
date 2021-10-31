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
	prevError(0),
	propGain(Kp),
	diffGain(Kd)
{
}

/**
 * \brief Sets the reference position of the PD loop
 * \param angle Angular position in tenths of degrees
 */
void RotatingPlate::Rotate(uint16_t angle)
{
	target = (target + angle) % angleUnitsPerRev;
}

/**
 * \brief PD control loop update routine
 */
void RotatingPlate::LoopUpdate()
{
#ifdef USE_INCREMENTAL_ENCODER
	position = IncrementalEncoder::GetPosition();
#endif

	int32_t actualPos = position;
	int32_t targetPos = target;

	/* Estimate position error considering angle wrapping */
	int32_t error = targetPos - actualPos;
	/* Only forwards for now */
	if(error < 0) error += angleUnitsPerRev;

	/* Calculate proportional gain component */
	float pPos = propGain * error;

	/* Calculate differential gain component */
	int32_t deltaE = prevError - error;
	float diffPos = diffGain * deltaE * updateFrequency;

	/* Record the new error for the next update */
	prevError = error;

	/* Calculate the PD controller loop output */
	float controlVar = pPos + diffPos;

	/* Feed the calculated control signal to the motor driver */
	Motor(controlVar);
}

void RotatingPlate::Motor(float control)
{
	/* Limit the excitation signal */
	float excitation = (control > 1.0) ? 1.0 : control;
	motor.Drive(excitation);

#ifndef USE_INCREMENTAL_ENCODER
	//If using open loop control, update position after motoring
#endif
}