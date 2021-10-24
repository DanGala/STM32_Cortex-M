/**
 * @file BLDC_Motor.cpp
 * @author DGM
 * @version 0.1
 * @date 2021-10-22
 * @copyright Copyright (c) 2021
 */

#include "SystemHeaders.h"

#ifdef USE_BLDC_MOTOR

BLDC_Motor::SixStepStage BLDC_Motor::currentStage = BLDC_Motor::SixStepStage::A_B;
bool BLDC_Motor::forwards = true;
bool BLDC_Motor::kickStartRequired = true;
bool BLDC_Motor::running = false;

/**
 * \brief Default c'tor
 */
BLDC_Motor::BLDC_Motor()
{
}

/**
 * \brief Initializer
 */
void BLDC_Motor::Initialize()
{
	InitializeHardware();
	currentStage = DetectCurrentStage(); //Detect initial position
}

/**
 * \brief Drives the BLDC motor according to the excitation signal provided
 * \param excitation Excitation voltage to drive the BLDC motor
 */
void BLDC_Motor::Drive(float excitation)
{
	bool previouslyForwards = forwards;
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
		running = true;
		forwards = false;
		excitationAbs = -excitation;
	}

	/* When direction changes or the motor has been stopped, a kickstart may be required to start running again */
	kickStartRequired = ( (previouslyForwards != forwards) || !running );

	/* Update PWM duty cycle based on excitation */
	SetDutyCycle(excitationAbs / NOMINAL_VOLTAGE);
}

/**
 * \brief Energizes the motor's coils according to the Six Step Mode sequence
 */
void BLDC_Motor::SixStepUpdate()
{
	if(running)
	{
		if(kickStartRequired)
		{
			currentStage = DetectCurrentStage();
		}

		switch (currentStage)
		{
			case SixStepStage::A_B:
				/* Disable phase C, and enable phases A and B with inverse polarities */
				DeEnergizeCoil(PHASE_C);
				EnergizeCoil(PHASE_A, true);
				EnergizeCoil(PHASE_B, false);
				break;
			case SixStepStage::A_C:
				/* Disable phase B, and enable phases A and C with inverse polarities */
				DeEnergizeCoil(PHASE_B);
				EnergizeCoil(PHASE_A, true);
				EnergizeCoil(PHASE_C, false);
				break;
			case SixStepStage::B_A:
				/* Disable phase C, and enable phases B and A with inverse polarities */
				DeEnergizeCoil(PHASE_C);
				EnergizeCoil(PHASE_B, true);
				EnergizeCoil(PHASE_A, false);
				break;
			case SixStepStage::B_C:
				/* Disable phase A, and enable phases B and C with inverse polarities */
				DeEnergizeCoil(PHASE_A);
				EnergizeCoil(PHASE_B, true);
				EnergizeCoil(PHASE_C, false);
				break;
			case SixStepStage::C_A:
				/* Disable phase B, and enable phases C and A with inverse polarities */
				DeEnergizeCoil(PHASE_B);
				EnergizeCoil(PHASE_C, true);
				EnergizeCoil(PHASE_A, false);
				break;
			case SixStepStage::C_B:
				/* Disable phase A, and enable phases C and B with inverse polarities */
				DeEnergizeCoil(PHASE_A);
				EnergizeCoil(PHASE_C, true);
				EnergizeCoil(PHASE_B, false);
				break;
			default:
				assert(0); //we should never end up here
		}
	}
	else
	{
		/* Disable all phases */
		DeEnergizeCoil(PHASE_A);
		DeEnergizeCoil(PHASE_B);
		DeEnergizeCoil(PHASE_C);
	}
}

/**
 * \brief Callback routine for Hall sensor detection events. Updates the internal 6-Step stage and energizes the appropriate coils
 */
void BLDC_Motor::HallSensorCallback()
{
	if(forwards)
	{
		currentStage++;
	}
	else
	{
		currentStage--;
	}
	SixStepUpdate();
}

/**
 * \brief Postfix increment operator overload for SixStepStage enum class
 */
BLDC_Motor::SixStepStage operator++(BLDC_Motor::SixStepStage& s, int)
{
	BLDC_Motor::SixStepStage old = s;
	s = static_cast<BLDC_Motor::SixStepStage>((static_cast<uint8_t>(s) + 1) % static_cast<uint8_t>(BLDC_Motor::SixStepStage::STAGE_COUNT));
	return old;
}

/**
 * \brief Postfix decrement operator overload for SixStepStage enum class
 */
BLDC_Motor::SixStepStage operator--(BLDC_Motor::SixStepStage& s, int)
{
	BLDC_Motor::SixStepStage old = s;
	if(s == BLDC_Motor::SixStepStage::A_B)
	{
		s = BLDC_Motor::SixStepStage::C_B;
	}
	else
	{
		s = static_cast<BLDC_Motor::SixStepStage>(static_cast<uint8_t>(s) - 1);
	}
	return old;
}

#endif //#ifdef USE_BLDC_MOTOR