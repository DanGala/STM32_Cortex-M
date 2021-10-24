#ifndef __BLDC_MOTOR_H_
#define __BLDC_MOTOR_H_

#ifdef USE_BLDC_MOTOR

class BLDC_Motor
{
public:
	BLDC_Motor();
	static void Initialize();
	static void Drive(float excitation);
	static void HallSensorCallback();

	enum class SixStepStage : uint8_t { A_B, A_C, B_C, B_A, C_A, C_B, STAGE_COUNT };
	static constexpr uint8_t PHASE_A = 0;
	static constexpr uint8_t PHASE_B = 1;
	static constexpr uint8_t PHASE_C = 2;
	static constexpr uint8_t PHASE_COUNT = 3;
	
private:
	static constexpr float NOMINAL_VOLTAGE = MOTOR_UMAX;
	friend SixStepStage operator++(SixStepStage& s, int);
	friend SixStepStage operator--(SixStepStage& s, int);

	static void InitializeHardware();
	static void SetDutyCycle(float duty);
	static void SixStepUpdate();
	static void EnergizeCoil(uint8_t phase, bool polarity);
	static void DeEnergizeCoil(uint8_t phase);
	static SixStepStage DetectCurrentStage();

	static SixStepStage currentStage; /**< Current stage in the Six Step Mode control sequence */
	static bool forwards; /**< Indicates whether the motor is spinning forwards or backwards */
	static bool kickStartRequired; /**< Indicates whether kickstarting the motor is required on the next update */
	static bool running; /**< Indicates whether the motor is running or stopped */
};

#endif //#ifdef USE_BLDC_MOTOR

#endif //#ifndef __BLDC_MOTOR_H_