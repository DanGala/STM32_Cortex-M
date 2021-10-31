#ifndef __BDC_MOTOR_H_
#define __BDC_MOTOR_H_

#ifdef USE_BDC_MOTOR

class BDC_Motor
{
public:
	BDC_Motor();
	static void Initialize();
	void Drive(float excitation);

private:
	static constexpr float NOMINAL_VOLTAGE = MOTOR_UMAX;

	static void InitializeHardware();
	void SetDutyCycle(float duty);

	static uint16_t ctorCount;
	uint16_t index;
	bool forwards; /**< Indicates whether the motor is spinning forwards or backwards */
	bool running; /**< Indicates whether the motor is running or stopped */
};

#endif //#ifdef USE_BDC_MOTOR

#endif //#ifndef __BDC_MOTOR_H_