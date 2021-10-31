#ifndef __ROTATING_PLATE_H_
#define __ROTATING_PLATE_H_

class RotatingPlate
{
public:
	RotatingPlate(float Kp, float Kd);
	static void Initialize();
	void Rotate(uint16_t angle);
	void LoopUpdate();
	
private:
	static constexpr uint16_t angleUnitsPerRev = 3600u; /**< Maximum angle value in tenths of degrees before wrapping */
	static constexpr float updateFrequency = 1; /**< Control loop update frequency in kHz - must match the frequency of the routine calling LoopUpdate() */

	void Motor(float control);

#ifdef USE_BDC_MOTOR
	BDC_Motor motor;
#elif defined(USE_BLDC_MOTOR)
	BLDC_Motor motor;
#else
	#error No motor support defined!
#endif
	uint16_t position; /**< Current angular position in tenths of degrees */
	uint16_t target; /**< Target angular position in tenths of degrees */
	int32_t prevError; /**< Previous loop update error */
	float propGain; /**< Proportional gain value for the PI loop */
	float diffGain; /**< Integral gain value for the PI loop */
};

#endif //#ifndef __ROTATING_PLATE_H_