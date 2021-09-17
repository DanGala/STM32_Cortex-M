#ifndef __ROTATING_PLATE_H_
#define __ROTATING_PLATE_H_

class RotatingPlate
{
public:
	RotatingPlate(float Kp, float Kd);
	void Rotate(uint16_t angle);
	void LoopUpdate();
	
private:
 	/** Maximum angle value in tens of degrees before wrapping */
	static constexpr uint16_t ANGLE_WRAP_AROUND = 3600u;
	/** Control loop update frequency in kHz - must match the frequency of the routine calling LoopUpdate() */
	static constexpr float UPDATE_FREQ = 1;

	void Motor(float control);

	uint16_t position; /**< Current angular position in tens of degrees */
	uint16_t target; /**< Target angular position in tens of degrees */
	uint16_t prevError; /**< Previous loop update error */
	float propGain; /**< Proportional gain value for the PI loop */
	float diffGain; /**< Integral gain value for the PI loop */
};

#endif //#ifndef __ROTATING_PLATE_H_