#ifndef __MOISTURE_SENSOR_H_
#define __MOISTURE_SENSOR_H_

/* https://makersportal.com/blog/2020/5/26/capacitive-soil-moisture-calibration-with-arduino */

class MoistureSensor
{
public:
	MoistureSensor(uint16_t adcChannelIndex);
	float GetMoistureLevel();
	void SetOverrideMoisture(float moisture, bool enabled);
	
private:
	uint16_t adcChannelIndex;
	float lastValue;
	float overrideMoisture;
	bool overrideEnabled;
};

#endif //#ifndef __MOISTURE_SENSOR_H_