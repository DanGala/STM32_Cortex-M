#ifndef __ADC_H_
#define __ADC_H_

#include "SystemHeaders.h"

#ifdef __cplusplus

class ADCChannel
{
public:
	ADCChannel(const char * name, float scale, float offset);
	uint32_t GetRaw();
	float GetScaled();
private:
	static uint16_t ctorCount;
	uint16_t index;
	const char * name;
	float scale;
	float offset;
};

class ADConverter
{
public:
	ADConverter();
	static void Initialize();
	static float AnalogReadScaled(uint16_t index);
	static uint32_t AnalogReadRaw(uint16_t index);

private:
	static void AnalogTask(void *pvParams);
	static void InitializeHardware();
	static void Start();
	static void Calibrate();
	
	static xTaskHandle analogTask_Handle;
	static ADCChannel channels[ADC_CHANNEL_COUNT];
};

#endif //#ifdef __cplusplus

#endif //#ifndef __ADC_H_