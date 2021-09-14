#ifndef __ADC_H
#define __ADC_H

#include "FreeRTOS.h"
#include "task.h"

#ifdef __cplusplus

class ADCChannel
{
public:
	ADCChannel(uint16_t channel, float scale, float offset);
	uint32_t GetRawValue();
	float GetScaledValue();
private:
	uint16_t channel;
	float scale;
	float offset;
};

class ADConverter
{
public:
	ADConverter();
	static void Initialize();
private:
	static void AnalogTask(void *pvParams);
	static void InitializeHardware();
	static void Calibrate();
	
	static xTaskHandle analogTask_Handle;
	static ADCChannel channels[1];
};

#endif //#ifdef __cplusplus

#endif //#ifndef __ADC_H