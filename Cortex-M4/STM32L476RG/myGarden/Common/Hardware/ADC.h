#ifndef __ADC_H_
#define __ADC_H_

#ifdef __cplusplus

class ADCChannel
{
public:
	ADCChannel(float scale, float offset);
	uint16_t GetRaw();
	float GetScaled();
private:
	static uint16_t ctorCount;
	uint16_t index;
	float scale;
	float offset;
};

class ADConverter
{
public:
	ADConverter();
	static void Initialize();
	static float AnalogReadScaled(uint16_t index);
	static uint16_t AnalogReadRaw(uint16_t index);
	static void BufferDoneInterrupt(uint8_t _doneBufferIndex);

private:
	static void InitializeHardware();
	static void Calibrate();
	
	static ADCChannel channels[ADC_CHANNEL_COUNT];
};

#endif //#ifdef __cplusplus

#endif //#ifndef __ADC_H_