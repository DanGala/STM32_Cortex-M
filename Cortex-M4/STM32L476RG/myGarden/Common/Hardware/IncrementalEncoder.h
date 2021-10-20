#ifndef __INCREMENTAL_ENCODER_H_
#define __INCREMENTAL_ENCODER_H_

#ifdef __cplusplus

class IncrementalEncoder
{
public:
	IncrementalEncoder();
	static void Initialize();
	static uint16_t GetPosition();
	
private:
	static void InitializeHardware();
};

#endif //#ifdef __cplusplus

#endif //#ifndef __INCREMENTAL_ENCODER_H_