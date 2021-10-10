#ifndef __PUMP_H_
#define __PUMP_H_

class Pump
{
public:
	Pump();
	static void Initialize();
	void StartPumping();
	void StopPumping();
	
private:
	static void InitializeHardware();

	static uint16_t ctorCount;
	uint16_t index;
	bool pumping; /**< Flag to indicate whether the pump is currently pumping */
};

#endif //#ifndef __PUMP_H_