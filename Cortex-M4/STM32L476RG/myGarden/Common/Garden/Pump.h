#ifndef __PUMP_H_
#define __PUMP_H_

class Pump
{
public:
	Pump();
	void StartPumping();
	void StopPumping();
	
private:
	bool pumping; /**< Flag to indicate whether the pump is currently pumping */
};

#endif //#ifndef __PUMP_H_