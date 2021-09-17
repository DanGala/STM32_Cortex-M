#ifndef __POT_H_
#define __POT_H_

//Forward declarations
class Plant;
class RotatingPlate;
class Pump;
class MoistureSensor;

class Pot
{
public:
	Pot(Plant plant, RotatingPlate plate, Pump waterPump, MoistureSensor moistureSense);
	void Water();
	void Rotate(uint16_t angle);
	static void WaterAll();
	static void RotateAll(uint16_t angle);
	
private:
	Plant& plant;
	RotatingPlate& plate;
	Pump& waterPump;
	MoistureSensor& moistureSense;
	Pot * nextPot;
	static Pot * pots;
};

#endif //#ifndef __POT_H_