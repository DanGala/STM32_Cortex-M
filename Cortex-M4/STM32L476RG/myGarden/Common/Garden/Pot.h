#ifndef __POT_H_
#define __POT_H_

//Forward declarations
#include "Plant.h"
#include "RotatingPlate.h"
#include "Pump.h"
#include "MoistureSensor.h"

class Pot
{
public:
	Pot(Plant plant, RotatingPlate plate, Pump waterPump, MoistureSensor moistureSense);
	static void Initialize();
	void Water();
	void Rotate(uint16_t angle);
	static void WaterAll();
	static void RotateAll(uint16_t angle);
	void UpdatePlantHealth();
	static void UpdateHealthAll();
	
private:
	static void RotationTask(void *pvParams);

	Plant plant;
	RotatingPlate plate;
	Pump waterPump;
	MoistureSensor moistureSense;
	Pot * nextPot;
	static Pot * pots;
	static TaskHandle_t rotationTask_Handle;
};

#endif //#ifndef __POT_H_