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

	Plant plant; /**< A plant (hopefully) growing in the pot */
	RotatingPlate plate; /**< A rotating plate onto which the pot is placed */
	Pump waterPump; /**< An irrigation pump whose outlet pumps water into the pot */
	MoistureSensor moistureSense; /**< A soil moisture sensor placed inside the pot */
	float soilMoisture; /**< Current soil moisture level */
	Pot * nextPot; /**< Pointer to the next pot on the list */
	static Pot * pots; /**< A singly-linked list of pots */
	static TaskHandle_t rotationTask_Handle; /**< RTOS task handle for the RotationTask() */
};

#endif //#ifndef __POT_H_