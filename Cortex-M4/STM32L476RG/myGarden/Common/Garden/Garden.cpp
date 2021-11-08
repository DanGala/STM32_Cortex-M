/**
 * @file Garden.cpp
 * @author DGM
 * @version 0.1
 * @date 2021-10-10
 * @copyright Copyright (c) 2021
 */

#include "SystemHeaders.h"
#include "Garden.h"
#include "Pot.h"
#include "GardenMonitor.h"

TaskHandle_t Garden::gardeningTask_Handle = nullptr;
Pot Garden::gardenPots[POT_COUNT] = POTS_INIT;

/**
 * \brief Default c'tor
 */
Garden::Garden()
{
}

/**
 * \brief Initializer
 */
void Garden::Initialize()
{
	Pump::Initialize();
	Pot::Initialize();
	GardenMonitor::Initialize();

	/* Create monitoring task */
	xTaskCreate(GardeningTask, "GardeningTask", 70, NULL, 1, &gardeningTask_Handle);
}

/**
 * \brief FreeRTOS task responsible for taking care of all plants
 * \param pvParams 
 */
void Garden::GardeningTask(void * pvParams)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	uint32_t rotatePeriodCounter = 0;

	while(1)
	{
		vTaskDelayUntil(&xLastWakeTime, (UPDATE_PERIOD_MS / portTICK_PERIOD_MS));
		
		/* Update health status for all pots */
		Pot::UpdateHealthAll();

		/* Water pots that need it */
		Pot::WaterAll();

		/* Rotate all pots a bit every now and then */
		if(rotatePeriodCounter++ == (DAY_TO_MS / UPDATE_PERIOD_MS))
		{
			Pot::RotateAll(ROTATION_PER_DAY);
			rotatePeriodCounter = 0;
		}

	}
}