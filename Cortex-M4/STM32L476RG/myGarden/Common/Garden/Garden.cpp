/**
 * @file Garden.cpp
 * @author DGM
 * @version 0.1
 * @date 2021-10-10
 * @copyright Copyright (c) 2021
 */

#include "SystemHeaders.h"
#include "Garden.h"

TaskHandle_t Garden::monitoringTask_Handle = nullptr;
Pot Garden::gardenPots[POT_COUNT] = POTS_INIT;

Garden::Garden()
{
}

void Garden::Initialize()
{
	Pump::Initialize();
	Pot::Initialize();

	/* Create monitoring task */
	xTaskCreate(MonitoringTask, "MonitoringTask", 70, NULL, 1, &monitoringTask_Handle);
}

/**
 * \brief FreeRTOS task responsible for processing values converted by the ADC peripheral
 * \param pvParams 
 */
void Garden::MonitoringTask(void * pvParams)
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