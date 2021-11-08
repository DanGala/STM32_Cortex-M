/**
 * @file GardenMonitor.cpp
 * @author DGM
 * @version 0.1
 * @date 2021-10-21
 * @copyright Copyright (c) 2021
 */

#include "SystemHeaders.h"
#include "GardenMonitor.h"

TaskHandle_t GardenMonitor::monitoringTask_Handle = nullptr;
xSemaphoreHandle GardenMonitor::txSemaphore;

/**
 * \brief Default c'tor
 */
GardenMonitor::GardenMonitor()
{
}

/**
 * \brief Initializer
 */
void GardenMonitor::Initialize()
{
	/* Semaphore initialization */
	txSemaphore = xSemaphoreCreateBinary();
	if(!txSemaphore)
	{
		assert(0);
	}

	/* Register serial command for printing an update */
	SerialCommand::Register("MONITOR_SYNC", &GardenMonitor::TriggerSyncEvent);
	
	/* Create monitoring task */
	xTaskCreate(MonitoringTask, "MonitoringTask", 70, NULL, 3, &monitoringTask_Handle);
}

void GardenMonitor::TriggerSyncEvent()
{
	portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
	xSemaphoreGive(txSemaphore);
	portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

/**
 * \brief FreeRTOS task responsible for monitoring the system and reporting its status
 * \param pvParams 
 */
void GardenMonitor::MonitoringTask(void * pvParams)
{	
	while(1)
	{
		if(xSemaphoreTake(txSemaphore, 1000) == pdTRUE)
		{
			///TODO: implement data packing and transmission
			//Get system information (water tank level, ambient temperature and humidity, sensor health...)

			//Get pots information (health, soil moisture, orientation?, luminosity)

			//Fill buffer with data and send

			//Wait indefinitely until next transmission is due
		}
	}
}