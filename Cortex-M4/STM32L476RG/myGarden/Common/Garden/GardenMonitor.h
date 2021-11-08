#ifndef __GARDEN_MONITOR_H_
#define __GARDEN_MONITOR_H_

//Forward declaration
class Garden;

class GardenMonitor
{
	friend class Garden;
public:
	GardenMonitor();
	static void Initialize();
	static void MonitoringTask(void *pvParams);

private:
	static void TriggerSyncEvent();

	static xSemaphoreHandle txSemaphore;
	static TaskHandle_t monitoringTask_Handle;
};

#endif //#ifndef __GARDEN_MONITOR_H_