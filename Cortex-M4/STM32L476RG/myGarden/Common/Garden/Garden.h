#ifndef __GARDEN_H_
#define __GARDEN_H_

#include "Pot.h"

class Garden
{
public:
	Garden();
	static void Initialize();
	static void MonitoringTask(void *pvParams);

private:
	static constexpr uint32_t HOURS_TO_MS = 3600000;
	static constexpr uint32_t UPDATE_PERIOD_MS = 1; //1 * HOURS_TO_MS;
	static constexpr uint32_t DAY_TO_MS = 24 * HOURS_TO_MS;
	static constexpr uint16_t ROTATION_PER_DAY = 450; // 1/8 rev per day

 	static Pot gardenPots[POT_COUNT];
	static TaskHandle_t monitoringTask_Handle;
};

#endif //#ifndef __GARDEN_H_