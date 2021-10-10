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
	static constexpr uint32_t ROTATING_PERIOD_SECS = 3600; //1h
	static constexpr uint16_t DEFAULT_ROTATION_ANGLE = 100; //1 rev every 36h

 	static Pot gardenPots[POT_COUNT];
	static TaskHandle_t monitoringTask_Handle;
};

#endif //#ifndef __GARDEN_H_