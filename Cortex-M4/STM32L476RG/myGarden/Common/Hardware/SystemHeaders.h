#ifndef __SYSTEM_HEADERS_H_
#define __SYSTEM_HEADERS_H_

/* STM libraries includes */
#include "stm32l4xx_hal.h"
#include "stm32l4xx_nucleo.h"
/* Standard libraries includes */
#include <stdlib.h>
#include <stdint.h>
#include <limits>
/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
/* Business Logic includes */
#include "Pot.h"
#include "Plant.h"
#include "Pump.h"
#include "RotatingPlate.h"
/* Hardware includes */
#include "ADC.h"
#include "MoistureSensor.h"

#endif //#ifndef __SYSTEM_HEADERS_H_