#ifndef __SYSTEM_HEADERS_H_
#define __SYSTEM_HEADERS_H_

/* Project configuration */
#include "myGardenConfig.h"
/* STM libraries includes */
#include "stm32l4xx_hal.h"
#include "stm32l4xx_nucleo.h"
/* Standard libraries includes */
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <limits>
#include <assert.h>
/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
/* Hardware includes */
#include "ADC.h"
#include "Serial.h"
#include "MoistureSensor.h"
#ifdef USE_INCREMENTAL_ENCODER
#include "IncrementalEncoder.h"
#endif //#ifdef USE_INCREMENTAL_ENCODER
#ifdef USE_BLDC_MOTOR
#include "BLDC_Motor.h"
#endif //#ifdef USE_BLDC_MOTOR
#ifdef USE_BDC_MOTOR
#include "BDC_Motor.h"
#endif //#ifdef USE_BDC_MOTOR

#endif //#ifndef __SYSTEM_HEADERS_H_