/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#define ARM_MATH_CM4

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include <stdlib.h> // For math abs() function

/* Exported types ------------------------------------------------------------*/
/**
  * @brief Instance structure for the floating-point PID Control.
  **/
typedef struct
{
  float windup_guard; /* Windup guarding for integral component. */
  float Kp;           /* The proportional gain. */
  float Ki;           /* The integral gain. */
  float Kd1;          /* The direct derivative gain. */
  float Kd2;          /* The parallel derivative gain */
  float prevErr;      /* Previous error entry. */
  float prevPos;      /* Previous rotor position */
  float intErr;       /* Integral error. */
} pid_d_instance;

/* Exported constants --------------------------------------------------------*/
/* Definition for IOs Pins */
#define PHASEA	        GPIO_PIN_8
#define PHASEB	        GPIO_PIN_9
#define PHASEC	        GPIO_PIN_10

#define PHASEAEN        GPIO_PIN_10
#define PHASEBEN        GPIO_PIN_11
#define PHASECEN        GPIO_PIN_12

#define ENCODERA		    GPIO_PIN_6
#define ENCODERB		    GPIO_PIN_7

#define HALLA		        GPIO_PIN_15
#define HALLB		        GPIO_PIN_3
#define HALLC		        GPIO_PIN_10

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
