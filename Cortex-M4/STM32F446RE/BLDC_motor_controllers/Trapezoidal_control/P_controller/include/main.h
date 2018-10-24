/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include <stdlib.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Definition for IOs Pins */

#define PHASEA			    GPIO_PIN_8
#define PHASEB			    GPIO_PIN_9
#define PHASEC			    GPIO_PIN_10

#define PHASEAEN		    GPIO_PIN_10
#define PHASEBEN		    GPIO_PIN_11
#define PHASECEN		    GPIO_PIN_12

#define ENCODERA		    GPIO_PIN_6
#define ENCODERB		    GPIO_PIN_7

#define HALLA			      GPIO_PIN_15
#define HALLB			      GPIO_PIN_3
#define HALLC			      GPIO_PIN_10

/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
