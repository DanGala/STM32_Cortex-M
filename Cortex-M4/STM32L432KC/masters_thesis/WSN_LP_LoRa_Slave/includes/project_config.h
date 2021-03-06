//*****************************************************************************
//
// project_config.h
//
//*****************************************************************************

/** 
 *  @file project_config.h

 @brief Main definitions of specific hardware

 @author Daniel Gala Montes
 @author Robolabo
 @author www.robolabo.etsit.upm.es
 @date 2018/05/21 

 CREATIVE COMMONS PUBLIC LICENSE:

 THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). 
 THE WORK IS PROTECTED BY COPYRIGHT AND/OR OTHER APPLICABLE LAW. 
 ANY USE OF THE WORK OTHER THAN AS AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.

 BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO BE BOUND BY THE TERMS OF THIS LICENSE. 
 TO THE EXTENT THIS LICENSE MAY BE CONSIDERED TO BE A CONTRACT, 
 THE LICENSOR GRANTS YOU THE RIGHTS CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND CONDITIONS.
 */

#include "stm32l4xx_hal.h"
#include "stm32l4xx_nucleo_32.h"
#include "stdlib.h"
/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#ifndef PROJECT_CONFIG_H_
#define PROJECT_CONFIG_H_

/** @brief Board Version */
#define BOARD_ED_A
/** @brieg Software Version */
#define SOFTWARE_VERSION  "LoRa Slave. Rev 1. 21-05-2018"

//------------------Configuracion de LORA----version a utilizar---------------

//#define CFG_1272_SEMTECH		1						//Selecciona el CHIP SX1272 de SEMTECH*/

//#define CFG_1278_DRF1278F		1						//Selecciona el CHIP SX1278 de DORJI version de tarjeta DRF1278F

#define CFG_1278_NICERF1278	1						//Selecciona el CHIP SX1278 de NICERF version de tarjeta LORA1278*/

/* Exported functions ------------------------------------------------------- */

//*********************************************************
//***************         SPI        *******************
//*********************************************************
#define LORA_SPI				      SPI1
#define LORA_SPI_CLK_ENABLE		__HAL_RCC_SPI1_CLK_ENABLE
#define LORA_SPI_IRQn		      SPI1_IRQn

// SPI CS/NSS
#define LORA_NSS_PIN		GPIO_PIN_1
#define LORA_NSS_PORT   GPIOA

// SPI SCK
#define LORA_SCK_PIN		GPIO_PIN_3
#define LORA_SCK_PORT   GPIOB

// SPI MISO
#define LORA_MISO_PIN		GPIO_PIN_4
#define LORA_MISO_PORT  GPIOB

// SPI MOSI
#define LORA_MOSI_PIN		GPIO_PIN_5
#define LORA_MOSI_PORT  GPIOB

#define EXT_ADC_SPI           SPI1
#define EXT_ADC_SPI_CLK       __HAL_RCC_SPI1_CLK_ENABLE
#define EXT_ADC_SPI_IRQn      SPI1_IRQn
#define EXT_ADC_SPI_ALT       GPIO_AF5_SPI1

// SPI CS/NSS
#define EXT_ADC_NSS_PIN       GPIO_PIN_0
#define EXT_ADC_NSS_PORT      GPIOB

// SPI SCK
#define EXT_ADC_SCK_PIN       GPIO_PIN_3
#define EXT_ADC_SCK_PORT      GPIOB

// SPI MISO
#define EXT_ADC_MISO_PIN      GPIO_PIN_4
#define EXT_ADC_MISO_PORT     GPIOB

// SPI MOSI
#define EXT_ADC_MOSI_PIN      GPIO_PIN_5
#define EXT_ADC_MOSI_PORT     GPIOB

//SPI RDY INTERRUPT - SAME AS MISO
#define EXT_ADC_DRDY_PORT          GPIOB
#define EXT_ADC_DRDY_PIN           GPIO_PIN_4
#define EXT_ADC_DRDY_EXTI          EXTI4_IRQn
#define EXTADC_DRDY_ISR_PRIORITY   8

//*********************************************************
//***************          I2C          *******************
//*********************************************************
/* Definition for I2Cx clock resources */
#define I2Cx                            I2C1
#define RCC_PERIPHCLK_I2Cx              RCC_PERIPHCLK_I2C1
#define RCC_I2CxCLKSOURCE               RCC_I2C1CLKSOURCE_HSI
#define I2Cx_CLK_ENABLE()               __HAL_RCC_I2C1_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE() 

#define I2C_TIMING      0x30420F13
#define I2C_ADDRESS     0x30F
#define FLAG_TIMEOUT    (uint32_t)0x1000

#define I2Cx_FORCE_RESET()              __HAL_RCC_I2C1_FORCE_RESET()
#define I2Cx_RELEASE_RESET()            __HAL_RCC_I2C1_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define I2Cx_SCL_PIN                    GPIO_PIN_6
#define I2Cx_SCL_GPIO_PORT              GPIOB
#define I2Cx_SDA_PIN                    GPIO_PIN_7
#define I2Cx_SDA_GPIO_PORT              GPIOB
#define I2Cx_SCL_SDA_AF                 GPIO_AF4_I2C1


/* Definition for I2Cx's NVIC */
#define I2Cx_EV_IRQn                    I2C1_EV_IRQn
#define I2Cx_ER_IRQn                    I2C1_ER_IRQn
#define I2Cx_EV_IRQHandler              I2C1_EV_IRQHandler
#define I2Cx_ER_IRQHandler              I2C1_ER_IRQHandler

/* Due to connection through soldering bridges SB16 and SB18, PA5 and PA6
 * must be configured as input floating to allow I2C communication on the
 * desired pins on the expansion board */
#define I2C_BRIDGE_PORT                   GPIOA
#define I2C_BRIDGE_SCL_PIN                GPIO_PIN_6
#define I2C_BRIDGE_SDA_PIN                GPIO_PIN_5
#define I2C_BRIDGE_MODE                   GPIO_MODE_INPUT
#define I2C_BRIDGE_SCL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#define I2C_BRIDGE_SDA_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()

//*********************************************************
//***************         LPTIM         *******************
//*********************************************************
/* Definition for LPTIMx clock resources */
#define LPTIMx                           LPTIM1
#define LPTIMx_CLK_ENABLE                __HAL_RCC_LPTIM1_CLK_ENABLE

/* Set the Maximum value of the counter (Auto-Reload) that defines the Period */           
#define LPTIM_PERIOD (uint32_t)65535

/* Set the Timeout value */
#define LPTIM_TIMEOUT (uint32_t)(61440 - 1)
/*#define LPTIM_TIMEOUT (uint32_t)(30720 - 1)*/

//*********************************************************
//***************  TASK's PRIO y STACKS *******************
//*********************************************************
/* Priority is higher as higher is its value 
 * It is important to check the configMAX_PRIORITIES value in 
 * includes/FreeRTOSConfig.h. Priorities should not go higher than
 * that value, because they will lowered to it */

#define TEST_LORA_TASK_PRIORITY		1	
#define TEST_LORA_TASK_STACK			256

#define TEST_ADC_TASK_PRIORITY      1
#define TEST_ADC_TASK_STACK     128

#endif /*PROJECT_CONFIG_H_*/
