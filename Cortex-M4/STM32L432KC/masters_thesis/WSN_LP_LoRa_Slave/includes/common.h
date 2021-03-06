/** 
 *	@file common.h

		@brief Common definitions to all files

		@author Alvaro Gutierrez
		@author Robolabo
		@author www.robolabo.etsit.upm.es
		@date 2013/05/22 

    CREATIVE COMMONS PUBLIC LICENSE:
		
		THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). 
		THE WORK IS PROTECTED BY COPYRIGHT AND/OR OTHER APPLICABLE LAW. 
		ANY USE OF THE WORK OTHER THAN AS AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.

		BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO BE BOUND BY THE TERMS OF THIS LICENSE. 
		TO THE EXTENT THIS LICENSE MAY BE CONSIDERED TO BE A CONTRACT, 
		THE LICENSOR GRANTS YOU THE RIGHTS CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND CONDITIONS.
*/

#ifndef __COMMON_H_
#define __COMMON_H_
#include "FreeRTOS.h"

#define CR 		13		/**< ENTER */

/* Board ID definition */
#define THERMOCOUPLE 	0
#define RTD 		1
#define ANALOG		2
#define VDCIN		3
#define VDCOUT		4
#define VACOUT		5
#define SAI		7

/* Software version*/
static uint16_t software_version = 1;

/**********************************************************//**
*						Types
************************************************************/

typedef union
{
	uint32_t word32;
	struct
	{
		uint8_t byte0;   /**< byte 0 is the LESS SIGNIFICATIVE */
		uint8_t byte1;
		uint8_t byte2;
		uint8_t byte3;
	}bytes;
} t_word32;

typedef union
{
	uint16_t word16;
	struct
	{
		uint8_t byte_up;
		uint8_t byte_down;
	}bytes;
} t_word16;

typedef union
{
	uint8_t byte8;
	struct
	{
		uint8_t bit0:1;  /**< bit 0 is the LESS SIGNIFICATIVE  */
		uint8_t bit1:1;
		uint8_t bit2:1;
		uint8_t bit3:1;
		uint8_t bit4:1;
		uint8_t bit5:1;
		uint8_t bit6:1;
		uint8_t bit7:1;
	} bits;
} t_byte8;

typedef union
{
  float float32;
  struct
  {
    uint8_t byte0;  /**< byte 0 is the LESS SIGNIFICATIVE */
    uint8_t byte1;
    uint8_t byte2;
    uint8_t byte3;
  }bytes;
}t_float32;

#endif
