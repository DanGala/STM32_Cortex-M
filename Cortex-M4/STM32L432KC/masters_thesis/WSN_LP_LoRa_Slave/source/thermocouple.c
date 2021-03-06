/** @file thermocouple.c

	@author Alvaro GUTIERREZ
	@author Robolabo
	@date 2013/07/11 - Edici�n base.

	Este archivo ser� el que se encargue de manejar el file system de acuerdo
	a las necesidades espec�ficar de la aplicaci�n

    COPYRIGHT NOTICE:
    This software is property of Robolabo. Its reproduction,  total or  par-
    tial, by any means, in any form, permanent or temporary, is forbidden
    unless explicitly authorised by Robolabo.
    Any adaptation, amendment, translation or transformation, as well as
    the decompiling or disassembly of this software  product, shall only
    be performed with the explicit authorization of Robolabo.
    The user of the present software product shall be allowed  to make a
    backup copy  as long as it is necessary  for the utilization  of the
    same.
    The terms stated above shall be understood  affecting that stated in
    applicable Laws. */

/* Includes ------------------------------------------------------------------*/
/* Board Configuration File */
#include "project_config.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "common.h"
#include "stmX_hal_i2c.h"
#define THERM_TABLES 1 //In order to have thermocouple.c only linking the tables, if not memory problems
#include "thermocouple.h"

extern I2C_HandleTypeDef I2cHandle;
/*****************************
 * ***************************/
void Therm_Init ( void )
{
  uint8_t i, v_sensor_config;

  for ( i = 0 ; i < THERM_N_SENSORS ; i++ )
  {
    Therm_SetType(i, TYPE_T);
  }

  /* Reset sensor */
  v_sensor_config = 0x2F;
  if(xHAL_I2C_Master_Transmit(&I2cHandle, (uint16_t)THERM_SENSOR_I2C_ADDRESS<<1, &v_sensor_config, 1U, FLAG_TIMEOUT) != HAL_OK)
  {
    /* Writting process error */
    while(1);
  }
  while (xHAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
  {
    /* Wait for the end of transfer */
    vTaskDelay(1);
  }

  /* Internal sensor configuration */
  v_sensor_config = 0x00;

  if(xHAL_I2C_Write_Register(&I2cHandle, (uint16_t)THERM_SENSOR_I2C_ADDRESS<<1, (uint8_t)THERM_SENSOR_CONFIG_REG, &v_sensor_config, 1U, FLAG_TIMEOUT) != HAL_OK)
  {
    /* Writing process error */
    while(1);
  }
  while (xHAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
  {
    /* Wait for the end of transfer */
    vTaskDelay(1);
  }

  v_sensor_config = THERM_SENSOR_CONFIG_VALUE;
  if(xHAL_I2C_Write_Register(&I2cHandle, (uint16_t)THERM_SENSOR_I2C_ADDRESS<<1, (uint8_t)THERM_SENSOR_CONFIG_REG, &v_sensor_config, 1U, FLAG_TIMEOUT) != HAL_OK)
  {
    /* Writing process error */
    while(1);
  }
  while (xHAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
  {
    /* Wait for the end of transfer */
    vTaskDelay(1);
  }

  /* Init ExtADC */
  ExtADC_Init();

  /* Reset ADC */
  ExtADC_Reset();
}

/*****************************
 * ***************************/

void Therm_SetType ( uint8_t channel, uint8_t type )
{
	uThermType[channel] = type;
}

/*****************************
 * ***************************/

uint8_t Therm_GetType ( uint8_t channel )
{
	return uThermType[channel];
}

/*****************************
 * ***************************/

int16_t Therm_ReadTemp ( uint8_t channel )
{
	uint8_t 	u8_junction_temp[2];
	uint16_t        u16_junction_temp;
	float 	 	junction_temp 	= 0;
	float 		voltage 	= 0;
	float 		temperature 	= 0;
	
	u8_junction_temp[0]=0;
	u8_junction_temp[1]=0;
	
	uint8_t config_sensor_value = THERM_SENSOR_CONFIG_VALUE;
	/* Send gain to the ADC depending on the type */
	Therm_TypeToADCGain ( channel );
	voltage = ExtADC_ReadVoltageInput(channel);
	uVoltage[channel] = (uint8_t)voltage;	
	/*I2C_Config();*/
  if(xHAL_I2C_Read_Register(&I2cHandle, (uint16_t)THERM_SENSOR_I2C_ADDRESS<<1, (uint8_t)THERM_SENSOR_TEMP_REG, u8_junction_temp, 2U, FLAG_TIMEOUT) != HAL_OK)
  {
    /* Writting process error */
    while(1);
  }
  while (xHAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
  {
    /* Wait for the end of transfer */
    vTaskDelay(1);
  }
	u16_junction_temp = u8_junction_temp[0];
	u16_junction_temp = u16_junction_temp << 8;
	u16_junction_temp += u8_junction_temp[1];
	if(u16_junction_temp > 32767)
		junction_temp = ((float) u16_junction_temp - 65536)/128;
	else
		junction_temp = ((float) u16_junction_temp)/128;
	
	sensorLastTemp = junction_temp;	
	/*junction_temp = ExtADC_ReadTempSensor();*/
	//JUST FOR TEST. BE CAREFULL
	/*junction_temp = 25.0;*/

	temperature = Therm_VoltageToTemp(voltage, junction_temp, Therm_GetType(channel));
	
	return (int16_t) (temperature * 10);
}

/*****************************
 * ***************************/

void Therm_TypeToADCGain ( uint8_t channel )
{
	switch (uThermType[channel])
	{
		/* TYPE J: [-210�C, 1200�C] --> [-8.095mV, 69.553mV]
		 * Set ADC in GAIN_32: [-78.125mV, 78.125mV] */
		case TYPE_J: 
			ExtADC_SetGain(channel, EXTADC_GAIN_32);
			break;
		/* TYPE K: [-270�C, 1370�C] --> [-6,458mV, 54.886mV]
		 * Set ADC in GAIN_32: [-78.125mV, 78.125mV] */
		case TYPE_K:
			ExtADC_SetGain(channel, EXTADC_GAIN_32);
			break;
		/* TYPE T: [-270�C, 400�C] --> [-6,258mV, 20.872mV]
		 * Set ADC in GAIN_128: [-19.53mV, 19.53mV] 
		 * Notice we have truncated in 370�C, if want it move to 
		 * GAIN_64*/
		case TYPE_T:
			ExtADC_SetGain(channel, EXTADC_GAIN_128);
			break;
	}
}

/*****************************
 * ***************************/
float Therm_VoltageToTemp ( float voltage, float junction_temp, uint8_t type )
{
	uint16_t 	index 						= 0;
	float 		fJunctionVoltage 	= 0;
	float 		fCorrectedVoltage = 0;

	float 		fM								= 0;
	float			fRoofVoltage			= 0;
	float 		fFloorVoltage			= 0;
	float 		fTemperature			= 0;

	fJunctionVoltage = Therm_JunctionTempToVoltage ( junction_temp, type);

	switch ( type )
	{
		case TYPE_T:
			fCorrectedVoltage = voltage + fJunctionVoltage;

			/* If voltage out of lower bound, return lower temp */
			if ( fCorrectedVoltage < fThermTableTypeT[0] )
				return TYPE_T_MIN_TEMP;
			/* If voltage out of upper bound, return higher temp */
			else if ( fCorrectedVoltage > fThermTableTypeT[TYPE_T_NUM_POINTS -1])
				return TYPE_T_MAX_TEMP;

			/* if voltage in bounds */
			else 
			{
				for ( index = 0 ; index < TYPE_T_NUM_POINTS; index++)
				{
					if (fCorrectedVoltage < fThermTableTypeT[index])
						break;
				}

				/* y = [(y2-y1)/(x2-x1)]�x + y1 - [(y2-y1)/(x2-x1)]�x1 */
				/*y = m�x + y1 - m�x1*/
				/*Notice that x2-x1 is always 1*/

				fRoofVoltage 	= fThermTableTypeT[index];
				fFloorVoltage	= fThermTableTypeT[index-1];
				fM = 1.0/(fRoofVoltage - fFloorVoltage ); 
				fTemperature = fM * fCorrectedVoltage + (float)(index-1+TYPE_T_MIN_TEMP) - fM * fFloorVoltage;

			}
			break;

		case TYPE_J:
			fCorrectedVoltage = voltage + fJunctionVoltage;

			/* If voltage out of lower bound, return lower temp */
			if ( fCorrectedVoltage < fThermTableTypeJ[0] )
				return TYPE_J_MIN_TEMP;
			/* If voltage out of upper bound, return higher temp */
			else if ( fCorrectedVoltage > fThermTableTypeJ[TYPE_J_NUM_POINTS -1])
				return TYPE_J_MAX_TEMP;

			/* if voltage in bounds */
			else 
			{
				for ( index = 0 ; index < TYPE_J_NUM_POINTS; index++)
				{
					if (fCorrectedVoltage < fThermTableTypeJ[index])
						break;
				}

				/* y = [(y2-y1)/(x2-x1)]�x + y1 - [(y2-y1)/(x2-x1)]�x1 */
				/* y = m�x + y1 - m�x1*/
				/* Notice that x2-x1 is always 1*/

				fRoofVoltage 	= fThermTableTypeJ[index];
				fFloorVoltage	= fThermTableTypeJ[index-1];
				fM = 1.0/(fRoofVoltage - fFloorVoltage ); 
				fTemperature = fM * fCorrectedVoltage + (float)(index-1+TYPE_J_MIN_TEMP) - fM * fFloorVoltage;

			}
			break;

		case TYPE_K:
			fCorrectedVoltage = voltage + fJunctionVoltage;

			/* If voltage out of lower bound, return lower temp */
			if ( fCorrectedVoltage < fThermTableTypeK[0] )
				return TYPE_K_MIN_TEMP;
			/* If voltage out of upper bound, return higher temp */
			else if ( fCorrectedVoltage > fThermTableTypeK[TYPE_K_NUM_POINTS -1])
				return TYPE_K_MAX_TEMP;

			/* if voltage in bounds */
			else 
			{
				for ( index = 0 ; index < TYPE_K_NUM_POINTS; index++)
				{
					if (fCorrectedVoltage < fThermTableTypeK[index])
						break;
				}

				/* y = [(y2-y1)/(x2-x1)]�x + y1 - [(y2-y1)/(x2-x1)]�x1 */
				/*y = m�x + y1 - m�x1*/
				/*Notice that x2-x1 is always 1*/

				fRoofVoltage 	= fThermTableTypeK[index];
				fFloorVoltage	= fThermTableTypeK[index-1];
				fM = 1.0/(fRoofVoltage - fFloorVoltage ); 
				fTemperature = fM * fCorrectedVoltage + (float)(index-1+TYPE_K_MIN_TEMP) - fM * fFloorVoltage;

			}
			break;
	}

	return fTemperature;
}

/*****************************
 * ***************************/
float	Therm_JunctionTempToVoltage	( float junction_temp, uint8_t type )
{
	int16_t 	sFloorTemp 		= 0;
	float 		fFloorVoltage	= 0;
	float 		fRoofVoltage		= 0;
	float 		fM							= 0;
	float 		fVoltage 			= 0;

	switch ( type )
	{
		case TYPE_T:
			/* If junction_temp out of lower bound, return lower voltage */
			if (junction_temp <= TYPE_T_MIN_TEMP )
				return fThermTableTypeT[0];

			/* If junction_temp out of upper bound, return higher voltage */
			else if ( junction_temp >= TYPE_T_MAX_TEMP )
				return fThermTableTypeT[TYPE_T_NUM_POINTS-1];

			/* If junction_temp in bounds */
			else
			{
				sFloorTemp 		= (int16_t) junction_temp;	
				fFloorVoltage 	= fThermTableTypeT[sFloorTemp-TYPE_T_MIN_TEMP];
				fRoofVoltage 	= fThermTableTypeT[sFloorTemp+1-TYPE_T_MIN_TEMP];

				/* y = [(y2-y1)/(x2-x1)]�x + y1 - [(y2-y1)/(x2-x1)]�x1 */
				/*y = m�x + y1 - m�x1*/
				/*Notice that x2-x1 is always 1*/
				fM = (fRoofVoltage - fFloorVoltage ); 
				fVoltage = fM * junction_temp + fFloorVoltage - fM * (float) sFloorTemp;

			}
			break;

		case TYPE_J:
			/* If junction_temp out of lower bound, return lower voltage */
			if (junction_temp <= TYPE_J_MIN_TEMP )
				return fThermTableTypeJ[0];

			/* If junction_temp out of upper bound, return higher voltage */
			else if ( junction_temp >= TYPE_J_MAX_TEMP )
				return fThermTableTypeJ[TYPE_J_NUM_POINTS-1];

			/* If junction_temp in bounds */
			else
			{
				sFloorTemp 		= (int16_t) junction_temp;	
				fFloorVoltage 	= fThermTableTypeJ[sFloorTemp-TYPE_J_MIN_TEMP];
				fRoofVoltage 	= fThermTableTypeJ[sFloorTemp+1-TYPE_J_MIN_TEMP];

				/* y = [(y2-y1)/(x2-x1)]�x + y1 - [(y2-y1)/(x2-x1)]�x1 */
				/* y = m�x + y1 - m�x1*/
				/* Notice that x2-x1 is always 1*/
				fM = (fRoofVoltage - fFloorVoltage ); 
				fVoltage = fM * junction_temp + fFloorVoltage - fM * (float) sFloorTemp;

			}
			break;

		case TYPE_K:
			/* If junction_temp out of lower bound, return lower voltage */
			if (junction_temp <= TYPE_K_MIN_TEMP )
				return fThermTableTypeK[0];

			/* If junction_temp out of upper bound, return higher voltage */
			else if ( junction_temp >= TYPE_K_MAX_TEMP )
				return fThermTableTypeK[TYPE_K_NUM_POINTS-1];

			/* If junction_temp in bounds */
			else
			{
				sFloorTemp 		= (int16_t) junction_temp;	
				fFloorVoltage 	= fThermTableTypeK[sFloorTemp-TYPE_K_MIN_TEMP];
				fRoofVoltage 	= fThermTableTypeK[sFloorTemp+1-TYPE_K_MIN_TEMP];

				/* y = [(y2-y1)/(x2-x1)]�x + y1 - [(y2-y1)/(x2-x1)]�x1 */
				/* y = m�x + y1 - m�x1*/
				/* Notice that x2-x1 is always 1*/
				fM = (fRoofVoltage - fFloorVoltage ); 
				fVoltage = fM * junction_temp + fFloorVoltage - fM * (float) sFloorTemp;
			}
			break;
	}
	return fVoltage;
}

/*****************************
 * ***************************/
uint16_t Therm_GetSensorTemp()
{
	uint16_t value = (uint16_t) (sensorLastTemp * 10.0);
	return value;
}
/*****************************
 ****************************/
uint8_t Therm_GetVoltage(uint8_t channel)
{
	return (uint8_t)uVoltage[channel];
}
