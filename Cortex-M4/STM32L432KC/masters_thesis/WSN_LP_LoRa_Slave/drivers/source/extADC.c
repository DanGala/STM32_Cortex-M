/** 
 *	@file extADC.c

		@brief Functions that handle the external ADC

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

/* Includes ------------------------------------------------------------------*/
/* Board Configuration File */
#include "project_config.h"

/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "extADC.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
// -----------------------------------------------------------------------------
	/* Create TypeDef vars */
	/* Create SPIs struct */
	SPI_InitTypeDef SPI_InitStructure;
  /* Create SPIs struct */
  SPI_HandleTypeDef SPI_Handle;
	/* Create GPIOs struct */
	GPIO_InitTypeDef GPIO_InitStructure;
// -----------------------------------------------------------------------------

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

void ExtADC_Init ( void )
{

	uint8_t i;

  /* Activate Clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  EXT_ADC_SPI_CLK();

  /* Config GPIOs */
  /* Configure SCK as Alternative Function PP */
  GPIO_InitStructure.Pin = EXT_ADC_MISO_PIN;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  GPIO_InitStructure.Alternate = EXT_ADC_SPI_ALT;
  HAL_GPIO_Init(EXT_ADC_MISO_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = EXT_ADC_MOSI_PIN;
  HAL_GPIO_Init(EXT_ADC_MOSI_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = EXT_ADC_SCK_PIN;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Alternate = EXT_ADC_SPI_ALT;
  HAL_GPIO_Init(EXT_ADC_SCK_PORT, &GPIO_InitStructure);

  /* Configure NSS as GPIO Output PP */
  GPIO_InitStructure.Pin = EXT_ADC_NSS_PIN;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(EXT_ADC_NSS_PORT, &GPIO_InitStructure);

  HAL_GPIO_WritePin(EXT_ADC_NSS_PORT, EXT_ADC_NSS_PIN, GPIO_PIN_SET);

  /* Deinitialize previous SPI configurations */
  if(HAL_SPI_DeInit(&SPI_Handle) != HAL_OK)
  {
    /* Deinitialization error */
    while(1);
  }

  /* Config SPI mode */
  SPI_Handle.Instance = EXT_ADC_SPI;
  SPI_Handle.Init.Direction = SPI_DIRECTION_2LINES;
  SPI_Handle.Init.Mode = SPI_MODE_MASTER;
  SPI_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
  SPI_Handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
  SPI_Handle.Init.CLKPhase = SPI_PHASE_2EDGE; 
  SPI_Handle.Init.NSS = SPI_NSS_SOFT;
  SPI_Handle.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  SPI_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  SPI_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
  SPI_Handle.Init.CRCPolynomial = 7;
  SPI_Handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  SPI_Handle.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  SPI_Handle.Init.TIMode = SPI_TIMODE_DISABLE;

  if(HAL_SPI_Init(&SPI_Handle) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Not all channels are used in differential configuration */
	for ( i = 0 ; i < NUM_DIFF_CHANNELS ; i++)
	{
		uChannelGain[i]		= EXTADC_GAIN_1;
		uChannelDifferential[i]	= 1;
	}
  for ( i = NUM_DIFF_CHANNELS ; i < NUM_CHANNELS ; i++ )
  {
    uChannelGain[i]   = EXTADC_GAIN_1;
    uChannelDifferential[i] = 0;
  }
  
}

void ExtADC_DeInit ( void )
{

  /* Deconfig GPIOs */
  /* Deconfigure SCK as Alternative Function PP */
  HAL_GPIO_DeInit(EXT_ADC_SCK_PORT, EXT_ADC_SCK_PIN);

  /* Deinitialize previous SPI configurations */
  if(HAL_SPI_DeInit(&SPI_Handle) != HAL_OK)
  {
    /* Deinitialization error */
    while(1);
  }

  /* Deselect extADC module on SPI bus */
  HAL_GPIO_WritePin(EXT_ADC_NSS_PORT, EXT_ADC_NSS_PIN, GPIO_PIN_SET);
}

/**	EXTADC_EnableDRDYInt()
 *  Habilita la interrupción externa que ataca a la línea DRDY
 */
void ExtADC_EnableDRDYInt(){

  GPIO_InitStructure.Pin = EXT_ADC_DRDY_PIN;
  GPIO_InitStructure.Speed  = GPIO_SPEED_FREQ_VERY_HIGH;                                              
	GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  HAL_GPIO_Init(EXT_ADC_DRDY_PORT, &GPIO_InitStructure);
}

/**	EXTADC_DisableDRDYInt()
 *  Deshabilita la interrupción externa que ataca a la línea DRDY
 */
void ExtADC_DisableDRDYInt(){
/*	EXTI_InitTypeDef EXTI_InitStructure;

	EXTI_InitStructure.EXTI_Line = EXT_ADC_DRDY_EXTI;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
	EXTI_Init(&EXTI_InitStructure); */
  HAL_GPIO_DeInit(EXT_ADC_DRDY_PORT, EXT_ADC_DRDY_PIN);
}

/**	EXTADC_SpiOn()
 *  Activa el bus SPI para recepción de datos del ADC externo
 */
void ExtADC_SpiOn(){
	/*SPI_I2S_ITConfig(EXT_ADC_SPI, SPI_I2S_IT_RXNE, ENABLE);*/
	//SPI_Cmd(EXT_ADC_SPI, ENABLE);
}

/********************************************
 * *****************************************/

/**	EXTADC_SpiOff()
 *  Desactiva el bus SPI para recepción de datos del ADC externo
 */
void ExtADC_SpiOff(){
	/*SPI_I2S_ITConfig(EXT_ADC_SPI, SPI_I2S_IT_RXNE, DISABLE);*/
	//SPI_Cmd(EXT_ADC_SPI, DISABLE);
}

/********************************************
 * *****************************************/

void ExtADC_NSSOn()
{
  HAL_GPIO_WritePin(EXT_ADC_NSS_PORT, EXT_ADC_NSS_PIN, GPIO_PIN_RESET);
}

/********************************************
 * *****************************************/

void ExtADC_NSSOff()
{
  HAL_GPIO_WritePin(EXT_ADC_NSS_PORT, EXT_ADC_NSS_PIN, GPIO_PIN_SET);
}



/*u8 ExtADC_GetFlag ()*/
/*{*/
/*return uExtADCReadyFlag;*/
/*}*/

/*t_u32_in_4 ExtADC_GetData ()*/
/*{*/
/*return uExtADCData;*/
/*}*/

/********************************************
 * *****************************************/
void ExtADC_Reset ( void )
{
  uint8_t out = 0xFF;
  uint8_t res;
	/* Enable NSS */
	ExtADC_NSSOn();
	/* Go for reset */
  if (HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY) != HAL_OK) while(1);
  if (HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY) != HAL_OK) while(1);
  if (HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY) != HAL_OK) while(1);
  if (HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY) != HAL_OK) while(1);
  if (HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY) != HAL_OK) while(1);
	/* Disable NSS */
	ExtADC_NSSOff();
}

/********************************************
 * *****************************************/
uint32_t ExtADC_ReadAnalogInput ( uint8_t channel )
{
		uint32_t 	data;
    uint8_t out;
    uint8_t res;
		t_u32_in_4 confReg; 
		t_u32_in_4 modeReg; 

		/*uint8_t diffChannel = ExtADC_GetDiffConf(channel);*/
		confReg = ExtADC_CalcCONFReg ( channel );
		modeReg = ExtADC_CalcMODEReg ( channel );

		/* Enable NSS */
		ExtADC_NSSOn();
		/* Write communication register to allow a writing to the configuration register */ 
    out = WRITE_CONF_REG;
    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);
	
		/* Write data to the configuration REGISTER to read tmp: 
		 * Temp ON: bit 16
		 * BUF ON: bit 4
		 * GAIN MAX (111): bit 2, 1 and 0 
		 * TOTAL: 0000 0001 0000 0000 0001 0111 --> 0x010017
		 * */
    out = confReg.bytes.byte2;
    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);
    out = confReg.bytes.byte1;
    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);
    out = confReg.bytes.byte0;
    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);

		/* Write communication register to allow writing to the Mode register */
    out = WRITE_MODE_REG;
    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);

		/* Write data to the MODE REG, to allow a single reading*/
    out = modeReg.bytes.byte2;
    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);
    out = modeReg.bytes.byte1;
    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);
    out = modeReg.bytes.byte0;
    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);

		/* Wait for RDY line down */
		while (HAL_GPIO_ReadPin(EXT_ADC_MISO_PORT, EXT_ADC_MISO_PIN) == GPIO_PIN_SET)
			vTaskDelay(1);
		
		/* Read Data REG */	
		data = ExtADC_ReadDataReg();
		return data;
}
/********************************************
 * *****************************************/

float ExtADC_ReadVoltageInput ( uint8_t channel)
{
	uint32_t 	code  	= 0;
	float 		voltage = 0;

	code = ExtADC_ReadAnalogInput ( channel );
	/* CODE = 2^(N-1) * [(AIN * Gain/V_ref) +1]
	 * AIN 	= ((CODE / 2^(N-1)) -1 ) * (V_ref/Gain)
	 */
	if (uChannelDifferential[channel])
	{
		voltage = (float) code / (float) EXTADC_HALF_RESOL;
		voltage -= 1.0;
		voltage *= (float) EXTADC_REFERENCE;
		voltage /= (float) ExtADC_CalcRealGain(uChannelGain[channel]);
		voltage *= 1000; //Go for mV

	}
	/* CODE = (2^N * AIN * Gain)/V_ref
	 * AIN 	= (CODE * V_ref)/(2^N * Gain) 
	 * */
	else
	{
		voltage = (float) code * (float) EXTADC_REFERENCE;
		voltage /= (float) EXTADC_RESOL;
		voltage /= (float) ExtADC_CalcRealGain(uChannelGain[channel]);
		voltage *= 1000; //Go for mV
	}

	return voltage;
}

/********************************************
 * *****************************************/
uint8_t	ExtADC_GetDiffConf		( uint8_t channel )
{
	switch (channel)
	{
		case 0:
			return CH0_DIFF_CONF;
			break;
		case 1:
			return CH1_DIFF_CONF;
			break;
		case 2:
			return CH2_DIFF_CONF;
			break;
		case 3:
			return CH3_DIFF_CONF;
			break;
		default:
			return 0x00;
			break;
	}
}

/********************************************
 * *****************************************/
uint8_t	ExtADC_GetPseudoConf		( uint8_t channel )
{
	switch (channel)
	{
    case 2:
      return AIN7_PSEUDO_CONF;
      break;
    case 3:
      return AIN9_PSEUDO_CONF;
      break;
		default:
			return 0x00;
			break;
	}
}

/********************************************
 * *****************************************/

float ExtADC_ReadTempSensor ( void )
{
    uint8_t out;
    uint8_t res;
		uint32_t 	data;
		float temp;

		/* Enable NSS */
		ExtADC_NSSOn();
		/* Write communication register to allow a writing to the configuration register */ 
    out = WRITE_CONF_REG;
    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);
	
		/* Write data to the configuration REGISTER to read tmp: 
		 * Temp ON: bit 16
		 * BUF ON: bit 4
		 * GAIN MAX (111): bit 2, 1 and 0 
		 * TOTAL: 0000 0001 0000 0000 0001 0111 --> 0x010017
		 * */
    out = 0x01;
    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);
    out = 0x00;
    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);
    out = 0x17;
    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);
	
		/* Write communication register to allow writing to the Mode register */
    out = WRITE_MODE_REG;
    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);

		/* Write data to the MODE REG, to allow a single reading*/
    out = 0x28;
    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);
    out = 0x00;
    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);
    out = 0x60;
    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);

		/* Wait for RDY line down */
		while (HAL_GPIO_ReadPin(EXT_ADC_MISO_PORT, EXT_ADC_MISO_PIN) == GPIO_PIN_SET)
			vTaskDelay(1);
		
		/* Read Data REG */	
		data = ExtADC_ReadDataReg();

		/* Convert data to degreees */
		temp = (float) (data - 0x800000) / 2815; //In Kelvin
		temp -= 273; //in celsius

		return temp;
}

/********************************************
 * *****************************************/

uint32_t ExtADC_ReadFullScaleReg ( void )
{
		uint32_t 	data;

		data = ExtADC_Read24bitsReg ( READ_FULLSCALE_REG);
		return data;
}

/********************************************
 * *****************************************/

uint32_t ExtADC_ReadOffsetReg ( void )
{
		uint32_t 	data;

		data = ExtADC_Read24bitsReg ( READ_OFFSET_REG);
		return data;
}

/********************************************
 * *****************************************/

uint32_t ExtADC_ReadConfReg ( void )
{
		uint32_t 	data;

		data = ExtADC_Read24bitsReg ( READ_CONF_REG);
		return data;
}

/********************************************
 * *****************************************/

uint32_t ExtADC_ReadDataReg ( void )
{
		uint32_t 	data;

		data = ExtADC_Read24bitsReg ( READ_DATA_REG);
		return data;
}

/********************************************
 * *****************************************/

uint32_t ExtADC_ReadModeReg ( void )
{
		uint32_t 	data;

		data = ExtADC_Read24bitsReg ( READ_MODE_REG);
		return data;
}

/********************************************
 * *****************************************/

uint8_t ExtADC_ReadGPOCONReg ( void )
{
		uint8_t 	data;
	
		data = ExtADC_Read8bitsReg ( READ_GPOCON_REG);
		return data;
}

/********************************************
 * *****************************************/

uint8_t ExtADC_ReadIDReg ( void )
{
		
		uint8_t 	data;
	
		data = ExtADC_Read8bitsReg ( READ_ID_REG);
		return data;
}

/********************************************
 * *****************************************/

uint8_t ExtADC_Read8bitsReg ( uint8_t reg )
{
		
    uint8_t   out, res;
		uint8_t 	data = 0;
		
		/* Enable NSS */
		ExtADC_NSSOn();
	
		/* Write communication register to allow a reading of the full-scale reg */
    /* HAL function TransmitReceive also clears trash in RX buffer */
    out = reg;
    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);

		/* A dummy write must be done in order to generate the clock and 
		 * allow the slave to put data in the MISO */
    out = 0x00;
    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &data, 1U, HAL_MAX_DELAY);

		/* Disable NSS */
		ExtADC_NSSOff();
		
		return data;
}

/********************************************
 * *****************************************/

uint32_t ExtADC_Read24bitsReg ( uint8_t reg )
{
	  uint8_t   out,res;	
		uint8_t 	i;
		uint8_t 	singleData[3];
		uint32_t 	data=0;

		for ( i = 0 ; i < 3 ; i++)
			singleData[i] = 0;
		
		/* Enable NSS */
		ExtADC_NSSOn();
	
		/* Write communication register to allow a reading of the full-scale reg */
    /* HAL function TransmitReceive also clears trash in RX buffer */
    HAL_SPI_TransmitReceive(&SPI_Handle, &reg, &res, 1U, HAL_MAX_DELAY);

		/* A dummy write must be done in order to generate the clock and 
		 * allow the slave to put data in the MISO */
    out = 0x00;
    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);
		singleData[0] = res;

    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);
		singleData[1] = res;
		
    HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);
		singleData[2] = res;

		/* Disable NSS */
		ExtADC_NSSOff();
		
		data = 	((uint32_t) singleData[0] << 16) + 
						((uint32_t) singleData[1] << 8) + 
						singleData[2];
		return data;
}

/********************************************
 * *****************************************/

t_u32_in_4 ExtADC_CalcCONFReg	( uint8_t channel )
{
	t_u32_in_4 confReg;

  if ( uChannelDifferential[channel] )
  {
    confReg.bytes.byte2 = 0x80;
    confReg.bytes.byte1 = ExtADC_GetDiffConf(channel);
    confReg.bytes.byte0 = uChannelGain[channel];//0x10 + uChannelGain[channel]; 
  }
  else
  {
    confReg.bytes.byte2 = 0x84;
    confReg.bytes.byte1 = ExtADC_GetPseudoConf(channel);
    confReg.bytes.byte0 = 0x08 + uChannelGain[channel]; 
  }

	return confReg; 
}

/********************************************
 * *****************************************/

t_u32_in_4 ExtADC_CalcMODEReg	( uint8_t channel )
{
	t_u32_in_4 modeReg;

	modeReg.bytes.byte2 = 0x28;
	modeReg.bytes.byte1 = 0x00;
	modeReg.bytes.byte0 = 0x60;

	return modeReg;
}

/********************************************
 * *****************************************/
void ExtADC_SetGain	( uint8_t channel, uint8_t gain )
{
	uChannelGain[channel] = gain;
}

/********************************************
 * *****************************************/
uint8_t ExtADC_GetGain ( uint8_t channel)
{
	return uChannelGain[channel];
}

/********************************************
 * *****************************************/

void ExtADC_SetDifferential	( uint8_t channel, uint8_t mode )
{	
	uChannelDifferential[channel] = mode;
}

/********************************************
 * *****************************************/

uint8_t ExtADC_GetDifferential	( uint8_t channel )
{
	return uChannelDifferential[channel];
}

/********************************************
 * *****************************************/

uint8_t 	ExtADC_CalcRealGain 		( uint8_t gain )
{
	switch (gain)
	{
		case EXTADC_GAIN_1:
			return 1;
			break;
		case EXTADC_GAIN_8:	
			return 8;
			break;
		case EXTADC_GAIN_16:
			return 16;
			break;
		case EXTADC_GAIN_32:
			return 32;
			break;
		case EXTADC_GAIN_64:
			return 64;
			break;
		case EXTADC_GAIN_128:
			return 128;
			break;
		default: 
			return 0;
			break;
	}
}

/******************** Creative Commons -- Robolabo *****************************/
/***************************** END OF FILE *************************************/

