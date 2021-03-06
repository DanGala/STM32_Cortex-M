/*******************************************************************************
 * HAL compatible con arduino para comunicacion con el LORA
 *******************************************************************************/


#include <stdio.h>
#include "main.h"
#include "halLora.h"
#include "radio.h"

// -----------------------------------------------------------------------------
  /* Create SPIs struct */
  SPI_HandleTypeDef SPI_Handle;
// -----------------------------------------------------------------------------
//Inicio de configuracion de pines I/O-> NSS, RXTX, RST, DIO0, DIO3
//->NSS
//->RST
//<-DIO0
//<-DIO3
//<-RXTX No se usa
void hal_io_init ()
{
  //Configuracion de pines de comuncionacion para el LoRa
  /* Create structs */
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Activate Clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure RST as OUTPUT */
  GPIO_InitStructure.Pin 	= LORA_RST_PIN;
  GPIO_InitStructure.Speed	= GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructure.Mode 	= LORA_RST_MODE;
  HAL_GPIO_Init(LORA_RST_PORT, &GPIO_InitStructure);


#if (defined CFG_1278_DRF1278F)  ||  (defined CFG_1272_SEMTECH)                        //DEFINE la version/fabricante del SX1278 de DORJI o SX1272 de SEMTECH

  // Configure DIO3 as INPUT
  GPIO_InitStructure.Pin 	= LORA_DIO3_PIN;
  GPIO_InitStructure.Speed	= GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructure.Mode 	= LORA_DIO3_MODE;
  HAL_GPIO_Init(LORA_DIO3_PORT, &GPIO_InitStructure);
#endif

#if defined(CFG_1278_NICERF1278)                          //Selecciona el CHIP SX1278 de NICERF

  /* Configure TXEN as OUTPUT */
  GPIO_InitStructure.Pin 	= LORA_TXEN_PIN;
  GPIO_InitStructure.Speed	= GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructure.Mode 	= LORA_TXEN_MODE;
  HAL_GPIO_Init(LORA_TXEN_PORT, &GPIO_InitStructure);

  /* Configure RXEN as OUTPUT */
  GPIO_InitStructure.Pin 	= LORA_RXEN_PIN;
  GPIO_InitStructure.Speed	= GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructure.Mode 	= LORA_RXEN_MODE;
  HAL_GPIO_Init(LORA_RXEN_PORT, &GPIO_InitStructure);
#endif




  /* Configure DIO0 as INPUT/INTERRUPT */
  GPIO_InitStructure.Pin 	= LORA_DIO0_PIN;
  GPIO_InitStructure.Speed	= GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructure.Mode 	= LORA_DIO0_MODE;
  HAL_GPIO_Init(LORA_DIO0_PORT, &GPIO_InitStructure);

  /* Enable and set EXTI lines 9 to 5 Interrupt to the defined priority */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, LORA_DIO0_ISR_PRIORITY, 0);
  //HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* NVIC_InitTypeDef is deprecated
  NVIC_InitStructure.NVIC_IRQChannel 	= EXTI9_5_IRQn; //External Line[9:5] Interrupts
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = LORA_DIO0_ISR_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //No importa, estamos en el grupo 4
  NVIC_InitStructure.NVIC_IRQChannelCmd 		  = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  */


}

//Se utiliza para hacer el reset en el Lora. 0/1 -> salida digital. 2-> entrada o pin flotante
void hal_pin_rst (uint8_t val) 
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Configure RST as OUTPUT */
  GPIO_InitStructure.Pin 	= LORA_RST_PIN;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  if(val == 0 || val == 1)			// drive pin    0->SX1278    1->SX1272
  {
    GPIO_InitStructure.Mode 	= LORA_RST_MODE;
    HAL_GPIO_Init(LORA_RST_PORT, &GPIO_InitStructure);
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, val?GPIO_PIN_SET:GPIO_PIN_RESET);
  }
  else
  {
    GPIO_InitStructure.Mode 	= GPIO_MODE_INPUT;
    HAL_GPIO_Init(LORA_RST_PORT, &GPIO_InitStructure);
  }
}

//Utilizada para configurar los pines de SPI
void hal_spi_init ()
{

  /* Create TypeDef vars */
  /* Create GPIOs struct */
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Create NVICs struct */
  /*NVIC_InitTypeDef NVIC_InitStructure;*/
  /* Create EXTIs struct */
  /*EXTI_InitTypeDef EXTI_InitStructure;*/

  /* Init TypeDefs */
  /* Init GPIOs struct */
  /*GPIO_StructInit(&GPIO_InitStructure);*/
  /* Init NVICs struct */
  /*NVIC_StructInit(&NVIC_InitStructure);*/
  /* Init SPIs struct */
  /*SPI_StructInit(&SPI_InitStructure);*/
  /* Init EXTIs struct */
  /*EXTI_StructInit(&EXTI_InitStructure);*/


  __HAL_RCC_SPI1_CLK_ENABLE();
  /* Activate Clock */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Config GPIOs */
  /* Configure SCK as Alternative Function PP */
  GPIO_InitStructure.Pin = LORA_SCK_PIN | LORA_MISO_PIN | LORA_MOSI_PIN;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  GPIO_InitStructure.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(LORA_SPI_PORT, &GPIO_InitStructure);

  /* Configure NSS as GPIO Output PP */
  GPIO_InitStructure.Pin = LORA_NSS_PIN;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(LORA_NSS_PORT, &GPIO_InitStructure);



  /* Config SPI */

  /* Config SPI Interrupt */
  /*NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);*/

  // Configure and enable SPI_MASTER interrupt -------------------------------
  /*NVIC_InitStructure.NVIC_IRQChannel = LORA_SPI_IRQn;*/
  /*NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;*/
  /*NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;*/
  /*NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;*/
  /*NVIC_Init(&NVIC_InitStructure);*/

  /* Config SPI mode */
  SPI_Handle.Instance = LORA_SPI;
  SPI_Handle.Init.Direction = SPI_DIRECTION_2LINES;
  SPI_Handle.Init.Mode = SPI_MODE_MASTER;
  SPI_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
  SPI_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
  SPI_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
/*SPI_Handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
  SPI_Handle.Init.CLKPhase = SPI_PHASE_2EDGE; */
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

  hal_pin_nss(1);					//NSS disable

  /*SPI_BiDirectionalLineConfig(LORA_SPI, SPI_Direction_Tx); */
  /*SPI_I2S_ITConfig(LORA_SPI, SPI_I2S_IT_TXNE, ENABLE);*/
  /*SPI_I2S_ITConfig(LORA_SPI, SPI_I2S_IT_RXNE, ENABLE);*/
  /*SPI_Cmd(LORA_SPI, ENABLE);*/

}

//Habilita o no el bus SPI
//->val 0->habilita el bus    /1->Deshabilita el bus
void hal_pin_nss (uint8_t val) {
  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, val?GPIO_PIN_SET:GPIO_PIN_RESET);
}



// Hace la tranferencia SPI con el radio. Tranfiere out, recibe/retorna res
//->out
//<-res
uint8_t hal_spi (uint8_t out) {

  uint8_t res;		
  HAL_SPI_TransmitReceive(&SPI_Handle, &out, &res, 1U, HAL_MAX_DELAY);
  return res;
}

//Hace retardos de ms segun el parametro time_delay
//-> time_delay-> crea una tarea
void hal_delay(uint8_t time_delay)
{
  vTaskDelay(time_delay);
}


//Se desarrolla la interrupcion por cambio de flanco ascendente por TXDONE o RXDONE
//Detecta si existio o no ErrorCRC comprobando bit activo en DIO3
void hal_ISR_check()				//en esta funcion se desarrollara la interrupcion
{
  uint8_t errorCRC = 0;

#if (defined CFG_1278_DRF1278F)  ||  (defined CFG_1272_SEMTECH)                        //DEFINE la version/fabricante del SX1278 de DORJI o SX1272 de SEMTECH

  if (HAL_GPIO_ReadPin(LORA_DIO3_PORT,LORA_DIO3_PIN) == GPIO_PIN_SET )
  {
    errorCRC = 1;
  }
#endif

  radio_irq_handler(errorCRC);
}



//Inicializa los pines I/O y SPI
void hal_init () 					//Configuracion de PINES para comunicaicon digital y SPI con el LORA
{
  hal_io_init();				//Configura los pines I/O control LORA
  hal_spi_init();				//Configura los pines de SPI LORA
}


//Habilitacion de funciones del SX12378 de NICERF
#if defined(CFG_1278_NICERF1278)                          //Selecciona el CHIP SX1278 de NICERF
void hal_TX_RX_en (u1_t estadoLora) {						//Se utiliza para hacer seleccionar TX o RX con la version SX1278 del NICERF
  if (estadoLora == TX_EN_OK)
  {
    HAL_GPIO_WritePin(LORA_TXEN_PORT, LORA_TXEN_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LORA_RXEN_PORT, LORA_RXEN_PIN, GPIO_PIN_RESET);
  }
  else if (estadoLora == RX_EN_OK)
  {
    HAL_GPIO_WritePin(LORA_TXEN_PORT, LORA_TXEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_RXEN_PORT, LORA_RXEN_PIN, GPIO_PIN_SET);
  }
  else							//Seleccionado TX_RX_EN_SLEEP
  {
    HAL_GPIO_WritePin(LORA_TXEN_PORT, LORA_TXEN_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_RXEN_PORT, LORA_RXEN_PIN, GPIO_PIN_RESET);
  }
}
#endif



void EXTI9_5_IRQHandler(void)
{
  /*portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;*/

  /*// Vamos a comenzar a recibir un nuevo dato: reinicio el contador de bytes recibidos*/
  /**//*EXTADC_RestartNumBytes();*/

  /*// Desactivo la interrupción de DRDY*/
  /*ExtADC_DisableDRDYInt();*/

  /*// Activo el SPI*/
  /*EXT_ADC_SPI->DR = 0;*/
  /*SPI_I2S_ReceiveData(EXT_ADC_SPI);*/
  /*SPI_I2S_ClearITPendingBit(EXT_ADC_SPI, SPI_IT_CRCERR | SPI_I2S_IT_OVR);*/

  /**//*ExtADC_SpiOn();*/

  /* Toggle Led 0 */
  /* limpio la interrupcion */
  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_8);
  hal_ISR_check();

  /*//Flag interrupt, should be changed by a Semaphore*/
  /*uExtADCReadyFlag = TRUE;*/

  /**//* Now the buffer is empty we can switch context if necessary. */
  /*if( xHigherPriorityTaskWoken ){*/
  /*portYIELD();*/
  /*}*/

}
