/**
  ******************************************************************************
  * @file    FSM_LP_Slave/source/application.c
  * @author  Daniel Gala Montes
  * @brief   Implementation of a FSM that performs the configuration and use 
  *          of a LoRa module to work as a slave in a multipoint network,
  *          waking up every minute to transmit sensed data before going back
  *          into a low power mode.
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "project_config.h"
#include "extADC.h"
#include "common.h"
#include "stmX_hal_i2c.h"
#include "thermocouple.h"
#include "halLora.h"
#include "radio.h"
#include "fsm.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t in_comm = 0; // 1 if a communication between slave and master is ongoing
uint8_t woken = 1; // 1 if system is not in LP mode
uint8_t acknowledged = 1; // 1 if last transmission has been acknowledged by master

/* I2C handle declaration */
I2C_HandleTypeDef I2cHandle;
/* LPTIM handle declaration */
LPTIM_HandleTypeDef LptimHandle;
/* FreeRTOS Task handle declaration */
xTaskHandle application_Task_Handle;

/* Timeout counter variables */
TickType_t xLastTickCount_RX;
TickType_t xLastTickCount_TX;

TickType_t ticks_timeout_RX;
TickType_t ticks_timeout_TX;

TickType_t xLastTickCount_200ms;
TickType_t ticks_desfase_tx;

/* Frame variables */
u1_t masterID;            //Define el numero del maestro
u1_t slaveID;            //Define el numero del esclavo
u1_t slaveVar;            //Define la variable enviada por el esclavo

u1_t flagEnvio;
u2_t contadorBytesRX;

//Tramas utilizadas para enviar o recibir
u1_t envioSensor[8]; // 0_masterID{0}   1_slaveID{1-3}   2_slaveVar{0-255}   3_RandomFrameID  4_varIndex 5_UNUSED 6-7_Data

u1_t ackReceived[4];
//ackReceived   0->Destino  1-> Fuente  2-> tipo sensor  3->Numero random de la trama

//Almacenamiento TRAMAS
u1_t tramaPendienteTX[200];
u1_t cantidadPendienteTX;
u1_t NumeroRandomTrama;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void I2C_Config(void);
void LPTIM_Config(void);
static void GPIO_ConfigAN(void);
void UserLed_Config(void);

/* FreeRTOS tasks */
void application_Task();

/* Private functions ---------------------------------------------------------*/

/* FSM functions */
/* Guard functions */
static int tx_ready(fsm_t* this);
static int tx_done(fsm_t* this);
static int tx_timeout(fsm_t* this);
static int rx_done(fsm_t* this);
static int rx_timeout(fsm_t* this);

static int data_ready(fsm_t* this);
static int comm_over(fsm_t* this);
static int wake_up(fsm_t* this);

/* Output functions */
static void transmit(fsm_t* this);
static void receive(fsm_t* this);
static void process(fsm_t* this);
static void reset_tx(fsm_t* this);
static void reset_rx(fsm_t* this);

static void begin_comm(fsm_t* this);
static void sleep(fsm_t* this);
static void start_sensing(fsm_t* this);

enum app_state
{
  SLEEP,
  SENSE,
  COMM
};

enum lora_state
{
  RST,
  TX,
  RX
};

static fsm_trans_t app_tt[] = {
  { SLEEP,  wake_up,      SENSE,  start_sensing },
  { SENSE,  data_ready,   COMM,   begin_comm    },
  { COMM,   comm_over,    SLEEP,  sleep         },
  { -1,     NULL,         -1,     NULL          },
};

static fsm_trans_t lora_tt[] = {
  { RST,  tx_ready,   TX,   transmit    },
  { TX,   tx_done,    RX,   receive     },
  { TX,   tx_timeout, RST,  reset_tx    },
  { RX,   rx_done,    RST,  process     },
  { RX,   rx_timeout, RST,  reset_rx    },
  { -1,   NULL,       -1,   NULL        },
};

/********************************************************************************
 application_Task
********************************************************************************/
void application_Task()
{
  ticks_timeout_TX = 2*1000 / portTICK_PERIOD_MS;
  ticks_timeout_RX = 5*1000 / portTICK_PERIOD_MS;

  masterID = 0;            //Define el numero del maestro
  slaveID = 1;             //Define el numero del esclavo
  slaveVar = 0;            //Define la variable enviada por el esclavo

  flagEnvio = 0;
  contadorBytesRX = 0x0000;

  //Tramas utilizadas para enviar o recibir
  envioSensor[0] = masterID;
  envioSensor[1] = slaveID;
  envioSensor[2] = 0;
  envioSensor[3] = 0;
  envioSensor[4] = 0;
  envioSensor[5] = 0;
  envioSensor[6] = 0;
  envioSensor[7] = 0;
  //envioSensor: 0_masterID{0}   1_slaveID{1-3}   2_slaveVar{0-255}   3_RandomFrameID  4_varIndex 5_UNUSED 6-7_Data

  //Inicializacion variable globales
  xLastTickCount_TX = xTaskGetTickCount();
  xLastTickCount_RX = xTaskGetTickCount();

  //Configuracion de parametros
  RADIO.flagTx = 0;      //Inicializacion de variables
  RADIO.flagRx = 0;
  RADIO.crc 	 = 0;
  
  int k;
  for(k=0; k<200; k++){
    tramaPendienteTX[k] = 0;
  }
  cantidadPendienteTX = 0;

  //SX1278
  RADIO.freq = 433800000;   // Use a frequency in the g3 which allows 10% duty cycling.
  RADIO.txpow = 17;         // Maximum TX power
  RADIO.imax = 100;         // Por defecto 100mA   45mA <=Imax <= 240mA

  //{ CR_4_5, CR_4_6, CR_4_7, CR_4_8 };					//Usado para CR
  //{ SF_6, SF_7, SF_8, SF_9, SF_10, SF_11, SF_12 };		//Usado para SF
  //{ BW7_8, BW10_4, BW15_6, BW20_8, BW31_25, BW41_7, BW62_5, BW125, BW250, BW500, BWrfu };

  RADIO.sf = SF_9;              //Configuracion del Modem SF a baja velocidad
  RADIO.bw = BW500;              //Configuracion del ancho de banda reducido
  RADIO.cr = CR_4_5;              //Configuracion de Coding rate

  hal_init();						//inicia el hardware
  radio_init();					//inicia el radio

  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn); //Activación de las interrupciones por DIO0

  srand( (unsigned int) ( xTaskGetTickCount() ) );
  ticks_desfase_tx = (rand() % 20) * 200 / portTICK_PERIOD_MS;  //Establece un valor random de desfase entre 0-20 (0-4s)

  fsm_t* app_fsm = fsm_new(app_tt);
  fsm_t* lora_fsm = fsm_new(lora_tt);

  while(1)
  {
    fsm_fire(app_fsm);
    while((cantidadPendienteTX > 0) && (in_comm == 1))
    {
      fsm_fire(lora_fsm);
      vTaskDelay(1);
    }
    vTaskDelay(1);
  }
}

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{
  /* STM32L4xx HAL library initialization:
     - Configure the Flash prefetch
     - Systick timer is configured by default as source of time base, but user 
     can eventually implement his proper time base source (a general purpose 
     timer for example or other time source), keeping in mind that Time base 
     duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
     handled in milliseconds basis.
     - Set NVIC Group Priority to 4
     - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 80 MHz */
  SystemClock_Config();

  HAL_Delay(2000);

  LPTIM_Config();

  UserLed_Config();

  xTaskCreate(application_Task, "application_Task", TEST_LORA_TASK_STACK, NULL, 1, &application_Task_Handle);
  vTaskStartScheduler();
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_PeriphCLKInitTypeDef  RCC_PeriphCLKInitStruct;

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_MSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Re-target the HSI to Clock the I2C Interface ################# */
  /* Select the LSE clock as I2C peripheral clock */
  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2Cx;
  RCC_PeriphCLKInitStruct.I2c1ClockSelection = RCC_I2CxCLKSOURCE;
  if(HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct) != HAL_OK)
  {
    while(1);
  }

  /* Re-target the LSE to Clock the LPTIM Counter ################# */
  /* Select the LSE clock as LPTIM peripheral clock */
  RCC_PeriphCLKInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
  RCC_PeriphCLKInitStruct.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSE;
  HAL_RCCEx_PeriphCLKConfig(&RCC_PeriphCLKInitStruct);
}

/* FSM functions */
/* Guard functions */
static int tx_ready(fsm_t* this)
{
  return ((cantidadPendienteTX>0) && ((xTaskGetTickCount() - xLastTickCount_200ms) >= ticks_desfase_tx) && (flagEnvio == 0));
}

static int tx_done(fsm_t* this)
{
  return ((flagEnvio == 1)  && (RADIO.flagTx == 1));
}

static int tx_timeout(fsm_t* this)
{
  return ((xTaskGetTickCount() - xLastTickCount_TX) >= ticks_timeout_TX);
}

static int rx_done(fsm_t* this)
{
  return RADIO.flagRx;
}

static int rx_timeout(fsm_t* this)
{
  return ((xTaskGetTickCount() - xLastTickCount_RX) >= ticks_timeout_RX);
}

static int data_ready(fsm_t* this)
{
  return (cantidadPendienteTX > 0);
}

static int comm_over(fsm_t* this)
{
  return (in_comm && cantidadPendienteTX==0 && acknowledged);
}

static int wake_up(fsm_t* this)
{
  return woken;
}

/* Output functions */
static void transmit(fsm_t* this)
{
  radio_mode(RADIO_RST);                                  //Detiene la RX antes
  flagEnvio = 1;
  xLastTickCount_200ms = xTaskGetTickCount();
  RADIO.flagTx = 0;

  NumeroRandomTrama = tramaPendienteTX[1];               //Este numero se utilizara para comprobar que se TX

  envioSensor[2] = tramaPendienteTX[0];
  envioSensor[3] = tramaPendienteTX[1];
  envioSensor[4] = tramaPendienteTX[2]; 
  envioSensor[6] = tramaPendienteTX[4]; 
  envioSensor[7] = tramaPendienteTX[5]; 
  //envioSensor: 0_masterID{0}   1_slaveID{1-3}   2_slaveVar{0-255}   3_RandomFrameID  4_varIndex 5_UNUSED 6-7_Data

  radio_buffer_to_frameTX(envioSensor, 8);                  //Carga 8 bytes a la FIFO de LORa-> Activa TX
  radio_mode(RADIO_TX);     //Configura la TX

  xLastTickCount_TX = xTaskGetTickCount();    //Inicializa contador de segundos TX
  xLastTickCount_RX = xTaskGetTickCount();    //Inicializa contador de segundos RX
}

static void receive(fsm_t* this)
{
  flagEnvio = 0;
  RADIO.flagTx = 0;
  RADIO.flagRx = 0;

  radio_mode(RADIO_RX);                     //Pasa a MODO RX
  xLastTickCount_RX = xTaskGetTickCount();    //Inicializa contador de segundos RX
  xLastTickCount_TX = xTaskGetTickCount();    //Inicializa contador de segundos TX
}

static void process(fsm_t* this)
{
  flagEnvio = 0;                          //Fuerza al estado de TX siempre y cuando se cumplan las condiciones

  if(RADIO.crc == 1)                      //ERROR (posible estado extra de tratamiento del error)
  {
    RADIO.crc = 0;
  }

  if(RADIO.dataLenRX == 4)       //Nota, son 4 bytes de confirmacion
  {
    int i;
    for (i=0; i< 4; i++)                        //Extrae los datos del  los datos de SENSOR a la TRAMA
    {
      ackReceived[i] = RADIO.frameRX[i];
    }
    if( (ackReceived[0] == slaveID) && (ackReceived[1] == masterID) )
    {
      if(ackReceived[3] == NumeroRandomTrama)         //Se confirma que la ultima trama, la recibio el maestro
      {
        radio_mode(RADIO_RST);          //Coloca el dispositivo a Dormir
        if(cantidadPendienteTX > 1)          //Tiene pendiente la TX de antiguas tramas?
        {//Es necesario desplazar/rotar tramaPendienteTX[]  6 posiciones
          cantidadPendienteTX--;
          for (i=0; i<(cantidadPendienteTX*6);i++)
          {
            tramaPendienteTX[i]=tramaPendienteTX[i+6];
          }
        }
        else
        {
          cantidadPendienteTX = 0;      //No tiene tramas pendientes por enviar
          acknowledged=1;
        }
      }
    }
  }
  srand( (unsigned int) ( xTaskGetTickCount() ) );
  ticks_desfase_tx = (rand() % 20) * 200 / portTICK_PERIOD_MS;  //Establece un valor random de desfase entre 0-20 (0-4s)
  xLastTickCount_200ms = xTaskGetTickCount();
}

static void reset_tx(fsm_t* this)
{
  flagEnvio = 0;
  xLastTickCount_RX = xTaskGetTickCount();    //Inicializa contador de segundos RX
  xLastTickCount_TX = xTaskGetTickCount();    //Inicializa contador de segundos TX
  xLastTickCount_200ms = xTaskGetTickCount() - ticks_desfase_tx;  //Fuerza a que no exista desfase
}

static void reset_rx(fsm_t* this)
{
  flagEnvio = 0;
  xLastTickCount_RX = xTaskGetTickCount();    //Inicializa contador de segundos RX
  xLastTickCount_TX = xTaskGetTickCount();    //Inicializa contador de segundos TX
  srand( (unsigned int) ( xTaskGetTickCount() ) );
  ticks_desfase_tx = (rand() % 20) * 200 / portTICK_PERIOD_MS;  //Establece un valor random de desfase entre 0-20 (0-4s)
  xLastTickCount_200ms = xTaskGetTickCount();
}

static void begin_comm(fsm_t* this)
{
  in_comm = 1;
  acknowledged=0;
}

static void sleep(fsm_t* this)
{
  woken = 0;
  in_comm = 0;
  
  /* Pause SPI for Lora module */
  hal_spi_pause();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Prepare for low power mode entry*/
  vTaskSuspendAll();

  /* Disable I2C peripheral to avoid malfunction #2 of the errata sheet */
  xHAL_I2C_DeInit(&I2cHandle);

  /* Configure all GPIO's to AN to reduce the power consumption*/
  //GPIO_ConfigAN();

  /*Suspend Tick increment to prevent wakeup by Systick interrupt.*/ 
  //HAL_SuspendTick();

  /* Ensure that MSI is wake-up system clock */
  __HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_MSI);

  /* Start LPTIM timeout to wake up from Stop mode */
  HAL_LPTIM_TimeOut_Start_IT(&LptimHandle, LPTIM_PERIOD, LPTIM_TIMEOUT);

  /* Enter STOP 1 mode */
  HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);

  /* Stop LPTIM Timeout */
  HAL_LPTIM_TimeOut_Stop_IT(&LptimHandle);

  /* Re-configure the system clock to 80 MHz based on MSI, enable and
     select PLL as system clock source (PLL is disabled in STOP mode) */
  SystemClock_Config();

  //HAL_ResumeTick();

  /* Re-configure all GPIO's to the application's configuration */
  UserLed_Config();
  hal_init();

  xTaskResumeAll();

}

static void start_sensing(fsm_t* this)
{
  int j;
  float voltage;
  t_word16 var;

  I2C_Config();
  /* Initialize thermocouples */
  Therm_Init();

  /* Prepare first transmission frame */
  cantidadPendienteTX++;
  j=(cantidadPendienteTX -1)*6;                 //Solamente almacena el Byte Random y los 5 bytes de informacion
  tramaPendienteTX[j] = THERMOCOUPLE;           //establece la variable de la Trama a termopares
  tramaPendienteTX[j+1] = rand() % 255;         //establece el numero RANDOM de la Trama, como comprobacion
  tramaPendienteTX[j+2] = 0x00;       // indica que es la medida del primer canal de termopares
  var.word16 = Therm_ReadTemp(0);  // Obtain temperature value from thermocouple connected to ADC's channel 0
  tramaPendienteTX[j+4] = var.bytes.byte_up;    //First data byte is channel 0 temperature's MSB
  tramaPendienteTX[j+5] = var.bytes.byte_down;  //Second data byte is channel 0 temperature's LSB

  /* Prepare second transmission frame */
  cantidadPendienteTX++;
  j=(cantidadPendienteTX -1)*6;                 //Solamente almacena el Byte Random y los 5 bytes de informacion
  tramaPendienteTX[j] = THERMOCOUPLE;    //establece la variable de la Trama a termopares
  tramaPendienteTX[j+1] = rand() % 255;         //establece el numero RANDOM de la Trama, como comprobacion
  tramaPendienteTX[j+2] = 0x01;       // indica que es la medida del segundo canal de termopares
  var.word16 = Therm_ReadTemp(1);  // Obtain temperature value from thermocouple connected to ADC's channel 1
  tramaPendienteTX[j+4] = var.bytes.byte_up;    //First data byte is channel 1 temperature's MSB
  tramaPendienteTX[j+5] = var.bytes.byte_down;  //Second data byte is channel 1 temperature's LSB

  /* Prepare third transmission frame */
  cantidadPendienteTX++;
  j=(cantidadPendienteTX -1)*6;                 //Solamente almacena el Byte Random y los 5 bytes de informacion
  tramaPendienteTX[j] = ANALOG;             //establece la variable de la Trama a analog
  tramaPendienteTX[j+1] = rand() % 255;         //establece el numero RANDOM de la Trama, como comprobacion
  tramaPendienteTX[j+2] = 0x00;            // indica que es la medida del primer canal de analogs
  voltage = ExtADC_ReadVoltageInput(2);
  var.word16 = (uint16_t)(voltage);
  tramaPendienteTX[j+4] = var.bytes.byte_up;    //First data byte is channel 2 voltage's MSB
  tramaPendienteTX[j+5] = var.bytes.byte_down;  //Second data byte is channel 2 voltage's LSB

  /* Prepare fourth transmission frame */
  cantidadPendienteTX++;
  j=(cantidadPendienteTX -1)*6;                 //Solamente almacena el Byte Random y los 5 bytes de informacion
  tramaPendienteTX[j] = ANALOG;             //establece la variable de la Trama a analog
  tramaPendienteTX[j+1] = rand() % 255;         //establece el numero RANDOM de la Trama, como comprobacion
  tramaPendienteTX[j+2] = 0x01;            // indica que es la medida del segundo canal de analogs
  voltage = ExtADC_ReadVoltageInput(3);
  var.word16 = (uint16_t)(voltage);
  tramaPendienteTX[j+4] = var.bytes.byte_up;    //First data byte is channel 2 voltage's MSB
  tramaPendienteTX[j+5] = var.bytes.byte_down;  //Second data byte is channel 2 voltage's LSB

  /* Pause SPI for external ADC */
  ExtADC_DeInit();
  /* Initialize SPI for LoRa */
  hal_spi_init();
}

/**************************************************************************/
/*I2C configuration                                                       */
/*  Configure I2C in order to enable communication with devices connected */
/**************************************************************************/
void I2C_Config(void)
{ 
  /*##-2- Enable peripherals and GPIO Clocks #################################*/
  /* Enable I2Cx clock */
  I2Cx_CLK_ENABLE();

  /*##-3- Configure the I2C peripheral ######################################*/
  I2cHandle.Instance              = I2Cx;
  I2cHandle.Init.Timing           = I2C_TIMING;
  I2cHandle.Init.OwnAddress1      = 0x00;
  I2cHandle.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  I2cHandle.Init.OwnAddress2      = 0x00;
  I2cHandle.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  I2cHandle.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

  if(xHAL_I2C_Init(&I2cHandle) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}

/**
 * @brief  Configure LPTIM in order to generate a wake-up interrupt every minute
 * @param  None
 * @retval None
 */
void LPTIM_Config(void)
{
  /* ### - 1 - Initialize LPTIM peripheral ################################## */
  /*
   *  Instance        = LPTIM1.
   *  Clock Source    = APB or LowPowerOSCillator
   *  Counter source  = Internal event.   
   *  Clock prescaler = 32.
   *  Counter Trigger = Software trigger
   */

  LptimHandle.Instance = LPTIM1;

  LptimHandle.Init.Clock.Source       = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  LptimHandle.Init.Clock.Prescaler    = LPTIM_PRESCALER_DIV32;
  LptimHandle.Init.Trigger.Source     = LPTIM_TRIGSOURCE_SOFTWARE;
  LptimHandle.Init.CounterSource      = LPTIM_COUNTERSOURCE_INTERNAL;
  LptimHandle.Init.Input1Source       = LPTIM_INPUT1SOURCE_GPIO;
  LptimHandle.Init.Input2Source       = LPTIM_INPUT2SOURCE_GPIO;

  /* Initialize LPTIM peripheral according to the passed parameters */
  if (HAL_LPTIM_Init(&LptimHandle) != HAL_OK)
  {
    while(1);
  }

  /* ### - 2 - Start the Timeout function in interrupt mode ################# */
  /*
   *  Period = 65535
   *  Pulse  = 61439
   *  According to this configuration (LPTIMER clocked by LSE & compare = 61439,
   *  the Timeout period = (compare + 1)/LSE_Frequency = 60s
   */
  //HAL_LPTIM_TimeOut_Start_IT(&LptimHandle, LPTIM_PERIOD, LPTIM_TIMEOUT);

}

/**
 * @brief  Compare match callback in non blocking mode 
 * @param  hlptim : LPTIM handle
 * @retval None
 */
void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
  /* Timeout was reached, confirm wakeup */
  woken = 1;
}

/**
 * @brief  Configure all GPIO's to AN to reduce the power consumption
 * @param  None
 * @retval None
 */
static void GPIO_ConfigAN(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Configure all GPIO as analog to reduce current consumption on non used IOs */
  /* Enable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();  
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);  
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* Disable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();   
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();
}

void UserLed_Config(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef     GPIO_InitStruct;
  /* Configure PA0 as GPIO Output PP for LED debugging */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
}

void PreSleepProcessing(uint32_t ulExpectedIdleTime)
{
}

void PostSleepProcessing(uint32_t ulExpectedIdleTime)
{
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
