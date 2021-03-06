/**
  ******************************************************************************
  * @file    Multipunto/Master/source/main.c
  * @author  Daniel Gala Montes
  * @brief   This example describes how to configure and use a LoRa module through
  *          the STM32L4xx HAL API to perform a demo point-multipoint net app.
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "halLora.h"
#include "radio.h"
#include "uartDriver.h"
#include "fsm.h"
#include "common.h"
/* Scheduler includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
xTaskHandle lora_Task_Handle;

TickType_t xLastTickCount_RX;
TickType_t xLastTickCount_TX;
TickType_t ticks_timeout_RX;
TickType_t ticks_timeout_TX;

u1_t NumeroMaestro;            //Define el numero del maestro
u1_t cantidadEsclavo;          //Define el numero de dispositivos esclavos

u1_t FlagRecibir;
u2_t contadorBytesRX;
u1_t transmiteConfirmacion[4];  //transmiteConfirmacion   0->Destino  1-> Fuente  2-> tipo sensor  3->Número random de la trama

uint8_t printbuf[30];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void lora_Task();

/* Private functions ---------------------------------------------------------*/

/* FSM functions */
/* Guard functions */
static int is_idle(fsm_t* this);
static int rx_done(fsm_t* this);
static int rx_err(fsm_t* this);
static int rx_timeout(fsm_t* this);
static int tx_done(fsm_t* this);
static int tx_timeout(fsm_t* this);
/* Output functions */
static void err_handle(fsm_t* this);
static void receive(fsm_t* this);
static void process(fsm_t* this);
static void reset(fsm_t* this);
static void wait_rx(fsm_t* this);
static void start_rx(fsm_t* this);

enum lora_state
{
  RST,
  RX,
  TX
};

static fsm_trans_t lora_tt[] = {
  { RST,  is_idle,      RX,   start_rx    },
  { RX,   rx_done,      TX,   process     },
  { RX,   rx_err,       RST,  err_handle  },
  { RX,   rx_timeout,   RX,   wait_rx     },
  { TX,   tx_done,      RX,   receive     },
  { TX,   tx_timeout,   RST,  reset       },
  { -1,   NULL,         -1,   NULL        },
};

/********************************************************************************
 lora_Task
 Prueba para puesta en funcionamiento de LORA: demo Multipunto - Maestro
********************************************************************************/
void lora_Task()
{
  println("MASTER\r\n",8);

  NumeroMaestro = 0;            //Define el numero del maestro
  cantidadEsclavo = 3;            //Define la cantidad de dispositivos esclavos

  ticks_timeout_TX = 3*1000 / portTICK_PERIOD_MS;
  ticks_timeout_RX = 5*1000 / portTICK_PERIOD_MS;

  FlagRecibir        = 0;
  contadorBytesRX    = 0x0000;
  transmiteConfirmacion[0] = 0; 
  transmiteConfirmacion[1] = NumeroMaestro; 
  transmiteConfirmacion[2] = 0; 
  transmiteConfirmacion[3] = 0; 

  //Inicialización de variables del driver
  RADIO.flagTx = 0;      
  RADIO.flagRx =0;
  RADIO.crc = 0;

  //Intervalo de frecuencia de 433.050 MHz to 434.790 MHz: BW: 1.74 MHz
  //Configuración de frecuencia
  RADIO.freq = 433800000;             //Frecuencias ISM para SX1278
  //RADIO.freq = 869525000;           //Frecuencias ISM para SX172

  //Configuración de potencia 
  RADIO.txpow = 17;                   //Maxima TX power
  RADIO.imax = 100;                   //Por defecto 100mA   45mA <=Imax <= 240mA

  /*
     Aumento máximo de ganancia. LNA máx. corriente 150%  LNA_RX_GAIN
     Máxima capacidad del Payload 64 
     LORARegSymbTimeoutLsb tiempo Timeout de símbolos es 0 
     Palabra reservada para Lora networks LORARegSyncWord 0x34
     Configuración de Pines DIO0=RXDONE DIO3=PAYLOADCRCERROR
     Configura LoRa modem (RegModemConfig1, RegModemConfig2) valores de RADIO.sf RADIO.bw RADIO.cr
     Configura el Canal de acuerdo con la frecuencia de RADIO.freq en 3 registros 
     Configura la potencia a Max potencia RADIO.txpow. Potencia limitada hasta 20dBm
     RADIO.sf = SF_6, SF_7, SF_8, SF_9, SF_10, SF_11, SF_12 
     RADIO.cr = CR_4_5, CR_4_6, CR_4_7, CR_4_8       
     RADIO.bw = BW7_8, BW10_4, BW15_6, BW20_8, BW31_25, BW41_7, BW62_5, BW125, BW250, BW500, BWrfu 
     */

  //Variables de configuración de la comunicación LoRa
  RADIO.sf = SF_9;                //Configuración del Spread factor
  RADIO.bw = BW500;               //Configuración del ancho de canal 
  RADIO.cr = CR_4_5;              //Configuracion de Coding rate

  RADIO.statusRXTX = RADIO_RST;  //Se incorpora para modificar en el main*loop

  hal_init();                     //Configuración de puertos de arduino
  radio_init();                   //Configuración de inicio para los LoRa

  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn); //Activación de las interrupciones por DIO0

  println("Inicia\r\n",8);

  fsm_t* lora_fsm = fsm_new(lora_tt);

  while(1)
  {
    fsm_fire(lora_fsm);
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
  
  /* Configure serial communication through UART */
  usartInit();

  xTaskCreate(lora_Task, "lora_Task", TEST_LORA_TASK_STACK, NULL, 1, &lora_Task_Handle);
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

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
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
}

/* FSM functions */
/* Guard functions */
static int is_idle(fsm_t* this)
{
  return (FlagRecibir == 0);
}

static int rx_done(fsm_t* this)
{
  return ( (RADIO.crc == 0) && 
           (RADIO.flagRx == 1) && 
           (RADIO.dataLenRX == 8) && 
           (RADIO.frameRX[0] == NumeroMaestro) && 
           (RADIO.frameRX[1] > 0) && 
           (RADIO.frameRX[1] <= cantidadEsclavo) );
}

static int rx_err(fsm_t* this)
{
  return RADIO.crc; 
}

static int rx_timeout(fsm_t* this)
{
  return ((RADIO.flagRx == 0) && ((xTaskGetTickCount() - xLastTickCount_RX) >= ticks_timeout_RX));
}

static int tx_done(fsm_t* this)
{
  return RADIO.flagTx;
}

static int tx_timeout(fsm_t* this)
{
  return ((xTaskGetTickCount() - xLastTickCount_TX) >= ticks_timeout_TX);
}

/* Output functions */
static void process(fsm_t* this)
{
  RADIO.flagRx = 0;                             //Apaga flag RX
  if(RADIO.dataLenRX == 8)                   //Nota, son 8 bytes de confirmacion
  {
    if( (RADIO.frameRX[0] == NumeroMaestro) && (RADIO.frameRX[1]>0) && (RADIO.frameRX[1]<=cantidadEsclavo) )    
    {//identifica que la informacion sea de un esclavo
      println("Info Esclavo:\r\n",15);
      snprintf(printbuf, 10, "%hu", RADIO.frameRX[1]);
      println(printbuf,strlen(printbuf));
      println("\r\nTrama OK!<<\r\n",15);

      transmiteConfirmacion[0] = RADIO.frameRX[1];        //Almacena número de Esclavo
      transmiteConfirmacion[2] =  RADIO.frameRX[2];       //Almacena tipo de variable del sensor
      transmiteConfirmacion[3] =  RADIO.frameRX[3];       //Almacena Numero Random de la Comunicación

      RADIO.statusRXTX = RADIO_TX;                         //Flag para determinar donde esta el LORA-> esto se puede hacer desde dentro las líbrerias      
      radio_buffer_to_frameTX(transmiteConfirmacion, 4);          //trasnmite 4 bytes

      println("Envia>>\r\n",9);

      RADIO.flagTx = 0;
      radio_mode(RADIO_TX);                                 //Configura la TX

      println(">>FIFO\r\n",8);
    }
    else
    {
      RADIO.statusRXTX = RADIO_RST;                         //Flag para determinar donde esta el LORA-> esto se puede hacer desde dentro las líbrerias      
      println("Info sin formato!\r\n",19);
    }
  }
  else
  {
    RADIO.statusRXTX = RADIO_RST;                         //Flag para determinar donde esta el LORA-> esto se puede hacer desde dentro las líbrerias      
    println("Exceso en Bytes!\r\n",18);
  }  

  xLastTickCount_TX = xTaskGetTickCount();    //Inicializa contador de segundos TX
}

static void receive(fsm_t* this)
{
  t_word16 var;
  var.bytes.byte_up  = RADIO.frameRX[7];
  var.bytes.byte_down  = RADIO.frameRX[6];
  println("Confirma TX!\r\n",14);

  RADIO.flagTx = 0;      
  contadorBytesRX = contadorBytesRX + 8;

  println("TOTAL recibidos ",16);
  snprintf(printbuf, 10, "%hu", contadorBytesRX);
  println(printbuf,strlen(printbuf));
  println(" datos\r\n",8);
  if (RADIO.frameRX[2] == 0){
    if (RADIO.frameRX[4] == 0){
      snprintf(printbuf, 12, "temp0: %lu", var.word16);
    } else {
      snprintf(printbuf, 12, "temp1: %lu", var.word16);
    }
  } else {
    if (RADIO.frameRX[4] == 0){
      snprintf(printbuf, 14, "analog0: %lu", var.word16);
    } else {
      snprintf(printbuf, 14, "analog1: %lu", var.word16);
    }
  }
  //snprintf(printbuf, 20, "%hu%hu%hu%hu", RADIO.frameRX[4],RADIO.frameRX[5],RADIO.frameRX[6],RADIO.frameRX[7]);
  println(printbuf,strlen(printbuf));
  println("\r\n",2);

  radio_mode(RADIO_RX);                   //Pasa a MODO RX
  RADIO.statusRXTX = RADIO_RX;            // Utilizado como flag coloca la variable de modo en RX
  xLastTickCount_RX = xTaskGetTickCount();    //Inicializa contador de segundos RX
}

static void err_handle(fsm_t* this)
{
  FlagRecibir = 0;
  RADIO.flagRx = 0;                             //Apaga flag RX
  RADIO.statusRXTX = RADIO_RST;                 //Forza al estado de TX siempre y cuando se cumplan las condiciones
  println("Datos Errados!!! sin acciones\r\n",31);
  RADIO.crc = 0;      
}

static void reset(fsm_t* this)
{
  if ( RADIO.statusRXTX != RADIO_RST ){
    println("Superado el tiempo de TX. Vuelve a TX\r\n",39);
    RADIO.statusRXTX = RADIO_RST;           //<- RST
  }
  FlagRecibir = 0;
}

static void wait_rx(fsm_t* this)
{
  println("No ha recibido datos...\r\n",25);
  xLastTickCount_RX = xTaskGetTickCount();    //Inicializa contador de segundos RX
}

static void start_rx(fsm_t* this)
{
  FlagRecibir = 1;
  RADIO.flagRx = 0;      
  println("Espera dato RX\r\n",16);
  radio_mode(RADIO_RX);                     //Pasa a MODO RX
  RADIO.statusRXTX = RADIO_RX;              // Utilizado como flag coloca la variable de modo en RX
  xLastTickCount_RX = xTaskGetTickCount();    //Inicializa contador de segundos RX
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
