/**
  ***********************************************************************************
  * @file	    P_Controller/src/main.c
  * @author	  Daniel Gala Montes
  * @version	V1.0
  * @date	    7-December-2016
  * @brief	  This code implements a proportional controller for a BLDC motor with 
		          Hall sensors for motor driving and an incremental enconder for
              positioning.
  ***********************************************************************************
**/

/* Includes -----------------------------------------------------------------------*/
#include "main.h"

/* Scheduler includes -------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

xTaskHandle pControllerTask_Handle;
void pController_Task();

/* Private typedef ----------------------------------------------------------------*/
/* Private define -----------------------------------------------------------------*/
#define PERIOD_VALUE	(uint32_t)(359)	/* Period Value */
#define CPR       (uint32_t)(2000)	/* Quadrature cycles per revolution */
#define REDUCT		(float)(13.795918)	/* Gear reduction ratio */
#define PI		    (float)(3.14159265358979323846) /* Math Pi constant */
#define REF		    (float)(PI/4) 	/* Reference signal in radians */
#define U_NOM		  (uint32_t)(24)	/* Nominal motor voltage */
#define U_MAX		  (float)(24)	/* Maximum voltage to drive the motor with speed limit */
#define KP		    (float)(25)    /* Proportional gain */
#define LENGTHB   (uint32_t)(10)   /* Length of the interpolation polynomial vector */

/* Private macro ------------------------------------------------------------------*/
/* Private variables --------------------------------------------------------------*/
volatile uint32_t       pulseValue;   /* Duty cycle for PWM signals */
volatile int32_t        pulses; /* Pulses on the encoder lines */
volatile int32_t        currentState;   /* Current state of the motor driver */
volatile int32_t        stop;   /* Motor stop flag */
volatile int32_t        rotation;       /* Sense of rotation */
const int32_t           hallState[8] = {-1, 5, 3, 4, 1, 0, 2, -1}; /* Hall sensors sector associated driving state */
volatile uint8_t        initState;  /* Hall sensors init state */
const int8_t            encoderState[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; /* Encoder state transition result */
volatile uint8_t        RotatoryCurrentState; /* Current state of the encoder channels */
volatile uint8_t        RotatoryTransition; /* Transition between current and last encoder channels state */
volatile int32_t        firstStep;      /* Initialization flag for 6Step control */
volatile float          position; /* Angular position of the motor in radians */ 
volatile float          u;  /* Ideal voltage to apply */
volatile float          uEq; /* Equivalent voltage to apply considering nonlinear behaviour */
const float             b[LENGTHB] = {2.890995842970301,
                                     -0.138481392424684,
                                    0.00401997409010371,
                                   -5.66545056846648e-5,
                                    4.40683499622806e-7,
                                   -2.02269663620287e-9,
                                    5.6106480364326e-12,
                                  -9.23735018122149e-15,
                                   8.30707063167539e-18,
                                  -3.14281819671145e-21}; /* Interpolation polynomial vector */

/* GPIO Configuration Structure declaration */
GPIO_InitTypeDef GPIO_InitStruct;

/* Timer handler declaration */
TIM_HandleTypeDef TimHandle;
TIM_HandleTypeDef Encoder_Handle;
TIM_HandleTypeDef Hall_Handle;

/*Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sConfig;

/* Timer Hall sensor Configuration Structure declaration */
TIM_HallSensor_InitTypeDef sHallConfig;

/* Timer Encoder Configuration Structure declaration */
TIM_Encoder_InitTypeDef sEncoderConfig;

/* Counter Prescaler value */
uint32_t uhPrescalerValue = 0;

/* Private function prototypes ----------------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
static void driveMotor(void);
static float calculateEqV(float v);
static void setVoltage(float v);

/* Private functions --------------------------------------------------------------*/

/**
  * @brief	Main program.
  * @param	None
  * @retval	None
**/
int main(void)
{

  pulseValue = 0;
  pulses = 0;
  position = 0;
  RotatoryCurrentState = 0x00;
  RotatoryTransition = 0;
  rotation = 1;
  firstStep = 1;
  stop = 0;
  currentState = 5;
  u = 0;
  uEq = 0;

  /* STM32F4xx HAL library initialization:
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

  /* Configure the system clock to 180 MHz */
  SystemClock_Config();

  /* -1- Enable each GPIO Clock (to be able to program the configuration registers) */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  
  /* -2- Configure IOs in output push-pull mode */
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pin = GPIO_PIN_5;            // User LED 
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Mode	= GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull	= GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pin = PHASEAEN | PHASEBEN | PHASECEN;	// Phase enable pins 
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pin = PHASEA | PHASEB | PHASEC; // Phase PWM output signals 
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pin = ENCODERA | ENCODERB;  //Encoder channel pins 
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pin = HALLA; // Hall sensor A pin 
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pin = HALLB | HALLC; // Hall sensors B and C pins
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* ---------------------------------------------------------------------------*/

  /* Encoder init state */
  RotatoryCurrentState = (HAL_GPIO_ReadPin(GPIOC, ENCODERA) << 1) | HAL_GPIO_ReadPin(GPIOC, ENCODERB);
  RotatoryTransition = (RotatoryTransition << 2) | RotatoryCurrentState;

  /* Hall sensors init state */
  initState = 0x00;
  initState |= (HAL_GPIO_ReadPin(GPIOA,HALLA) << 2) | (HAL_GPIO_ReadPin(GPIOB,HALLB) << 1) | HAL_GPIO_ReadPin(GPIOB,HALLC);
  currentState = hallState[initState & 0x07];

/* -----------------------------------------------------------------------------

##-1- Configure the TIM peripheral ###########################################

  TIM1 Configuration: generate 3 PWM signals with the same duty cycle.

    In this code TIM1 input clock (TIM1CLK) is set to APB1 clock x 4,
    since APB1 prescaler is equal to 4.
      TIM1CLK = APB1CLK*4
      APB1CLK = HCLK/4
      => TIM1CLK = HCLK = SystemCoreClock
    To get TIM1 counter clock at 16 MHz, the prescaler is computed as follows:
      Prescaler = (TIM1CLK / TIM1 counter clock) - 1
      Prescaler = ((SystemCoreClock) / 16 MHz) - 1
    To get TIM1 output clock at 50 KHz, the period (ARR)) is computed as follows:
      ARR = (TIM1 counter clock / TIM1 output clock) - 1
          = 319

    TIM1 duty cycle = (TIM1_CCR1 / TIM1_ARR + 1)* 100 = 25%
------------------------------------------------------------------------------*/

  /* Compute the prescaler value to have TIM1 counter clock equal to 16 MHz */
  uhPrescalerValue = (uint32_t)(SystemCoreClock / 18000000) - 1;

  /* Initialize TIM1 peripheral as follows:
	+ Prescaler = (SystemCoreClock / 18000000) - 1
	+ Period = (359)
	+ ClockDivision = 0
	+ Counter direction = Up
  */

  TimHandle.Instance 		           = TIM1;
  TimHandle.Init.Prescaler	       = uhPrescalerValue;
  TimHandle.Init.Period 	         = PERIOD_VALUE;
  TimHandle.Init.ClockDivision 	   = 0;
  TimHandle.Init.CounterMode 	     = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;

  if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the PWM channels ##############################################*/
  sConfig.OCMode	        = TIM_OCMODE_PWM1;
  sConfig.OCNPolarity	    = TIM_OCNPOLARITY_HIGH;
  sConfig.OCNIdleState	  = TIM_OCNIDLESTATE_RESET;
  sConfig.OCIdleState	    = TIM_OCNIDLESTATE_RESET;
  sConfig.Pulse 	        = pulseValue;
  
  /* Set the pulse value for channel 1 */
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  } 

  /* Set the pulse value for channel 2 */
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
  {
  
    /* Configuration Error */
    Error_Handler();
  }

  /* Set the pulse value for channel 3 */
  if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

/* ------------------------------------------------------------------------------
  TIM2 Configuration: Hall sensor interface mode.

  - Configure 3 timer inputs ORed to the TI1 input channel by writing the TI1S bit
    in TIM2_CR2 register to '1'.
  - Program the time base: write the TIM2_ARR to the max value (the counter must
    cleared by the TI1 change. Set the prescaler to get a maximum counter period
    longer than the time between 2 changes on the sensors.
  - Program channel 1 in capture mode (TRC selected): write the CC1S bits in the
    TIM2_CCMR1 register to '01'. The user can also program the digital filter if
    needed.
--------------------------------------------------------------------------------*/
  /* Initialize TIMx peripheral as follows:
        + Prescaler = 0
        + Period = 0xFFFF
        + ClockDivision = 0
        + Counter direction = Up
  */

  Hall_Handle.Instance               = TIM2;
  Hall_Handle.Init.Prescaler         = 0;
  Hall_Handle.Init.Period            = 0xFFFF;
  Hall_Handle.Init.ClockDivision     = 0;
  Hall_Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  Hall_Handle.Init.RepetitionCounter = 0;

  if (HAL_TIM_IC_Init(&Hall_Handle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
/*##-2- Configure the Input Capture channel ################################*/
  /* Configure the Input Capture of channel 2 */
  sHallConfig.IC1Polarity  = TIM_ICPOLARITY_BOTHEDGE;
  sHallConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sHallConfig.IC1Filter    = 0;
  sHallConfig.Commutation_Delay = 0;
  if (HAL_TIMEx_HallSensor_Init(&Hall_Handle, &sHallConfig) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

/*##-3- Start the Hall sensor interface mode ##############################*/
  if (HAL_TIMEx_HallSensor_Start_IT(&Hall_Handle) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }
/*##-4- Enable the interrupt for TIM2 ####################################*/
  HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
/*--------------------------------------------------------------------------*/

  /* -1- Initialize TIM3 to handle the encoder sensor */
  /* Initialize TIM3 peripheral as follow:
   *        + Period = 65535
   *        + Prescaler = 0
   *        + ClockDivision = 0
   *        + Counter direction = Up
  */
  Encoder_Handle.Instance = TIM3;

  Encoder_Handle.Init.Period            = 0xFFFF;
  Encoder_Handle.Init.Prescaler         = 0;
  Encoder_Handle.Init.ClockDivision     = 0;
  Encoder_Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  Encoder_Handle.Init.RepetitionCounter = 0;

  sEncoderConfig.EncoderMode        = TIM_ENCODERMODE_TI12;

  sEncoderConfig.IC1Polarity        = TIM_ICPOLARITY_BOTHEDGE;
  sEncoderConfig.IC1Selection       = TIM_ICSELECTION_DIRECTTI;
  sEncoderConfig.IC1Prescaler       = TIM_ICPSC_DIV1;
  sEncoderConfig.IC1Filter          = 0;

  sEncoderConfig.IC2Polarity        = TIM_ICPOLARITY_BOTHEDGE;
  sEncoderConfig.IC2Selection       = TIM_ICSELECTION_DIRECTTI;
  sEncoderConfig.IC2Prescaler       = TIM_ICPSC_DIV1;
  sEncoderConfig.IC2Filter          = 0;

  if(HAL_TIM_Encoder_Init(&Encoder_Handle, &sEncoderConfig) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Start the encoder interface */
  HAL_TIM_Encoder_Start_IT(&Encoder_Handle, TIM_CHANNEL_ALL);

  /* Enable the interrupt for TIM3 */
  HAL_NVIC_SetPriority(TIM3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

/*--------------------------------------------------------------------------*/
  /* Disable all channels */
  HAL_GPIO_WritePin(GPIOC, PHASEAEN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, PHASEBEN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, PHASECEN, GPIO_PIN_RESET);

  /* Stop all channels */
  if (HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  if (HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }
  if (HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_3) != HAL_OK)
  {
    /* PWM Generation Error */
    Error_Handler();
  }

/*--------------------------------------------------------------------------*/
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
/*--------------------------------------------------------------------------*/

/*#################################### USER CODE ########################################*/

  xTaskCreate(pController_Task, "pController_Task", 70, NULL, 1, &pControllerTask_Handle);
  vTaskStartScheduler();

}

/**
  * @brief	This function is executed to drive the BLDC motor depending on
		the current state of the excitation of phases and the sign of the
		duty cycle of the PWM signals.
  * @param	None
  * @retval	None
**/
static void driveMotor(void)
{

  if(stop){
    /* Disable all channels */
    HAL_GPIO_WritePin(GPIOC, PHASEAEN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, PHASEBEN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, PHASECEN, GPIO_PIN_RESET);

    HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_3);

  } else {

    if (firstStep){
      /* Hall sensors init state */
      initState = 0x00;
      initState |= (HAL_GPIO_ReadPin(GPIOA,HALLA) << 2) | (HAL_GPIO_ReadPin(GPIOB,HALLB) << 1) | HAL_GPIO_ReadPin(GPIOB,HALLC);
      currentState = hallState[initState & 0x07];

      /* Clear initialization flag */
      firstStep = 0;
    } 

    if (rotation == 0) {
      currentState++;
      if (currentState > 5) currentState = 0;
    } else {
      currentState--;
      if (currentState < 0) currentState = 5;
    }

    switch (currentState) {

      case 0:     //B+, C-
        /* Disable channel 1 */
        HAL_GPIO_WritePin(GPIOC, PHASEAEN, GPIO_PIN_RESET);
        /* Enable channel 2 */
        HAL_GPIO_WritePin(GPIOC, PHASEBEN, GPIO_PIN_SET);
        /* Enable channel 3 */
        HAL_GPIO_WritePin(GPIOC, PHASECEN, GPIO_PIN_SET);
        /* Stop channel 1 */
        HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_1);
        /* Start channel 2 */
        HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2);
        /* Stop channel 3 */
        HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_3);
        break;

      case 1:     //B+, A-
        /* Enable channel 1 */
        HAL_GPIO_WritePin(GPIOC, PHASEAEN, GPIO_PIN_SET);
        /* Enable channel 2 */
        HAL_GPIO_WritePin(GPIOC, PHASEBEN, GPIO_PIN_SET);
        /* Disable channel 3 */
        HAL_GPIO_WritePin(GPIOC, PHASECEN, GPIO_PIN_RESET);
        /* Stop channel 1 */
        HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_1);
        /* Start channel 2 */
        HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2);
        /* Stop channel 3 */
        HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_3);
        break;

      case 2:     //C+, A-
        /* Enable channel 1 */
        HAL_GPIO_WritePin(GPIOC, PHASEAEN, GPIO_PIN_SET);
        /* Disable channel 2 */
        HAL_GPIO_WritePin(GPIOC, PHASEBEN, GPIO_PIN_RESET);
        /* Enable channel 3 */
        HAL_GPIO_WritePin(GPIOC, PHASECEN, GPIO_PIN_SET);
        /* Stop channel 1 */
        HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_1);
        /* Stop channel 2 */
        HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_2);
        /* Start channel 3 */
        HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_3);
        break;

      case 3:     //C+, B-
        /* Disable channel 1 */
        HAL_GPIO_WritePin(GPIOC, PHASEAEN, GPIO_PIN_RESET);
        /* Enable channel 2 */
        HAL_GPIO_WritePin(GPIOC, PHASEBEN, GPIO_PIN_SET);
        /* Enable channel 3 */
        HAL_GPIO_WritePin(GPIOC, PHASECEN, GPIO_PIN_SET);
        /* Stop channel 1 */
        HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_1);
        /* Stop channel 2 */
        HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_2);
        /* Start channel 3 */
        HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_3);
        break;

      case 4:     //A+, B-
        /* Disable channel 1 */
        HAL_GPIO_WritePin(GPIOC, PHASEAEN, GPIO_PIN_SET);
        /* Enable channel 2 */
        HAL_GPIO_WritePin(GPIOC, PHASEBEN, GPIO_PIN_SET);
        /* Enable channel 3 */
        HAL_GPIO_WritePin(GPIOC, PHASECEN, GPIO_PIN_RESET);
        /* Stop channel 1 */
        HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1);
        /* Start channel 2 */
        HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_2);
        /* Stop channel 3 */
        HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_3);
        break;

      case 5:     //A+, C-
        /* Disable channel 1 */
        HAL_GPIO_WritePin(GPIOC, PHASEAEN, GPIO_PIN_SET);
        /* Enable channel 2 */
        HAL_GPIO_WritePin(GPIOC, PHASEBEN, GPIO_PIN_RESET);
        /* Enable channel 3 */
        HAL_GPIO_WritePin(GPIOC, PHASECEN, GPIO_PIN_SET);
        /* Stop channel 1 */
        HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1);
        /* Start channel 2 */
        HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_2);
        /* Stop channel 3 */
        HAL_TIM_PWM_Stop(&TimHandle, TIM_CHANNEL_3);
        break;
    }
  }        
}

void pController_Task (void * pvParams)
{
  /* Block for 1ms */
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;

  while(1)
  {
    // Control algorithm
    position = (float)(2*PI*pulses/(CPR*REDUCT));

    u = KP * (REF - position);
    if (u > U_MAX) u = U_MAX;
    if (u < -U_MAX) u = -U_MAX;

   uEq = calculateEqV(u);
    if (uEq > U_MAX) uEq = U_MAX;
    if (uEq < -U_MAX) uEq = -U_MAX;

    // Set new PWM duty
    setVoltage(uEq);

    vTaskDelay(xDelay);
  }
}

static float calculateEqV(float v)
{
  if (abs(v)<3) return v;
  int n;
  float result;
  result = b[LENGTHB-1]*v;
  for (n=LENGTHB-2; n>=0; n--){
      result = (result*v + b[n])*v;
  }
  return result;
}

static void setVoltage(float v)
{
  if (v<0){  // Drive in negative sense of rotation
    uEq = -v;
    pulseValue = (uint32_t)(((PERIOD_VALUE+1) * uEq / U_NOM)-1);
    TIM1->CCR1 = pulseValue;

    /* Is it not driving in negative sense of rotation? */
    if (rotation == 1){
      rotation = !rotation;
      firstStep = 1;
    }
  } else {  // Drive in positive sense of rotation
    uEq = v; 
    pulseValue = (uint32_t)(((PERIOD_VALUE+1) * uEq / U_NOM)-1);
    TIM1->CCR1 = pulseValue;

    /* Is it not driving in positive sense of rotation? */
    if (rotation == 0){
      rotation = !rotation;
      firstStep = 1;
    }
  }
  /* If motor needs a firstStep call driveMotor() */
  if (firstStep) driveMotor();
}

void TIM2_IRQHandler(void)
{ 
  HAL_TIM_IRQHandler(&Hall_Handle);
}

void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&Encoder_Handle);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) driveMotor();
  if (htim->Instance == TIM3)
  {
    RotatoryCurrentState = (HAL_GPIO_ReadPin(GPIOC, ENCODERA) << 1) | HAL_GPIO_ReadPin(GPIOC, ENCODERB);
    RotatoryTransition = (RotatoryTransition << 2) | RotatoryCurrentState;
    pulses += encoderState[RotatoryTransition & 0x0F];
  }
}

/**
  * @brief	This function is executed in case of error occurence.
  * @param	None
  * @retval	None
**/
static void Error_Handler(void)
{
  while(1){
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_Delay(1000);
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows: 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            PLL_R                          = 6
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 6;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  { 
    Error_Handler();
  }

   /* Activate the OverDrive to reach the 180 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
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
