/**
  ***********************************************************************************
  * @file	  PID-D_ControllerFreeRTOS/src/main.c
  * @author	  Daniel Gala Montes
  * @version  V1.0
  * @date	  20-February-2017
  * @brief	  This code implements a PID-D controller for a BLDC motor with Hall
  *           sensors for trapezoidal control and an incremental enconder for 
  *           positioning.
  ***********************************************************************************
**/

/**
  ************************************************************************************
  * TODO: Nothing for the time being
  ************************************************************************************ 
**/

/**
  ************************************************************************************
  * Current motor model parameters:
  *   pM = 38.1374
  *   KM = 1105.7
  ************************************************************************************ 
**/

/* Includes -----------------------------------------------------------------------*/
#include "main.h"

/* Scheduler includes -------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Private define -----------------------------------------------------------------*/
#define PI            (float)(3.14159265358979323846) /* Math Pi constant */
#define CPR		        (uint32_t)(2000)	/* Quadrature cycles per revolution */
#define REDUCT		    (float)(13.795918)	/* Gear reduction ratio */
#define PERIOD_VALUE  (uint32_t)(359)	/* Period Value for PWM signals */
#define REF		        (float)(PI/4) 	/* Reference signal in radians */
#define U_NOM		      (uint32_t)(24)	/* Nominal motor voltage */
#define U_MAX		      (float)(24)	/* Maximum voltage to drive the motor with speed limit */
#define LENGTHB       (uint32_t)(14)   /* Length of the interpolation polynomial vector */
#define PID_D_KP      (float)(15.8551) /* Proportional gain */
#define PID_D_KD1     (float)(0.8559)  /* Derivative direct gain */
#define PID_D_KD2     (float)(-0.0642)  /* Derivative parallel gain */
#define PID_D_KI      (float)(146.4913) /* Integral gain */
#define TS            (float)(0.001)  /* Sampling period in seconds */

/* Private macro ------------------------------------------------------------------*/
/* Private variables --------------------------------------------------------------*/
volatile  uint8_t  uwStep;   /* Step index */
          uint8_t  firstStep; /* Motor start-up flag */
volatile uint32_t  pulseValue;   /* Duty cycle for PWM signals */
volatile  int32_t  pulses; /* Pulses on the encoder lines */
const      int8_t  hallState[8] = {-1, 6, 2, 1, 4, 5, 3, -1}; /* Hall sensors sector associated driving state */
volatile  uint8_t  initState;  /* Hall sensors init state */
const      int8_t  encoderState[16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; /* Encoder state transition result */
volatile  uint8_t  RotatoryCurrentState; /* Current state of the encoder channels */
volatile  uint8_t  RotatoryTransition; /* Transition between current and last encoder channels state */
volatile  uint8_t  stop;   /* Motor stop flag */
volatile  uint8_t  rotation;       /* Sense of rotation */
volatile    float  u;  /* Ideal voltage to apply */
volatile    float  uEq; /* Equivalent voltage to apply considering nonlinear behaviour */
volatile    float  position; /* Relative motor position [rad] */
volatile    float  pid_dError; /* Position error - PID-D controller input */
const       float  b[LENGTHB] = {1.98418343786294,
                                -0.0683862815716846,
                                 0.00254257231573365,
                                -5.34998131101186e-5,
                                 6.92172268134585e-7,
                                -5.85342798222869e-9,
                                 3.36570846233264e-11,
                                -1.34618716901714e-13,
                                 3.77891450508021e-16,
                                -7.40895443342920e-19,
                                 9.92547791605037e-22,
                                -8.65172057920249e-25,
                                 4.41926766031534e-28,
				                        -1.00322003575332e-31}; /* Interpolation polynomial vector */

/* GPIO Configuration Structure declaration */
GPIO_InitTypeDef GPIO_InitStruct;

/* Timer handler declaration */
TIM_HandleTypeDef TimHandle;
TIM_HandleTypeDef Encoder_Handle;
TIM_HandleTypeDef Hall_Handle;

/*Timer Output Compare Configuration Structure declaration */
TIM_OC_InitTypeDef sPWMConfig;

/* Timer Break Configuration Structure declaration */
TIM_BreakDeadTimeConfigTypeDef sBreakConfig;

/* Timer Hall sensor Configuration Structure declaration */
TIM_HallSensor_InitTypeDef sHallConfig;

/* Timer Encoder Configuration Structure declaration */
TIM_Encoder_InitTypeDef sEncoderConfig;

/* PID-D controller task handle */
xTaskHandle pid_dControllerTask_Handle;

/* Motor start-up task handle */
xTaskHandle startupTask_Handle;

/* PID-D Instance */
pid_d_instance PID_D;

/* Counter Prescaler value */
uint32_t uhPrescalerValue = 0;

/* Private function prototypes ----------------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);
static void EXTI15_10_IRQHandler_Config(void);
static float calculateEqV(float v);
static void setVoltage(float v);
void startup_Task();
void pid_dController_Task();
void pid_dInit(pid_d_instance * S);
float pid_dUpdate(pid_d_instance * S, float err);

/* Private functions --------------------------------------------------------------*/

/**
  * @brief	Main program.
  * @param	None
  * @retval	None
**/
int main(void)
{

  pulseValue = (PERIOD_VALUE+1)/4 - 1;
  pulses = 0;
  rotation = 1;
  stop = 0;
  firstStep = 1;
  initState = 0x00;
  RotatoryCurrentState = 0x00;
  RotatoryTransition = 0;
  u = 0;
  uEq = 0;
  position = 0;
  pid_dError = REF;

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
  /* User LED */
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Phase enables */
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pin = PHASEAEN | PHASEBEN | PHASECEN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Phases */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pin = PHASEA | PHASEB | PHASEC;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Encoder channels */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pin = ENCODERA | ENCODERB;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Hall sensor A */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pin = HALLA;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Hall sensors B and C */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Pin = HALLB | HALLC;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/*--------------------------------------------------------------------------*/

  EXTI15_10_IRQHandler_Config();

/*--------------------------------------------------------------------------*/

  /* PID-D structure initialization */
  pid_dInit(&PID_D);

/* -----------------------------------------------------------------------------
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
  HAL_NVIC_SetPriority(TIM2_IRQn, 2, 1);
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

  /* Encoder initial state */
  RotatoryCurrentState = (HAL_GPIO_ReadPin(GPIOC, ENCODERA) << 1) | HAL_GPIO_ReadPin(GPIOC, ENCODERB);
  RotatoryTransition = (RotatoryTransition << 2) | RotatoryCurrentState;

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
      Prescaler = ((SystemCoreClock) / 18 MHz) - 1
    To get TIM1 output clock at 50 KHz, the period (ARR)) is computed as follows:
      ARR = (TIM1 counter clock / TIM1 output clock) - 1
          = 359

    TIM1 duty cycle = (TIM1_CCR1 / TIM1_ARR + 1)* 100 = 25%
------------------------------------------------------------------------------*/

  /* Counter Prescaler value */
  uint32_t uhPrescalerValue;

  /* Compute the prescaler value to have TIM1 counter clock equal to 16 MHz */
  uhPrescalerValue = (uint32_t)(SystemCoreClock / 18000000) - 1;

  /* Initialize TIM1 peripheral as follows:
      + Prescaler = (SystemCoreClock / 18000000) - 1
      + Period = (359)
      + ClockDivision = 0
      + Counter direction = Up
  */

  TimHandle.Instance               = TIM1;
  TimHandle.Init.Prescaler         = uhPrescalerValue;
  TimHandle.Init.Period            = PERIOD_VALUE;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;

  if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure the PWM channels ##############################################*/
  sPWMConfig.OCMode       = TIM_OCMODE_TIMING;
  sPWMConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sPWMConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sPWMConfig.OCIdleState  = TIM_OCIDLESTATE_SET;
  sPWMConfig.OCNIdleState = TIM_OCNIDLESTATE_SET;
  sPWMConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sPWMConfig.Pulse        = (uint32_t)(V*(float)((PERIOD_VALUE+1)/24) - 1);

  /* Set the pulse value for channel 1 */
  if(HAL_TIM_OC_ConfigChannel(&TimHandle, &sPWMConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  } 

  /* Set the pulse value for channel 2 */
  if(HAL_TIM_OC_ConfigChannel(&TimHandle, &sPWMConfig, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /* Set the pulse value for channel 3 */
  if(HAL_TIM_OC_ConfigChannel(&TimHandle, &sPWMConfig, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /*##-3- Configure the Break stage ##########################################*/
  sBreakConfig.OffStateRunMode  = TIM_OSSR_ENABLE;
  sBreakConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
  sBreakConfig.BreakState       = TIM_BREAK_DISABLE;
  sBreakConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
  sBreakConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_ENABLE;
  sBreakConfig.DeadTime         = 1;

  if(HAL_TIMEx_ConfigBreakDeadTime(&TimHandle, &sBreakConfig) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

  /*##-4- Configure the commutation event: software event ####################*/
  /* Enable the TIM1 global Interrupt & set priority */
  HAL_TIMEx_ConfigCommutationEvent_IT(&TimHandle, TIM_TS_NONE, TIM_COMMUTATION_SOFTWARE);

  HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);

  /*##-5- Start signals generation ###########################################*/

  /* Start channel 1 */
  if(HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /* Start channel 1N */
  if(HAL_TIMEx_OCN_Start(&TimHandle, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  /* Start channel 2 */
  if(HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /* Start channel 2N */
  if(HAL_TIMEx_OCN_Start(&TimHandle, TIM_CHANNEL_2) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  /* Start channel 3 */
  if(HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /* Start channel 3N */
  if(HAL_TIMEx_OCN_Start(&TimHandle, TIM_CHANNEL_3) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

/*--------------------------------------------------------------------------*/

  xTaskCreate(startup_Task, "startup_Task", 63, NULL, 0, &startupTask_Handle);
  xTaskCreate(pid_dController_Task, "pid_dController_Task", 70, NULL, 1, &pid_dControllerTask_Handle);
  vTaskStartScheduler();

}

/* Start-up algorithm for hall-sensor automated trapezoidal control */
void startup_Task (void * pvParams)
{
  /* Block for 10ms */
  const TickType_t xDelay = 5 / portTICK_PERIOD_MS;

  while(1)
  {
    HAL_TIMEx_CommutationCallback(&TimHandle);              // Read Hall sensors to know the current hall sector
    HAL_TIM_GenerateEvent(&TimHandle, TIM_EventSource_COM); // Trigger first step manually to overcome friction and inertia
    vTaskDelay(xDelay);                                     // Wait for the rotation to begin
    if (pulses > 0)                                         // If there's been movement, 
    {
      vTaskDelete(startupTask_Handle);                      // Delete the start-up task
    }                                                       // Else, repeat
  }
}


/**
  * @brief  Commutation event callback in non blocking mode
  * @param  htim : Timer handle
  * @retval None
  */
void HAL_TIMEx_CommutationCallback(TIM_HandleTypeDef *htim)
{
  if(stop)
  {
    /* Disable all channels */
    HAL_GPIO_WritePin(GPIOC, PHASEAEN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, PHASEBEN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, PHASECEN, GPIO_PIN_RESET);
    HAL_TIM_OC_Stop(&TimHandle, TIM_CHANNEL_1);
    HAL_TIMEx_OCN_Stop(&TimHandle, TIM_CHANNEL_1);
    HAL_TIM_OC_Stop(&TimHandle, TIM_CHANNEL_2);
    HAL_TIMEx_OCN_Stop(&TimHandle, TIM_CHANNEL_2);
    HAL_TIM_OC_Stop(&TimHandle, TIM_CHANNEL_3);
    HAL_TIMEx_OCN_Stop(&TimHandle, TIM_CHANNEL_3);
  }
  /* Entry state */
  if (firstStep)
  {
    /* Hall sensors init state */
    initState = 0x00;
    initState |= (HAL_GPIO_ReadPin(GPIOA,HALLA) << 2) | (HAL_GPIO_ReadPin(GPIOB,HALLB) << 1) | HAL_GPIO_ReadPin(GPIOB,HALLC);
    uwStep = hallState[initState & 0x07];

    uint32_t activeChannel;
    uint32_t compChannel;
    uint32_t idleChannel;

    switch(uwStep)
    {
      case 1:
        activeChannel = TIM_CHANNEL_1;
        compChannel = TIM_CHANNEL_3;
        idleChannel = TIM_CHANNEL_2;
        break;
      case 2:
        activeChannel = TIM_CHANNEL_1;
        compChannel = TIM_CHANNEL_2;
        idleChannel = TIM_CHANNEL_3;
        break;
      case 3:
        activeChannel = TIM_CHANNEL_3;
        compChannel = TIM_CHANNEL_2;
        idleChannel = TIM_CHANNEL_1;
        break;
      case 4:
        activeChannel = TIM_CHANNEL_3;
        compChannel = TIM_CHANNEL_1;
        idleChannel = TIM_CHANNEL_2;
        break;
      case 5:
        activeChannel = TIM_CHANNEL_2;
        compChannel = TIM_CHANNEL_1;
        idleChannel = TIM_CHANNEL_3;
        break;
      case 6:
        activeChannel = TIM_CHANNEL_2;
        compChannel = TIM_CHANNEL_3;
        idleChannel = TIM_CHANNEL_1;
        break;
    }

    /* Next step: Step 1 Configuration -------------------------------------- */
    /* Active channel configuration */
    sPWMConfig.OCMode     = TIM_OCMODE_PWM1;
    HAL_TIM_PWM_ConfigChannel(&TimHandle, &sPWMConfig, activeChannel);
    HAL_TIM_PWM_Start(&TimHandle, activeChannel);
    HAL_TIMEx_OCN_Stop(&TimHandle, activeChannel);

    /* Complementary channel configuration */
    sPWMConfig.OCMode     = TIM_OCMODE_PWM1;
    HAL_TIM_PWM_ConfigChannel(&TimHandle, &sPWMConfig, compChannel);
    HAL_TIMEx_OCN_Start(&TimHandle, compChannel);
    HAL_TIM_PWM_Stop(&TimHandle, compChannel);

    /* Idle channel configuration */
    HAL_TIM_OC_Stop(&TimHandle, idleChannel);
    HAL_TIMEx_OCN_Stop(&TimHandle, idleChannel);
  }

  switch (uwStep)
  {
    case 1: // A+ , B-

      /* Next step: Step 2 Configuration -------------------------------------- */
      /*  Channel1 configuration */
      /* Same configuration as the previous step */

      /*  Channel2 configuration */
      sPWMConfig.OCMode     = TIM_OCMODE_PWM1;
      HAL_TIM_PWM_ConfigChannel(&TimHandle, &sPWMConfig, TIM_CHANNEL_2);
      HAL_TIMEx_OCN_Start(&TimHandle, TIM_CHANNEL_2);

      /*  Channel3 configuration */
      HAL_TIMEx_OCN_Stop(&TimHandle, TIM_CHANNEL_3);

      uwStep++;
      break;

    case 2: // C+ , B-

      /* Next step: Step 3 Configuration -------------------------------------- */
      /*  Channel2 configuration */
      /* Same configuration as the previous step */

      /*  Channel3 configuration */
      sPWMConfig.OCMode     = TIM_OCMODE_PWM1;
      HAL_TIM_PWM_ConfigChannel(&TimHandle, &sPWMConfig, TIM_CHANNEL_3);
      HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_3);

      /*  Channel1 configuration */
      HAL_TIM_OC_Stop(&TimHandle, TIM_CHANNEL_1);

      uwStep++;
      break;

    case 3: // C+ , A-

      /* Next step: Step 4 Configuration -------------------------------------- */
      /*  Channel3 configuration */
      /* Same configuration as the previous step */

      /*  Channel2 configuration */
      HAL_TIMEx_OCN_Stop(&TimHandle, TIM_CHANNEL_2);

      /*  Channel1 configuration */
      sPWMConfig.OCMode     = TIM_OCMODE_PWM1;
      HAL_TIM_PWM_ConfigChannel(&TimHandle, &sPWMConfig, TIM_CHANNEL_1);
      HAL_TIMEx_OCN_Start(&TimHandle, TIM_CHANNEL_1);

      uwStep++;
      break;

    case 4: // B+ , A-

      /* Next step: Step 5 Configuration -------------------------------------- */
      /*  Channel3 configuration */
      HAL_TIM_OC_Stop(&TimHandle, TIM_CHANNEL_3);

      /*  Channel1 configuration */
      /* Same configuration as the previous step */

      /*  Channel2 configuration */
      sPWMConfig.OCMode     = TIM_OCMODE_PWM1;
      HAL_TIM_PWM_ConfigChannel(&TimHandle, &sPWMConfig, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2);

      uwStep++;
      break;

    case 5: // B+ , C-

      /* Next step: Step 6 Configuration -------------------------------------- */
      /*  Channel3 configuration */
      sPWMConfig.OCMode     = TIM_OCMODE_PWM1;
      HAL_TIM_PWM_ConfigChannel(&TimHandle, &sPWMConfig, TIM_CHANNEL_3);
      HAL_TIMEx_OCN_Start(&TimHandle, TIM_CHANNEL_3);

      /*  Channel1 configuration */
      HAL_TIMEx_OCN_Stop(&TimHandle, TIM_CHANNEL_1);

      /*  Channel2 configuration */
      /* Same configuration as the previous step */

      uwStep++;
      break;

    default: // A+ , C-

      /* Next step: Step 1 Configuration -------------------------------------- */
      /*  Channel1 configuration */
      sPWMConfig.OCMode     = TIM_OCMODE_PWM1;
      HAL_TIM_PWM_ConfigChannel(&TimHandle, &sPWMConfig, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1);

      /*  Channel3 configuration */
      /* Same configuration as the previous step */

      /*  Channel2 configuration */
      HAL_TIM_OC_Stop(&TimHandle, TIM_CHANNEL_2);

      uwStep = 1;
      break;
  }

  if (firstStep)
  {
    /* Enable all channels */
    HAL_GPIO_WritePin(GPIOC, PHASEAEN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, PHASEBEN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, PHASECEN, GPIO_PIN_SET);
    firstStep = 0;
  }
}
void pid_dController_Task (void * pvParams)
{
  /* Periodic 1ms task */
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 1 / portTICK_PERIOD_MS;

  // Initialise the xLastWakeTime variable with the current time.
   xLastWakeTime = xTaskGetTickCount();

  while(1)
  {
    // Control algorithm
    position = (float)(2*PI*pulses/(CPR*REDUCT));

    pid_dError = (float)(REF) - position;

    u = pid_dUpdate(&PID_D, pid_dError);
    if (u > U_MAX) u = U_MAX;
    if (u < -U_MAX) u = -U_MAX;

    uEq = calculateEqV(u);

    // Set new PWM duty
    setVoltage(uEq);
//    setVoltage(u);

    /* Block for 1ms */
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

static float calculateEqV(float v)
{
//  if ((abs(v)<3)) return v;
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
      rotation = 0;
    }
  } else {  // Drive in positive sense of rotation
    uEq = v; 
    pulseValue = (uint32_t)(((PERIOD_VALUE+1) * uEq / U_NOM)-1);
    TIM1->CCR1 = pulseValue;

    /* Is it not driving in positive sense of rotation? */
    if (rotation == 0){
      rotation = 1;
    }
  }
  if (firstStep) HAL_TIMEx_CommutationCallback(&TimHandle);
}

/**    
  * @brief      Initialization function for the PID-D Control.   
  * @param      *S points to an instance of the PID-D structure.   
  * @retval     None  
**/

void pid_dInit(pid_d_instance * S)
{
  S->Kp = PID_D_KP;
  S->Kd1 = PID_D_KD1;
  S->Kd2 = PID_D_KD2;
  S->Ki = PID_D_KI;
  S->windup_guard = U_MAX;
  S->prevErr = REF;
  S->prevPos = 0;
  S->intErr = 0;
}

/**
  * @brief      Control output signal update.
  * @param      *S points to an instance of the PID-D structure.
  * @param      err is the current error signal input
  * @retval     out is the control signal produced.
**/
float pid_dUpdate(pid_d_instance * S, float err)
{
  float out;

  /* Integration with windup guarding */
  S->intErr += err * TS;
  if (S->intErr < -(S->windup_guard)) S->intErr = -(S->windup_guard) / S->Ki;
  else if (S->intErr > S->windup_guard) S->intErr = S->windup_guard / S->Ki;

  /* Output signal calculation */
  out = (S->Kp * err) + (S->Kd1 * (err - S->prevErr) / TS) + (S->Ki * S->intErr * TS) - (S->Kd2 * (position - S->prevPos) / TS);

  /* State update */
  S->prevErr = err;
  S->prevPos = position;

  return out;
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
 * @brief  Configures EXTI lines 15 to 10 (connected to PC.13 pin) in interrupt mode
 * @param  None
 * @retval None
**/
static void EXTI15_10_IRQHandler_Config(void)
{
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Configure PC.13 pin as input floating */
  GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Enable and set EXTI lines 15 to 10 Interrupt to the highest priority */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/**
 * @brief EXTI line detection callbacks
 * @param GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
**/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13)
  {
    stop = 1;
    TIM1->CCR1 = 0;
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
