/**
 * @file Serial_STM32.cpp
 * @author DGM
 * @version 0.1
 * @date 2021-11-03
 */

#include "SystemHeaders.h"

static UART_HandleTypeDef huart;

/**
 * \brief Initializes the USART in asynchronous mode, with reception handled by interrupts and transmission handled by DMA
 */
void Serial::InitializeHardware()
{
	/* Enable clocks */
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* Configure UART pins */
	GPIO_InitTypeDef initStructure;
	initStructure.Mode = GPIO_MODE_AF_PP;
	initStructure.Pull = GPIO_PULLDOWN;
	initStructure.Speed = GPIO_SPEED_MEDIUM;
	initStructure.Alternate = GPIO_AF7_USART1;
	initStructure.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	HAL_GPIO_Init(GPIOB, &initStructure);

	/* Configure the USART peripheral in asynchronous mode */
	huart.Instance = USART1;
	huart.Init.BaudRate = 115200;
	huart.Init.WordLength = UART_WORDLENGTH_8B;
	huart.Init.StopBits = UART_STOPBITS_1;
	huart.Init.Parity = UART_PARITY_NONE;
	huart.Init.Mode = UART_MODE_TX_RX;
	huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_DeInit(&huart);
	HAL_UART_Init(&huart);

	/* Enable UART receive interrupt */
	__HAL_UART_ENABLE_IT(&huart, UART_IT_RXNE);

	/* Configure UART interrupt */
	HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * \brief Friend RX function to access the private data members of the serial class from an external C wrapper
 */
void SerialRxFriend()
{
	Serial::RxISR();
}

/**
 * \brief Friend TX function to access the private data members of the serial class from an external C wrapper
 */
void SerialTxFriend()
{
	Serial::TxISR();
}

/**
 * \brief RX interrupt service routine
 */
void Serial::RxISR()
{
	portBASE_TYPE rxHigherPriorityTaskWoken = pdFALSE;

	if(!(USART1->ISR & UART_FLAG_RXNE))
	{
		uint32_t dummy = USART1->RDR; //Read from DR to clear RXNE interrupt and prevent repeated firing of the ISR
		UNUSED(dummy);
	}

	//Process incoming data
	while(USART1->ISR & UART_FLAG_RXNE)
	{
		uint8_t data = (USART1->RDR & 0xFF);
		if(xQueueSendToBackFromISR(rxQueue, &data, &rxHigherPriorityTaskWoken) == errQUEUE_FULL)
		{
			assert(0); //RX queue overrun
		}
	}

	//Parity error
	if(__HAL_UART_GET_FLAG(&huart, UART_FLAG_PE) && __HAL_UART_GET_IT_SOURCE(&huart, UART_IT_PE))
	{
		__HAL_UART_CLEAR_PEFLAG(&huart);
	}
	//Frame error
	if(__HAL_UART_GET_FLAG(&huart, UART_FLAG_FE) && __HAL_UART_GET_IT_SOURCE(&huart, UART_IT_ERR))
	{
		__HAL_UART_CLEAR_FEFLAG(&huart);
	}
	//Noise error
	if(__HAL_UART_GET_FLAG(&huart, UART_FLAG_NE) && __HAL_UART_GET_IT_SOURCE(&huart, UART_IT_ERR))
	{
		__HAL_UART_CLEAR_NEFLAG(&huart);
	}
	//Overrun
	if(__HAL_UART_GET_FLAG(&huart, UART_FLAG_ORE) && __HAL_UART_GET_IT_SOURCE(&huart, UART_IT_ERR))
	{
		__HAL_UART_CLEAR_OREFLAG(&huart);
	}

	//Yield if a higher priority task was woken due to an element being enqueued
	portEND_SWITCHING_ISR(rxHigherPriorityTaskWoken);
}

/**
 * \brief TX interrupt service routine
 */
void Serial::TxISR()
{
	portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

	//Wait until TC flag is set before transmitting the next byte
	uint32_t counter = 0;
	while(!__HAL_UART_GET_FLAG(&huart, UART_FLAG_TC))
	{
		if(++counter > 200)
		{
			assert(0); //Timeout
		}
	}

	//Are we done sending?
	if(txCircularBuffer.Empty())
	{
		huart.Instance->CR1 &= ~USART_CR1_TXEIE;
		huart.Instance->CR1 &= ~USART_CR1_TCIE;
		xSemaphoreGiveFromISR(txSemaphore, &higherPriorityTaskWoken);
	}
	else
	{
		huart.Instance->TDR = (uint8_t)(txCircularBuffer.Pop());
	}

	//Yield if a higher priority task was woken due to the semaphore being released
	portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

/**
 * \brief Trigger the DMA transfer by setting the pending bit on the selected IRQ
 */
void Serial::TriggerTransmission()
{
	huart.Instance->CR1 |= USART_CR1_TXEIE;
	HAL_NVIC_SetPendingIRQ(USART1_IRQn);
}

extern "C" void USART1_IRQHandler(void)
{
	//Receiver mode
	if(__HAL_UART_GET_FLAG(&huart, UART_FLAG_RXNE) && __HAL_UART_GET_IT_SOURCE(&huart, UART_IT_RXNE))
	{
		SerialRxFriend();
	}

	//Transmitter mode
	if((__HAL_UART_GET_FLAG(&huart, UART_FLAG_TXE) && __HAL_UART_GET_IT_SOURCE(&huart, UART_IT_TXE)) ||
		(__HAL_UART_GET_FLAG(&huart, UART_FLAG_TC) && __HAL_UART_GET_IT_SOURCE(&huart, UART_IT_TC)))
	{
		SerialTxFriend();
	}
}