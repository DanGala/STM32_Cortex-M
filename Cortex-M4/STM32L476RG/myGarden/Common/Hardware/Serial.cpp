/**
 * @file Serial.cpp
 * @author DGM
 * @version 0.1
 * @date 2021-11-03
 */

#include "SystemHeaders.h"

char Serial::txBuffer[TX_BUF_LEN] = {0};
CircularBuffer<char> Serial::txCircularBuffer(TX_BUF_LEN, txBuffer);
char Serial::command[MAX_COMMAND_LEN] = {0};
xQueueHandle Serial::rxQueue = nullptr;
xSemaphoreHandle Serial::txSemaphore = nullptr;
uint8_t Serial::rxBytes = 0;
xTaskHandle Serial::serialCommsTask_Handle = nullptr;
SerialCommand * SerialCommand::firstCmd = nullptr;

/**
 * \brief Default c'tor
 */
SerialCommand::SerialCommand() :
	command(nullptr),
	callback(nullptr),
	next(nullptr)
{
}

/**
 * \brief Registers a callback function that will be invoked when a string literal is received over serial
 * \param command The string literal associated to the callback function
 * \param callback The callback function to be invoked
 */
void SerialCommand::Register(const char * command, void(*callback)(void))
{
	SerialCommand *newCommand = new SerialCommand();
	assert(newCommand != nullptr);
	newCommand->command = command;
	newCommand->callback = callback;
	newCommand->next = nullptr;
	if(firstCmd == nullptr)
	{
		firstCmd = newCommand;
	}
	else
	{
		SerialCommand * iterator = firstCmd;
		while(iterator->next)
		{
			iterator = iterator->next;
		}
		iterator->next = newCommand;
	}
}

/**
 * \brief C'tor
 */
Serial::Serial()
{
}

/**
 * \brief Initializer
 */
void Serial::Initialize()
{
	/* Create RX queue */
	rxQueue = xQueueCreate(RX_QUEUE_LEN, 1);
	assert(rxQueue != nullptr);

	/* Create the TX semaphore */
	vSemaphoreCreateBinary(txSemaphore);
	assert(txSemaphore != nullptr);

	/* Initialize hardware driver */
	InitializeHardware();

	/* Create rotation task */
	xTaskCreate(SerialCommsTask, "SerialCommsTask", 70, NULL, 4, &serialCommsTask_Handle);
}

/**
 * \brief Adds a string of data to the transmission buffer
 * \param data Pointer to data to be transmitted
 * \return false if there's not enough free space left on the buffer, true otherwise
 */
bool Serial::SendToBuffer(const char * data)
{
	return SendToBuffer(data, strlen(data));
}

/**
 * \brief Adds a non-terminated string of data to the transmission buffer
 * \param data Pointer to data to be transmitted
 * \param length Number of characters to add
 * \return false if there's not enough free space left on the buffer, true otherwise
 */
bool Serial::SendToBuffer(const char * data, uint32_t length)
{
	uint32_t pushed = 0;
	while( !txCircularBuffer.Full() && (pushed < length))
	{
		txCircularBuffer.Push(*(data++));
		pushed++;
	}
	return (pushed == length);
}

/**
 * \brief Flush out the TX buffer
 */
void Serial::WriteBuffer()
{
	if(xSemaphoreTake(txSemaphore, portMAX_DELAY) == pdPASS)
	{
		TriggerTransmission();
	}
}

/**
 * \brief Compare the received command with the registered ones and invoke their callback if there's a match
 */
void Serial::ProcessCommand(const char * command)
{
	SerialCommand * iterator = SerialCommand::firstCmd;
	while(iterator != nullptr)
	{
		if(strcmp(iterator->command, command) == 0)
		{
			if(iterator->callback) iterator->callback();
			return;
		}
		iterator = iterator->next;
	}
	SendToBuffer("Unrecognised command\r\n");
	WriteBuffer();
}

/**
 * \brief FreeRTOS task responsible for handling serial communications
 * \param pvParams 
 */
void Serial::SerialCommsTask(void * pvParams)
{
	static char * nextCharacter = command;
	while(1)
	{
		if(xQueueReceive(rxQueue, nextCharacter, 10) != errQUEUE_EMPTY)
		{
			if(++rxBytes == MAX_COMMAND_LEN)
			{
				assert(0); //RX buffer overrun
			}
			else if(*nextCharacter == '\n')
			{
				//Replace the newline character with a null terminator
				*nextCharacter = '\0';
				//Process the command
				ProcessCommand(command);
				//Reset the RX buffer
				nextCharacter = command;
				rxBytes = 0;
			}
			else
			{
				//Keep receiving until either an error or '\n' is detected
				nextCharacter++;
			}
		}
	}
}