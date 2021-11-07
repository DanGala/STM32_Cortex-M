#ifndef __SERIAL_H_
#define __SERIAL_H_

#ifdef __cplusplus

#include "CircularBuffer.h"

class SerialCommand
{
	friend class Serial;
public:
	explicit SerialCommand();
	static void Register(const char * command, void(*callback)(void));
private:
	const char * command;
	void (*callback)(void);
	SerialCommand * next;
	static SerialCommand * firstCmd;
};

class Serial
{
	friend void SerialRxFriend();
	friend void SerialTxFriend();

public:
	Serial();
	static void Initialize();
	static bool SendToBuffer(const char * data);
	static bool SendToBuffer(const char * data, uint32_t length);
	static void WriteBuffer();
	static void SerialCommsTask(void *pvParams);

private:
	static void InitializeHardware();
	static void ProcessCommand(const char * command);
	static void Transmit();
	static void RxISR();
	static void TxISR();
	static void TriggerTransmission();

	static constexpr uint16_t TX_BUF_LEN = 512u;
	static constexpr uint16_t RX_QUEUE_LEN = 100u;
	static constexpr uint16_t MAX_COMMAND_LEN = 50u;

	static __attribute__((aligned(32))) char txBuffer[TX_BUF_LEN];
	static CircularBuffer<char> txCircularBuffer;
	static __attribute__((aligned(32))) char command[MAX_COMMAND_LEN];
	static xQueueHandle rxQueue;
	static xSemaphoreHandle txSemaphore;
	static uint8_t rxBytes;
	static xTaskHandle serialCommsTask_Handle;
};

#endif //#ifdef __cplusplus

#endif //#ifndef __SERIAL_H_