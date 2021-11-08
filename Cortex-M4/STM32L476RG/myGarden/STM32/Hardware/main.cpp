#include "SystemHeaders.h"
#include "Garden.h"

ADConverter adc;
Garden myGarden;

/* Declaring protected assert variables */
static constexpr size_t ASSERT_BUFFER_SIZE = 100;
static constexpr int ASSERT_CODE = 0x42;
__attribute__((section(".assert_data.FILE"))) char assertFile[ASSERT_BUFFER_SIZE];
__attribute__((section(".assert_data.FUNC"))) char assertFunc[ASSERT_BUFFER_SIZE];
__attribute__((section(".assert_data.TEXT"))) char assertText[ASSERT_BUFFER_SIZE];
__attribute__((section(".assert_data.LINE"))) int assertLine;
__attribute__((section(".assert_data.CODE"))) int assertCode;

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
 	 * in the RCC_OscInitTypeDef structure. */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 3;
	RCC_OscInitStruct.PLL.PLLN = 8;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);
	/** Initializes the CPU, AHB and APB buses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);

	/** Configure the main internal regulator output voltage */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
}

extern "C" uint32_t HAL_GetTick(void)
{
	//https://www.keil.com/pack/doc/CMSIS/Core/html/group__Core__Register__gr.html#ga2c32fc5c7f8f07fb3d436c6f6fe4e8c8
	if(__get_IPSR() > 15)
	{
		return xTaskGetTickCountFromISR();
	}
	else
	{
		return xTaskGetTickCount();
	}
}

static constexpr uint8_t APP_VERSION_MAJOR = 0;
static constexpr uint8_t APP_VERSION_MINOR = 1;
/**
 * \brief Prints details of the app name and current version over serial
 */
void PrintStartupInfo()
{
	char startupInfo[50];
	sprintf(startupInfo, "\r\nmyGarden App v%d.%2d starting up!\r\n", APP_VERSION_MAJOR, APP_VERSION_MINOR);
	Serial::SendToBuffer(startupInfo);
	Serial::WriteBuffer();
}

/**
 * \brief If the controller reset was caused by an assert, prints the details of the assert over serial
 */
void PrintAssertInfo()
{
	if(assertCode == ASSERT_CODE)
	{
		char assertInfo[230];
		sprintf(assertInfo, "\r\nAssert at: %s:%d (\"%s\")\r\n", assertFile, assertLine, assertText);
		Serial::SendToBuffer(assertInfo);
		Serial::WriteBuffer();
	}
	else
	{
		assertLine = 0;
		for(size_t i = 0; i < ASSERT_BUFFER_SIZE; i++)
		{
			assertFile[i] = 0;
			assertFunc[i] = 0;
			assertText[i] = 0;
		}
	}
	assertCode = 0;
}

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{
	/* Initialize HAL libraries */
	HAL_Init();

	/* Configure the system clock to 64 MHz */
	SystemClock_Config();

	/* Initialize serial communications as soon as possible */
	Serial::Initialize();

	/* Print startup info */
	PrintStartupInfo();

	/* Print assert reset info */
	PrintAssertInfo();

	/* Configure the ADC peripheral */
	ADConverter::Initialize();

	/* Initialize business logic */
	Garden::Initialize();

	/* Start the scheduler - should never return */
	vTaskStartScheduler();

	return 0;
}

extern "C" void assert_failed(uint8_t* file, uint32_t line)
{
	__assert_func((const char*)file, line, "_unknown", "HAL_assert");
}

extern "C" void __assert_func(const char * file, int line, const char * func, const char * text)
{
	/* Copy assert information into uninitialised data section */
	strncpy(assertFile, const_cast<char *>(file), ASSERT_BUFFER_SIZE-1 );
	assertFile[ASSERT_BUFFER_SIZE-1] = '\0';
	strncpy(assertFunc, const_cast<char *>(func), ASSERT_BUFFER_SIZE-1 );
	assertFunc[ASSERT_BUFFER_SIZE-1] = '\0';
	strncpy(assertText, const_cast<char *>(text), ASSERT_BUFFER_SIZE-1 );
	assertText[ASSERT_BUFFER_SIZE-1] = '\0';
	assertLine = line;
	assertCode = ASSERT_CODE;

	//Trigger a software reset
	NVIC_SystemReset();

	while(1);
}