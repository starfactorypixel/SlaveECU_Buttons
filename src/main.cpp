#include "main.h"
#include <ConstantLibrary.h>
#include <LoggerLibrary.h>
#include "SPI.h"
#include "About.h"
#include "Leds.h"
#include "CANLogic.h"
#include "Buttons.h"
#include "Out.h"

ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan;
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef hDebugUart;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	
	MX_GPIO_Init();
	MX_CAN_Init();
	MX_SPI2_Init();
	MX_USART1_UART_Init();
	MX_ADC1_Init();
	
	About::Setup();
	Leds::Setup();
	CANLib::Setup();
	ButtonsLeds::Setup();
	SPI::Setup();
	Out::Setup();
	
	Leds::obj.SetOn(Leds::LED_GREEN, 50, 1950);
	
	uint32_t current_time = HAL_GetTick();
	while (1)
	{
		// don't need to update current_time because it is always updated by Loop() functions
		// current_time = HAL_GetTick();

		About::Loop(current_time);
		Leds::Loop(current_time);
		CANLib::Loop(current_time);
		ButtonsLeds::Loop(current_time);
		SPI::Loop(current_time);
		Out::Loop(current_time);
	}
}

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
	if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_ADC1_Init(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if(HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_CAN_Init(void)
{
	CAN_FilterTypeDef sFilterConfig;
	
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 4;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = ENABLE;
	hcan.Init.AutoWakeUp = ENABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = ENABLE;
	hcan.Init.TransmitFifoPriority = ENABLE;
	if(HAL_CAN_Init(&hcan) != HAL_OK)
	{
		Error_Handler();
	}

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	// sFilterConfig.SlaveStartFilterBank = 14;
	if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_SPI2_Init(void)
{
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if(HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_USART1_UART_Init(void)
{
	hDebugUart.Instance = USART1;
	hDebugUart.Init.BaudRate = 500000;
	hDebugUart.Init.WordLength = UART_WORDLENGTH_8B;
	hDebugUart.Init.StopBits = UART_STOPBITS_1;
	hDebugUart.Init.Parity = UART_PARITY_NONE;
	hDebugUart.Init.Mode = UART_MODE_TX_RX;
	hDebugUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	hDebugUart.Init.OverSampling = UART_OVERSAMPLING_16;
	if(HAL_UART_Init(&hDebugUart) != HAL_OK)
	{
		Error_Handler();
	}
}

static void MX_GPIO_Init(void)
{
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

/*
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // Enable A.F. clock
	//AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE; // JTAG is disabled, SWD is enabled

	AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1;
*/
/*
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

GPIO_InitTypeDef GPIO_InitStruct = {0};
GPIO_InitStruct.Pin = GPIO_PIN_15;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // Настройка как выход в режиме Push-Pull
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

// Установить высокий уровень на PA15
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
*/


}

void Error_Handler(void)
{
	Leds::obj.SetOff();
	Leds::obj.SetOn(Leds::LED_RED);
	
	__disable_irq();
	while (1)
	{

	}
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
