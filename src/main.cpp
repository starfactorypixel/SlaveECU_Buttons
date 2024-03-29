/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <ConstantLibrary.h>
#include <LoggerLibrary.h>
#include <About.h>
#include <Leds.h>
#include <CANLogic.h>
#include <ButtonsLeds.h>

#define IO_PIN_1 GPIO_PIN_5  // GPIOB
#define IO_PIN_2 GPIO_PIN_6  // GPIOB
#define IO_PIN_3 GPIO_PIN_7  // GPIOB
#define IO_PIN_4 GPIO_PIN_8  // GPIOB
#define IO_PIN_5 GPIO_PIN_9  // GPIOB
#define IO_PIN_6 GPIO_PIN_13 // GPIOC
#define IO_PIN_7 GPIO_PIN_14 // GPIOC
#define IO_PIN_8 GPIO_PIN_15 // GPIOC
#define IO_PIN_9 GPIO_PIN_0  // GPIOA
#define IO_PIN_10 GPIO_PIN_1 // GPIOA

#define IO_INPUT_MODE(PIN, PORT) (PORT)->ODR |= PIN;                        // настроить пин на вход
#define IO_OUTPUT_MODE(PIN, PORT) (PORT)->IDR |= PIN                        // настроить пин на выход
#define IO_LOW(PIN, PORT) HAL_GPIO_WritePin((PORT), (PIN), GPIO_PIN_RESET); // лог 0 на выход
#define IO_HIGH(PIN, PORT) HAL_GPIO_WritePin((PORT), (PIN), GPIO_PIN_SET);  // лог 1 на выход
#define IO_READ(PIN, PORT) HAL_GPIO_ReadPin((PORT), (PIN))                  // чтение пина

#define STBY_CAN_H() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);   // установить выход защелка в High
#define STBY_CAN_L() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // установить выход защелка в Low
#define VIO_CAN_H() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);   // установить выход защелка в High
#define VIO_CAN_L() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // установить выход защелка в Low

#define ClearBit(reg, bit) reg &= (~(1 << (bit)))        // пример: ClearBit(PORTB, 1); //сбросить 1-й бит PORTB
#define SetBit(reg, bit) reg |= (1 << (bit))             // пример: SetBit(PORTB, 3); //установить 3-й бит PORTB
#define BitIsClear(reg, bit) ((reg & (1 << (bit))) == 0) // пример: if (BitIsClear(PORTB,1)) {...} //если бит очищен
#define BitIsSet(reg, bit) ((reg & (1 << (bit))) != 0)   // пример: if(BitIsSet(PORTB,2)) {...} //если бит установлен

ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan;
SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef hDebugUart;

uint32_t Timer1 = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI2_Init(void);



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef RxHeader = {0};
	uint8_t RxData[8] = {0};
	
	if( HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK )
	{
		CANLib::can_manager.IncomingCANFrame(RxHeader.StdId, RxData, RxHeader.DLC);
	}
	
	return;
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	Leds::obj.SetOn(Leds::LED_RED, 100);
	
	DEBUG_LOG_TOPIC("CAN", "RX error event, code: 0x%08lX\n", HAL_CAN_GetError(hcan));
	
	return;
}

void HAL_CAN_Send(can_object_id_t id, uint8_t *data, uint8_t length)
{
	CAN_TxHeaderTypeDef TxHeader = {0};
	uint8_t TxData[8] = {0};
    uint32_t TxMailbox = 0;
	
	TxHeader.StdId = id;
	TxHeader.ExtId = 0;
	TxHeader.RTR  = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = length;
	TxHeader.TransmitGlobalTime = DISABLE;
	memcpy(TxData, data, length);
	
	while( HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0 )
	{
		Leds::obj.SetOn(Leds::LED_RED);
	}
	Leds::obj.SetOff(Leds::LED_RED);
	
	if( HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK )
	{
		Leds::obj.SetOn(Leds::LED_RED, 100);

		DEBUG_LOG_TOPIC("CAN", "TX error event, code: 0x%08lX\n", HAL_CAN_GetError(&hcan));
	}
	
	return;
}



/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();

  HAL_TIM_Base_Start_IT(&htim1);

  STBY_CAN_L();
  VIO_CAN_H();

  /* активируем события которые будут вызывать прерывания  */
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);

  HAL_CAN_Start(&hcan);

  IO_OUTPUT_MODE(IO_PIN_1, GPIOB); // настроить пин на выход
  IO_LOW(IO_PIN_1, GPIOB);         // установить лог 0 на выходе
  IO_HIGH(IO_PIN_1, GPIOB);        // установить лог 1 на выходе
  IO_INPUT_MODE(IO_PIN_1, GPIOB);  // настроить пин на вход

  About::Setup();
  Leds::Setup();
  CANLib::Setup();
  ButtonsLeds::Setup();

  Leds::obj.SetOn(Leds::LED_WHITE, 50, 1950);

  uint32_t current_time = HAL_GetTick();
  while (1)
  {
    // don't need to update current_time because it is always updated by Loop() functions
    // current_time = HAL_GetTick();

    About::Loop(current_time);
    Leds::Loop(current_time);
    CANLib::Loop(current_time);
    ButtonsLeds::Loop(current_time);
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  // Common config
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  // Configure Regular Channel
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{
    // https://istarik.ru/blog/stm32/159.html

    CAN_FilterTypeDef sFilterConfig;

    // CAN interface initialization
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 4;
    hcan.Init.Mode = CAN_MODE_NORMAL; // CAN_MODE_NORMAL
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;   // DISABLE
    hcan.Init.AutoBusOff = ENABLE;           // DISABLE
    hcan.Init.AutoWakeUp = ENABLE;           // DISABLE
    hcan.Init.AutoRetransmission = DISABLE;  // DISABLE
    hcan.Init.ReceiveFifoLocked = ENABLE;    // DISABLE
    hcan.Init.TransmitFifoPriority = ENABLE; // DISABLE
    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        Error_Handler();
    }

    // CAN filtering initialization
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
    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{
  // SPI2 parameter configuration
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 89;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{
  hDebugUart.Instance = USART3;
  hDebugUart.Init.BaudRate = 500000;
  hDebugUart.Init.WordLength = UART_WORDLENGTH_8B;
  hDebugUart.Init.StopBits = UART_STOPBITS_1;
  hDebugUart.Init.Parity = UART_PARITY_NONE;
  hDebugUart.Init.Mode = UART_MODE_TX_RX;
  hDebugUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hDebugUart.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&hDebugUart) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // GPIO Ports Clock Enable
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Configure GPIO pin Output Level
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_3, GPIO_PIN_RESET);

  // Configure GPIO pin Output Level
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_RESET);

  // Configure GPIO pins : PC13 PC14 PC15
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // Configure GPIO pins : PA0 PA1 PA2 PA3 PA4 PA5 PA6 PA7 PA15
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure GPIO pins : PB2 PB12 PB3 PB4
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure GPIO pins : PA8 PA9 PA10
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure GPIO pins : PB5 PB6 PB7
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure GPIO pins : PB8 PB9
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1)
  {
    Timer1++;
  }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	Leds::obj.SetOn(Leds::LED_RED);
	Leds::obj.SetOff(Leds::LED_WHITE);
	
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
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
