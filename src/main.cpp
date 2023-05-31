/* USER CODE BEGIN Header */
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <LoggerLibrary.h>
#include <About.h>
#include <Leds.h>
#include <CANLogic.h>
#include <ButtonsLeds.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	
#define LedGreen_ON				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)			//
#define LedGreen_OFF			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)		//
#define LedRed_ON					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET)			//
#define LedRed_OFF				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET)		//


#define IO_PIN_1                    GPIO_PIN_5			// GPIOB
#define IO_PIN_2                    GPIO_PIN_6			// GPIOB
#define IO_PIN_3                    GPIO_PIN_7			// GPIOB
#define IO_PIN_4                    GPIO_PIN_8			// GPIOB
#define IO_PIN_5                    GPIO_PIN_9			// GPIOB
#define IO_PIN_6                    GPIO_PIN_13			// GPIOC
#define IO_PIN_7                    GPIO_PIN_14			// GPIOC
#define IO_PIN_8                    GPIO_PIN_15			// GPIOC
#define IO_PIN_9                    GPIO_PIN_0			// GPIOA
#define IO_PIN_10                   GPIO_PIN_1			// GPIOA


#define IO_INPUT_MODE(PIN,PORT)			(PORT)->ODR |= PIN;         // настроить пин на вход
#define IO_OUTPUT_MODE(PIN,PORT)		(PORT)->IDR |= PIN          // настроить пин на выход
#define IO_LOW(PIN,PORT)						HAL_GPIO_WritePin((PORT), (PIN), GPIO_PIN_RESET); 		// лог 0 на выход
#define IO_HIGH(PIN,PORT)						HAL_GPIO_WritePin((PORT), (PIN), GPIO_PIN_SET);       // лог 1 на выход
#define IO_READ(PIN,PORT)           HAL_GPIO_ReadPin((PORT), (PIN))   // чтение пина



#define STBY_CAN_H()      	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);	// установить выход защелка в High
#define STBY_CAN_L()       HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);	// установить выход защелка в Low
#define VIO_CAN_H()      	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);	// установить выход защелка в High
#define VIO_CAN_L()       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);	// установить выход защелка в Low


#define ClearBit(reg, bit)       reg &= (~(1<<(bit)))   //пример: ClearBit(PORTB, 1); //сбросить 1-й бит PORTB
#define SetBit(reg, bit)          reg |= (1<<(bit))     //пример: SetBit(PORTB, 3); //установить 3-й бит PORTB
#define BitIsClear(reg, bit)    ((reg & (1<<(bit))) == 0)		//пример: if (BitIsClear(PORTB,1)) {...} //если бит очищен
#define BitIsSet(reg, bit)       ((reg & (1<<(bit))) != 0)		//пример: if(BitIsSet(PORTB,2)) {...} //если бит установлен

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef hDebugUart;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

	uint32_t byteswritten,bytesread;
	uint8_t result;
	uint8_t readCAN =0;
	
// For CAN
	CAN_TxHeaderTypeDef TxHeader;
	CAN_RxHeaderTypeDef RxHeader;
	uint8_t TxData[8] = {0,};
	uint8_t RxData[8] = {0,};
	uint32_t TxMailbox = 0;
	uint8_t trans_str[30];

// For Timers
	uint32_t Timer1 = 0;
	
// For Regester
	uint8_t LedBuf[2];
	uint8_t ButtonBuf[2] = {0,};
	uint8_t Button[2];
	uint8_t Led[2] = {0,};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/*
void HC595_Write_HC165_Read(){
	CE_L();        		// Write enadle
	SH_LD_165_L();		// Reset HC165
	SH_LD_165_H();
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) LedBuf, (uint8_t *)ButtonBuf, 2, 5000);
	CE_H();       		// Write disable
}
*/



/// @brief Callback function of CAN receiver.
/// @param hcan Pointer to the structure that contains CAN configuration.
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8] = {0};

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        CANLib::can_manager.IncomingCANFrame(RxHeader.StdId, RxData, RxHeader.DLC);
		CANLib::can_manager_light.IncomingCANFrame(RxHeader.StdId, RxData, RxHeader.DLC);
        // DEBUG_LOG("RX: CAN 0x%04lX", RxHeader.StdId);
    }
}

/// @brief Callback function for CAN error handler
/// @param hcan Pointer to the structure that contains CAN configuration.
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t er = HAL_CAN_GetError(hcan);
    DEBUG_LOG("CAN ERROR: %lu %08lX", (unsigned long)er, (unsigned long)er);
}

/// @brief Sends data via CAN bus
/// @param id CANObject ID
/// @param data Pointer to the CAN frame data buffer (8 bytes max)
/// @param length Length of the CAN frame data buffer
void HAL_CAN_Send(can_object_id_t id, uint8_t *data, uint8_t length)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8] = {0};
    uint32_t TxMailbox = 0;

    TxHeader.StdId = id;                   // Standard frame ID (sets to 0 if extended one used)
    TxHeader.ExtId = 0;                    // Extended frame ID (sets to 0 if standard one used)
    TxHeader.RTR = CAN_RTR_DATA;           // CAN_RTR_DATA: CAN frame with data will be sent
                                           // CAN_RTR_REMOTE: remote CAN frame will be sent
    TxHeader.IDE = CAN_ID_STD;             // CAN_ID_STD: CAN frame with standard ID
                                           // CAN_ID_EXT: CAN frame with extended ID
    TxHeader.DLC = length;                 // Data length of the CAN frame
    TxHeader.TransmitGlobalTime = DISABLE; // Time Triggered Communication Mode

    memcpy(TxData, data, length);

    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0)
    {
        Leds::obj.SetOn(Leds::LED_RED);
    }
    Leds::obj.SetOff(Leds::LED_RED);

    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
        DEBUG_LOG("CAN TX ERROR: 0x%04lX", TxHeader.StdId);
    }
}




/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	


	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);

	STBY_CAN_L();
	VIO_CAN_H();

	LedGreen_OFF;
	LedRed_OFF;
	
	//HC595_Write_HC165_Read();
	Button[0] = ButtonBuf[0];
	Button[1] = ButtonBuf[1];
	
		/* 
		Заполняем структуру отвечающую за отправку кадров
		StdId — это идентификатор стандартного кадра.
		ExtId — это идентификатор расширенного кадра.
		RTR = CAN_RTR_DATA — это говорит о том, что мы отправляем кадр с данными (Data Frame). Если указать CAN_RTR_REMOTE, тогда это будет Remote Frame.
		IDE = CAN_ID_STD — это говорит о том, что мы отправляем стандартный кадр. Если указать CAN_ID_EXT, тогда это будет расширенный кадр. В StdId нужно будет указать 0, а в ExtId записать расширенный идентификатор.
		DLC = 8 — количество полезных байт передаваемых в кадре (от 1 до 8).
		TransmitGlobalTime — относится к Time Triggered Communication Mode, мы это не используем поэтому пишем 0.
	*/
	TxHeader.StdId = 0x07B0;
	TxHeader.ExtId = 0;
	TxHeader.RTR = CAN_RTR_DATA; //CAN_RTR_REMOTE
	TxHeader.IDE = CAN_ID_STD;   // CAN_ID_EXT
	TxHeader.DLC = 8;
	TxHeader.TransmitGlobalTime = DISABLE;
	
	
	/* активируем события которые будут вызывать прерывания  */
	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);
	
	HAL_CAN_Start(&hcan);
	
	TxHeader.StdId = 0x07B0;
	TxHeader.DLC = 8;
	TxData[0] = 0x44;
	TxData[1] = 0x53;
	TxData[2] = 0x46;
	TxData[3] = 0x30;
	TxData[4] = 0x30;
	TxData[5] = 0x30;
	TxData[6] = 0x33;
	TxData[7] = 0x00;

	
	
	

	IO_OUTPUT_MODE(IO_PIN_1,GPIOB);				// настроить пин на выход
	IO_LOW(IO_PIN_1,GPIOB);         			// установить лог 0 на выходе
	IO_HIGH(IO_PIN_1,GPIOB);         			// установить лог 1 на выходе
	IO_INPUT_MODE(IO_PIN_1,GPIOB);				// настроить пин на вход




	About::Setup();
	Leds::Setup();
	CANLib::Setup();
	ButtonsLeds::Setup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint8_t cn=0;


	uint32_t current_time = HAL_GetTick();
	
	while (1)
	{
        // don't need to update current_time because it is always updated by Loop() functions
        // current_time = HAL_GetTick();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		About::Loop(current_time);
		Leds::Loop(current_time);
		//CANLib::Loop(current_time);
		ButtonsLeds::Loop(current_time);
		
		
		
		if(readCAN != 0){
			readCAN = 0;
			if(RxData[6] == 0x31){
        LedGreen_ON;
			}
			if(RxData[6] == 0x32){
				LedGreen_OFF;
			}
		}
		
		cn++;

		LedBuf[0] = cn;
		LedBuf[1] = cn;
		
		//HC595_Write_HC165_Read();		// запись и чтение сдвиговых регистров

		// если есть нажатие
		if(Button[0] != ButtonBuf[0] || Button[1] != ButtonBuf[1]){
			Button[0] = ButtonBuf[0];
			Button[1] = ButtonBuf[1];
			
			TxHeader.StdId = 0x07B0;
			TxHeader.DLC = 8;
			TxData[0] = 0x44;
			TxData[1] = 0x53;
			TxData[2] = 0x46;
			TxData[3] = 0x30;
			TxData[4] = 0x30;
			TxData[5] = 0x30;
			TxData[6] = ButtonBuf[0];
			TxData[7] = ButtonBuf[1];
			//HAL_CAN_Send();
			
			if(BitIsSet(Button[0], 0)) {
				LedGreen_OFF;
			}
			if(BitIsSet(Button[0], 1)) {
				LedGreen_ON;
			}
			
		}

	//HAL_Delay(50);
		
	}
  /* USER CODE END 3 */
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
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
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
	CAN_FilterTypeDef  sFilterConfig;
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; 
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	//sFilterConfig.SlaveStartFilterBank = 14;

	if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
	Error_Handler();
	}
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
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
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
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
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
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
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB3 PB4 */
  GPIO_InitStruct.Pin = /*GPIO_PIN_2|*/GPIO_PIN_12|GPIO_PIN_3/*|GPIO_PIN_4*/;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim1)
	{
		Timer1++;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
