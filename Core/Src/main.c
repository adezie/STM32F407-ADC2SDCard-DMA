/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#define ADC_BUFF 3
#define SD_BUFF 25*1024
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//uint16_t adc_buf[ADC_BUFF];
uint16_t sdcard_buf[SD_BUFF];
uint8_t log_buf[1024];
//uint8_t log_buf[SD_BUFF];
//char stringa[(5 * SD_BUFF)];
//char stringa[SD_BUFF];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
volatile unsigned long ulHighFrequencyTimerTicks;
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

TIM_HandleTypeDef htim10;

DMA_HandleTypeDef hdma_memtomem_dma2_stream1;
/* Definitions for blink1 */
osThreadId_t blink1Handle;
const osThreadAttr_t blink1_attributes = {
  .name = "blink1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for SD_Log */
osThreadId_t SD_LogHandle;
const osThreadAttr_t SD_Log_attributes = {
  .name = "SD_Log",
  .stack_size = 3500 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for storeSD */
osSemaphoreId_t storeSDHandle;
const osSemaphoreAttr_t storeSD_attributes = {
  .name = "storeSD"
};
/* USER CODE BEGIN PV */
void configureTimerForRunTimeStats(void) {//RTOS Debugging session
  ulHighFrequencyTimerTicks = 0;
  HAL_TIM_Base_Start_IT(&htim10);
}

unsigned long getRunTimeCounterValue(void) {//RTOS debugging session
  return ulHighFrequencyTimerTicks;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM10_Init(void);
void startblink1(void *argument);
void StartLog(void *argument);

/* USER CODE BEGIN PFP */
FRESULT res; /* FatFs function common result code */
uint32_t byteswritten, bytesread; /* File write/read counts */
uint8_t wtext[] = "ADC_DMA test"; /* File write buffer */
uint8_t rtext[_MAX_SS];/* File read buffer */
uint8_t fileOnce = 0;
FATFS myfile;
uint8_t feror = 0;
uint8_t fName[] = "logdata.dat\0";
FIL file;
FRESULT fR;
UINT bytesCnt = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	/* Implement your write code here, this is used by puts and printf for example */
	int i = 0;
	for (i = 0; i < len; i++)
		ITM_SendChar((*ptr++));
	return len;
}

/* USER CODE END 0 */

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
  MX_CAN1_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) sdcard_buf, SD_BUFF);
	__HAL_DMA_DISABLE_IT(&hdma_adc1, DMA_IT_HT);
	__HAL_DMA_DISABLE_IT(&hdma_memtomem_dma2_stream1, DMA_IT_HT);
//  __HAL_DMA_DISABLE_IT(&hdma_memtomem_dma2_stream1,DMA_IT_TC);
//  HAL_DMA_Init(&hdma_memtomem_dma2_stream1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of storeSD */
  storeSDHandle = osSemaphoreNew(1, 1, &storeSD_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of blink1 */
  blink1Handle = osThreadNew(startblink1, NULL, &blink1_attributes);

  /* creation of SD_Log */
  SD_LogHandle = osThreadNew(StartLog, NULL, &SD_Log_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 1679;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream1
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream1 on DMA2_Stream1 */
  hdma_memtomem_dma2_stream1.Instance = DMA2_Stream1;
  hdma_memtomem_dma2_stream1.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_memtomem_dma2_stream1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream1.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_memtomem_dma2_stream1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream1.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream1.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream1) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	osSemaphoreRelease(storeSDHandle); //giving semaphore to logging after
									   //the ADC data conversion completed
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_startblink1 */
/**
 * @brief  Function implementing the blink1 thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_startblink1 */
void startblink1(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);			// and stop data logging
		osDelay(1000);						 // after 1000 ms
		osSemaphoreDelete(storeSDHandle);//for debugging purpose, delete SDHandle semaphore
//		printf("oke");
	}
//  osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLog */
/**
 * @brief Function implementing the SD_Log thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLog */
void StartLog(void *argument)
{
  /* USER CODE BEGIN StartLog */
//	HAL_StatusTypeDef cek;
	MX_SDIO_SD_Init();
	MX_DMA_Init();
	/* Infinite loop */
	for (;;) {
		osSemaphoreAcquire(storeSDHandle, osWaitForever);//Don't start SDCard logging before the data ready
		HAL_DMA_Start(&hdma_memtomem_dma2_stream1, (uint32_t) sdcard_buf,
									(uint32_t) log_buf, sizeof(log_buf));//start the memory to memory DMA after ADC data ready
		HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream1,
									HAL_DMA_FULL_TRANSFER, HAL_MAX_DELAY);
		fR = f_mount(&myfile, SDPath, 1);
		if (fR == FR_OK) {
			if (f_open(&file, (char*) fName, FA_OPEN_APPEND | FA_WRITE)
					== FR_OK) {
//				sprintf(stringa,
//						"\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u\n%u,%u,%u",
//						log_buf[0], log_buf[1], log_buf[2], log_buf[3],
//						log_buf[4], log_buf[5], log_buf[6], log_buf[7],
//						log_buf[8], log_buf[9], log_buf[10], log_buf[11],
//						log_buf[12], log_buf[13], log_buf[14], log_buf[15],
//						log_buf[16], log_buf[17], log_buf[18], log_buf[19],
//						log_buf[20], log_buf[21], log_buf[22], log_buf[23],
//						log_buf[24], log_buf[25], log_buf[26], log_buf[27],
//						log_buf[28], log_buf[29], log_buf[30], log_buf[31],
//						log_buf[32], log_buf[33], log_buf[34], log_buf[35],
//						log_buf[36], log_buf[37], log_buf[38], log_buf[39],
//						log_buf[40], log_buf[41], log_buf[42], log_buf[43],
//						log_buf[44], log_buf[45], log_buf[46], log_buf[47],
//						log_buf[48], log_buf[49], log_buf[50], log_buf[51],
//						log_buf[52], log_buf[53], log_buf[54], log_buf[55],
//						log_buf[56], log_buf[57], log_buf[58], log_buf[59],
//						log_buf[60], log_buf[61], log_buf[62], log_buf[63],
//						log_buf[64], log_buf[65], log_buf[66], log_buf[67],
//						log_buf[68], log_buf[69], log_buf[70], log_buf[71],
//						log_buf[72], log_buf[73], log_buf[74], log_buf[75],
//						log_buf[76], log_buf[77], log_buf[78], log_buf[79],
//						log_buf[80], log_buf[81], log_buf[82], log_buf[83],
//						log_buf[84], log_buf[85], log_buf[86], log_buf[87],
//						log_buf[88], log_buf[89], log_buf[90], log_buf[91],
//						log_buf[92], log_buf[93], log_buf[94], log_buf[95],
//						log_buf[96], log_buf[97], log_buf[98], log_buf[99],
//						log_buf[100], log_buf[101], log_buf[102], log_buf[103],
//						log_buf[104], log_buf[105], log_buf[106], log_buf[107],
//						log_buf[108], log_buf[109], log_buf[110], log_buf[111],
//						log_buf[112], log_buf[113], log_buf[114], log_buf[115],
//						log_buf[116], log_buf[117], log_buf[118], log_buf[119],
//						log_buf[120], log_buf[121], log_buf[122], log_buf[123],
//						log_buf[124], log_buf[125], log_buf[126], log_buf[127],
//						log_buf[128], log_buf[129], log_buf[130], log_buf[131],
//						log_buf[132], log_buf[133], log_buf[134], log_buf[135],
//						log_buf[136], log_buf[137], log_buf[138], log_buf[139],
//						log_buf[140], log_buf[141], log_buf[142], log_buf[143],
//						log_buf[144], log_buf[145], log_buf[146], log_buf[147],
//						log_buf[148], log_buf[149], log_buf[150], log_buf[151],
//						log_buf[152], log_buf[153], log_buf[154], log_buf[155],
//						log_buf[156], log_buf[157], log_buf[158], log_buf[159],
//						log_buf[160], log_buf[161], log_buf[162], log_buf[163],
//						log_buf[164], log_buf[165], log_buf[166], log_buf[167],
//						log_buf[168], log_buf[169], log_buf[170], log_buf[171],
//						log_buf[172], log_buf[173], log_buf[174], log_buf[175],
//						log_buf[176], log_buf[177], log_buf[178], log_buf[179],
//						log_buf[180], log_buf[181], log_buf[182], log_buf[183],
//						log_buf[184], log_buf[185], log_buf[186], log_buf[187],
//						log_buf[188], log_buf[189], log_buf[190], log_buf[191],
//						log_buf[192], log_buf[193], log_buf[194], log_buf[195],
//						log_buf[196], log_buf[197], log_buf[198], log_buf[199],
//						log_buf[200], log_buf[201], log_buf[202], log_buf[203],
//						log_buf[204], log_buf[205], log_buf[206], log_buf[207],
//						log_buf[208], log_buf[209], log_buf[210], log_buf[211],
//						log_buf[212], log_buf[213], log_buf[214], log_buf[215],
//						log_buf[216], log_buf[217], log_buf[218], log_buf[219],
//						log_buf[220], log_buf[221], log_buf[222], log_buf[223],
//						log_buf[224], log_buf[225], log_buf[226], log_buf[227],
//						log_buf[228], log_buf[229], log_buf[230], log_buf[231],
//						log_buf[232], log_buf[233], log_buf[234], log_buf[235],
//						log_buf[236], log_buf[237], log_buf[238], log_buf[239],
//						log_buf[240], log_buf[241], log_buf[242], log_buf[243],
//						log_buf[244], log_buf[245], log_buf[246], log_buf[247],
//						log_buf[248], log_buf[249], log_buf[250], log_buf[251],
//						log_buf[252], log_buf[253], log_buf[254], log_buf[255],
//						log_buf[256], log_buf[257], log_buf[258], log_buf[259],
//						log_buf[260], log_buf[261], log_buf[262], log_buf[263],
//						log_buf[264], log_buf[265], log_buf[266], log_buf[267],
//						log_buf[268], log_buf[269], log_buf[270], log_buf[271],
//						log_buf[272], log_buf[273], log_buf[274], log_buf[275],
//						log_buf[276], log_buf[277], log_buf[278], log_buf[279],
//						log_buf[280], log_buf[281], log_buf[282], log_buf[283],
//						log_buf[284], log_buf[285], log_buf[286], log_buf[287],
//						log_buf[288], log_buf[289], log_buf[290], log_buf[291],
//						log_buf[292], log_buf[293], log_buf[294], log_buf[295],
//						log_buf[296], log_buf[297], log_buf[298], log_buf[299],
//						log_buf[300], log_buf[301], log_buf[302], log_buf[303],
//						log_buf[304], log_buf[305], log_buf[306], log_buf[307],
//						log_buf[308], log_buf[309], log_buf[310], log_buf[311],
//						log_buf[312], log_buf[313], log_buf[314], log_buf[315],
//						log_buf[316], log_buf[317], log_buf[318], log_buf[319],
//						log_buf[320], log_buf[321], log_buf[322], log_buf[323],
//						log_buf[324], log_buf[325], log_buf[326], log_buf[327],
//						log_buf[328], log_buf[329], log_buf[330], log_buf[331],
//						log_buf[332], log_buf[333], log_buf[334], log_buf[335],
//						log_buf[336], log_buf[337], log_buf[338], log_buf[339],
//						log_buf[340], log_buf[341], log_buf[342], log_buf[343],
//						log_buf[344], log_buf[345], log_buf[346], log_buf[347],
//						log_buf[348], log_buf[349], log_buf[350], log_buf[351],
//						log_buf[352], log_buf[353], log_buf[354], log_buf[355],
//						log_buf[356], log_buf[357], log_buf[358], log_buf[359],
//						log_buf[360], log_buf[361], log_buf[362], log_buf[363],
//						log_buf[364], log_buf[365], log_buf[366], log_buf[367],
//						log_buf[368], log_buf[369], log_buf[370], log_buf[371],
//						log_buf[372], log_buf[373], log_buf[374], log_buf[375],
//						log_buf[376], log_buf[377], log_buf[378], log_buf[379],
//						log_buf[380], log_buf[381], log_buf[382], log_buf[383],
//						log_buf[384], log_buf[385], log_buf[386], log_buf[387],
//						log_buf[388], log_buf[389], log_buf[390], log_buf[391],
//						log_buf[392], log_buf[393], log_buf[394], log_buf[395],
//						log_buf[396], log_buf[397], log_buf[398], log_buf[399],
//						log_buf[400], log_buf[401], log_buf[402], log_buf[403],
//						log_buf[404], log_buf[405], log_buf[406], log_buf[407],
//						log_buf[408], log_buf[409], log_buf[410], log_buf[411],
//						log_buf[412], log_buf[413], log_buf[414], log_buf[415],
//						log_buf[416], log_buf[417], log_buf[418], log_buf[419],
//						log_buf[420], log_buf[421], log_buf[422], log_buf[423],
//						log_buf[424], log_buf[425], log_buf[426], log_buf[427],
//						log_buf[428], log_buf[429], log_buf[430], log_buf[431],
//						log_buf[432], log_buf[433], log_buf[434], log_buf[435],
//						log_buf[436], log_buf[437], log_buf[438], log_buf[439],
//						log_buf[440], log_buf[441], log_buf[442], log_buf[443],
//						log_buf[444], log_buf[445], log_buf[446], log_buf[447],
//						log_buf[448], log_buf[449], log_buf[450], log_buf[451],
//						log_buf[452], log_buf[453], log_buf[454], log_buf[455],
//						log_buf[456], log_buf[457], log_buf[458], log_buf[459],
//						log_buf[460], log_buf[461], log_buf[462], log_buf[463],
//						log_buf[464], log_buf[465], log_buf[466], log_buf[467],
//						log_buf[468], log_buf[469], log_buf[470], log_buf[471],
//						log_buf[472], log_buf[473], log_buf[474], log_buf[475],
//						log_buf[476], log_buf[477], log_buf[478], log_buf[479],
//						log_buf[480], log_buf[481], log_buf[482], log_buf[483],
//						log_buf[484], log_buf[485], log_buf[486], log_buf[487],
//						log_buf[488], log_buf[489], log_buf[490], log_buf[491],
//						log_buf[492], log_buf[493], log_buf[494], log_buf[495],
//						log_buf[496], log_buf[497], log_buf[498], log_buf[499],
//						log_buf[500], log_buf[501], log_buf[502], log_buf[503],
//						log_buf[504], log_buf[505], log_buf[506], log_buf[507],
//						log_buf[508], log_buf[509], log_buf[510], log_buf[511],
//						log_buf[512], log_buf[513], log_buf[514], log_buf[515],
//						log_buf[516], log_buf[517], log_buf[518], log_buf[519],
//						log_buf[520], log_buf[521], log_buf[522], log_buf[523],
//						log_buf[524], log_buf[525], log_buf[526], log_buf[527],
//						log_buf[528], log_buf[529], log_buf[530], log_buf[531],
//						log_buf[532], log_buf[533], log_buf[534], log_buf[535],
//						log_buf[536], log_buf[537], log_buf[538], log_buf[539],
//						log_buf[540], log_buf[541], log_buf[542], log_buf[543],
//						log_buf[544], log_buf[545], log_buf[546], log_buf[547],
//						log_buf[548], log_buf[549], log_buf[550], log_buf[551],
//						log_buf[552], log_buf[553], log_buf[554], log_buf[555],
//						log_buf[556], log_buf[557], log_buf[558], log_buf[559],
//						log_buf[560], log_buf[561], log_buf[562], log_buf[563],
//						log_buf[564], log_buf[565], log_buf[566], log_buf[567],
//						log_buf[568], log_buf[569], log_buf[570], log_buf[571],
//						log_buf[572], log_buf[573], log_buf[574], log_buf[575],
//						log_buf[576], log_buf[577], log_buf[578], log_buf[579],
//						log_buf[580], log_buf[581], log_buf[582], log_buf[583],
//						log_buf[584], log_buf[585], log_buf[586], log_buf[587],
//						log_buf[588], log_buf[589], log_buf[590], log_buf[591],
//						log_buf[592], log_buf[593], log_buf[594], log_buf[595],
//						log_buf[596], log_buf[597], log_buf[598], log_buf[599],
//						log_buf[600], log_buf[601], log_buf[602], log_buf[603],
//						log_buf[604], log_buf[605], log_buf[606], log_buf[607],
//						log_buf[608], log_buf[609], log_buf[610], log_buf[611],
//						log_buf[612], log_buf[613], log_buf[614], log_buf[615],
//						log_buf[616], log_buf[617], log_buf[618], log_buf[619],
//						log_buf[620], log_buf[621], log_buf[622], log_buf[623],
//						log_buf[624], log_buf[625], log_buf[626], log_buf[627],
//						log_buf[628], log_buf[629], log_buf[630], log_buf[631],
//						log_buf[632], log_buf[633], log_buf[634], log_buf[635],
//						log_buf[636], log_buf[637], log_buf[638], log_buf[639],
//						log_buf[640], log_buf[641], log_buf[642], log_buf[643],
//						log_buf[644], log_buf[645], log_buf[646], log_buf[647],
//						log_buf[648], log_buf[649], log_buf[650], log_buf[651],
//						log_buf[652], log_buf[653], log_buf[654], log_buf[655],
//						log_buf[656], log_buf[657], log_buf[658], log_buf[659],
//						log_buf[660], log_buf[661], log_buf[662], log_buf[663],
//						log_buf[664], log_buf[665], log_buf[666], log_buf[667],
//						log_buf[668], log_buf[669], log_buf[670], log_buf[671],
//						log_buf[672], log_buf[673], log_buf[674], log_buf[675],
//						log_buf[676], log_buf[677], log_buf[678], log_buf[679],
//						log_buf[680], log_buf[681], log_buf[682], log_buf[683],
//						log_buf[684], log_buf[685], log_buf[686], log_buf[687],
//						log_buf[688], log_buf[689], log_buf[690], log_buf[691],
//						log_buf[692], log_buf[693], log_buf[694], log_buf[695],
//						log_buf[696], log_buf[697], log_buf[698], log_buf[699],
//						log_buf[700], log_buf[701], log_buf[702], log_buf[703],
//						log_buf[704], log_buf[705], log_buf[706], log_buf[707],
//						log_buf[708], log_buf[709], log_buf[710], log_buf[711],
//						log_buf[712], log_buf[713], log_buf[714], log_buf[715],
//						log_buf[716], log_buf[717], log_buf[718], log_buf[719],
//						log_buf[720], log_buf[721], log_buf[722], log_buf[723],
//						log_buf[724], log_buf[725], log_buf[726], log_buf[727],
//						log_buf[728], log_buf[729], log_buf[730], log_buf[731],
//						log_buf[732], log_buf[733], log_buf[734], log_buf[735],
//						log_buf[736], log_buf[737], log_buf[738], log_buf[739],
//						log_buf[740], log_buf[741], log_buf[742], log_buf[743],
//						log_buf[744], log_buf[745], log_buf[746], log_buf[747],
//						log_buf[748], log_buf[749], log_buf[750], log_buf[751],
//						log_buf[752], log_buf[753], log_buf[754], log_buf[755],
//						log_buf[756], log_buf[757], log_buf[758], log_buf[759],
//						log_buf[760], log_buf[761], log_buf[762], log_buf[763],
//						log_buf[764], log_buf[765], log_buf[766], log_buf[767],
//						log_buf[768], log_buf[769], log_buf[770], log_buf[771],
//						log_buf[772], log_buf[773], log_buf[774], log_buf[775],
//						log_buf[776], log_buf[777], log_buf[778], log_buf[779],
//						log_buf[780], log_buf[781], log_buf[782], log_buf[783],
//						log_buf[784], log_buf[785], log_buf[786], log_buf[787],
//						log_buf[788], log_buf[789], log_buf[790], log_buf[791],
//						log_buf[792], log_buf[793], log_buf[794], log_buf[795],
//						log_buf[796], log_buf[797], log_buf[798], log_buf[799],
//						log_buf[800], log_buf[801], log_buf[802], log_buf[803],
//						log_buf[804], log_buf[805], log_buf[806], log_buf[807],
//						log_buf[808], log_buf[809], log_buf[810], log_buf[811],
//						log_buf[812], log_buf[813], log_buf[814], log_buf[815],
//						log_buf[816], log_buf[817], log_buf[818], log_buf[819],
//						log_buf[820], log_buf[821], log_buf[822], log_buf[823],
//						log_buf[824], log_buf[825], log_buf[826], log_buf[827],
//						log_buf[828], log_buf[829], log_buf[830], log_buf[831],
//						log_buf[832], log_buf[833], log_buf[834], log_buf[835],
//						log_buf[836], log_buf[837], log_buf[838], log_buf[839],
//						log_buf[840], log_buf[841], log_buf[842], log_buf[843],
//						log_buf[844], log_buf[845], log_buf[846], log_buf[847],
//						log_buf[848], log_buf[849], log_buf[850], log_buf[851],
//						log_buf[852], log_buf[853], log_buf[854], log_buf[855],
//						log_buf[856], log_buf[857], log_buf[858], log_buf[859],
//						log_buf[860], log_buf[861], log_buf[862], log_buf[863],
//						log_buf[864], log_buf[865], log_buf[866], log_buf[867],
//						log_buf[868], log_buf[869], log_buf[870], log_buf[871],
//						log_buf[872], log_buf[873], log_buf[874], log_buf[875],
//						log_buf[876], log_buf[877], log_buf[878], log_buf[879],
//						log_buf[880], log_buf[881], log_buf[882], log_buf[883],
//						log_buf[884], log_buf[885], log_buf[886], log_buf[887],
//						log_buf[888], log_buf[889], log_buf[890], log_buf[891],
//						log_buf[892], log_buf[893], log_buf[894], log_buf[895],
//						log_buf[896], log_buf[897], log_buf[898], log_buf[899],
//						log_buf[900], log_buf[901], log_buf[902], log_buf[903],
//						log_buf[904], log_buf[905], log_buf[906], log_buf[907],
//						log_buf[908], log_buf[909], log_buf[910], log_buf[911],
//						log_buf[912], log_buf[913], log_buf[914], log_buf[915],
//						log_buf[916], log_buf[917], log_buf[918], log_buf[919],
//						log_buf[920], log_buf[921], log_buf[922], log_buf[923],
//						log_buf[924], log_buf[925], log_buf[926], log_buf[927],
//						log_buf[928], log_buf[929], log_buf[930], log_buf[931],
//						log_buf[932], log_buf[933], log_buf[934], log_buf[935],
//						log_buf[936], log_buf[937], log_buf[938], log_buf[939],
//						log_buf[940], log_buf[941], log_buf[942], log_buf[943],
//						log_buf[944], log_buf[945], log_buf[946], log_buf[947],
//						log_buf[948], log_buf[949], log_buf[950], log_buf[951],
//						log_buf[952], log_buf[953], log_buf[954], log_buf[955],
//						log_buf[956], log_buf[957], log_buf[958], log_buf[959],
//						log_buf[960], log_buf[961], log_buf[962], log_buf[963],
//						log_buf[964], log_buf[965], log_buf[966], log_buf[967],
//						log_buf[968], log_buf[969], log_buf[970], log_buf[971],
//						log_buf[972], log_buf[973], log_buf[974], log_buf[975],
//						log_buf[976], log_buf[977], log_buf[978], log_buf[979],
//						log_buf[980], log_buf[981], log_buf[982], log_buf[983],
//						log_buf[984], log_buf[985], log_buf[986], log_buf[987],
//						log_buf[988], log_buf[989], log_buf[990], log_buf[991],
//						log_buf[992], log_buf[993], log_buf[994], log_buf[995],
//						log_buf[996], log_buf[997], log_buf[998], log_buf[999],
//						log_buf[1000], log_buf[1001], log_buf[1002],
//						log_buf[1003], log_buf[1004], log_buf[1005],
//						log_buf[1006], log_buf[1007], log_buf[1008],
//						log_buf[1009], log_buf[1010], log_buf[1011],
//						log_buf[1012], log_buf[1013], log_buf[1014],
//						log_buf[1015], log_buf[1016], log_buf[1017],
//						log_buf[1018], log_buf[1019], log_buf[1020],
//						log_buf[1021], log_buf[1022], log_buf[1023],
//						log_buf[1024], log_buf[1025], log_buf[1026],
//						log_buf[1027], log_buf[1028], log_buf[1029],
//						log_buf[1030], log_buf[1031], log_buf[1032],
//						log_buf[1033], log_buf[1034], log_buf[1035],
//						log_buf[1036], log_buf[1037], log_buf[1038],
//						log_buf[1039], log_buf[1040], log_buf[1041],
//						log_buf[1042], log_buf[1043], log_buf[1044],
//						log_buf[1045], log_buf[1046], log_buf[1047],
//						log_buf[1048], log_buf[1049], log_buf[1050],
//						log_buf[1051], log_buf[1052], log_buf[1053],
//						log_buf[1054], log_buf[1055], log_buf[1056],
//						log_buf[1057], log_buf[1058], log_buf[1059],
//						log_buf[1060], log_buf[1061], log_buf[1062],
//						log_buf[1063], log_buf[1064], log_buf[1065],
//						log_buf[1066], log_buf[1067], log_buf[1068],
//						log_buf[1069], log_buf[1070], log_buf[1071],
//						log_buf[1072], log_buf[1073], log_buf[1074],
//						log_buf[1075], log_buf[1076], log_buf[1077],
//						log_buf[1078], log_buf[1079], log_buf[1080],
//						log_buf[1081], log_buf[1082], log_buf[1083],
//						log_buf[1084], log_buf[1085], log_buf[1086],
//						log_buf[1087], log_buf[1088], log_buf[1089],
//						log_buf[1090], log_buf[1091], log_buf[1092],
//						log_buf[1093], log_buf[1094], log_buf[1095],
//						log_buf[1096], log_buf[1097], log_buf[1098],
//						log_buf[1099], log_buf[1100], log_buf[1101],
//						log_buf[1102], log_buf[1103], log_buf[1104],
//						log_buf[1105], log_buf[1106], log_buf[1107],
//						log_buf[1108], log_buf[1109], log_buf[1110],
//						log_buf[1111], log_buf[1112], log_buf[1113],
//						log_buf[1114], log_buf[1115], log_buf[1116],
//						log_buf[1117], log_buf[1118], log_buf[1119],
//						log_buf[1120], log_buf[1121], log_buf[1122],
//						log_buf[1123], log_buf[1124], log_buf[1125],
//						log_buf[1126], log_buf[1127], log_buf[1128],
//						log_buf[1129], log_buf[1130], log_buf[1131],
//						log_buf[1132], log_buf[1133], log_buf[1134],
//						log_buf[1135], log_buf[1136], log_buf[1137],
//						log_buf[1138], log_buf[1139], log_buf[1140],
//						log_buf[1141], log_buf[1142], log_buf[1143],
//						log_buf[1144], log_buf[1145], log_buf[1146],
//						log_buf[1147], log_buf[1148], log_buf[1149],
//						log_buf[1150], log_buf[1151], log_buf[1152],
//						log_buf[1153], log_buf[1154], log_buf[1155],
//						log_buf[1156], log_buf[1157], log_buf[1158],
//						log_buf[1159], log_buf[1160], log_buf[1161],
//						log_buf[1162], log_buf[1163], log_buf[1164],
//						log_buf[1165], log_buf[1166], log_buf[1167],
//						log_buf[1168], log_buf[1169], log_buf[1170],
//						log_buf[1171], log_buf[1172], log_buf[1173],
//						log_buf[1174], log_buf[1175], log_buf[1176],
//						log_buf[1177], log_buf[1178], log_buf[1179],
//						log_buf[1180], log_buf[1181], log_buf[1182],
//						log_buf[1183], log_buf[1184], log_buf[1185],
//						log_buf[1186], log_buf[1187], log_buf[1188],
//						log_buf[1189], log_buf[1190], log_buf[1191],
//						log_buf[1192], log_buf[1193], log_buf[1194],
//						log_buf[1195], log_buf[1196], log_buf[1197],
//						log_buf[1198], log_buf[1199], log_buf[1200],
//						log_buf[1201], log_buf[1202], log_buf[1203],
//						log_buf[1204], log_buf[1205], log_buf[1206],
//						log_buf[1207], log_buf[1208], log_buf[1209],
//						log_buf[1210], log_buf[1211], log_buf[1212],
//						log_buf[1213], log_buf[1214], log_buf[1215],
//						log_buf[1216], log_buf[1217], log_buf[1218],
//						log_buf[1219], log_buf[1220], log_buf[1221],
//						log_buf[1222], log_buf[1223], log_buf[1224],
//						log_buf[1225], log_buf[1226], log_buf[1227],
//						log_buf[1228], log_buf[1229], log_buf[1230],
//						log_buf[1231], log_buf[1232], log_buf[1233],
//						log_buf[1234], log_buf[1235], log_buf[1236],
//						log_buf[1237], log_buf[1238], log_buf[1239],
//						log_buf[1240], log_buf[1241], log_buf[1242],
//						log_buf[1243], log_buf[1244], log_buf[1245],
//						log_buf[1246], log_buf[1247], log_buf[1248],
//						log_buf[1249], log_buf[1250], log_buf[1251],
//						log_buf[1252], log_buf[1253], log_buf[1254],
//						log_buf[1255], log_buf[1256], log_buf[1257],
//						log_buf[1258], log_buf[1259], log_buf[1260],
//						log_buf[1261], log_buf[1262], log_buf[1263],
//						log_buf[1264], log_buf[1265], log_buf[1266],
//						log_buf[1267], log_buf[1268], log_buf[1269],
//						log_buf[1270], log_buf[1271], log_buf[1272],
//						log_buf[1273], log_buf[1274], log_buf[1275],
//						log_buf[1276], log_buf[1277], log_buf[1278],
//						log_buf[1279], log_buf[1280], log_buf[1281],
//						log_buf[1282], log_buf[1283], log_buf[1284],
//						log_buf[1285], log_buf[1286], log_buf[1287],
//						log_buf[1288], log_buf[1289], log_buf[1290],
//						log_buf[1291], log_buf[1292], log_buf[1293],
//						log_buf[1294], log_buf[1295], log_buf[1296],
//						log_buf[1297], log_buf[1298], log_buf[1299],
//						log_buf[1300], log_buf[1301], log_buf[1302],
//						log_buf[1303], log_buf[1304], log_buf[1305],
//						log_buf[1306], log_buf[1307], log_buf[1308],
//						log_buf[1309], log_buf[1310], log_buf[1311],
//						log_buf[1312], log_buf[1313], log_buf[1314],
//						log_buf[1315], log_buf[1316], log_buf[1317],
//						log_buf[1318], log_buf[1319], log_buf[1320],
//						log_buf[1321], log_buf[1322], log_buf[1323],
//						log_buf[1324], log_buf[1325], log_buf[1326],
//						log_buf[1327], log_buf[1328], log_buf[1329],
//						log_buf[1330], log_buf[1331], log_buf[1332],
//						log_buf[1333], log_buf[1334], log_buf[1335],
//						log_buf[1336], log_buf[1337], log_buf[1338],
//						log_buf[1339], log_buf[1340], log_buf[1341],
//						log_buf[1342], log_buf[1343], log_buf[1344],
//						log_buf[1345], log_buf[1346], log_buf[1347],
//						log_buf[1348], log_buf[1349], log_buf[1350],
//						log_buf[1351], log_buf[1352], log_buf[1353],
//						log_buf[1354], log_buf[1355], log_buf[1356],
//						log_buf[1357], log_buf[1358], log_buf[1359],
//						log_buf[1360], log_buf[1361], log_buf[1362],
//						log_buf[1363], log_buf[1364], log_buf[1365],
//						log_buf[1366], log_buf[1367], log_buf[1368],
//						log_buf[1369], log_buf[1370], log_buf[1371],
//						log_buf[1372], log_buf[1373], log_buf[1374],
//						log_buf[1375], log_buf[1376], log_buf[1377],
//						log_buf[1378], log_buf[1379], log_buf[1380],
//						log_buf[1381], log_buf[1382], log_buf[1383],
//						log_buf[1384], log_buf[1385], log_buf[1386],
//						log_buf[1387], log_buf[1388], log_buf[1389],
//						log_buf[1390], log_buf[1391], log_buf[1392],
//						log_buf[1393], log_buf[1394], log_buf[1395],
//						log_buf[1396], log_buf[1397], log_buf[1398],
//						log_buf[1399], log_buf[1400], log_buf[1401],
//						log_buf[1402], log_buf[1403], log_buf[1404],
//						log_buf[1405], log_buf[1406], log_buf[1407],
//						log_buf[1408], log_buf[1409], log_buf[1410],
//						log_buf[1411], log_buf[1412], log_buf[1413],
//						log_buf[1414], log_buf[1415], log_buf[1416],
//						log_buf[1417], log_buf[1418], log_buf[1419],
//						log_buf[1420], log_buf[1421], log_buf[1422],
//						log_buf[1423], log_buf[1424], log_buf[1425],
//						log_buf[1426], log_buf[1427], log_buf[1428],
//						log_buf[1429], log_buf[1430], log_buf[1431],
//						log_buf[1432], log_buf[1433], log_buf[1434],
//						log_buf[1435], log_buf[1436], log_buf[1437],
//						log_buf[1438], log_buf[1439], log_buf[1440],
//						log_buf[1441], log_buf[1442], log_buf[1443],
//						log_buf[1444], log_buf[1445], log_buf[1446],
//						log_buf[1447], log_buf[1448], log_buf[1449],
//						log_buf[1450], log_buf[1451], log_buf[1452],
//						log_buf[1453], log_buf[1454], log_buf[1455],
//						log_buf[1456], log_buf[1457], log_buf[1458],
//						log_buf[1459], log_buf[1460], log_buf[1461],
//						log_buf[1462], log_buf[1463], log_buf[1464],
//						log_buf[1465], log_buf[1466], log_buf[1467],
//						log_buf[1468], log_buf[1469], log_buf[1470],
//						log_buf[1471], log_buf[1472], log_buf[1473],
//						log_buf[1474], log_buf[1475], log_buf[1476],
//						log_buf[1477], log_buf[1478], log_buf[1479],
//						log_buf[1480], log_buf[1481], log_buf[1482],
//						log_buf[1483], log_buf[1484], log_buf[1485],
//						log_buf[1486], log_buf[1487], log_buf[1488],
//						log_buf[1489], log_buf[1490], log_buf[1491],
//						log_buf[1492], log_buf[1493], log_buf[1494],
//						log_buf[1495], log_buf[1496], log_buf[1497],
//						log_buf[1498], log_buf[1499]);

				fR = f_write(&file, log_buf, 2*SD_BUFF , &bytesCnt);
				f_close(&file);
//				printf("ok");
//				memset(&stringa[0], 0, sizeof(stringa));
			}
		}
	}
  /* USER CODE END StartLog */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM13 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM13) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
//	  printf("error Occured \n");
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
