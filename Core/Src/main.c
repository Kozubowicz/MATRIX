/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>


#include "max7219_matrix.h"
#include "GFX_BW.h"
#include "fonts/fonts.h"
#include <string.h>
char Text[50];
char str[255] ="Systemy Operacyjne Mikrokontrolerow";

int sensor = 0;
int sensor11 = 0;
int sensor12 = 0;
uint16_t del = 100;
uint8_t flag = 2;
uint8_t intesivity = 99;

/* Variables for AHT10 */
uint8_t AHT10_RX_Data[6];
uint32_t AHT10_ADC_Raw;
float AHT10_Temperature = 0;
float AHT10_Humidity;
uint8_t AHT10_TmpHum_Cmd = 0xAC;

#define AHT10_Adress 0x38 << 1

uint8_t img[]={0x00,0x66,0xFF,0xFF,0xFF,0x7E,0x3C,0x18,0x00,0x66,0xFF,0xFF,0xFF,0x7E,0x3C,0x18};

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c3;

RNG_HandleTypeDef hrng;

SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MartixTask */
osThreadId_t MartixTaskHandle;
const osThreadAttr_t MartixTask_attributes = {
  .name = "MartixTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for PotentionTask */
osThreadId_t PotentionTaskHandle;
const osThreadAttr_t PotentionTask_attributes = {
  .name = "PotentionTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SwitchTask */
osThreadId_t SwitchTaskHandle;
const osThreadAttr_t SwitchTask_attributes = {
  .name = "SwitchTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TempTask */
osThreadId_t TempTaskHandle;
const osThreadAttr_t TempTask_attributes = {
  .name = "TempTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = {
  .name = "UartTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UartQueue */
osMessageQueueId_t UartQueueHandle;
const osMessageQueueAttr_t UartQueue_attributes = {
  .name = "UartQueue"
};
/* USER CODE BEGIN PV */
uint8_t uart2RecBuf = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RNG_Init(void);
void StartDefaultTask(void *argument);
void StartMartixTask(void *argument);
void StartPotentionTask(void *argument);
void StartSwitchTask(void *argument);
void StartTempTask(void *argument);
void StartUartTask(void *argument);

/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (1) // interrupt rx
	{
		//osMessageQueuePut(mq_id, msg_ptr, msg_prio, timeout)
		osMessageQueuePut(UartQueueHandle, &uart2RecBuf, 0, 0);
		//mask= mask? 0:1 ;
		HAL_UART_Receive_IT(&huart2, &uart2RecBuf, 1);
	}
}

uint8_t scan(uint8_t *stt){
	uint8_t f = 0;
	for (int i = 1; i <255; i++){
		if(stt[i] == '#' ){
			f = 1;
		}
		if(f == 0){
			str[i-1]=stt[i];
		}
		else{
			str[i-1]= NULL;
		}
	}

}

void graf(uint8_t *stt){
	char ff[16];
	uint8_t ii = 1;
	for (int i = 0; i < 16; i++){
		ii += 6;

	}

}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
  MAX7219_Init(&hspi3);
  HAL_Delay(100);
  GFX_SetFont(font_8x5);
  GFX_SetFontSize(1);
  MAX7219_Clear(MAX7219_BLACK);
  MAX7219_Display();
  HAL_UART_Receive_IT(&huart2, &uart2RecBuf, 1);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of UartQueue */
  UartQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &UartQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MartixTask */
  MartixTaskHandle = osThreadNew(StartMartixTask, NULL, &MartixTask_attributes);

  /* creation of PotentionTask */
  PotentionTaskHandle = osThreadNew(StartPotentionTask, NULL, &PotentionTask_attributes);

  /* creation of SwitchTask */
  SwitchTaskHandle = osThreadNew(StartSwitchTask, NULL, &SwitchTask_attributes);

  /* creation of TempTask */
  TempTaskHandle = osThreadNew(StartTempTask, NULL, &TempTask_attributes);

  /* creation of UartTask */
  UartTaskHandle = osThreadNew(StartUartTask, NULL, &UartTask_attributes);

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
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RNG|RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10707DBC;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartMartixTask */
/**
* @brief Function implementing the MartixTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMartixTask */
void StartMartixTask(void *argument)
{
  /* USER CODE BEGIN StartMartixTask */
  /* Infinite loop */
	for(;;)
	  {
		if (flag == 0 || flag == 1){
			if (flag == 0){
				sprintf(Text, "%d#", sensor);
			}
			else if (flag == 1){
				sprintf(Text, "%d$", sensor);
			}
			MAX7219_Clear(MAX7219_BLACK);
			GFX_DrawString(0,0, Text, MAX7219_WHITE, MAX7219_BLACK);
			MAX7219_Display();
			osDelay(200);
		}
		else if(flag == 2){

			uint8_t w=sizeof(img);

			uint8_t h=8;
			uint8_t i, j, jj;


				MAX7219_Clear(MAX7219_BLACK);
				for(j = 0; j < h; j++)
				{
					for(i = 0; i < w; i++)
					{
						jj=(i)/8;
						if(img[(j+jj*7) + i /8] & (128 >> (i&7)) ){
							GFX_DrawPixel(i, j, MAX7219_WHITE);
						}
					}
				}
				MAX7219_Display();
				if(flag!=2)break;
				osDelay(100);

		}
		else if (flag == 3){
			uint8_t TextSize = strlen(str);
				for(int i = MAX7219_X_PIXELS; i > ((TextSize * (GFX_GetFontWidth() + 1)) * -1); i--){
				MAX7219_Clear(MAX7219_BLACK);
				GFX_DrawString(i,0, str, MAX7219_WHITE, MAX7219_BLACK);
				MAX7219_Display();
				if(flag!=3)break;
				osDelay(del);
			}
		}

	  }
  /* USER CODE END StartMartixTask */
}

/* USER CODE BEGIN Header_StartPotentionTask */
/**
* @brief Function implementing the PotentionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPotentionTask */
void StartPotentionTask(void *argument)
{
  /* USER CODE BEGIN StartPotentionTask */
  /* Infinite loop */

  for(;;)
  {
	int AD_RES = 0;

	  // Start ADC Conversion
	HAL_ADC_Start(&hadc1);
	         // Poll ADC1 Perihperal & TimeOut = 1mSec
	HAL_ADC_PollForConversion(&hadc1, 1);
	         // Read The ADC Conversion Result & Map It To PWM DutyCycle
	AD_RES = HAL_ADC_GetValue(&hadc1);

	if((AD_RES/500)!= intesivity ){
		intesivity = 8 - (AD_RES/500);

		MAX7219_SetIntensity(0,intesivity);
		MAX7219_SetIntensity(1,intesivity);
	}
    osDelay(100);
  }
  /* USER CODE END StartPotentionTask */
}

/* USER CODE BEGIN Header_StartSwitchTask */
/**
* @brief Function implementing the SwitchTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSwitchTask */
void StartSwitchTask(void *argument)
{
  /* USER CODE BEGIN StartSwitchTask */
  /* Infinite loop */
  for(;;)
  {

	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET){
		  osDelay(300);
		  if(flag==0){flag=1;}
		  else if (flag==1){flag=2;}
		  else if (flag==2){flag=3;}
		  else if (flag==3){flag=0;}

	  }
	  if(flag == 0){sensor=sensor11;}
	  else if (flag == 1){sensor=sensor12;}

    osDelay(1);
  }
  /* USER CODE END StartSwitchTask */
}

/* USER CODE BEGIN Header_StartTempTask */
/**
* @brief Function implementing the TempTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTempTask */
void StartTempTask(void *argument)
{
  /* USER CODE BEGIN StartTempTask */
  /* Infinite loop */
  for(;;)
  {

	  HAL_I2C_Master_Transmit(&hi2c3, AHT10_Adress,  &AHT10_TmpHum_Cmd, 1,100); // Send command (trigger measuremetns) + parameters
	  osDelay(100);

	  HAL_I2C_Master_Receive(&hi2c3, AHT10_Adress,  (uint8_t*)AHT10_RX_Data, 6,100); // Receive data: STATUS[1]:HIMIDITY[2.5]:TEMPERATURE[2.5]


	  AHT10_ADC_Raw = (((uint32_t)AHT10_RX_Data[3] & 15) << 16) | ((uint32_t)AHT10_RX_Data[4] << 8) | AHT10_RX_Data[5];
	  AHT10_Temperature = (float)(AHT10_ADC_Raw * 200.00 / 1048576.00) - 50.00;

	  AHT10_ADC_Raw = ((uint32_t)AHT10_RX_Data[1] << 12) | ((uint32_t)AHT10_RX_Data[2] << 4) | (AHT10_RX_Data[3] >> 4);
	  AHT10_Humidity = (float)(AHT10_ADC_Raw*100.00/1048576.00);

	  sensor11=(int)AHT10_Temperature;
	  sensor12=(int)AHT10_Humidity;
	  osDelay(2000);

  }
  /* USER CODE END StartTempTask */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the UartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument)
{
  /* USER CODE BEGIN StartUartTask */
		uint8_t accepted[] = "accepted\n" ;
		uint8_t error[] = "error\n" ;

		uint8_t uartCMD[255] ;
		uint16_t fromQueue ;
		uint8_t index = 0 ;
		uint8_t ii = 0;
		uint16_t sl = 0;
	  /* Infinite loop */
	  for(;;){

	    osDelay(1);
	    osMessageQueueGet(UartQueueHandle, &fromQueue, 0, osWaitForever);


	    uartCMD[index] = (uint8_t) fromQueue ; index = (index<255? index+1 : 0) ;

	    if (fromQueue == 0x0A) {
	    	index = 0 ;
	        ii = 1;

	        switch(uartCMD[0]) {

	        	case '0' :

	        		flag = 0;
	        		HAL_UART_Transmit_IT(&huart2, "Temperatura\n", sizeof("Temperatura\n"));
	        		break;

	        	case '1' :

	        		flag = 1;
	        		HAL_UART_Transmit_IT(&huart2, "Wilgotnosc\n", sizeof("Wilgotnosc\n"));
	        		break;

	        	case '2':

	        		flag = 2;
	        		HAL_UART_Transmit_IT(&huart2, "GFX\n", sizeof("GFX\n"));
	        		break;

	        	case '3':

	        		flag = 3;
	        		HAL_UART_Transmit_IT(&huart2, "Text\n", sizeof("Text\n"));
	        		break;

	        	case 's':


        				for(int j = 0 ; j < 4 ; j++){

        					if (uartCMD[j] <= 0x39){
        						uartCMD[j] = uartCMD[j] - 0x30;  // 1 - 9
        					}
        					else{
        						uartCMD[j] = uartCMD[j] - 0x37;  //preserve A - F
        					}

        				}
        				sl = uartCMD[1] << 8 | uartCMD[2] << 4 | uartCMD[3] ;

        				HAL_UART_Transmit_IT(&huart2, "Slide\n", sizeof("Slide\n"));
        				del = sl;
	        		break;

	        	case 'g':

	        		if (uartCMD[49] == '#'){

	        			for (int i = 1; i < 17; i++){
	        				for(int j = 0 ; j < 2 ; j++){

	        					if (uartCMD[ii+j] <= 0x39){
	        						uartCMD[ii+j] = uartCMD[ii+j] - 0x30;  //preserve: 1 - 9
	        					}
	        					else{
	        						uartCMD[ii+j] = uartCMD[ii+j] - 0x37;  //preserve A - F
	        					}

	        				}

	        				img[i-1] =  uartCMD[ii] << 4 | uartCMD[ii+1]  ;
	        				ii += 3;

	        			}

	        			HAL_UART_Transmit_IT(&huart2, accepted, sizeof(accepted));
	        			flag = 2;
	        		}
	        		else{
	        			HAL_UART_Transmit_IT(&huart2, error, sizeof(error));
	        		}

	        		break ;

	        	case 't':

	        		scan(uartCMD);
	        		HAL_UART_Transmit_IT(&huart2, accepted, sizeof(accepted));
	        		flag = 3;

	        		break ;
	        	}
	        }

	  }
  /* USER CODE END StartUartTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
  while (1)
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

