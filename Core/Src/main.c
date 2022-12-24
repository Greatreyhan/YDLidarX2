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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <cmath>
#include <string.h>
#include <stdio.h>
#include "lcd16x2_i2c.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define dataLong 360

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/************************************* Variables ******************************/

uint8_t ReceiverBuffer[dataLong] ;
uint8_t logData[dataLong*3];
uint8_t pointLog = 0;

// Power Up Scan
uint8_t modelLidar;
uint8_t firmwareVersionMinor;
uint8_t firmwareVersionMajor;
uint8_t hardwareVersion;
uint8_t serialNumber[16];
uint8_t dataLengthPower;
uint8_t modeScanPower;
uint8_t typeCodePower;
uint8_t contentPower[20];
uint8_t flagPower = 0;

// Sampling & Ranging
int dataLengthScan;
uint8_t modeScan;
uint8_t typeCodeScan;
uint8_t contentScan[60];
uint8_t scanLog[120];
uint8_t startingPoint[1];
uint8_t flagScan = 0;

// Data Protocol
uint8_t PH[2];
uint8_t CT[1];
uint8_t LSN[1];
uint8_t FSA[2];
uint8_t LSA[2];
uint8_t CS[2];
uint8_t SI[2];
uint8_t logDataProtocol[80];
float distanceBuf;
float angleStart;
float angleEnd;
float angleAtIPosition;
float angleCorrectionAtIPosition;
float trueAngle;

// Main Data
float distance[360];
float angle[360];
unsigned int distacePosition = 0;
int flagDataSimilar = 0;

// Normalisasi Data
float angleToDistance[360];
short flagNorm = 0;

/************************************* Call Back ******************************/

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
}

/************************************* Receiving Data **************************/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Get Data
//	for(int i = 0; i < dataLong; i++){
//		logData[i+(pointLog*dataLong)] = ReceiverBuffer[i];
//	}
//	pointLog++;
//	
	// Scan for Power Information
	for(int i = 0; i < dataLong; i++){
		if(flagPower == 0){
			if(ReceiverBuffer[i] == 0xA5){
				if(ReceiverBuffer[i+1] == 0x5A){
					
					dataLengthPower = ((ReceiverBuffer[i+5]) << 3 | (ReceiverBuffer[i+4]) << 2 | (ReceiverBuffer[i+3]) << 1 | (ReceiverBuffer[i+2])); 
					
					// Mode Scan
					if(ReceiverBuffer[i+5] == 0x00){
						modeScanPower = 0x00;
					}
					else{
						modeScanPower = 0x01;
					}
					
					// Type Code
					typeCodePower = ReceiverBuffer[i+6];
					
					// Content
					modelLidar = ReceiverBuffer[i+7];
					firmwareVersionMajor = ReceiverBuffer[i+9];
					firmwareVersionMinor = ReceiverBuffer[i+8];
					hardwareVersion = ReceiverBuffer[i+10];
					
					// Flag On
					flagPower = 1;
				}
			}
		}
	}
	
	
	// Scan for sampling
	for(int i = 0; i < dataLong; i++){
		if(flagPower == 1){
			if(ReceiverBuffer[i] == 0xA5){
				if(ReceiverBuffer[i+1] == 0x5A){
					if(ReceiverBuffer[i+2] == 0x05){
						 
						dataLengthScan = (ReceiverBuffer[i+5] << 24)|(ReceiverBuffer[i+4] << 16) | (ReceiverBuffer[i+3] << 8) | (ReceiverBuffer[i+2]);
						
						// Mode Scan
						if(ReceiverBuffer[i+5] == 0x00){
							modeScan = 0x00;
						}
						else{
							modeScan = 0x01;
						}
						
						// Type Code
						typeCodeScan = ReceiverBuffer[i+6];
						
						// Flag Scan ON
						flagScan = 1;			
					}
				}
			}
		}
	}
	
	// Scan for continuous data
	if(flagScan == 1){
		for(int k = 0; k < dataLong; k++){
		logDataProtocol[k] = ReceiverBuffer[k];
		if(ReceiverBuffer[k] == 0xAA){
			if(ReceiverBuffer[k+1] == 0x55){
				
				PH[0] = ReceiverBuffer[k];
				PH[1] = ReceiverBuffer[k+1];
				CT[0] = ReceiverBuffer[k+2];
				LSN[0] = ReceiverBuffer[k+3];
				FSA[0] = ReceiverBuffer[k+4];
				FSA[1] = ReceiverBuffer[k+5];
				LSA[0] = ReceiverBuffer[k+6];
				LSA[1] = ReceiverBuffer[k+7];
				CS[0] = ReceiverBuffer[k+8];
				CS[1] = ReceiverBuffer[k+9];
									
				// Calculate Starting Angle
				angleStart = (FSA[0]| FSA[1] << 8) >> 1 ;
				angleStart = (float)angleStart/64;
									
				// Calculate Ending Angle
				angleEnd = (LSA[0] | LSA[1] << 8) >> 1;
				angleEnd = (float)angleEnd/64;
									
				
				// Combine All Data Angle and Distance
					int z = 0;
					int y = 0;
					for(int x = 0; x < ((int)LSN[0])*2; x++){
						int flag = 0;
						// filter for Distance to only take data every odd number
						if( x%2 == 0){
							SI[0] = ReceiverBuffer[k+10+x];
							SI[1] = ReceiverBuffer[k+11+x];
							distanceBuf = SI[0] | SI[1] << 8;
							distanceBuf = (float)distanceBuf/4;
							distance[z] = distanceBuf;
							z++;
							flag = 1;
						}
						
						if(flag == 1){
							if(distanceBuf > 0.1){
								
								// Reading Next Angle
								float angleAtIPositionB = (float)(((angleEnd - angleStart)/(((int)LSN[0])-1))*(x-1))+angleStart;
								float angleCorrectionAtIPositionB = atan(21.8*(155.3-distanceBuf)/(155.3*distanceBuf));
								angleAtIPositionB = angleAtIPositionB + angleCorrectionAtIPositionB;
								
								// Reading Next Distance
								uint8_t SIB[2];
								SIB[0] = ReceiverBuffer[k+10+x];
								SIB[1] = ReceiverBuffer[k+11+x];
								float distanceBufB = SIB[0] | SIB[1] << 8;
								distanceBufB = (float)distanceBufB/4;
								
								// Reading Flag Similar to Continue if reading the same point
								if(flagDataSimilar == 1){
									flagDataSimilar = 0;
									continue;
								}	
								else if((int)angleAtIPosition == (int)angleAtIPositionB){
									int bufDistance = (distanceBuf + distanceBufB)/2;
									angleToDistance[(int)angleAtIPosition] = bufDistance;
									flagDataSimilar = 1;
								}
								else{
									angleToDistance[(int)angleAtIPosition] = distanceBuf;
								}
								
								
								
							}
							flag = 0;
						}
						
						
						
						// filter for Angle to only take data along LSN size
						if( x < ((int)LSN[0])){
							angleAtIPosition = (float)(((angleEnd - angleStart)/(((int)LSN[0])-1))*(x-1))+angleStart;
							angleCorrectionAtIPosition = atan(21.8*(155.3-distanceBuf)/(155.3*distanceBuf));
							angleAtIPosition = angleAtIPosition + angleCorrectionAtIPosition;
							angle[y] = angleAtIPosition;
							y++;
						}
						
						
					}
					
				}
			}
		}
	}
	
	
	HAL_UART_Receive_DMA(&huart2, ReceiverBuffer, dataLong);

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
	/********************************** Start LCD Function ****************************/
	
	if(lcd16x2_i2c_init(&hi2c1)){
		lcd16x2_i2c_clear();
	}
	
	/********************************** End LCD Function ******************************/
	
	/********************************** Start PWM Generator ***************************/
	
	TIM1->CCR1 = (100*166667/100);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	
	/********************************** End PWM Generator *****************************/
	
	/********************************** Start UART Receiver ***************************/
	
	HAL_UART_Receive_DMA(&huart2, ReceiverBuffer, dataLong);
	
	/********************************** End UART Receiver *****************************/
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
//		lcd16x2_i2c_clear();
//		lcd16x2_i2c_setCursor(0,0);
//		lcd16x2_i2c_printf("Dist : %f",distance[0]);
//		lcd16x2_i2c_setCursor(1,0);
//		lcd16x2_i2c_printf("Angle : %f", angle[0]);
//		HAL_Delay(500);
//		flagNorm = 0;
//		for(int i = 0; i < 40 ; i++){
//							
//			// Normalisasi Data
//			if(distance[i] > 0.1 ){
//				
////				char bufTransmit[34];
//				int bufAngleA = (int) angle[i];
//				int bufAngleB = (int) angle[i+1];
//				
//				if(flagNorm == 1){
//					flagNorm = 0;
//					continue;
//				}
//				else if(bufAngleA == bufAngleB){
//					int bufDistance = (distance[i] + distance[i+1])/2;
//					angleToDistance[bufAngleA] = bufDistance;
//					flagNorm = 1;
//				}
//				else{
//					angleToDistance[bufAngleA] = distance[i];
//				}
//					
//				sprintf(bufTransmit, "D%d: %.2f || A%d: %d \r\n", i,distance[i], i, bufAngleA);
//				HAL_UART_Transmit(&huart3, (uint8_t*)bufTransmit, sizeof(bufTransmit), HAL_MAX_DELAY);
//			}
//			
//		}
		
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 168-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
