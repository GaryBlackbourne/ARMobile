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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// forward and backward moving speed
#define FWD_SPEED 70
#define BWD_SPEED 70
// speed for the outer wheels when turning
#define TURN_SPEED 50
// backwards speed for the inner wheels when turning
#define ANTITURN_SPEED 10
// time to wait before processing next keypress
#define DELAY_TIME 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

typedef enum DirectionTypeDef{
	FORWARD,
	BACKWARD
}DirectionTypeDef;

volatile uint8_t data = 0;
volatile uint8_t ctrl_msg = 0;

uint8_t left_presses = 0, right_presses = 0, fwd_presses = 0, bwd_presses = 0;

const uint8_t err_bluetooth[] = "Illegal Bluetooth character\r\n";
const uint8_t msg_init[] = "Initialized, version " __DATE__ " " __TIME__ "\r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

static void Stop(void);

inline static void Set_Speed_Right(uint8_t);		// 0 <= param < 100
inline static void Set_Speed_Left(uint8_t);			// 0 <= param < 100


static void Set_Dir_Right(DirectionTypeDef);
static void Set_Dir_Left(DirectionTypeDef);

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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Transmit(&huart2, msg_init, sizeof(msg_init), 100);

  // comm:
  HAL_UART_Receive_IT(&huart3, &ctrl_msg, 1);
  HAL_UART_Receive_IT(&huart2, &data, 1);

  // PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  // Init movement
  Stop();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  while(left_presses>0){
//		  Set_Dir_Left(FORWARD);
//		  Set_Dir_Right(BACKWARD);
//		  Set_Speed_Left(TURN_SPEED);
//		  Set_Speed_Right(ANTITURN_SPEED);
//		  left_presses--;
//		  HAL_Delay(DELAY_TIME);
//	  }
//	  while(right_presses>0){
//		  Set_Dir_Left(BACKWARD);
//		  Set_Dir_Right(FORWARD);
//		  Set_Speed_Left(ANTITURN_SPEED);
//		  Set_Speed_Right(TURN_SPEED);
//		  right_presses--;
//		  HAL_Delay(DELAY_TIME);
//	  }
//	  while(fwd_presses>0){
//		  Set_Dir_Left(FORWARD);
//		  Set_Dir_Right(FORWARD);
//		  Set_Speed_Left(FWD_SPEED);
//		  Set_Speed_Right(FWD_SPEED);
//		  fwd_presses--;
//		  HAL_Delay(DELAY_TIME);
//	  }
//	  while(bwd_presses>0){
//		  Set_Dir_Left(BACKWARD);
//		  Set_Dir_Right(BACKWARD);
//		  Set_Speed_Left(BWD_SPEED);
//		  Set_Speed_Right(BWD_SPEED);
//		  bwd_presses--;
//		  HAL_Delay(DELAY_TIME);
//	  }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_TIM2|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, dir_right_2_Pin|dir_right_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(dir_left_1_GPIO_Port, dir_left_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(dir_left_2_GPIO_Port, dir_left_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : dir_right_2_Pin dir_right_1_Pin */
  GPIO_InitStruct.Pin = dir_right_2_Pin|dir_right_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : dir_left_1_Pin */
  GPIO_InitStruct.Pin = dir_left_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(dir_left_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : dir_left_2_Pin */
  GPIO_InitStruct.Pin = dir_left_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(dir_left_2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart){
	if(huart->Instance == huart3.Instance){
		// bluetooth message
		HAL_UART_Transmit(&huart2, &ctrl_msg, 1, 100);

		switch(ctrl_msg){

		case 'l':
			// left_presses++;
			Set_Dir_Right(BACKWARD);
			Set_Dir_Left(FORWARD);
			Set_Speed_Left(30);
			Set_Speed_Right(30);
			break;
		case 'r':
			// right_presses++;
			Set_Dir_Right(FORWARD);
			Set_Dir_Left(BACKWARD);
			Set_Speed_Left(30);
			Set_Speed_Right(30);
			break;
		case 'f':
			// fwd_presses++;
			Set_Dir_Right(FORWARD);
			Set_Dir_Left(FORWARD);
			Set_Speed_Left(30);
			Set_Speed_Right(30);
			break;
		case 'b':
			// bwd_presses++;
			Set_Dir_Right(BACKWARD);
			Set_Dir_Left(BACKWARD);
			Set_Speed_Left(30);
			Set_Speed_Right(30);
			break;
		case 's':
			Stop();
			break;
		default:
			//HAL_UART_Transmit(&huart2, err_bluetooth, sizeof(err_bluetooth), 100);
			break;
		}

		HAL_UART_Receive_IT(&huart3, &ctrl_msg, 1);

	}else if(huart->Instance == huart2.Instance){
		// serial terminal message
		HAL_UART_Transmit(&huart2, &data, 1, 100);
		switch(data){

		case 'e':
			Set_Dir_Right(FORWARD);
			break;
		case 'd':
			Set_Dir_Right(BACKWARD);
			break;
		case 'q':
			Set_Dir_Left(FORWARD);
			break;
		case 'a':
			Set_Dir_Left(BACKWARD);
			break;
		case 'r':
			Set_Speed_Left(70);
			break;
		case 'f':
			Set_Speed_Left(30);
			break;
		case 't':
			Set_Speed_Right(70);
			break;
		case 'g':
			Set_Speed_Right(30);
			break;
		case 's':
			Stop();
			break;
		default:
			break;
		}
		HAL_UART_Receive_IT(&huart2, &data, 1);
	}
	return;
}

void Stop(void){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);

	Set_Speed_Left(0);
	Set_Speed_Right(0);

	return;
}

void Set_Speed_Right(uint8_t param){
	TIM3->CCR1 = param;
	return;
}

void Set_Speed_Left(uint8_t param){
	TIM2->CCR3 = param;
	return;
}

static void Set_Dir_Right(DirectionTypeDef dir){
	switch (dir){
	case FORWARD:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
		break;
	case BACKWARD:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
		break;
	default:
		break;
	}
	return;
}

static void Set_Dir_Left(DirectionTypeDef dir){
	switch (dir){
		case FORWARD:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
			break;
		case BACKWARD:
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
			break;
		default:
			break;
		}
	return;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
