/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
#include "ring_buffer.h"
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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint32_t left_toggles = 0;
uint32_t right_toggles = 0;
uint32_t hazard_toggles = 0;

volatile uint8_t left_flag = 0;
volatile uint8_t right_flag = 0;
volatile uint8_t hazard_flag = 0;

/* Variables para el Ring Buffer */
#define BUFFER_CAPACITY (20)

// Buffer de datos Uart1
uint8_t ring_buffer_1_data[BUFFER_CAPACITY];
uint8_t ring_buffer_2_data[BUFFER_CAPACITY];
RingBuffer ring_buffer_1;
static uint8_t data_1;

// Buffer de datos Uart2
uint8_t ring_buffer_2_data[BUFFER_CAPACITY];
RingBuffer ring_buffer_2;
static uint8_t data_2;

volatile uint8_t data_ready = 0;

// Mi ID
#define MY_ID "1087705844"
#define MY_NAME "Jonny Sinisterra"
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void turn_signal_left(void){
    static uint32_t turn_toggle_tick = 0;
    static uint8_t blink_count = 0;  // Variable para contar el número de parpadeos
    if (blink_count < 6) {  // Parpadea solo si no se han completado los tres parpadeos
        if (turn_toggle_tick < HAL_GetTick()){
            turn_toggle_tick =  HAL_GetTick() + 670;
            HAL_GPIO_TogglePin(D3_GPIO_Port,D3_Pin);
            blink_count++;  // Incrementa el contador de parpadeos
        }
    }
}


void turn_signal_right(void){
    static uint32_t turn_toggle_tick = 0;
    static uint8_t blink_count = 0;  // Variable para contar el número de parpadeos
    if (blink_count < 6) {  // Parpadea solo si no se han completado los tres parpadeos
        if (turn_toggle_tick < HAL_GetTick()){
            turn_toggle_tick =  HAL_GetTick() + 670;
            HAL_GPIO_TogglePin(D4_GPIO_Port,D4_Pin);
            blink_count++;  // Incrementa el contador de parpadeos
        }
    }
}


void turn_signal_hazard(void){
    static uint32_t turn_toggle_tick = 0;
    static uint8_t blink_count = 0;  // Variable para contar el número de parpadeos
    if (blink_count < 6) {  // Parpadea solo si no se han completado los tres parpadeos
        if (turn_toggle_tick < HAL_GetTick()){
            turn_toggle_tick =  HAL_GetTick() + 670;
            HAL_GPIO_TogglePin(D3_GPIO_Port,D3_Pin);
            HAL_GPIO_TogglePin(D4_GPIO_Port,D4_Pin);
            HAL_GPIO_TogglePin(D1_GPIO_Port,D1_Pin);
            blink_count++;  // Incrementa el contador de parpadeos
        }
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if(huart->Instance == USART1){
    ring_buffer_put(&ring_buffer_1, data_1);
    HAL_UART_Receive_IT(&huart1, &data_1, 1);
  } else

	if (huart->Instance == USART2) {
        ring_buffer_put(&ring_buffer_2, data_2);
        HAL_UART_Receive_IT(&huart2, &data_2, 1);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  //UNUSED(GPIO_Pin);
  if (GPIO_Pin==S1_Pin){// LEFT
	  HAL_UART_Transmit(&huart2, (uint8_t*)"LEFT\r\n", 6, 10);
	  left_flag = 1;
  } else if (GPIO_Pin==S3_Pin){//RIGHT
	  HAL_UART_Transmit(&huart2, (uint8_t*)"RIGHT\r\n", 7, 10);
	  right_flag = 1;
  } else if (GPIO_Pin==S2_Pin){//HAZARD
	  HAL_UART_Transmit(&huart2, (uint8_t*)"HAZARD\r\n",8, HAL_MAX_DELAY);
	  hazard_flag = 1;
  }

}

void hearbeat(void){
	static uint32_t heartbeat_tick =0;
	if (heartbeat_tick < HAL_GetTick()){
		heartbeat_tick =  HAL_GetTick()+1000;
		HAL_GPIO_TogglePin(D1_GPIO_Port,D1_Pin);
	}
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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  ring_buffer_init(&ring_buffer_2, ring_buffer_2_data, BUFFER_CAPACITY);
  HAL_UART_Receive_IT(&huart2, &data_2, 1);// Inicializar la recepción de datos por interrupción
  HAL_UART_Receive_IT(&huart1, &data_2, 1);// Inicializar la recepción de datos por interrupción

  while (1)
  {
    /* USER CODE END WHILE */
	  hearbeat();
	  if (ring_buffer_is_full(&ring_buffer_2) != 0) {
	      data_ready = ring_buffer_get(&ring_buffer_2, &data_2);
	          int8_t id_incorrect = 0;
	          char my_id[] = MY_ID;
	          for(uint8_t idx =0; idx < sizeof(my_id); idx++){
	            if(ring_buffer_get(&ring_buffer_2, &data_2) != 0){
	              if (data_2 != my_id[idx]){
	                  id_incorrect = 1;
	              }
	            }
	          }
	          if(id_incorrect == 0){
	              HAL_UART_Transmit(&huart2, (uint8_t*)MY_NAME, strlen(MY_NAME), HAL_MAX_DELAY);
	          } else {
	              HAL_UART_Transmit(&huart2, (uint8_t*)"ID incorrecto\r\n", 15, HAL_MAX_DELAY);
	          }
	      }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D1_Pin|D2_Pin|D3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : S1_Pin S2_Pin */
  GPIO_InitStruct.Pin = S1_Pin|S2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_Pin D2_Pin D3_Pin */
  GPIO_InitStruct.Pin = D1_Pin|D2_Pin|D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : S3_Pin */
  GPIO_InitStruct.Pin = S3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(S3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : D4_Pin */
  GPIO_InitStruct.Pin = D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(D4_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
