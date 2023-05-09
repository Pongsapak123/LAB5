/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t RxBuffer[20];
uint8_t TxBuffer[255];
uint8_t input;
uint8_t prev_input;
uint8_t input_state;
uint8_t input_state_prev;

uint8_t led_state = 1;

uint8_t STATE = 0;

int8_t hz = 1;
int8_t hz_prev = 1;
typedef enum {
  STATE_MAIN_MENU,
  STATE_SUBMENU_1,
  STATE_SUBMENU_2,
} state_t;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void UARTPollingMethod();
void LedTask();
void UARTInterruptConfig();
void UARTDMAconfig();
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

//  uint8_t TEXT[] = "HELLO FIBO"; // 10+1 have \0
//  HAL_UART_Transmit(&huart2, TEXT, 11, 10);
  sprintf((char*)TxBuffer,"\n\n\n\n\n\n\n\r==========Main Menu==========\r\n");
  HAL_UART_Transmit_IT(&huart2, TxBuffer, strlen((char*)TxBuffer));

  sprintf((char*)TxBuffer,"Press 0 : Led Control\r\n");
  HAL_UART_Transmit_IT(&huart2, TxBuffer, strlen((char*)TxBuffer));

  sprintf((char*)TxBuffer,"Press 1 : ButtonStatus\r\n");
    HAL_UART_Transmit_IT(&huart2, TxBuffer, strlen((char*)TxBuffer));
  UARTInterruptConfig();

//  state_t state = STATE_MAIN_MENU;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(input != prev_input ){
		  switch (STATE) {
			default:
			case 0:
				if(RxBuffer[0] == 48){ //0
					sprintf((char*)TxBuffer,"\n\n\n\n\n\n\r\n==========Menu 0==========\r\n%d",hz);
					HAL_UART_Transmit_IT(&huart2, TxBuffer, strlen((char*)TxBuffer));
					STATE = 1;
				}else if (RxBuffer[0] == 49) { //1
					sprintf((char*)TxBuffer,"\n\n\n\n\n\n\r\n==========Menu 1==========\r\n%d",HAL_GPIO_ReadPin(LD2_GPIO_Port, LD2_Pin));
					HAL_UART_Transmit_IT(&huart2, TxBuffer, strlen((char*)TxBuffer));
					STATE = 2;
				}else{
					sprintf((char*)TxBuffer,"\n\n\n\n\n\n\r\n Error try again\r\n");
					HAL_UART_Transmit_IT(&huart2, TxBuffer, strlen((char*)TxBuffer));
				}
			break;

			case 1:
				if(RxBuffer[0] == 97){ //a
					hz = hz + 1;
					sprintf((char*)TxBuffer,"\n\n\n\n\n\n\r\n==========Menu 0==========\r\n%d",hz);
					HAL_UART_Transmit_IT(&huart2, TxBuffer, strlen((char*)TxBuffer));
				}else if (RxBuffer[0] == 115) { // s
					hz = hz - 1;
					if(hz<=0){hz=0;}
					sprintf((char*)TxBuffer,"\n\n\n\n\n\n\r\n==========Menu 0==========\r\n%d",hz);
					HAL_UART_Transmit_IT(&huart2, TxBuffer, strlen((char*)TxBuffer));
				}else if (RxBuffer[0] == 100) { // d
					if(led_state==1){led_state=0;}
					else if (led_state==0) {led_state=1;}

					if(led_state == 0){
						hz_prev = hz;
						hz = 0;
					}else if(led_state == 1){
						hz = hz_prev;
					}
//					HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, RESET);
				}else if (RxBuffer[0] == 120) { // x
					sprintf((char*)TxBuffer,"\n\n\n\n\n\n\r\n==========Choose 0/1==========\r\n");
					HAL_UART_Transmit_IT(&huart2, TxBuffer, strlen((char*)TxBuffer));
					STATE = 0;
				}else{
					sprintf((char*)TxBuffer,"\n\n\n\n\n\n\r\n Error try again\r\n");
					HAL_UART_Transmit_IT(&huart2, TxBuffer, strlen((char*)TxBuffer));
				}
			break;

			case 2:
				if(RxBuffer[0] == 120){ // x
					sprintf((char*)TxBuffer,"\n\n\n\n\n\n\r\n==========Choose 0/1==========\r\n");
					HAL_UART_Transmit_IT(&huart2, TxBuffer, strlen((char*)TxBuffer));
					STATE = 0;
				}else if(HAL_GPIO_ReadPin(LD2_GPIO_Port, LD2_Pin)==0){
					sprintf((char*)TxBuffer,"\n\n\n\n\n\n\r\n==========Menu 1==========\r\n%d",HAL_GPIO_ReadPin(LD2_GPIO_Port, LD2_Pin));
					HAL_UART_Transmit_IT(&huart2, TxBuffer, strlen((char*)TxBuffer));
				}
				else{
					sprintf((char*)TxBuffer,"\n\n\n\n\n\n\r\n Error try again\r\n");
					HAL_UART_Transmit_IT(&huart2, TxBuffer, strlen((char*)TxBuffer));
				}
			break;
			  }
	  }
	  input = '\0';
	  prev_input = input;
	  LedTask();
  /* USER CODE END 3 */
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 57600;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void LedTask(){
	static uint32_t timestamp = 0;
	if(HAL_GetTick()>=timestamp){
		timestamp = HAL_GetTick()+(1000/hz);
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
}

void UARTInterruptConfig(){
	HAL_UART_Receive_IT(&huart2, RxBuffer, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	if(huart == &huart2){

		RxBuffer[1] = '\0';
		input = RxBuffer[0];
//		sprintf((char*)TxBuffer,"Received : %s\r\n",RxBuffer);

	  HAL_UART_Transmit_IT(&huart2, TxBuffer, strlen((char*)TxBuffer));


	  HAL_UART_Receive_IT(&huart2, RxBuffer, 1);
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
