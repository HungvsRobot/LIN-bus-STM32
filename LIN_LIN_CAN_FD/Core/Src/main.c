/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

uint8_t ID_LIN = 0x34;
uint8_t ID_pid = 0;
uint8_t buff_data[8];
uint8_t leng_buff_data = 0; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t pid_Calc(uint8_t ID){
  if(ID >= 0x3F) return 0xFF;

  uint8_t ID_bit[6]; 
  for(uint8_t i = 0; i < 6; i++){
    ID_bit[i] = (ID >> i) & 0x01; 
  }

  uint8_t P0 = (ID_bit[0] ^  ID_bit[1] ^  ID_bit[2] ^  ID_bit[4]) & 0x01;
  uint8_t P1 = (~(ID_bit[1] ^  ID_bit[3] ^  ID_bit[4] ^  ID_bit[5])) & 0x01;

  ID = ID | (P0 << 6)  | (P1 << 7); 

  return ID;  
}

uint8_t ID_Decode(uint8_t ID_pid){
    return ID_pid &= 0x3F;
}

uint8_t CheckSum_Calc(uint8_t PID, uint8_t *data, uint8_t size){
  uint16_t Sum = PID; 

  for(uint8_t i = 0; i < size; i++){
    Sum += data[i];
    if(Sum > 0xFF ){
      Sum = (Sum & 0xFF) + 1; 
    }
  }
  Sum = 0xFF - Sum; 
  return Sum; 
}


void LIN_Master_Request(UART_HandleTypeDef *huart_LIN, uint8_t ID, uint8_t *buff, uint8_t size){
  uint8_t Buff_tx[size + 2];  Buff_tx[0] = 0x55;
  Buff_tx[1] = pid_Calc(ID); 
  ID_pid = Buff_tx[1];
  for(uint8_t i = 2; i < 2+ size; i++){
    Buff_tx[i] = buff[i-2]; 
  }
  Buff_tx[size+2] = CheckSum_Calc(ID_pid, buff, size); 
  int a = 3;
  if(size == 0){a = 2;}
  HAL_LIN_SendBreak(huart_LIN);
  HAL_UART_Transmit(huart_LIN, Buff_tx, size + a, 10);
  
}

//=================== RX mode ======================//
uint8_t RxData[20];
uint8_t RxData_rec[8]; 
uint8_t Sts_LIN = 0; 
uint8_t Size_Rx = 8; 

uint8_t size_respond = 8;
uint8_t buff_feedback[8];

uint8_t flag_respond = 0;

uint8_t CheckSum_Rx = 0;


uint8_t LIN_Slave_Check_Request(uint8_t ID, uint8_t size){
	if(RxData[0] == 0x00 && RxData[1] == 0x55 && RxData[2] == pid_Calc(ID_LIN)){
		for(uint8_t i = 0; i< size; i++){
			RxData_rec[i] = RxData[i + 3];
			RxData[i+3] = 0;
		} 
		RxData[1] = 0;
		RxData[2] = 0;
		if(RxData[size + 3] == CheckSum_Calc(pid_Calc(ID), RxData_rec, size)){
		  return 1;
		}else{
		  return 2; //Trạng thái đúng nhưng check sum sai
		}
	}else{
    return 0; // data lỗi. 
  }
}

uint8_t  LIN_Slave_Respond(UART_HandleTypeDef *huart_LIN, uint8_t ID, uint8_t size, uint8_t *buff_feedback){
	uint8_t buff_respond[size + 1];
    uint8_t Sts_LIN_in = LIN_Slave_Check_Request(ID_LIN, Size_Rx);

	if(Sts_LIN_in == 2){
		for(uint8_t i = 0; i < size ; i++){
			buff_respond[i] = buff_feedback[i];
		}
		buff_respond[size] = CheckSum_Calc(pid_Calc(ID), buff_feedback, size);
		HAL_UART_Transmit(huart_LIN, buff_respond, size + 1 , 10);
		return 2;
	}else{
		return Sts_LIN_in;
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
  // Hung

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //HAL_LIN_Init(&huart1, 13);

  HAL_UARTEx_ReceiveToIdle_IT(&huart2, RxData, 20);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    LIN_Master_Request(&huart1, ID_LIN, buff_data, leng_buff_data);
    HAL_Delay(200);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_LIN_Init(&huart1, UART_LINBREAKDETECTLENGTH_10B) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_LIN_Init(&huart2, UART_LINBREAKDETECTLENGTH_11B) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


//  Lin Slave mode // Config // use LIN protocol
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  HAL_UARTEx_ReceiveToIdle_IT(&huart2, RxData, 20);


  Sts_LIN = LIN_Slave_Respond(&huart2,ID_LIN, size_respond, buff_feedback);

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
