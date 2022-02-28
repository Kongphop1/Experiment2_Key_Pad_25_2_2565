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
uint16_t Buttonstate = 0; // store 4x4 button state
int numafterconvert = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

 // declare Function at here
void ButtonMatrixRead();
int PinConvertMapping(uint16_t);
void checkmynumber(int x);

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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

   // Function to read Button
   ButtonMatrixRead();
   PinConvertMapping(Buttonstate);
   // checkmynumber(int x);

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|L4_Pin|L1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : L4_Pin L1_Pin */
  GPIO_InitStruct.Pin = L4_Pin|L1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : L2_Pin */
  GPIO_InitStruct.Pin = L2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : R1_Pin */
  GPIO_InitStruct.Pin = R1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(R1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R2_Pin R4_Pin R3_Pin */
  GPIO_InitStruct.Pin = R2_Pin|R4_Pin|R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : L3_Pin */
  GPIO_InitStruct.Pin = L3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


int PinConvertMapping(uint16_t y){
	switch(y){
		case 1:
			numafterconvert = 7;
			break;
		case 2:
			numafterconvert = 8;
			break;
		case 4:
			numafterconvert = 9;
			break;
//		case 8:
//			numafterconvert = 7;
//			break;
		case 16:
			numafterconvert = 4;
			break;
		case 32:
			numafterconvert = 5;
			break;
		case 64:
			numafterconvert = 6;
			break;
//		case 128:
//			numafterconvert = 7;
//			break;
		case 256:
			numafterconvert = 1;
			break;
		case 512:
			numafterconvert = 2;
			break;
		case 1024:
			numafterconvert = 3;
			break;
//		case 2048:
//			numafterconvert = 7;
//			break;
		case 4096:
			numafterconvert = 0;
			break;
//		case 8192:
//			numafterconvert = 7;
//			break;
//		case 16384:
//			numafterconvert = 7;
//			break;
//		case 32768:
//			numafterconvert = 7;
//			break;
	}
	return numafterconvert;
}


// �?ระ�?าศต�?ัว�?�?รอย�?า�?�?ี�?เ�?ราะเอามา�?า�? GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)

// ตัวเ�?�?�? Array �?ั�?�? R �?�?�?�?ุม port �?ั�? pin
GPIO_TypeDef *ButtonMatrixPortR[4] = {R1_GPIO_Port, R2_GPIO_Port, R3_GPIO_Port, R4_GPIO_Port};
uint16_t ButtonMatrixPinR[4] = {R1_Pin, R2_Pin, R3_Pin, R4_Pin };

// ตัวเ�?�?�? Array �?ั�?�? L �?�?�?�?ุม port �?ั�? pin
GPIO_TypeDef *ButtonMatrixPortL[4] = {L1_GPIO_Port, L2_GPIO_Port, L3_GPIO_Port, L4_GPIO_Port}; // เ�?�?�?�?�?อมูล GPIO ที�?�?ำลั�?�?ะ�?�?�?�?า�?�?ว�? port A,B,C
uint16_t ButtonMatrixPinL[4] = {L1_Pin, L2_Pin, L3_Pin, L4_Pin }; // เ�?�?�?�?�?อมูล GPIO ที�?�?ำลั�?�?ะ�?�?�?�?า�?�?ว�? pin มั�?เ�?�?�?ตัวเล�? เ�?�?�? 1 2 3

// Read Button state 4x4 Button

void ButtonMatrixRead()
{
	// �?ห�? function ถู�?เรีย�?ทุ�? 100 ms
	static uint32_t timeStamp = 0;
	static uint8_t CurrentL = 0; // เวลาที�? L run อยู�?รั�?ษา�?�?าเอา�?ว�?�?�?�?ั�?�?ุ�?ั�?

	if (HAL_GetTick() - timeStamp >= 100){
		timeStamp = HAL_GetTick();

		for(int i = 0; i<4;i++){
			if(HAL_GPIO_ReadPin(ButtonMatrixPortR[i], ButtonMatrixPinR[i])== GPIO_PIN_RESET){ // Press Button
				// set bit i to 1
				// i calculate form i(R) and CurrentL to set bit that relate to 4x4 button
				Buttonstate |= 1 << (i + (CurrentL*4));
				// ย�?ตัวอย�?า�?�?ุ�?มถู�?�?ด ถ�?า i = 3, currentL = 2 �?ะ�?ด�?ว�?าเรา�?ด�?ุ�?ม K12 อยู�? �?ะ�?ด�?อี�?ว�?า bit ที�? 11 ถู�?เ�?ลี�?ย�?�?ห�?เ�?�?�? 1
			}
			else{
				// set bit i to 0
				Buttonstate &= ~(1 << (i + (CurrentL*4)));
			}
		}
		// set currentL to Hi-z (Open drain) �?ดยเริ�?มต�?�?�?ล�?วทุ�?�?ถว�?ะเ�?�?�? HI �?ล�?ว�?ถวที�?�?ะหา�?ะเ�?�?�? LOW ว�?�?�?เรื�?อย�?
		HAL_GPIO_WritePin(ButtonMatrixPortL[CurrentL], ButtonMatrixPinL[CurrentL], GPIO_PIN_SET); // เ�?�?�?�?าร set �?ห�?�?า L เ�?�?�? High �?ว�?�?ดย�?ะ set ทั�?�? column �?อ�?�?ถวตัวเอ�?
		uint8_t nextL = (CurrentL + 1) %4 ; // เ�?�?�?�?ารเ�?ลี�?ย�?เ�?�?�? column  ต�?อ�?�?เ�?�?�?�?ถว�?หม�? (�?าร mod = �?ารหารเอาเศษ = เ�?�?�?�?าร�?ั�? overflow เ�?ื�?อ�?ห�? reset �?�?เริ�?ม�?ั�?�?�?า�?ถว(row)ที�? 1-4)
		// set NextL to Low
		HAL_GPIO_WritePin(ButtonMatrixPortL[nextL], ButtonMatrixPinL[nextL], GPIO_PIN_RESET); // เ�?�?�?�?าร set �?ห�?�?า L เ�?�?�? Low �?ว�?�?ดย�?ะ set ทั�?�? column �?อ�?�?ถว�?หม�?
		CurrentL = nextL; // หลั�?�?า�? set เสร�?�?�?ล�?ว�?ห�?เอา �?ถว ที�?ถู�? set �?ว�?�?ล�?วมาเ�?ลี�?ย�? เ�?�?�?ตำ�?ห�?�?�?�?ถว�?หม�?�?ห�?ตัวเอ�?

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

