/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EEPROM_ADDR 0x50 << 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
uint8_t buffer_eeprom [8];
uint8_t flag_comm;
uint8_t test;
uint8_t count = 0;
uint8_t receive_data[6], receive_sig;
uint8_t receive_eeprom [5] = "SANG.";
uint8_t word1[7] = "Command";
uint8_t word2[4] = "Set";
uint8_t clear [10];
uint8_t buffer_UART;
uint8_t Buffer_Con [10] ;
uint8_t key_code [8] = { 0x52, 0x51, 0x50, 0x4F, 0x1A, 0x16, 0x07, 0x08 }; // up, down, left, right, 'A', 'S', 'D', 'E'
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void Convert_data (void);
uint8_t String_Compare (uint8_t * pointer1, uint8_t * pointer2, uint8_t mode);
void Clear_Buffer (void);
uint8_t Set_Compare (void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Write_Eeprom (void);
void Read_Eeprom (void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern USBD_HandleTypeDef hUsbDeviceFS;

typedef struct
{
	uint8_t MODIFIER;
	uint8_t RESERVED;
	uint8_t KEYCODE1;
	uint8_t KEYCODE2;
	uint8_t KEYCODE3;
	uint8_t KEYCODE4;
	uint8_t KEYCODE5;
	uint8_t KEYCODE6;
} 	keyboardHID;

keyboardHID keyboardhid = {0,0,0,0,0,0,0,0};
void Read_Eeprom (void)
{
	HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDR, 0x0000, 2, buffer_eeprom, 8, 100);
	for (uint8_t i = 0; i < 8; i++)
	{
		key_code [i] = buffer_eeprom [i];
	}
}
void Write_Eeprom (void)
{
	HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDR, 0x0000, 2, key_code, 8, 100);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_10)
	{
		flag_comm = 0;
		Write_Eeprom();
		HAL_UART_Transmit(&huart2, (uint8_t *) "Set Up OK!!!\r\n", 14, 100);
	}
}
uint8_t Set_Compare (void)
{
	uint8_t variable = 1;
	uint8_t flag = 0;
	uint8_t buffer[4]= {'\0', '\0', '\0', '\0'};
	uint8_t numberical = Buffer_Con[3];
	uint8_t code1 = Buffer_Con[4] - 0x30;
	uint8_t code2 = Buffer_Con[5] - 0x30;
	for (uint8_t i = 0; i < 3; i++)
	{
		buffer [i] = Buffer_Con [i];
	}
	if (count == 6)
	{
	variable = String_Compare(word2, buffer, 1);
	if(variable == 1)
	{
		if ( numberical >= '0' && numberical <= '7')
		{
			key_code [numberical - 0x30] = code1*10 + code2;
		}
		else
		{
			flag = 0x03; //error second char
		}
	}
	else
	{
		flag  = 0x02; //error first char
	}
	}
	else
	{
		flag = 0x01; //error syntax
	}
	return flag;

}
void Clear_Buffer (void)
{
	for(uint8_t i = 0; i < 10; i++)
	{
		Buffer_Con[i] = '\0';
	}
}
void Convert_data (void)
{
	keyboardhid.KEYCODE1 = 0x00;
	keyboardhid.KEYCODE2 = 0x00;
	keyboardhid.KEYCODE3 = 0x00;
	keyboardhid.KEYCODE4 = 0x00;
	keyboardhid.KEYCODE5 = 0x00;
	keyboardhid.KEYCODE6 = 0x00;

	if (receive_data [0] ==  0x60)				//UP ARROW
		keyboardhid.KEYCODE1 = key_code[0];
	if (receive_data [0] ==  0x5A)				//DOWN ARROW
		keyboardhid.KEYCODE1 = key_code[1];
	if (receive_data [1] ==  0x5C)				//LEFT ARROW
		keyboardhid.KEYCODE2 = key_code[2];
	if (receive_data [1] ==  0x5E)				//RIGHT ARROW
		keyboardhid.KEYCODE2 = key_code[3];
	if (receive_data [2] ==  0x05)				//TOP
		keyboardhid.KEYCODE3 = key_code[4];
	if (receive_data [3] ==  0x05)				//BOT
		keyboardhid.KEYCODE4 = key_code[5];
	if (receive_data [4] ==  0x05)				//LEFT
		keyboardhid.KEYCODE5 = key_code[6];
	if (receive_data [5] ==  0x05)				//RIGHT
		keyboardhid.KEYCODE6 = key_code[7];
}
uint8_t String_Compare (uint8_t * pointer1, uint8_t * pointer2, uint8_t mode)
{
	uint8_t flag = 1;
//	uint8_t i = 0;
	while (*(pointer1) != '\0' || *(pointer2) != '\0')
	{
		if(*(pointer1) != *(pointer2))
		{
			flag = 0;
			break;
		}
		pointer1 ++;
		pointer2 ++;
	}
	if(mode == 0)
	{
	Clear_Buffer();
	}
	return flag;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim -> Instance == htim1.Instance)
	{
		Convert_data();
		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t *) &keyboardhid, sizeof(keyboardhid));
	}
	if (htim -> Instance == htim2.Instance)
	{
		HAL_TIM_Base_Stop_IT(&htim2);
		if(flag_comm == 0)
		{
			if(String_Compare(word1, Buffer_Con, 0))
			{
			HAL_UART_Transmit(&huart2, (uint8_t *) "OK\r\n", 4, 100);
			flag_comm = 1;
			}
			else
			{
			HAL_UART_Transmit(&huart2, (uint8_t *) "Error\r\n", 7, 100);
			}
		}
		else
		{
			test = Set_Compare();
			Clear_Buffer();
			if (test == 0)
			{
				HAL_UART_Transmit(&huart2, (uint8_t *) "OK\r\n", 4, 100);
			}
			if (test == 0x01)
			{
				HAL_UART_Transmit(&huart2, (uint8_t *) "Extra Characters\r\n", 18, 100);
			}
			if (test == 0x02)
			{
				HAL_UART_Transmit(&huart2, (uint8_t *) "Error First Block\r\n", 19, 100);
			}
			if (test == 0x03)
			{
				HAL_UART_Transmit(&huart2, (uint8_t *) "Error Second Block\r\n", 20, 100);
			}
		}
		count = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart -> Instance == huart1.Instance)
	{
		static uint8_t flag = 0;
		static uint8_t  num = 0;
		if ( flag == 1)
		{
			receive_data [num] = receive_sig;
			num ++;
		}
		else
		{
			if (receive_sig == 0x53)  //"S" Character
				flag = 1;
		}
		if (num >= 6)
		{
			flag = 0;
			num = 0;
		}
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
	if(huart -> Instance == huart2.Instance)
	{
		if (count == 0)
		{
		__HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		HAL_TIM_Base_Start_IT(&htim2);
		}
		Buffer_Con [count] = buffer_UART;
		count ++;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart1, &receive_sig, 1);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_UART_Receive_DMA(&huart2, &buffer_UART, 1);
  Read_Eeprom();
 // test = String_Compare(word1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  HAL_UART_Receive(&huart2, receive_data, 2, HAL_MAX_DELAY);
//	  keyboardhid.KEYCODE1 = 0;
//	  keyboardhid.KEYCODE2 = 0;
	  keyboardhid.KEYCODE1 = receive_data[0];
	  keyboardhid.KEYCODE2 = receive_data [1];
//	  keyboardhid.KEYCODE1 = 0x4F;
//	  USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t *) &keyboardhid, sizeof(keyboardhid));
//	  HAL_UART_Transmit(&huart2, receive_eeprom, 2, 100);
	  HAL_Delay(50);
//	  keyboardhid.KEYCODE1 = 0;
//	  keyboardhid.KEYCODE2 = 0;
//	  USBD_HID_SendReport(&hUsbDeviceFS, receive_data, sizeof(keyboardhid));
//	  HAL_Delay(100);
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 6;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50000;
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
  htim2.Init.Prescaler = 12 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60000 - 1;
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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(I2C_WP_GPIO_Port, I2C_WP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : I2C_WP_Pin */
  GPIO_InitStruct.Pin = I2C_WP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I2C_WP_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
