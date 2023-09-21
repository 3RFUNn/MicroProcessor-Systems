/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include <stdio.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RS 0x20 /* PB5 mask for reg select */
#define RW 0x40 /* PB6 mask for read/write */
#define EN 0x80 /* PB7 mask for enable */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
enum state
{
	stopped,
	counting,
	paused,
	off
};
static enum state chronometer_state = stopped;

static int MM = 0, SS = 0, MMM = 0, first, second, is_falling = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void delayMs(int n);
void LCD_command(unsigned char command);
void LCD_data(char data);
void LCD_init(void);
void show_time(void);
void turn_off(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
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
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim5);
	LCD_init();
	LCD_data('W');
	LCD_data('e');
	LCD_data('l');
	LCD_data('c');
	LCD_data('o');
	LCD_data('m');
	LCD_data('e');
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 while (1)
	{
		if (chronometer_state == counting)
		{
			show_time();
			delayMs(75);
		}
		else if (chronometer_state == off)
		{
			break;
		}
	}
	while (1)
	{
		LCD_data('t');
		LCD_data('u');
		LCD_data('r');
		LCD_data('n');
		LCD_data(' ');
		LCD_data('o');
		LCD_data('f');
		LCD_data('f');
		delayMs(500);
		LCD_command(1);
		delayMs(500);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
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
  htim2.Init.Prescaler = 280;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 16000;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 20000;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void LCD_init(void)
{
	delayMs(30); /* initialization sequence */
	LCD_command(0x30);
	delayMs(10);
	LCD_command(0x30);
	delayMs(1);
	LCD_command(0x30);

	LCD_command(0x38); /* set 8-bit data, 2-line, 5x7 font */
	LCD_command(0x06); /* move cursor right after each char */
	LCD_command(0x01); /* clear screen, move cursor to home */
	LCD_command(0x0F); /* turn on display, cursor blinking */
}

void LCD_command(unsigned char command)
{
	//GPIOB->BSRR = (RS | RW) << 16; /* RS = 0, R/W = 0 */
	HAL_GPIO_WritePin(GPIOB, RS, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, RW, GPIO_PIN_RESET);
	//GPIOC->ODR &= ~0x00FF;		   /* clear data bus */
	//GPIOC->ODR |= command;		   /* put command on data bus */
	for(uint16_t GPIO_Pin = GPIO_PIN_0; GPIO_Pin != GPIO_PIN_8; GPIO_Pin = GPIO_Pin << 1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_Pin, command % 2);
		command = command >> 1;
	}
	HAL_GPIO_WritePin(GPIOB, EN, GPIO_PIN_SET);
	//GPIOB->BSRR = EN;			   /* pulse E high */
	delayMs(0);
	HAL_GPIO_WritePin(GPIOB, EN, GPIO_PIN_RESET);
	//GPIOB->BSRR = EN << 16; /* clear E */

	if (command < 4)
		delayMs(2); /* command 1 and 2 needs up to 1.64ms */
	else
		delayMs(1); /* all others 40 us */
}

void LCD_data(char data)
{
	//GPIOB->BSRR = RS;		/* RS = 1 */
	//GPIOB->BSRR = RW << 16; /* R/W = 0 */
	HAL_GPIO_WritePin(GPIOB, RS, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, RW, GPIO_PIN_RESET);
	//GPIOC->ODR &= ~0x00FF;	/* clear data bus */
	//GPIOC->ODR |= data;		/* put data on data bus */
	for(uint16_t GPIO_Pin = GPIO_PIN_0; GPIO_Pin != GPIO_PIN_8; GPIO_Pin = GPIO_Pin << 1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_Pin, data % 2);
		data = data >> 1;
	}
	//GPIOB->BSRR = EN;		/* pulse E high */
	HAL_GPIO_WritePin(GPIOB, EN, GPIO_PIN_SET);
	delayMs(0);
	//GPIOB->BSRR = EN << 16; /* clear E */
	HAL_GPIO_WritePin(GPIOB, EN, GPIO_PIN_RESET);
	delayMs(1);
}

void show_time(void)
{
	LCD_command(1);
	char print[16];
	sprintf(print, "%.2d:%.2d:%.3d", MM, SS, MMM);
	for (int i = 0; i < 16 && print[i] != '\0'; i++)
		LCD_data(print[i]);
}

/* delay n milliseconds (16 MHz CPU clock) */
void delayMs(int n)
{
	int i;
	for (; n > 0; n--)
		for (i = 0; i < 3195; i++)
			__NOP();
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case GPIO_PIN_8:
			chronometer_state = counting;
			HAL_TIM_Base_Start_IT(&htim2);
			break;
		case GPIO_PIN_9:
			chronometer_state = paused;
			HAL_TIM_Base_Stop_IT(&htim2);
			show_time();
			break;
		case GPIO_PIN_10:
			chronometer_state = stopped;
			HAL_TIM_Base_Stop_IT(&htim2);
			MMM = 0;
			SS = 0;
			MM = 0;
			show_time();
			if (is_falling == 0)
			{
				//first = TIM5->CNT;
				first = __HAL_TIM_GetCounter(&htim5);
				is_falling = 1;
			}
			else
			{
				is_falling = 0;
				//second = TIM5->CNT;
				second = __HAL_TIM_GetCounter(&htim5);
				if (second - first > 300)
				{
					//__disable_irq();
					HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
					HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
					HAL_TIM_Base_Stop_IT(&htim2);
					HAL_TIM_Base_Stop(&htim5);
					LCD_command(1);
					chronometer_state = off;
				}
			}
			break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (chronometer_state == counting)
	{
		MMM++;
		if (MMM > 999)
		{
			MMM = 0;
			SS++;
		}
		if (SS > 59)
		{
			SS = 0;
			MM++;
		}
		if (MM > 99)
		{
			MM = 0;
		}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
