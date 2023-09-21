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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define KPA GPIO_PIN_8
#define KPB GPIO_PIN_9
#define KPC GPIO_PIN_10
#define KPD GPIO_PIN_11
#define KP1 GPIO_PIN_12
#define KP2 GPIO_PIN_13
#define KP3 GPIO_PIN_14

#define KP_GPIO GPIOC

#define RS GPIO_PIN_5
#define RW GPIO_PIN_6
#define EN GPIO_PIN_7

#define LCD_GPIO GPIOC

#define LED_Red GPIO_PIN_0
#define LED_Green GPIO_PIN_1
#define LED_Blue GPIO_PIN_2
#define LED_Yellow GPIO_PIN_3

#define LED_GPIO GPIOA

#define Button_Red GPIO_PIN_12
#define Button_Green GPIO_PIN_13
#define Button_Blue GPIO_PIN_14
#define Button_Yellow GPIO_PIN_15

#define Button_GPIO GPIOB	

#define SS1 GPIO_PIN_8
#define SS2 GPIO_PIN_9
#define SS3 GPIO_PIN_10
#define SS4 GPIO_PIN_11
#define SS5 GPIO_PIN_12
#define SS6 GPIO_PIN_13
#define SS7 GPIO_PIN_14	

#define SS_GPIO GPIOA	


#define GameTime 5



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
uint8_t clear_counter = 0, capture_counter = 0, seconds = 0, wrong_count = 0, match_studentID = 0;
uint16_t delay_value = 3000;

char studentID1[8] = {'9', '8', '2', '4', '3', '0', '4', '5'};
char studentID2[8] = {'9', '8', '2', '4', '3', '0', '2', '7'};
char studentName1[7] = {'F', 'o', 'u', 'l', 'a', 'd', 'i'};
char studentName2[6] = {'R', 'a', 'f', 'e', 'i', 'e'};

enum LED_states
{
	Red,
	Green,
	Blue,
	Yellow,
	None
};
enum LED_states LED_state = None;

enum states
{
	get_studentID,
	blink_name,
	pushed_once,
	game,
	win,
	lose
};
enum states state = get_studentID;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
char get_char(void);

void delayMs(int n);

void LCD_command(unsigned char command);
void LCD_data(char data);
void LCD_init(void);	

uint8_t Seven_Segment_Decoder(uint8_t num);
void Seven_Segment_Show(uint8_t num);

void initialize_game(void);
void gameover(void);
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
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  LCD_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (state == get_studentID)
		{
			LCD_command(1);
			char studentID[8];
			for(int i = 0; i < 8 ; i++)
			{
				studentID[i] = get_char();
				LCD_data(studentID[i]);
			}
			
			match_studentID = 1;
			for(int i = 0; i < 8 ; i++)
				if(studentID[i] != studentID1[i])
				{
					match_studentID = 0;
					break;
				}
				
			if(match_studentID == 1)
			{
				state = blink_name;
			}
			
			else
			{
				match_studentID = 2;
				for(int i = 0; i < 8 ; i++)
					if(studentID[i] != studentID2[i])
					{
						match_studentID = 0;
						break;
					}
				if(match_studentID == 2)
				{
					state = blink_name;
				}
			}
			if(state == blink_name)
			{
				LCD_command(1);
				HAL_TIM_Base_Start_IT(&htim2);
				HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
			}
		}
		else if (state == game)
		{
			if(LED_state != None)
			{
				wrong_count++;
				Seven_Segment_Show(wrong_count);
				if(wrong_count == 3)
				{
					state = lose;
					gameover();
				}

			}
			HAL_GPIO_WritePin(LED_GPIO, LED_Red, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_GPIO, LED_Green, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_GPIO, LED_Blue, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED_GPIO, LED_Yellow, GPIO_PIN_RESET);
			srand(__HAL_TIM_GetCounter(&htim3));
			uint8_t random = rand()%4;
			
			switch (random)
			{
				case 0:
					LED_state = Red;
					HAL_GPIO_WritePin(LED_GPIO, LED_Red, GPIO_PIN_SET);
					break;
				case 1:
					LED_state = Green;
					HAL_GPIO_WritePin(LED_GPIO, LED_Green, GPIO_PIN_SET);
					break;
				case 2:
					LED_state = Blue;
					HAL_GPIO_WritePin(LED_GPIO, LED_Blue, GPIO_PIN_SET);
					break;
				case 3:
					LED_state = Yellow;
					HAL_GPIO_WritePin(LED_GPIO, LED_Yellow, GPIO_PIN_SET);
					break;
			}
			delayMs(delay_value);
		}
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  htim2.Init.Prescaler = 16000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
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
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim5.Init.Period = 999;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 PC6 PC7
                           PC8 PC9 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA8 PA9
                           PA10 PA11 PA12 PA13
                           PA14 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2)
	{
		if(state == blink_name)
		{			
			if(clear_counter == 0)
			{
				if(match_studentID == 1)
					for(int i = 0; i < 7 ; i++)
						LCD_data(studentName1[i]);
				else
					for(int i = 0; i < 6 ; i++)
						LCD_data(studentName2[i]);
				clear_counter = 1;
			}	
			else
			{
				LCD_command(1);
				clear_counter = 0;
			}
		}
	}

	if(htim == &htim5)
	{	
		seconds++;
		LCD_command(1);
		LCD_data(((60 - seconds) / 10) % 10 + 48);
		LCD_data((60 - seconds) % 10 + 48);
		if(seconds == 9)
		{
			delay_value = 2000;
		}
		if(seconds == 15)
		{
			delay_value = 1000;
		}
		if(seconds >= 25)
		{
			delay_value = 900;
		}
		if(seconds == 35)
		{
			delay_value = 800;
		}
		if(seconds == 45)
		{
			delay_value = 700;
		}
		if(seconds == 55)
		{
			delay_value = 600;
		}
		if(seconds == GameTime)
		{
			state = win;
			gameover();
		}
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(capture_counter == 0)
	{
			__HAL_TIM_SetCounter(&htim3, 0);
			capture_counter = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_FALLING) ;
	}
	else
	{
		if(state == blink_name || state == pushed_once)
		{	
			if(__HAL_TIM_GetCounter(&htim3) > 50)
			{
				if(state == blink_name)
					state = pushed_once;	
				else
				{
					initialize_game();
				}
			}
		}
		else if (state == game || state == win || state == lose)
		{
			if(__HAL_TIM_GetCounter(&htim3) > 75)
			{
				initialize_game();
			}
		}
		capture_counter = 0;	
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1,TIM_INPUTCHANNELPOLARITY_RISING) ;
	}
}

char get_char(void)
{
		HAL_GPIO_WritePin(KP_GPIO, KPB, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(KP_GPIO, KPC, GPIO_PIN_RESET);
	while(1)
	{
		HAL_GPIO_WritePin(KP_GPIO, KPD, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(KP_GPIO, KPA, GPIO_PIN_SET);
		if (HAL_GPIO_ReadPin(KP_GPIO, KP1) == GPIO_PIN_SET)
		{
			while (HAL_GPIO_ReadPin(KP_GPIO, KP1) == GPIO_PIN_SET){}
			return '1';
		}
		if (HAL_GPIO_ReadPin(KP_GPIO, KP2) == GPIO_PIN_SET)
		{
			while (HAL_GPIO_ReadPin(KP_GPIO, KP2) == GPIO_PIN_SET){}
			return '2';
		}
		if (HAL_GPIO_ReadPin(KP_GPIO, KP3) == GPIO_PIN_SET)
		{
			while (HAL_GPIO_ReadPin(KP_GPIO, KP3) == GPIO_PIN_SET){}
			return '3';
		}
		
		HAL_GPIO_WritePin(KP_GPIO, KPA, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(KP_GPIO, KPB, GPIO_PIN_SET);
		if (HAL_GPIO_ReadPin(KP_GPIO, KP1) == GPIO_PIN_SET)
		{
			while (HAL_GPIO_ReadPin(KP_GPIO, KP1) == GPIO_PIN_SET){}
			return '4';
		}
		if (HAL_GPIO_ReadPin(KP_GPIO, KP2) == GPIO_PIN_SET)
		{
			while (HAL_GPIO_ReadPin(KP_GPIO, KP2) == GPIO_PIN_SET){}
			return '5';
		}
		if (HAL_GPIO_ReadPin(KP_GPIO, KP3) == GPIO_PIN_SET)
		{
			while (HAL_GPIO_ReadPin(KP_GPIO, KP3) == GPIO_PIN_SET){}
			return '6';
		}
		
		HAL_GPIO_WritePin(KP_GPIO, KPB, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(KP_GPIO, KPC, GPIO_PIN_SET);
		if (HAL_GPIO_ReadPin(KP_GPIO, KP1) == GPIO_PIN_SET)
		{
			while (HAL_GPIO_ReadPin(KP_GPIO, KP1) == GPIO_PIN_SET){}
			return '7';
		}
		if (HAL_GPIO_ReadPin(KP_GPIO, KP2) == GPIO_PIN_SET)
		{
			while (HAL_GPIO_ReadPin(KP_GPIO, KP2) == GPIO_PIN_SET){}
			return '8';
		}
		if (HAL_GPIO_ReadPin(KP_GPIO, KP3) == GPIO_PIN_SET)
		{
			while (HAL_GPIO_ReadPin(KP_GPIO, KP3) == GPIO_PIN_SET){}
			return '9';
		}
		
		HAL_GPIO_WritePin(KP_GPIO, KPC, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(KP_GPIO, KPD, GPIO_PIN_SET);
		if (HAL_GPIO_ReadPin(KP_GPIO, KP1) == GPIO_PIN_SET)
		{
			while (HAL_GPIO_ReadPin(KP_GPIO, KP1) == GPIO_PIN_SET){}
			return '*';
		}
		if (HAL_GPIO_ReadPin(KP_GPIO, KP2) == GPIO_PIN_SET)
		{
			while (HAL_GPIO_ReadPin(KP_GPIO, KP2) == GPIO_PIN_SET){}
			return '0';
		}
		if (HAL_GPIO_ReadPin(KP_GPIO, KP3) == GPIO_PIN_SET)
		{
			while (HAL_GPIO_ReadPin(KP_GPIO, KP3) == GPIO_PIN_SET){}
			return '#';
		}
	}
}

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
	/* RS = 0, R/W = 0 */
	HAL_GPIO_WritePin(GPIOB, RS, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, RW, GPIO_PIN_RESET);
	/* clear data bus */
	/* put command on data bus */
	for(uint16_t GPIO_Pin = GPIO_PIN_0; GPIO_Pin != GPIO_PIN_8; GPIO_Pin = GPIO_Pin << 1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_Pin, command % 2);
		command = command >> 1;
	}
	HAL_GPIO_WritePin(GPIOB, EN, GPIO_PIN_SET);
	/* pulse E high */
	delayMs(0);
	HAL_GPIO_WritePin(GPIOB, EN, GPIO_PIN_RESET);
	/* clear E */

	if (command < 4)
		delayMs(2); /* command 1 and 2 needs up to 1.64ms */
	else
		delayMs(1); /* all others 40 us */
}

void LCD_data(char data)
{
	/* RS = 1 */
	/* R/W = 0 */
	HAL_GPIO_WritePin(GPIOB, RS, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, RW, GPIO_PIN_RESET);
	/* clear data bus */
	/* put data on data bus */
	for(uint16_t GPIO_Pin = GPIO_PIN_0; GPIO_Pin != GPIO_PIN_8; GPIO_Pin = GPIO_Pin << 1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_Pin, data % 2);
		data = data >> 1;
	}
	/* pulse E high */
	HAL_GPIO_WritePin(GPIOB, EN, GPIO_PIN_SET);
	delayMs(0);
 /* clear E */
	HAL_GPIO_WritePin(GPIOB, EN, GPIO_PIN_RESET);
	delayMs(1);
}

uint8_t Seven_Segment_Decoder(uint8_t num)
{
	switch(num)
	{
		case 0:
			return 0x7E;
		case 1:
			return 0x30;
		case 2:
			return 0x6D;
		case 3:
			return 0x79;
	}
	return 0;
}

void Seven_Segment_Show(uint8_t num)
{
	uint8_t temp = Seven_Segment_Decoder(num);
	HAL_GPIO_WritePin(SS_GPIO, SS7, temp % 2);
	temp /= 2;
	HAL_GPIO_WritePin(SS_GPIO, SS6, temp % 2);
	temp /= 2;
	HAL_GPIO_WritePin(SS_GPIO, SS5, temp % 2);
	temp /= 2;
	HAL_GPIO_WritePin(SS_GPIO, SS4, temp % 2);
	temp /= 2;
	HAL_GPIO_WritePin(SS_GPIO, SS3, temp % 2);
	temp /= 2;
	HAL_GPIO_WritePin(SS_GPIO, SS2, temp % 2);
	temp /= 2;
	HAL_GPIO_WritePin(SS_GPIO, SS1, temp % 2);
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
	if(state == game)
	{
		HAL_GPIO_WritePin(LED_GPIO, LED_Red, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GPIO, LED_Green, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GPIO, LED_Blue, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LED_GPIO, LED_Yellow, GPIO_PIN_RESET);
		switch(GPIO_Pin)
		{
			case Button_Red:
				if(LED_state != Red)
				{
					wrong_count++;
				}
				break;
				case Button_Green:
				if(LED_state != Green)
				{
					wrong_count++;
				}
				break;
				case Button_Blue:
				if(LED_state != Blue)
				{
					wrong_count++;
				}
				break;
				case Button_Yellow:
				if(LED_state != Yellow)
				{
					wrong_count++;
				}
				break;	
		}
		if(wrong_count == 3)
		{
			state = lose;
			gameover();
		}
		Seven_Segment_Show(wrong_count);
		LED_state = None;
	}
}

void initialize_game(void)
{
	LED_state = None;
	HAL_GPIO_WritePin(LED_GPIO, LED_Red, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GPIO, LED_Green, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GPIO, LED_Blue, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GPIO, LED_Yellow, GPIO_PIN_RESET);
	state = game;
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim5);
	LCD_command(1);
	wrong_count = 0;
	Seven_Segment_Show(wrong_count);
	seconds = 0;
	LCD_data('6');
	LCD_data('0');
}

void gameover(void) 
{
	HAL_GPIO_WritePin(LED_GPIO, LED_Red, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GPIO, LED_Green, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GPIO, LED_Blue, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_GPIO, LED_Yellow, GPIO_PIN_RESET);
	LCD_command(1);
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_TIM_Base_Stop_IT(&htim5);
	Seven_Segment_Show(wrong_count);
	if(state == win)
		{	
			while(state == win)
			{
				LCD_data('W');
				LCD_data('I');
				LCD_data('N');
				LCD_data('N');
				LCD_data('E');
				LCD_data('R');
				HAL_GPIO_WritePin(LED_GPIO, LED_Green, GPIO_PIN_SET);
				delayMs(500);
				HAL_GPIO_WritePin(LED_GPIO, LED_Green, GPIO_PIN_RESET);
				delayMs(500);
				LCD_command(1);
				HAL_GPIO_WritePin(LED_GPIO, LED_Green, GPIO_PIN_SET);
				delayMs(500);
				HAL_GPIO_WritePin(LED_GPIO, LED_Green, GPIO_PIN_RESET);
				delayMs(500);
			}
		}
		else if(state == lose)
		{
			while(state == lose)
			{
				LCD_data('L');
				LCD_data('O');
				LCD_data('S');
				LCD_data('E');
				LCD_data('R');
				HAL_GPIO_WritePin(LED_GPIO, LED_Red, GPIO_PIN_SET);
				delayMs(500);
				HAL_GPIO_WritePin(LED_GPIO, LED_Red, GPIO_PIN_RESET);
				delayMs(500);
				LCD_command(1);
				HAL_GPIO_WritePin(LED_GPIO, LED_Red, GPIO_PIN_SET);
				delayMs(500);
				HAL_GPIO_WritePin(LED_GPIO, LED_Red, GPIO_PIN_RESET);
				delayMs(500);
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
