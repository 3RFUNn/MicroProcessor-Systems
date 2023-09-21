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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
const uint16_t Sampelling_Rate = 8000;
uint16_t Sampelling_Data[5000];
const uint16_t Window_Lenth = 500;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

// ROW
const uint32_t k697 = (uint32_t)(0.5 + ((Window_Lenth * 697) / Sampelling_Rate));
const uint32_t k_mul697 = (uint32_t)(0.5 + ((Window_Lenth * 697 * 2) / Sampelling_Rate));

const uint32_t k770 = (uint32_t)(0.5 + ((Window_Lenth * 770) / Sampelling_Rate));
const uint32_t k_mul770 = (uint32_t)(0.5 + ((Window_Lenth * 770 * 2) / Sampelling_Rate));

const uint32_t k852 = (uint32_t)(0.5 + ((Window_Lenth * 852) / Sampelling_Rate));
const uint32_t k_mul852 = (uint32_t)(0.5 + ((Window_Lenth * 852 * 2) / Sampelling_Rate));

const uint32_t k941 = (uint32_t)(0.5 + ((Window_Lenth * 941) / Sampelling_Rate));
const uint32_t k_mul941 = (uint32_t)(0.5 + ((Window_Lenth * 941 * 2) / Sampelling_Rate));

// COL
const uint32_t k1209 = (uint32_t)(0.5 + ((Window_Lenth * 1209) / Sampelling_Rate));
const uint32_t k_mul1209 = (uint32_t)(0.5 + ((Window_Lenth * 1209 * 2) / Sampelling_Rate));

const uint32_t k1336 = (uint32_t)(0.5 + ((Window_Lenth * 1336) / Sampelling_Rate));
const uint32_t k_mul1336 = (uint32_t)(0.5 + ((Window_Lenth * 1336 * 2) / Sampelling_Rate));

const uint32_t k1477 = (uint32_t)(0.5 + ((Window_Lenth * 1477) / Sampelling_Rate));
const uint32_t k_mul1477 = (uint32_t)(0.5 + ((Window_Lenth * 1477 * 2) / Sampelling_Rate));

const uint32_t k1633 = (uint32_t)(0.5 + ((Window_Lenth * 1633) / Sampelling_Rate));
const uint32_t k_mul1633 = (uint32_t)(0.5 + ((Window_Lenth * 1633 * 2) / Sampelling_Rate));


// DEFINE COEFK
// ROW
float coef_k697, coef_k_mul697;
float coef_k770, coef_k_mul770;
float coef_k852, coef_k_mul852;
float coef_k941, coef_k_mul941;

//COL
float coef_k1209, coef_k_mul1209;
float coef_k1336, coef_k_mul1336;
float coef_k1477, coef_k_mul1477;
float coef_k1633, coef_k_mul1633;


uint32_t Index = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
uint8_t Seven_Segment_Decoder(uint8_t num);
void goertzel();
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
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	coef_k697 = 2*cos((2*3.14159265359*(float)k697)/(float)Window_Lenth);
	coef_k770 = 2*cos((2*3.14159265359*(float)k770)/(float)Window_Lenth);
	coef_k852  = 2*cos((2*3.14159265359*(float)k852)/(float)Window_Lenth);
	coef_k941 = 2*cos((2*3.14159265359*(float)k941)/(float)Window_Lenth);
	coef_k1209 = 2*cos((2*3.14159265359*(float)k1209)/(float)Window_Lenth);
	coef_k1336 = 2*cos((2*3.14159265359*(float)k1336)/(float)Window_Lenth);
	coef_k1477 = 2*cos((2*3.14159265359*(float)k1477)/(float)Window_Lenth);
	coef_k1633 = 2*cos((2*3.14159265359*(float)k1633)/(float)Window_Lenth);

	coef_k_mul697 = 2*cos((2*3.14159265359*(float)k_mul697)/(float)Window_Lenth);
	coef_k_mul770 = 2*cos((2*3.14159265359*(float)k_mul770)/(float)Window_Lenth);
	coef_k_mul852 = 2*cos((2*3.14159265359*(float)k_mul852)/(float)Window_Lenth);
	coef_k_mul941 = 2*cos((2*3.14159265359*(float)k_mul941)/(float)Window_Lenth);
	coef_k_mul1209 = 2*cos((2*3.14159265359*(float)k_mul1209)/(float)Window_Lenth);
	coef_k_mul1336 = 2*cos((2*3.14159265359*(float)k_mul1336)/(float)Window_Lenth);
	coef_k_mul1477 = 2*cos((2*3.14159265359*(float)k_mul1477)/(float)Window_Lenth);
	coef_k_mul1633 = 2*cos((2*3.14159265359*(float)k_mul1633)/(float)Window_Lenth);

	
	HAL_TIM_Base_Start_IT(&htim3);
	
  /* USER CODE END 2 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 249;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC8 PC9
                           PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t Seven_Segment_Decoder(uint8_t num)
{
  switch (num)
  {
  case 0:
    return 0x7E;
  case 1:
    return 0x30;
  case 2:
    return 0x6D;
  case 3:
    return 0x79;
  case 4:
    return 0x33;
  case 5:
    return 0x5B;
  case 6:
    return 0x5F;
  case 7:
    return 0x70;
  case 8:
    return 0x7F;
  case 9:
    return 0x7B;
  case 10:
    return 0x77;
  case 11:
    return 0x1F;
  case 12:
    return 0x4E;
  case 13:
    return 0x3D;
  case 14:
    return 0x01;
  case 15:
    return 0x1D;
  }
  return 0;
}



uint16_t Get_Analog_Value()
{
	HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint16_t res = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return res;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  Sampelling_Data[Index] = Get_Analog_Value();
 
  if(Index % Window_Lenth == Window_Lenth - 1)
		goertzel();

  Index = Index == 4999 ? 0 : Index + 1;
}

float Power(float coef)
{
  float Q0, Q1, Q2;
  Q2 = 0;
  Q1 = 0;
  Q0 = 0;
  
  for(int i = Index - Window_Lenth + 1; i <= Index; i++){
    Q2 = Q1;
    Q1 = Q0;
    Q0 = (coef * Q1) - Q2 + (float)Sampelling_Data[i];
  }
  return Q1 * Q1 + Q2 * Q2 - (coef * Q1 * Q2); 
}

void goertzel()
{
  float myPowers_row[4];
  float myPowers_col[4];
  uint8_t ROW_MAX = 0, COL_MAX = 0;
  
  myPowers_row[0] = Power(coef_k697) + Power(coef_k_mul697);
  myPowers_row[1] = Power(coef_k770) + Power(coef_k_mul770);
  myPowers_row[2] = Power(coef_k852) + Power(coef_k_mul852);
  myPowers_row[3] = Power(coef_k941) + Power(coef_k_mul941);
  for(int j = 0; j < 3; j++)
    if(myPowers_row[j] <= myPowers_row[j + 1])
      ROW_MAX = j + 1;
  
  myPowers_col[0] = Power(coef_k1209) + Power(coef_k_mul1209);
  myPowers_col[1] = Power(coef_k1336) + Power(coef_k_mul1336);
  myPowers_col[2] = Power(coef_k1477) + Power(coef_k_mul1477);
  myPowers_col[3] = Power(coef_k1633) + Power(coef_k_mul1633);
  for(int j =0; j < 3; j++)
    if(myPowers_col[j] <= myPowers_col[j + 1])
      COL_MAX = j + 1;
		
  if(ROW_MAX == 0 && COL_MAX == 0)
      GPIOC->ODR = Seven_Segment_Decoder(1)<<8;
  if(ROW_MAX == 0 && COL_MAX == 1)
      GPIOC->ODR = Seven_Segment_Decoder(2)<<8;
  if(ROW_MAX == 0 && COL_MAX == 2)
      GPIOC->ODR = Seven_Segment_Decoder(3)<<8;
  if(ROW_MAX == 0 && COL_MAX == 3)
      GPIOC->ODR = Seven_Segment_Decoder(10)<<8;
  
  if(ROW_MAX == 1 && COL_MAX == 0)
      GPIOC->ODR = Seven_Segment_Decoder(4)<<8;
  if(ROW_MAX == 1 && COL_MAX == 1)
      GPIOC->ODR = Seven_Segment_Decoder(5)<<8;
  if(ROW_MAX == 1 && COL_MAX == 2)
      GPIOC->ODR = Seven_Segment_Decoder(6)<<8;
  if(ROW_MAX == 1 && COL_MAX == 3)
      GPIOC->ODR = Seven_Segment_Decoder(11)<<8;
  
  if(ROW_MAX == 2 && COL_MAX == 0)
      GPIOC->ODR = Seven_Segment_Decoder(7)<<8;
  if(ROW_MAX == 2 && COL_MAX == 1)
      GPIOC->ODR = Seven_Segment_Decoder(8)<<8;
  if(ROW_MAX == 2 && COL_MAX == 2)
      GPIOC->ODR = Seven_Segment_Decoder(9)<<8;
  if(ROW_MAX == 2 && COL_MAX == 3)
      GPIOC->ODR = Seven_Segment_Decoder(12)<<8;
  
  if(ROW_MAX == 3 && COL_MAX == 0)
      GPIOC->ODR = Seven_Segment_Decoder(14)<<8;
  if(ROW_MAX == 3 && COL_MAX == 1)
      GPIOC->ODR = Seven_Segment_Decoder(0)<<8;
  if(ROW_MAX == 3 && COL_MAX == 2)
      GPIOC->ODR = Seven_Segment_Decoder(15)<<8;
  if(ROW_MAX == 3 && COL_MAX == 3)
      GPIOC->ODR = Seven_Segment_Decoder(13)<<8;
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
  * @param  file: Index to the source file name
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
