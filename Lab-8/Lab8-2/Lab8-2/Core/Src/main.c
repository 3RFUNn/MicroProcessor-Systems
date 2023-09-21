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
#include <stdio.h>
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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t * rdata;
static int col = 0x0100, num1 = 0, num2 = 0, result = 0;
static char op = ' ';
enum state
{
	get_first_num,
	get_second_num,
	get_op,
	show_result
};
static char current_key = ' ', current_key_r;
static enum state calculator_state = get_first_num;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void delayMs(int n);
void LCD_command(unsigned char command);
void LCD_data(char data);
void LCD_init(void);
void PORTS_init(void);
void EXTI15_10_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void LCD_print_result(void);
void LCD_reset(void);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
		LCD_init();
		LCD_reset();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (col == 0x0800)
		{
			col = 0x0100;
		}
		else
			col = col << 1;

		GPIOC->ODR = col;
		delayMs(1);
    /* USER CODE END WHILE */
		//HAL_UART_Receive_IT(&huart1, data, sizeof(rdata));
		//HAL_UART_Transmit(&huart1, data, sizeof(data), HAL_MAX_DELAY);
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */


/* initialize port pins then initialize LCD controller */
void LCD_init(void)
{
	PORTS_init();

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

void PORTS_init(void)
{
	RCC->AHB1ENR |= 0x06;	/* enable GPIO B/C clock */
	RCC->APB2ENR |= 0x4000; /* enable SysConfig clock */
	
	/* PB5 for LCD R/S */
	/* PB6 for LCD R/W */
	/* PB7 for LCD EN */
	GPIOB->MODER = 0x00005400; /* set pin output mode */
	GPIOB->PUPDR = 0x000A5400; /* set pin output mode */
	GPIOB->BSRR = 0x00C00000;  /* turn off EN and R/W */

	/* PC0-PC7 for LCD D0-D7, respectively. */
	GPIOC->MODER = 0x00555555; /* set pin output mode */
	GPIOC->PUPDR = 0xAA000000; /* set pin output mode */

	SYSCFG->EXTICR[3] = 0x2222;
	SYSCFG->EXTICR[2] = 0x0011;
	EXTI->IMR = 0xF300;
	EXTI->RTSR = 0xF300;
	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	__enable_irq();
}

void LCD_command(unsigned char command)
{
	GPIOB->BSRR = (RS | RW) << 16; /* RS = 0, R/W = 0 */
	GPIOC->ODR &= ~0x00FF;		   /* clear data bus */
	GPIOC->ODR |= command;		   /* put command on data bus */
	GPIOB->BSRR = EN;			   /* pulse E high */
	delayMs(0);
	GPIOB->BSRR = EN << 16; /* clear E */

	if (command < 4)
		delayMs(2); /* command 1 and 2 needs up to 1.64ms */
	else
		delayMs(1); /* all others 40 us */
}

void LCD_data(char data)
{
	GPIOB->BSRR = RS;		/* RS = 1 */
	GPIOB->BSRR = RW << 16; /* R/W = 0 */
	GPIOC->ODR &= ~0x00FF;	/* clear data bus */
	GPIOC->ODR |= data;		/* put data on data bus */
	GPIOB->BSRR = EN;		/* pulse E high */
	GPIOB->BSRR = EN;		/* pulse E high */
	delayMs(0);
	GPIOB->BSRR = EN << 16; /* clear E */
	delayMs(1);
}

/* delay n milliseconds (16 MHz CPU clock) */
void delayMs(int n)
{
	int i;
	for (; n > 0; n--)
		for (i = 0; i < 3195; i++)
			__NOP();
}

void EXTI15_10_IRQHandler(void)
{
	while ((GPIOC->IDR & 0xF000) != 0)
	{
	}
	if (EXTI->PR & 0x1000)
	{
		switch (col)
		{
		case 0x0100:
			current_key = '7';
			break;
		case 0x0200:
			current_key = '8';
			break;
		case 0x0400:
			current_key = '9';
			break;
		case 0x0800:
			current_key = '/';
			break;
		}
		EXTI->PR |= 0x1000;
	}
	if (EXTI->PR & 0x2000)
	{
		switch (col)
		{
		case 0x0100:
			current_key = '4';
			break;
		case 0x0200:
			current_key = '5';
			break;
		case 0x0400:
			current_key = '6';
			break;
		case 0x0800:
			current_key = '*';
			break;
		}
		EXTI->PR |= 0x2000;
	}
	if (EXTI->PR & 0x4000)
	{
		switch (col)
		{
		case 0x0100:
			current_key = '1';
			break;
		case 0x0200:
			current_key = '2';
			break;
		case 0x0400:
			current_key = '3';
			break;
		case 0x0800:
			current_key = '-';
			break;
		}
		EXTI->PR |= 0x4000;
	}
	if (EXTI->PR & 0x8000)
	{
		switch (col)
		{
		case 0x0100:
			current_key = 'C';
			break;
		case 0x0200:
			current_key = '0';
			break;
		case 0x0400:
			current_key = '=';
			break;
		case 0x0800:
			current_key = '+';
			break;
		}
		EXTI->PR |= 0x8000;
	}
		HAL_UART_Receive_IT(&huart1, &current_key_r, 1);
	HAL_UART_Transmit(&huart1, &current_key, 1, HAL_MAX_DELAY);
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
}

void EXTI9_5_IRQHandler(void)
{
	if (calculator_state == show_result)
	{
		LCD_reset();
		if (EXTI->PR & 0x0100)
		{
			result++;
			EXTI->PR |= 0x0100;
		}
		if (EXTI->PR & 0x0200)
		{
			result--;
			EXTI->PR |= 0x0200;
		}
		LCD_print_result();
	}
	else
	{
		EXTI->PR |= 0x0300;
	}
	NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
}

void LCD_print_result(void)
{
	char charValue[16];
	sprintf(charValue, "%d", result);
	int i = 0;
	while (charValue[i] != '\0')
	{
		LCD_data(charValue[i]);
		i++;
	}
}
void LCD_reset(void)
{
	LCD_command(1);
	LCD_data('R');
	LCD_data('a');
	LCD_data('f');
	LCD_data('e');
	LCD_data('e');
	LCD_data('-');
	LCD_data('F');
	LCD_data('o');
	LCD_data('o');
	LCD_data('l');
	LCD_data('a');
	LCD_data('d');
	LCD_data('i');
	LCD_command(0xC0);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
if (current_key_r == 'C')
	{
		LCD_reset();
		calculator_state = get_first_num;
		num1 = 0;
		num2 = 0;
	}
	else if ((current_key_r == '+' || current_key_r == '-' || current_key_r == '*' || current_key_r == '/') &&
			 (calculator_state == get_first_num || calculator_state == get_op))
	{
		LCD_reset();
		op = current_key_r;
		LCD_data(op);
		calculator_state = get_op;
	}
	else if ((current_key_r == '+' || current_key_r == '-' || current_key_r == '*' || current_key_r == '/') &&
			 (calculator_state == get_second_num ))
	{
		LCD_reset();
		op = current_key_r;
		switch (op)
		{
		case '+':
			result = num1 + num2;
			break;
		case '-':
			result = num1 - num2;
			break;
		case '*':
			result = num1 * num2;
			break;
		case '/':
			result = num1 / num2;
			break;
		}
		num1 = result;
		num2 = 0;
		LCD_print_result();
	}
	else if (current_key_r == '=' && calculator_state == get_second_num)
	{
		LCD_reset();
		switch (op)
		{
		case '+':
			result = num1 + num2;
			break;
		case '-':
			result = num1 - num2;
			break;
		case '*':
			result = num1 * num2;
			break;
		case '/':
			result = num1 / num2;
			break;
		}
		calculator_state = show_result;
		LCD_print_result();
	}
	else if (calculator_state == get_first_num && (current_key_r >= '0' && current_key_r <= '9'))
	{
		if(num1 == 0)
			LCD_reset();
		num1 = num1 * 10 + ((int)current_key_r - 48);
		LCD_data(current_key_r);
	}
	else if (calculator_state == get_second_num && (current_key_r >= '0' && current_key_r <= '9'))
	{
		if(num2 == 0)
			LCD_reset();
		num2 = num2 * 10 + ((int)current_key_r - 48);
		LCD_data(current_key_r);
	}
	else if (calculator_state == show_result && (current_key_r >= '0' && current_key_r <= '9'))
	{
		LCD_reset();
		calculator_state = get_first_num;
		num1 = ((int)current_key_r - 48);
		LCD_data(current_key_r);
	}
	else if (calculator_state == get_op && (current_key_r >= '0' && current_key_r <= '9'))
	{
		LCD_reset();
		calculator_state = get_second_num;
		num2 = ((int)current_key_r - 48);
		LCD_data(current_key_r);
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
