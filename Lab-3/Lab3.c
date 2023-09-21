#include "stm32f4xx.h"
#include <stdio.h>

#define RS 0x20 /* PB5 mask for reg select */
#define RW 0x40 /* PB6 mask for read/write */
#define EN 0x80 /* PB7 mask for enable */

void delayMs(int n);
void LCD_command(unsigned char command);
void LCD_data(char data);
void LCD_init(void);
void PORTS_init(void);
void EXTI15_10_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void LCD_print_result(void);
void LCD_reset(void);

static int col = 0x0100, num1 = 0, num2 = 0, result = 0;
static char op = ' ';
enum state
{
	get_first_num,
	get_second_num,
	get_op,
	show_result
};
static char current_key = ' ';
static enum state calculator_state = get_first_num;

int main(void)
{
	LCD_init();
	LCD_reset();
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
	}
}

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
	if (current_key == 'C')
	{
		LCD_reset();
		calculator_state = get_first_num;
		num1 = 0;
		num2 = 0;
	}
	else if ((current_key == '+' || current_key == '-' || current_key == '*' || current_key == '/') &&
			 (calculator_state == get_first_num || calculator_state == get_op))
	{
		LCD_reset();
		op = current_key;
		LCD_data(op);
		calculator_state = get_op;
	}
	else if (current_key == '=' && calculator_state == get_second_num)
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
	else if (calculator_state == get_first_num && (current_key >= '0' && current_key <= '9'))
	{
		num1 = num1 * 10 + ((int)current_key - 48);
		LCD_data(current_key);
	}
	else if (calculator_state == get_second_num && (current_key >= '0' && current_key <= '9'))
	{
		num2 = num2 * 10 + ((int)current_key - 48);
		LCD_data(current_key);
	}
	else if (calculator_state == show_result && (current_key >= '0' && current_key <= '9'))
	{
		LCD_reset();
		calculator_state = get_first_num;
		num1 = ((int)current_key - 48);
		LCD_data(current_key);
	}
	else if (calculator_state == get_op && (current_key >= '0' && current_key <= '9'))
	{
		LCD_reset();
		calculator_state = get_second_num;
		num2 = ((int)current_key - 48);
		LCD_data(current_key);
	}

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
