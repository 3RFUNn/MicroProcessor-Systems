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
void TIM2_IRQHandler(void);
void show_time(void);
void turn_off(void);

enum state
{
	stopped,
	counting,
	paused,
	off
};

static enum state chronometer_state = stopped;

static int MM = 0, SS = 0, MMM = 0, first, second, is_falling = 0;

int main(void)
{
	LCD_init();
	LCD_data('W');
	LCD_data('e');
	LCD_data('l');
	LCD_data('c');
	LCD_data('o');
	LCD_data('m');
	LCD_data('e');
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
	GPIOB->PUPDR = 0x0150000;  /* set pin output mode */
	GPIOB->BSRR = 0x00C00000;  /* turn off EN and R/W */

	/* PC0-PC7 for LCD D0-D7, respectively. */
	GPIOC->MODER = 0x00005555; /* set pin output mode */

	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; /* enable TIM5 clock */
	TIM5->PSC = 16000 - 1;				/* divided by 16000 */
	TIM5->ARR = 20000 - 1;				/* divided by 20000 */
	TIM5->CR1 = TIM_CR1_CEN;			/* enable counter */

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	TIM2->PSC = 160 - 1;
	TIM2->ARR = 10 - 1;
	TIM2->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM2_IRQn); /* enable interrupt in NVIC */

	SYSCFG->EXTICR[2] = 0x0111;
	EXTI->IMR = 0x0700;
	EXTI->FTSR = 0x0700;
	EXTI->RTSR = 0x0400;
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
	chronometer_state = stopped;
	TIM2->CR1 &= ~TIM_CR1_CEN;
	MMM = 0;
	SS = 0;
	MM = 0;
	show_time();
	if (is_falling == 0)
	{
		first = TIM5->CNT;
		is_falling = 1;
	}
	else
	{
		is_falling = 0;
		second = TIM5->CNT;
		if (second - first > 300)
		{
			__disable_irq();
			LCD_command(1);
			chronometer_state = off;
		}
	}

	EXTI->PR |= 0x0400;
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
}

void EXTI9_5_IRQHandler(void)
{
	if (EXTI->PR & 0x0100)
	{
		chronometer_state = counting;
		TIM2->CR1 |= TIM_CR1_CEN;
		EXTI->PR |= 0x0100;
	}
	else if (EXTI->PR & 0x0200)
	{
		chronometer_state = paused;
		TIM2->CR1 &= ~TIM_CR1_CEN;
		show_time();
		EXTI->PR |= 0x0200;
	}
	NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
}

void TIM2_IRQHandler(void)
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
	NVIC_ClearPendingIRQ(TIM2_IRQn);
}
void show_time(void)
{
	LCD_command(1);
	char print[16];
	sprintf(print, "%.2d:%.2d:%.3d", MM, SS, MMM);
	for (int i = 0; i < 16 && print[i] != '\0'; i++)
		LCD_data(print[i]);
}
