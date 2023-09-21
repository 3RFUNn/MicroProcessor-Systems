#include "stm32F4xx.h"

void ADC_init(void);
void USART2_init(void);
void USART2_write(int c);
void USART2_IRQHandler(void);
void delayMs(int);

int main (void) {
		ADC_init();
    USART2_init();          /* initialize USART2 */
	
    while(1) {              
			ADC1->CR2 |= 0x40000000;        /* start a conversion */
			while(!(ADC1->SR & 2)) {}       /* wait for conv complete */
			uint32_t result = ADC1->DR;              /* read conversion result */
			double res = result;
			res /= 4095;
			res *= 500;			
			result = (uint32_t) res;
			uint16_t digits[3];
			digits[0] = result % 10;
			result /= 10;
			digits[1] = result % 10;
			result /= 10;
			digits[2] = result % 10;
			USART2_write(digits[2] + 48);
			USART2_write(digits[1] + 48);
			USART2_write(digits[0] + 48);
			USART2_write('\r');
			delayMs(1000);        /* leave a gap between messages */
    }
}

void ADC_init(void)
{
		RCC->AHB1ENR |=  1;	            /* enable GPIOA clock */
    GPIOA->MODER &= ~0x00000C00;    /* clear pin mode */
    GPIOA->MODER |=  0x00000400;    /* set pin to output mode */

    /* set up pin PA1 for analog input */
    RCC->AHB1ENR |=  1;	            /* enable GPIOA clock */
    GPIOA->MODER |=  0xC;           /* PA1 analog */

    /* setup ADC1 */
    RCC->APB2ENR |= 0x00000100;     /* enable ADC1 clock */
    ADC1->CR2 = 0;                  /* SW trigger */
    ADC1->SQR3 = 1;                 /* conversion sequence starts at ch 1 */
    ADC1->SQR1 = 0;                 /* conversion sequence length 1 */
    ADC1->CR2 |= 1;                 /* enable ADC1 */

}

void USART2_init (void) {
    RCC->AHB1ENR |= 1;          /* Enable GPIOA clock */
    RCC->APB1ENR |= 0x20000;    /* Enable USART2 clock */
    GPIOA->AFR[0] &= ~0x0F00;
    GPIOA->AFR[0] |=  0x0700;   /* alt7 for USART2 */
    GPIOA->MODER  &= ~0x00F0;
    GPIOA->MODER  |=  0x0020;   /* enable alternate function for PA2 */

    USART2->BRR = 0x0683;       /* 9600 baud @ 16 MHz */
    USART2->CR1 = 0x000C;       /* enable Tx, 8-bit data */
    USART2->CR2 = 0x0000;       /* 1 stop bit */
    USART2->CR3 = 0x0000;       /* no flow control */
    USART2->CR1 |= 0x2000;      /* enable USART2 */
	
		NVIC_EnableIRQ(USART2_IRQn);
}


void USART2_write (int ch) {
    while (!(USART2->SR & 0x0080)) {}   
    USART2->DR = (ch & 0xFF);
}


void delayMs(int n) {
    int i;
    for (; n > 0; n--)
        for (i = 0; i < 3195; i++) 
						__NOP();
}