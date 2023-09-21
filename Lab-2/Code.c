#include "stm32f4xx.h"


void delayMs(int n);
int BCD_to_seven_segment_decoder(int);

int button_1_counter, button_2_counter, pressed_1, pressed_2, counter_flag, ms;
int red_green = 0x28;
int counter = 9;

int main(void)
{
	RCC->AHB1ENR |=  4;
	GPIOC->MODER = 0x55550550; 
	GPIOC->PUPDR = 0x0000000A;
	button_1_counter = button_2_counter = pressed_1 = pressed_2 = counter_flag = ms = 0;
	
	while (1)
	{
		if((GPIOC->IDR  & 1) == 1 && pressed_1 == 0)
		{
			pressed_1 = 1;
		}
		
		else if((GPIOC->IDR & 2) == 2 && pressed_2 == 0)
		{
			pressed_2 = 1;
		}
		
		else if((GPIOC->IDR & 1) == 0 && pressed_1 != 0)
		{
			button_1_counter++;
			pressed_1 = 0;
		}
		else if((GPIOC->IDR & 2) == 0 && pressed_2 != 0)
		{
			button_2_counter++;
			pressed_2 = 0;
		}
		
		if (button_1_counter == 2)
		{
			button_1_counter = 0;
			red_green = 0x14;
			counter = 9;		
		}
		
		if (button_2_counter == 3)
			counter_flag = 1;
		
		
		if (button_2_counter == 4)
		{
			button_2_counter = 0;
			counter_flag = 0;
		}

		
		if(ms == 0 && counter_flag == 0)
		{
			GPIOC -> ODR = red_green;
			GPIOC -> ODR &= ~0xFF00;
			GPIOC -> ODR |= BCD_to_seven_segment_decoder(counter) << 8;
			
				if(counter == 0)
				{
					counter = 9;
					red_green = (red_green == 0x14) ? red_green << 1 : red_green >> 1;	
				}
				else
					counter--;
		}
		
		ms = ms < 999 ? ms + 1:  0;
		delayMs(1);
	}
}


int BCD_to_seven_segment_decoder(int digit)
{
	switch (digit)
	{
		case 0:
			return 0b0000001;
		case 1:
			return 0b1001111;
		case 2:
			return 0b0010010;
		case 3:
			return 0b0000110;
		case 4:
			return 0b1001100;
		case 5:
			return 0b0100100;
		case 6:
			return 0b0100000;
		case 7:
			return 0b0001111;
		case 8:
			return 0b0000000;
		case 9:
			return 0b0000100;
	}
	return 0;
}

void delayMs(int n)
{
	int i;
	for (; n > 0; n--)
		for (i = 0; i < 3195; i++)
			__NOP();
}
