//-----------------------------------------------------------------------------
// Copyright 2004 Silicon Laboratories, Inc.
//
// FILE NAME      : dtmfgen.c
// TARGET DEVICE  : C8051F300
// CREATED ON     : 25.05.2004
// CREATED BY     : SYRO
//
// Revision 1.0
// This file contains the source code of the DTMF generator.


//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
//
#include "c8051f300.h"                      // SFR declarations
#include "dtmfgen.h"


//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------
//
void main(void)
{
    EA = 0;                                 // All interrupts disabled
    PCA0MD &= ~0x40;                        // Clear watchdog timer

    SYSCLK_Init();                          // Configure system clock
    PORT_Init();                            // Configure Crossbar and GPIO
    PCA0_Init();                            // Configure PCA0
    Timer0_Init(SYSCLK/12/50);              // Configure Timer0 to overlow
                                            // every 20ms
    Interrupt_Init();                       // Initialize interrupts

    while(1)
    {		
        if(valid_key)                       // If a key was pressed
        {
            TR0 = 0;                        // Stop Timer0
            CR = 1;                         // Start PCA0 counter
            valid_key = 0;						

			if(XBR0 & 0x01)					// If P0.0 is skipped
				XBR0 = 0x7E;				// assign it to CEX0 
        }
    }
}


//-----------------------------------------------------------------------------
// SYSCLK_Init
//-----------------------------------------------------------------------------
//
// This routine initializes the system clock to use the internal oscillator 
// as its clock source. Also enables missing clock detector and VDD monitor.
//
void SYSCLK_Init(void)
{
    OSCICN = 0x07;                        // Configure internal oscillator for
                                          // its highest frequency
    RSTSRC = 0x06;                        // Enable missing clock detector and
                                          // VDD monitor
}


//-----------------------------------------------------------------------------
// PORT_Init
//-----------------------------------------------------------------------------
//
// Configure the Crossbar and GPIO ports.
// P0.0 - PWM output
// P0.1 - Keypad row 0
// P0.2 - Keypad row 1
// P0.3 - Keypad row 2
// P0.4 - Keypad row 3
// P0.5 - Keypad column 0
// P0.6 - Keypad column 1
// P0.7 - Keypad column 2
//
void PORT_Init (void)
{
    XBR0     = 0x7F;                        // Skip all pins
    XBR1     = 0x40;					    // CEX0 selected
    XBR2     = 0x40;                        // Enable crossbar and weak pull-ups

    P0MDIN   = ~0x00;	                    // No analog inputs
    P0MDOUT  = 0x01;                        // P0.0 set as push-pull output 

	PWM = 0;								// Hold P0.0 low
}


//-----------------------------------------------------------------------------
// PCA0_Init
//-----------------------------------------------------------------------------
//
// PCA0 configuration routine.
//
void PCA0_Init(void)
{
    PCA0MD = 0x89;                          // PCA0 clock source is system clock
                                            // Enable PCA0 overflow interrupt

    PCA0CPM0 = 0x4A;                        // Comparator function enabled
                                            // Match function enabled
                                            // 8-bit PWM enabled

    PCA0H = 0xFF;                           // Load PCA0 counter MSB
    PCA0L = 0x00;

	PCA0CPH0 = 0xFF;
	PCA0CPL0 = 0xFF;

	CR = 0;
}


//-----------------------------------------------------------------------------
// Timer0_Init
//-----------------------------------------------------------------------------
//
// Timer0 configuration routine.
//
void Timer0_Init(int counts)
{
    CKCON &= ~0x0B;                         // Timer0 clock source = SYSCLK/12

    TMOD &= ~0x0E;                          // Timer0 in 16 bit mode
	TMOD |= 0x01;

	TH0 = -counts >> 8;
	TL0 = -counts & 0xFF;
    TMR0RLH = TH0;
    TMR0RLL = TL0;

    TR0 = 1;
}


//-----------------------------------------------------------------------------
// Interrupt_Init
//-----------------------------------------------------------------------------
//
// Enables the interrupts and sets their priorities
//
void Interrupt_Init(void)
{
    IE = 0;                                 // All interrupts disabled
    EIE1 = 0;

    PT0 = 0;                                // Timer0 interrupt low priority
	ET0 = 1;                                // Timer0 interrupt enable

    EIP1 |= 0x08;                           // PCA0 interrupt high priority
    EIE1 |= 0x08;                           // PCA0 interrupt enable

    EA = 1;                                 // Enable interrupts
}


//-----------------------------------------------------------------------------
// PCA0 Interrupt Service Routine (ISR)
//-----------------------------------------------------------------------------
//
// PCA0 is configured to overflow at about 96KHz. The ISR updates the compare
// registers with values from the sine lookup table. At a 8kHz rate the values
// of the sine table indexes are updated accordingly with the key pressed.
// The routine also checks that a tone is generated for 100ms.
//
void PCA0_ISR(void) interrupt 9
{
    if(tone_duration >= 800)          		// If the tone was generated for
    {                                 		// 100ms

        CR = 0;                         	// Stop PCA0 counter
        TR0 = 1;                        	// Start Timer0
    	tone_duration = 0;
	}
	else
	{
	    // Load the PCA0 compare register MSB with the sum of the two DTMF 
    	// frequencies
	    PCA0CPH0 = sine_table[low_index] + sine_table[high_index];
	}

    if(++pwm_counter >= 12)                
    {
        // At a 8kHz rate, update the values of low_index and high_index
        low_index += increment[row];
        high_index += increment[col + 4];
    
        if(low_index >= SAMPLE_NO)
            low_index -= SAMPLE_NO;

        if(high_index >= SAMPLE_NO)
            high_index -= SAMPLE_NO;

        if(++tone_duration >= 800)
			PCA0CPL0 = 0xFF;

        pwm_counter = 0;
    }

    PCA0H = 0xFF;                           // Reload PCA0 counter MSB

    CF = 0;                                 // Clear interrupt flag
}


//-----------------------------------------------------------------------------
// Timer0 Interrupt Service Routine (ISR)
//-----------------------------------------------------------------------------
//
// Timer0 ISR is used to scan the matrix keypad once every 20ms. A zero is sent
// to one of the keypad rows, and the columns are read. If a valid keypress is
// detected, row and col are set properly.
//
void Timer0_ISR (void) interrupt 1
{
    TR0 = 0;                                // Stop Timer0
    TH0 = TMR0RLH;                          // Reload Timer0 counter
    TL0 = TMR0RLL;
    TR0 = 1;                                // Start Timer0

    P0 = ~(0x02 << row);                    // Send a 0 to one of the keypad
                                            // rows
	PWM = 0;

    while(TH0 < 0xC0);                      // Wait about 10ms

    col_val = P0 & 0xE0;                    // Read keypad columns

    switch(col_val)
    {
        case 0xC0:                          // Column 0
            while(C0==0);                   // Wait the key to be released
            col = 0;
            valid_key = 1;
            break;
        case 0xA0:                          // Column 1
            while(C1==0);                   // Wait the key to be released        
            col = 1;
            valid_key = 1;
            break;
        case 0x60:                          // Column 2
            while(C2==0);                   // Wait the key to be released
            col = 2;
            valid_key = 1;
            break;    
    }

    if(valid_key == 0)                      // A key was pressed
    {
        row++;                              // Go to next keypad row
        if(row >= 4)
            row = 0;
    }
}


//-----------------------------------------------------------------------------
// end of dtmfgen.c
//-----------------------------------------------------------------------------