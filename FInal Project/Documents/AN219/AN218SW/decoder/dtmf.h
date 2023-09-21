//-----------------------------------------------------------------------------
// Copyright 2004 Silicon Laboratories, Inc.
//
// FILE NAME      : dtmf.h
// TARGET DEVICE  : C8051F300
// CREATED ON     : 30.04.2004
// CREATED BY     : SYRO
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------
#define SYSCLK       24500000             // SYSCLK frequency in Hz
#define XTALCLK      32768                // XTALCLK frequency in Hz
#define BAUDRATE     9600                 // UART0 baud rate
#define SAMPLE_RATE  8000                 // Sample rate for ADC0 conversions
#define XMIN         20                   // Threshold value for the difference
                                          // between two succesive samples

#define EXT_OSC      0                    // Constants used by OSC_Change
#define INT_OSC      1                    // routine


//-----------------------------------------------------------------------------
//Structures and Types
//-----------------------------------------------------------------------------
struct ltype                              // In order to avoid truncation in the
{                                         // filter feedback stage, a long
    char space;                           // multiply and a divide are necessary.
    int intval;                           // This structure is used to avoid the
    char trunc;                           // long division.
};

typedef union ULONG                       // When the filter terms are calculated,
{                                         // the desired result ends up in the two
    long l;                               // middle bytes of the long variable.
    struct ltype hold;                    // Rather than divide by 256, the code
} ULONG;                                  // accesses the middle two bytes directly.


typedef union UINT                        // This type is used in much the same way
{                                         // as the previous one.  Rather than
    int i;                                // dividing the value to scale it down,
    char b[2];                            // the code accesses the high byte, which
} UINT;                                   // is equivalent to divide by 256.


typedef union LNG                         // Same as UINT but modified to access
{                                         // the high 16 bits of a long result.
   long l;
   int i[2];
} LNG;


//-----------------------------------------------------------------------------
// 16-bit SFR Definitions for 'F30x
//-----------------------------------------------------------------------------
sfr16 TMR2     = 0xcc;                    // Timer2 counter
sfr16 TMR2RL   = 0xca;                    // Timer2 reload value


//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------
char code dtmfchar[16] =                  // DTMF characters
{
    '1','2','3','A',
    '4','5','6','B',
    '7','8','9','C',
    '*','0','#','D'
};


char code display_codes[16] =             // Character codes for the 7-segment
{                                         // LED display
    0x60, 0xDA, 0xF2, 0xEE,
    0x66, 0xB6, 0xBE, 0x3E,
    0xE0, 0xFE, 0xF6, 0x9C,
    0x6E, 0xFC, 0x3A, 0x7A
};


code int coef1[2] = { 235, 437};          // Goertzel filter coefficients
code int coef2[2] = { 181, 421};
code int coef3[2] = { 118, 402};
code int coef4[2] = {  47, 378};
code int coef5[2] = {-165, 298};
code int coef6[2] = {-258, 255};
code int coef7[2] = {-349, 204};
code int coef8[2] = {-429, 146};


ULONG Qhold;                              // Temporary storage for fiter
                                          // calculations

ULONG Qthold;                             // Saves previous value for magnitude
                                          // calculations

UINT Q1[3];                               // These are the elements of the 8
UINT Q2[3];                               // Goertzel filters.  The filters are
UINT Q3[3];                               // 2 pole IIR filters and so require
UINT Q4[3];                               // 3 terms each.
UINT Q5[3];
UINT Q6[3];
UINT Q7[3];
UINT Q8[3];

idata UINT Qt1[2];                        // These 2 element arrays are used for
idata UINT Qt2[2];                        // storing the low two elements of the
idata UINT Qt3[2];                        // filter elements after N filter
idata UINT Qt4[2];                        // iterations.
idata UINT Qt5[2];
idata UINT Qt6[2];
idata UINT Qt7[2];
idata UINT Qt8[2];

idata int mag_squared1, mag_squared2;     // Store output of the Goertzel filters.
idata int mag_squared3, mag_squared4;
idata int mag_squared5, mag_squared6;
idata int mag_squared7, mag_squared8;

unsigned char TMR0RLH, TMR0RLL;           // Timer0 reload value

bit new_tone;                             // Flag for valid pause between tones

bit start_goertzel;                       // Flag for start of the decoding
                                          // process

bit gain_calc;                            // Flag for gain computing

bit done;                                 // Flag for starting character decoding

char base_freq;                           // Flag for base frequencies /
                                          // 2nd harmonics

int x;                                    // Current signal sample

int x_old;                                // Former signal sample

int high_x, low_x;                        // Minimum and maximum value of the
                                          // signal
int delta_x;

int gain;                                 // Gain computed in AGC block

unsigned char gain_cnt;                   // Gain stage sample counter

unsigned char sample_no;                  // Sample counter

unsigned char max_sample;                 // Total no. of samples for base freqs
                                          // and 2nd harmonics

unsigned char sig_present;                // For every frequency detected, a bit
                                          // in sig_present is set

unsigned char dtmf_index;                 // Index for dtmfchar array

unsigned char set1;                       // Hold the no. of freqs. detected
unsigned char set2;


unsigned char hour;
unsigned char minute;
unsigned char second;
unsigned char hundredth;

unsigned char temp_hour;
unsigned char temp_min;
unsigned char temp_sec;


//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------
void SYSCLK_Init(void);                   // System clock configuration

void PORT_Init(void);                     // I/O port configuration

void Timer0_Init(int counts);             // Timer0 configuration

void Timer2_Init(int counts);             // Timer2 configuration

void ADC0_Init(void);                     // ADC0 configuration

void CP0_Init(void);                      // Comparator0 configuration

void UART0_Init(void);                    // UART0 configuration

void PCA0_Init(void);                     // PCA0 configuration

void Interrupt_Init(void);                // Interrupt configuration

void DTMF_Detect(void);                   // Compute signal energy and
                                          // decode DTMF characters

void DTMF_Init(void);                     // Variable initializations

void Idle(void);                          // Switches to Idle Mode

void OSC_Change(char n);                  // Changes the system clock

void Display(unsigned char char_code);    // Displays a character on the
                                          // 7-segment LED display

void UART_display(unsigned char character);

void Delay(void);                         // Waits for 250us



//-----------------------------------------------------------------------------
// end of dtmf.h
//-----------------------------------------------------------------------------