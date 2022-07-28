/*
  system initialize for PIC32MX170F256B

  CPU clock 40MHz
   internal oscillator + PLL

  Peripheral clock 10MHz
*/

#include <xc.h>


// DEVCFG3
#pragma config PMDL1WAY = ON    // Peripheral Module Disable Configuration->Allow only one reconfiguration
#pragma config IOL1WAY = ON    // Peripheral Pin Select Configuration->Allow only one reconfiguration

// DEVCFG2
#pragma config FPLLIDIV = DIV_2    // PLL Input Divider->2x Divider
#pragma config FPLLMUL = MUL_20    // PLL Multiplier->20x Multiplier
#pragma config FPLLODIV = DIV_2    // System PLL Output Clock Divider->PLL Divide by 2

// DEVCFG1
#pragma config FNOSC = FRCPLL    // Oscillator Selection Bits->Fast RC Osc with PLL
#pragma config FSOSCEN = OFF    // Secondary Oscillator Enable->Disabled
#pragma config IESO = ON    // Internal/External Switch Over->Enabled
#pragma config POSCMOD = OFF    // Primary Oscillator Configuration->Primary osc disabled
#pragma config OSCIOFNC = OFF    // CLKO Output Signal Active on the OSCO Pin->Disabled
#pragma config FPBDIV = DIV_4    // Peripheral Clock Divisor->Pb_Clk is Sys_Clk/4
#pragma config FCKSM = CSDCMD    // Clock Switching and Monitor Selection->Clock Switch Disable, FSCM Disabled
#pragma config WDTPS = PS1048576    // Watchdog Timer Postscaler->1:1048576
#pragma config WINDIS = OFF    // Watchdog Timer Window Enable->Watchdog Timer is in Non-Window Mode
#pragma config FWDTEN = OFF    // Watchdog Timer Enable->WDT Disabled (SWDTEN Bit Controls)
#pragma config FWDTWINSZ = WINSZ_25    // Watchdog Timer Window Size->Window Size is 25%

// DEVCFG0
#pragma config DEBUG = OFF    // Background Debugger Enable->Debugger is Disabled
#pragma config JTAGEN = OFF    // JTAG Enable->JTAG Disabled
#pragma config ICESEL = ICS_PGx1    // ICE/ICD Comm Channel Select->Communicate on PGEC1/PGED1
#pragma config PWP = OFF    // Program Flash Write Protect->Disable
#pragma config BWP = OFF    // Boot Flash Write Protect bit->Protection Disabled
#pragma config CP = OFF    // Code Protect->Protection Disabled


/*!
  initialize I/O settings.
*/
void pin_init( void )
{
  // Setting the Output Latch SFR(s)
  LATA = 0x0000;
  LATB = 0x0000;

  // Setting the GPIO Direction SFR(s)
  TRISA = 0x001F & 0xfffc;	// A1,0:out for LED
  TRISB = 0xFFFF & 0xfffc;	// B1,0:out for LED

  // Setting the Weak Pull Up and Weak Pull Down SFR(s)
  CNPUA = 0x0000;
  CNPUB = 0x0000 | 0x0080;	// B7: pull-up for SW
  CNPDA = 0x0000;
  CNPDB = 0x0000;

  // Setting the Open Drain SFR(s)
  ODCA = 0x0000;
  ODCB = 0x0000;

  // Setting the Analog/Digital Configuration SFR(s)
  ANSELA = 0x0003 & 0x0000;	// all digital.
  ANSELB = 0xF00F & 0xc000;	// B14,15:analog
}


/*!
  initialize interrupt settings.
*/
void interrupt_init( void )
{
  // Enable the multi vector
  INTCONbits.MVEC = 1;
  // Enable Global Interrupts
  __builtin_mtc0(12,0,(__builtin_mfc0(12,0) | 0x0001));
}


/*!
  converts an I/O Pin number such as "A0" to an available OC number.

  @param  port	port such as A=1, B=2...
  @param  num	number 0..15
 */
int pin_to_oc_num( int port, int num )
{
  /* Output pin to OC number selection

     see Datasheet DS60001168L
         TABLE 11-2: OUTPUT PIN SELECTION
   */
  static const int8_t TBL_OC_NUM[3][16] = {
    /*      num  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 */
    /* PortA */ {1, 2, 4, 3, 4,-1,-1,-1, 2, 2,-1,-1,-1,-1,-1,-1},
    /* PortB */ {3, 2, 4, 1, 1, 2, 4, 1, 2, 3, 3, 2,-1, 4, 3, 1},
    /* PortC */ {1, 4, 3, 4, 3, 1, 4, 1, 2, 3,-1,-1,-1,-1,-1,-1},
  };

  return TBL_OC_NUM[ port-1 ][ num ];
}

/*!
  Assign PWM output pin.

  @param  port	port such as A=1, B=2...
  @param  num	number 0..15
  @param  oc_num OC number
*/
int assign_pwm_pin( int port, int num, int oc_num )
{
  static volatile uint32_t *RPxx[] = { &RPA0R, &RPB0R, &RPC0R };
  static const uint8_t OC_NUM[] = {5, 5, 5, 5, 6};

  RPxx[ port-1 ][ num ] = OC_NUM[ oc_num-1 ];

  return 0;
}
