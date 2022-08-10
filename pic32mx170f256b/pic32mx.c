/*
  system initialize for PIC32MX170F256B

  CPU clock 40MHz
   internal oscillator + PLL

  Peripheral clock 10MHz
*/

#include <xc.h>
#include "pic32mx.h"


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


/*! Input pin selection table.

  see Datasheet DS60001168L
  TABLE 11-1: INPUT PIN SELECTION
*/
static const struct {
  uint8_t port;
  uint8_t num;
} TBL_INPUT_PIN_SELECTION[4][5] = {
  {
    { 1, 0 },	// 0000 = RPA0
    { 2, 3 },	// 0001 = RPB3
    { 2, 4 },	// 0010 = RPB4
    { 2,15 },	// 0011 = RPB15
    { 2, 7 },	// 0100 = RPB7
  },{
    { 1, 1 },	// 0000 = RPA1
    { 2, 5 },	// 0001 = RPB5
    { 2, 1 },	// 0010 = RPB1
    { 2,11 },	// 0011 = RPB11
    { 2, 8 },	// 0100 = RPB8
  },{
    { 1, 2 },	// 0000 = RPA2
    { 2, 6 },	// 0001 = RPB6
    { 1, 4 },	// 0010 = RPA4
    { 2,13 },	// 0011 = RPB13
    { 2, 2 },	// 0100 = RPB2
  },{
    { 1, 3 },	// 0000 = RPA3
    { 2,14 },	// 0001 = RPB14
    { 2, 0 },	// 0010 = RPB0
    { 2,10 },	// 0011 = RPB10
    { 2, 9 },	// 0100 = RPB9
  }
};
static const int NUM_OF_TBL_INPUT_PIN_SELECTION = sizeof(TBL_INPUT_PIN_SELECTION[0]) / sizeof(TBL_INPUT_PIN_SELECTION[0][0]);


/*! Output pin selection table.

  see Datasheet DS60001168L
  TABLE 11-2: OUTPUT PIN SELECTION
*/
static const struct {
  uint8_t port;
  uint8_t num;
} TBL_OUTPUT_PIN_SELECTION[4][5] = {
  {
    { 1, 0 },	// RPA0
    { 2, 3 },	// RPB3
    { 2, 4 },	// RBP4
    { 2,15 },	// RBP15
    { 2, 7 },	// RPB7
  },{
    { 1, 1 },	// RPA1
    { 2, 5 },	// RPB5
    { 2, 1 },	// RPB1
    { 2,11 },	// RPB11
    { 2, 8 },	// RPB8
  },{
    { 1, 2 },	// RPA2
    { 2, 6 },	// RPB6
    { 1, 4 },	// RPA4
    { 2,13 },	// RPB13
    { 2, 2 },	// RPB2
  },{
    { 1, 3 },	// RPA3
    { 2,14 },	// RPB14
    { 2, 0 },	// RPB0
    { 2,10 },	// RPB10
    { 2, 9 },	// RPB9
  }
};
static const int NUM_OF_TBL_OUTPUT_PIN_SELECTION = sizeof(TBL_OUTPUT_PIN_SELECTION[0]) / sizeof(TBL_OUTPUT_PIN_SELECTION[0][0]);



volatile uint32_t *TBL_RPxnR[] = { &RPA0R, &RPB0R, &RPC0R };


/*!
  initialize I/O settings.

  方針: ハード的に用途が決定しているもの (LED, SW2, UART1) 以外は、
  　　　全デジタル入力、内部プルダウンとしておく。
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
  CNPDA = 0x0000 | 0x000c;	// ignore A0,A1,A4(UART)
  CNPDB = 0x0000 | 0xff6c;	// ignore B0,B1,B4(UART),B7

  // Setting the Open Drain SFR(s)
  ODCA = 0x0000;
  ODCB = 0x0000;

  // Setting the Analog/Digital Configuration SFR(s)
  ANSELA = 0x0003 & 0x0000;	// all digital.
  ANSELB = 0xF00F & 0x0000;
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
  assign the pin to digital input mode

  @param  port	port such as A=1, B=2...
  @param  num	number 0..15
  @return	dummy
*/
int set_pin_to_digital_input( int port, int num )
{
  ANSELxCLR(port) = (1 << num);
  TRISxSET(port) = (1 << num);
  CNPUxCLR(port) = (1 << num);
  CNPDxCLR(port) = (1 << num);

  return 0;
}


/*!
  assign the pin to digital output mode

  @param  port	port such as A=1, B=2...
  @param  num	number 0..15
  @return	dummy
*/
int set_pin_to_digital_output( int port, int num )
{
  ANSELxCLR(port) = (1 << num);
  TRISxCLR(port) = (1 << num);
  CNPUxCLR(port) = (1 << num);
  CNPDxCLR(port) = (1 << num);

  return 0;
}


/*!
  assign the pin to analog input mode

  @param  port	port such as A=1, B=2...
  @param  num	number 0..15
  @return	ADC cnannel number or -1 is error.
*/
int set_pin_to_analog_input( int port, int num )
{
  /* Output pin to  number selection
     see Datasheet DS60001168L  TABLE 3
   */
  static const int8_t TBL_AN[3][16] = {
    /*      num   0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 */
    /* PortA */ { 0, 1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
    /* PortB */ { 2, 3, 4, 5,-1,-1,-1,-1,-1,-1,-1,-1,12,11,10, 9},
    /* PortC */ {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
  };

  int ch = TBL_AN[ port-1 ][ num ];
  if( ch < 0 ) return ch;

  ANSELxSET(port) = (1 << num);
  TRISxSET(port) = (1 << num);
  CNPUxCLR(port) = (1 << num);
  CNPDxCLR(port) = (1 << num);

  return ch;
}


/*!
  assign the pin to pwm output mode.

  @param  port	port such as A=1, B=2...
  @param  num	number 0..15
  @return	OC(PWM) unit number or -1 is error.
*/
int set_pin_to_pwm( int port, int num )
{
  /* Output pin to OC number selection

     see Datasheet DS60001168L
         TABLE 11-2: OUTPUT PIN SELECTION
   */
  static const int8_t TBL_OC[3][16] = {
    /*      num  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 */
    /* PortA */ {1, 2, 4, 3, 4,-1,-1,-1, 2, 2,-1,-1,-1,-1,-1,-1},
    /* PortB */ {3, 2, 4, 1, 1, 2, 4, 1, 2, 3, 3, 2,-1, 4, 3, 1},
    /* PortC */ {1, 4, 3, 4, 3, 1, 4, 1, 2, 3,-1,-1,-1,-1,-1,-1},
  };

  if( port < 1 || port > 3 ) return -1;
  if( num > 15 ) return -1;

  int oc_num = TBL_OC[ port-1 ][ num ];

  // set pin to output mode and assign to pwm.
  set_pin_to_digital_output( port, num );

  static const uint8_t OC_NUM[] = {5, 5, 5, 5, 6};
  RPxnR(port, num) = OC_NUM[ oc_num-1 ];

  return oc_num;
}


/*!
  assign the pin to SPI

  @param  unit	SPI unit number 1 or 2
  @param  sdi_p	SDI port such as A=1, B=2...
  @param  sdi_n SDI port number 0..15
  @param  sdo_p	SDO port
  @param  sdo_n SDO port number
  @param  sck_p	SCK port
  @param  sck_n SCK port number
  @return	minus value is error.
*/
int set_pin_to_spi( int unit, int sdi_p, int sdi_n, int sdo_p, int sdo_n, int sck_p, int sck_n )
{
  if( unit < 1 || unit > 2 ) return -1;		// error return.

  /*
    Check argument. SDI pin.
  */
  static const int TBLIDX_SDI[] = {1, 2};
  int unit_idx = TBLIDX_SDI[unit-1];
  int i;
  for( i = 0; i < NUM_OF_TBL_INPUT_PIN_SELECTION; i++ ) {
    if(	sdi_p == TBL_INPUT_PIN_SELECTION[unit-1][i].port &&
	sdi_n == TBL_INPUT_PIN_SELECTION[unit-1][i].num ) {
      break;
    }
  }
  if( i == NUM_OF_TBL_INPUT_PIN_SELECTION ) return -1;		// error return
  int set_val_SDIxR = i;


  /*
    Check argument. SDO pin.

    see Datasheet DS60001168L
         TABLE 11-2: OUTPUT PIN SELECTION
  */
  static const struct {
    uint8_t port;
    uint8_t num;
  } TBL_OUTPUT_PIN_SELECTION[] = {
    { 1,  1 },	// RPA1
    { 2,  5 },	// RPB5
    { 2,  1 },	// RPB1
    { 2, 11 },	// RPB11
    { 2,  8 },	// RPB8
    { 1,  2 },	// RPA2
    { 2,  6 },	// RPB6
//  { 1,  4 },	// RPA4  confrict UART1 for console
    { 2, 13 },	// RPB13
    { 2,  2 },	// RPB2
  };
  static const int NUM_OF_TBL_OUTPUT_PIN_SELECTION = sizeof(TBL_OUTPUT_PIN_SELECTION) / sizeof(TBL_OUTPUT_PIN_SELECTION[0]);

  for( i = 0; i < NUM_OF_TBL_OUTPUT_PIN_SELECTION; i++ ) {
    if( sdo_p == TBL_OUTPUT_PIN_SELECTION[i].port &&
	sdo_n == TBL_OUTPUT_PIN_SELECTION[i].num ) break;
  }
  if( i == NUM_OF_TBL_OUTPUT_PIN_SELECTION ) return -2;		// error return

  /*
    Check argument. SCK pin.
  */
  static const struct {
    uint8_t unit;
    uint8_t port;
    uint8_t num;
  } TBL_SCK_PIN_SELECTION[] = {
    { 1, 2, 14 },  // SPI1: B14
    { 2, 2, 15 },  // SPI2: B15
  };
  static const int NUM_OF_TBL_SCK_PIN_SELECTION = sizeof(TBL_SCK_PIN_SELECTION) / sizeof(TBL_SCK_PIN_SELECTION[0]);

  for( i = 0; i < NUM_OF_TBL_SCK_PIN_SELECTION; i++ ) {
    if( unit  == TBL_SCK_PIN_SELECTION[i].unit &&
	sck_p == TBL_SCK_PIN_SELECTION[i].port &&
	sck_n == TBL_SCK_PIN_SELECTION[i].num ) break;
  }
  if( i == NUM_OF_TBL_SCK_PIN_SELECTION ) return -3;		// error return


  // assign pins.
  // SDI
  set_pin_to_digital_input( sdi_p, sdi_n );
  SDIxR(unit) = set_val_SDIxR;

  // SDO
  set_pin_to_digital_output( sdo_p, sdo_n );
  RPxnR(sdo_p-1, sdo_n) = unit + 2;	// SDO1=0x03 or SDO2=0x04

  // SCK
  set_pin_to_digital_output( sck_p, sck_n );

  return 0;
}


/*!
  assign the pin to UART

  @param  unit	UART unit number 1 or 2
  @param  txd_p	TxD port such as A=1, B=2...
  @param  txd_n TxD port number 0..15
  @param  rxd_p	RxD port
  @param  rxd_n RxD port number
  @return	minus value is error.
*/
int set_pin_to_uart( int unit, int txd_p, int txd_n, int rxd_p, int rxd_n )
{
  if( unit < 1 || unit > 2 ) return -1;		// error return.

  /*
    Check argument. TxD pin.

    DS60001168L  TABLE 11-2: OUTPUT PIN SELECTION
  */
  static const struct {
    uint8_t group;
    uint8_t set_val;
  } TBL_UxTX[/*unit*/] = {
    { 0, 0x01 },  // 0001 = U1TX
    { 3, 0x02 },  // 0010 = U2TX
  };

  int group = TBL_UxTX[unit-1].group;
  int i;
  for( i = 0; i < NUM_OF_TBL_OUTPUT_PIN_SELECTION; i++ ) {
    if( txd_p == TBL_OUTPUT_PIN_SELECTION[group][i].port &&
	txd_n == TBL_OUTPUT_PIN_SELECTION[group][i].num ) break;
  }
  if( i == NUM_OF_TBL_OUTPUT_PIN_SELECTION ) return -2;		// error return
  // int set_val_RPxnR = TBL_UxTX[unit-1].set_val;

  /*
    Check argument. RxD pin.

    DS60001168L  TABLE 11-1: INPUT PIN SELECTION
  */
  static const uint8_t TBL_UxRX[/*unit*/] = {
    2,	// U1RX
    1,	// U2RX
  };

  group = TBL_UxRX[unit-1];
  for( i = 0; i < NUM_OF_TBL_INPUT_PIN_SELECTION; i++ ) {
    if(	rxd_p == TBL_INPUT_PIN_SELECTION[group][i].port &&
	rxd_n == TBL_INPUT_PIN_SELECTION[group][i].num ) break;
  }
  if( i == NUM_OF_TBL_INPUT_PIN_SELECTION ) return -3;		// error return

  /* set output (TxD) pin.
     (note)
     Defaults to high level to keep high level when pin assignment is changed.
  */
  set_pin_to_digital_output( txd_p, txd_n );
  LATxSET( txd_p ) = (1 << txd_n);		// set high.
  RPxnR( txd_p, txd_n ) = TBL_UxTX[unit-1].set_val;

  // set input (RxD) pin.
  set_pin_to_digital_input( rxd_p, rxd_n );
  UxRXR(unit) = i;

  return 0;
}


/*!
  release assign the pin from peripheral.

  @param  port	port such as A=1, B=2...
  @param  num	number 0..15
  @return	-1 if error.
*/
int release_pin_from_peripheral( int port, int num )
{
  if( port < 1 || port > 3 ) return -1;
  if( num < 0 || num > 15 ) return -1;

  RPxnR( port, num ) = 0;

  return 0;
}
