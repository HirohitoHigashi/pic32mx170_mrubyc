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



static volatile uint32_t *RPxx[] = { &RPA0R, &RPB0R, &RPC0R };


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
  assign the pin to pwm output mode.

  @param  port	port such as A=1, B=2...
  @param  num	number 0..15
  @return	OC(PWM) unit number or -1 is error.
*/
int set_pin_for_pwm( int port, int num )
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

  // set pin to output mode.
  volatile uint32_t *ansel_x_clr = &ANSELACLR;
  volatile uint32_t *tris_x_clr = &TRISACLR;

  ansel_x_clr[ OFS_PORT(port) ] = (1 << num);
  tris_x_clr[ OFS_PORT(port) ] = (1 << num);

  // assign pin to pwm output.
  static const uint8_t OC_NUM[] = {5, 5, 5, 5, 6};

  RPxx[ port-1 ][ num ] = OC_NUM[ oc_num-1 ];

  return oc_num;
}


/*!
  assign the pin to ADC input mode.

  @param  port	port such as A=1, B=2...
  @param  num	number 0..15
  @return	ADC cnannel number or -1 is error.
 */
int set_pin_for_adc( int port, int num )
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

  // set pin to analog input mode.
  volatile uint32_t *ansel_x_set = &ANSELASET;
  volatile uint32_t *tris_x_set = &TRISASET;

  ansel_x_set[ OFS_PORT(port) ] = (1 << num);
  tris_x_set[ OFS_PORT(port) ] = (1 << num);

  return ch;
}


/*!
  assign the pin to SPI

  @param  unit	SPI unit number
  @param  sdi_p	SDI port such as A=1, B=2...
  @param  sdi_n SDI port number 0..15
  @param  sdo_p	SDI port
  @param  sdo_n SDI port number
  @param  sck_p	SDI port
  @param  sck_n SDI port number
  @return	minus value is error.
*/
int set_pin_for_spi( int unit, int sdi_p, int sdi_n, int sdo_p, int sdo_n, int sck_p, int sck_n )
{
  /*
    Check argument. unit and SDI pin.

    see Datasheet DS60001168L
         TABLE 11-1: INPUT PIN SELECTION
   */
  static const struct {
    uint8_t unit;	// 1..2
    uint8_t port;	// A=1, B=2,...
    uint8_t num;	// 0..15
    uint8_t sel_val;
  } TBL_INPUT_PIN_SELECTION[] = {
    {1, 1,  1, 0x00},	// A1 = 0000
    {1, 2,  5, 0x01},	// B5 = 0001
    {1, 2,  1, 0x02},	// B1 = 0010
    {1, 2, 11, 0x03},	// B11= 0011
    {1, 2,  8, 0x04},	// B8 = 0100

    {2, 1,  2, 0x00},	// A2 = 0000
    {2, 2,  6, 0x01},	// B6 = 0001
    {2, 1,  4, 0x02},	// A4 = 0010
    {2, 2, 13, 0x03},	// B13= 0011
    {2, 2,  2, 0x04},	// B2 = 0100
  };
  static const int NUM_OF_TBL_INPUT_PIN_SELECTION = sizeof(TBL_INPUT_PIN_SELECTION) / sizeof(TBL_INPUT_PIN_SELECTION[0]);

  int sel_val_SDIxR;
  int i;
  for( i = 0; i < NUM_OF_TBL_INPUT_PIN_SELECTION; i++ ) {
    if( unit  == TBL_INPUT_PIN_SELECTION[i].unit &&
	sdi_p == TBL_INPUT_PIN_SELECTION[i].port &&
	sdi_n == TBL_INPUT_PIN_SELECTION[i].num ) {
      sel_val_SDIxR = TBL_INPUT_PIN_SELECTION[i].sel_val;
      break;
    }
  }
  if( i == NUM_OF_TBL_INPUT_PIN_SELECTION ) return -1;		// error return

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
  volatile uint32_t *tris_x_set = &TRISASET;
  volatile uint32_t *tris_x_clr = &TRISACLR;
  volatile uint32_t *ansel_x_clr = &ANSELACLR;

  // SDI
  tris_x_set[ OFS_PORT(sdi_p) ] = (1 << sdi_n);
  ansel_x_clr[ OFS_PORT(sdi_p) ] = (1 << sdi_n);
  if( unit == 1 ) {
    SDI1R = sel_val_SDIxR;
  } else {
    SDI2R = sel_val_SDIxR;
  }

  // SDO
  tris_x_clr[ OFS_PORT(sdo_p) ] = (1 << sdo_n);
  ansel_x_clr[ OFS_PORT(sdo_p) ] = (1 << sdo_n);
  RPxx[ sdo_p-1 ][ sdo_n ] = unit + 2;	// SDO1=0x03 or SDO2=0x04

  // SCK
  tris_x_clr[ OFS_PORT(sck_p) ] = (1 << sck_n);
  ansel_x_clr[ OFS_PORT(sck_p) ] = (1 << sck_n);

  return 0;
}
