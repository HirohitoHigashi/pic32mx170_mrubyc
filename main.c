/* ************************************************************************** */
/** main

  @Company
    ShimaneJohoshoriCenter.inc

  @File Name
    main.c

  @Summary
    mruby/c firmware for Rboard.

  @Description
    main routine.
 */
/* ************************************************************************** */

#include <xc.h>
#include <sys/attribs.h>
#include <string.h>

#include "pic32mx.h"
#include "uart.h"
#include "i2c.h"
#include "adc.h"
#include "spi.h"
#include "digital.h"

#include "mrubyc.h"
#include "mrbc_firm.h"

#include "pic32mx.c"


#define MEMORY_SIZE (1024*40)
uint8_t memory_pool[MEMORY_SIZE];


/*
  HAL functions.
*/
int hal_write(int fd, const void *buf, int nbytes) {
  return uart_write(&uart1_handle, buf, nbytes );
}

int hal_flush(int fd) {
    return 0;
}

void hal_abort( const char *s )
{
  if( s ) {
    hal_write( 0, s, strlen(s) );
  }
  __delay_ms(5000);
  system_reset();
}

void _mon_putc( char c )
{
  uart_write(&uart1_handle, &c, 1);
}


/*
  Timer functions.

  using Timer1
        16bit modes.
	1kHz (1ms/cycle)
  see   DS60001105G
        14.3.4.2 16-BIT SYNCHRONOUS COUNTER INITIALIZATION STEPS
*/
static void tick_timer_init( void )
{
  T1CON = 0;		// count PBCLK 1:1
  TMR1 = 0;
  PR1 = PBCLK / 1000;

  IFS0CLR= 1 << _IFS0_T1IF_POSITION;	// Clear interrupt.
  IPC1bits.T1IP = 1;	// Interrupt priority.
  IPC1bits.T1IS = 0;
  IEC0bits.T1IE = 1;	// Enable interrupt
  T1CONbits.ON = 1;
}

// Timer1 interrupt handler.
void __ISR(_TIMER_1_VECTOR, IPL1AUTO) timer1_isr( void )
{
  mrbc_tick();
  IFS0CLR = (1 << _IFS0_T1IF_POSITION);
}


/*!
  Choose to enter programming mode or run mode.
*/
static int check_timeout( void )
{
  int i;
  for( i = 0; i < 50; i++ ) {
    LATAbits.LATA0 = 1;
    __delay_ms( 30 );
    LATAbits.LATA0 = 0;
    __delay_ms( 30 );
    if( uart_can_read_line( &uart1_handle ) ) return 1;
  }
  return 0;
}


/*!
  initialize pin assign.
*/
static void c_pin_init(mrb_vm *vm, mrb_value *v, int argc)
{
  pin_init();
}


/*!
  main
*/
int main(void)
{
  /* module init */
  pin_init();
  interrupt_init();
  uart_init();
  tick_timer_init();

  __delay_ms(1000);
  static const char ST_MSG[] = "\r\n\x1b(B\x1b)B\x1b[0m\x1b[2JRboard v*.*, mruby/c v3.1 start.\n";
  hal_write( 0, ST_MSG, sizeof(ST_MSG) );


  if (check_timeout()){
    add_code();
    //    memset( memory_pool, 0, sizeof(memory_pool));	// TODO
  };


  /* init mruby/c VM */
  mrbc_init(memory_pool, MEMORY_SIZE);
  mrbc_define_method(0, mrbc_class_object, "pinInit", c_pin_init);
  mrbc_init_class_adc(0);
  mrbc_init_class_i2c(0);
  mrbc_init_class_uart(0);
  mrbc_init_class_digital(0);
  mrbc_init_class_pwm(0);
  mrbc_init_class_onboard(0);
  mrbc_init_class_spi(0);

  /* load the ruby programs on flash memory. */
  const uint8_t *fl_addr = (uint8_t*)FLASH_SAVE_ADDR;
  static const char RITE[4] = "RITE";
  while( strncmp( fl_addr, RITE, sizeof(RITE)) == 0 ) {
    mrbc_create_task(fl_addr, 0);

    // get a next irep.
    uint32_t size = 0;
    int i;
    for( i = 0; i < 4; i++ ) {
      size = (size << 8) | fl_addr[8 + i];
    }
    fl_addr += ALIGN_ROW_SIZE( size );
  }

  /* and run! */
  mrbc_run();

#if 0
  while(1) {
    LATAbits.LATA0 = 1;
    __delay_ms( 1000 );
    LATAbits.LATA0 = 0;
    __delay_ms( 1000 );
  }
#endif
  return 1;
}
