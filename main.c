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
uint8_t t_count = 0;


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


static void c_pin_init(mrb_vm *vm, mrb_value *v, int argc) {
    pin_init();
}

void timer_init() {
    TMR1 = TMR2 = TMR3 = 0x0;
    PR1 = PR2 = 10000;
    PR3 = 10;
    T1CON = T2CON = T3CON = 0x8000;
}

int check_timeout(void)
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

/* mruby/c writer */

void __ISR(_TIMER_1_VECTOR, IPL1AUTO) _T1Interrupt (  ){
    mrbc_tick();
    IFS0CLR= 1 << _IFS0_T1IF_POSITION;
}

void __ISR(_TIMER_2_VECTOR, IPL2AUTO) _T2Interrupt (  ){
    t_count++;
    IFS0CLR = 1 << _IFS0_T2IF_POSITION;
}



int main(void)
{
    /* module init */
    pin_init();
    i2c_init();
    adc_init();
    uart_init();
    timer_init();

    /*Enable the interrupt*/
#if 0
    IPC9bits.U2IP = 4;
    IPC9bits.U2IS = 3;
    IFS1bits.U2RXIF = 0;
    IFS1bits.U2TXIF = 0;
    IEC1bits.U2RXIE = 1;
    IEC1bits.U2TXIE = 0;
#endif
    IEC0bits.T1IE = 1;
    IEC0bits.T2IE = 1;
    IPC6bits.FCEIP = 1;
    IPC6bits.FCEIS = 0;
    IPC1bits.T1IP = 1;
    IPC1bits.T1IS = 0;
    IPC2bits.T2IP = 2;
    IPC2bits.T2IS = 0;
    INTCONbits.MVEC = 1;
    __builtin_enable_interrupts();

    mrbc_print("\r\n\x1b(B\x1b)B\x1b[0m\x1b[2JRboard v*.*, mruby/c v3.1 start.\n");

    if (check_timeout()){
        /* IDE code */
        add_code();
	memset( memory_pool, 0, sizeof(memory_pool));
    };

    /* mruby/c */
    mrbc_init(memory_pool, MEMORY_SIZE);
    mrbc_define_method(0, mrbc_class_object, "pinInit", c_pin_init);
    mrbc_init_class_adc(0);
    mrbc_init_class_i2c(0);
    mrbc_init_class_uart(0);
    mrbc_init_class_digital(0);
    mrbc_init_class_pwm(0);
    mrbc_init_class_onboard(0);
    mrbc_init_class_spi(0);

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

    T1CONbits.ON = 1;
    mrbc_run();
    return 1;
}
