/* ************************************************************************** */
/** DIGITAL

  @Company
    ShimaneJohoshoriCenter.inc

  @File Name
    digital.c

  @Summary
    Digital & PWM processing

  @Description
    mruby/c function army

    以下のレジスタが、番号ごとに一定番地離れている事に依存しています。
      * GPIO 関連のレジスタが、ポートごとに 0x100番地
      * OC(PWM) 関連のレジスタが、0x200番地
    そうでないプロセッサに対応が必要になった場合は、戦略の変更が必要です。
 */
/* ************************************************************************** */

#include <stdint.h>
#include "pic32mx.h"
#include "digital.h"

//! get offset of port address.
#define OFS_PORT(n)	(0x100 / sizeof(uint32_t) * ((n) - 1))
#define OFS_OC(n)	(0x200 / sizeof(uint32_t) * ((n) - 1))

/* ================================ C codes ================================ */


static int set_gpio_handle( GPIO_HANDLE *h, mrbc_value v[] )
{
  if( v[1].tt == MRBC_TT_INTEGER ) {
    int ch = mrbc_integer(v[1]);
    if( ch <= 4 ) {
      h->port = 1;
      h->num = ch;
    } else {
      h->port = 2;
      h->num = ch-5;
    }

  } else if( v[1].tt == MRBC_TT_STRING ) {
    const char *s = mrbc_string_cstr(&v[1]);
    if( 'A' <= s[0] && s[0] <= 'G' ) {
      h->port = s[0] - 'A' + 1;
    } else if( 'a' <= s[0] && s[0] <= 'g' ) {
      h->port = s[0] - 'a' + 1;
    } else {
      return -1;
    }

    h->num = mrbc_atoi( s+1, 10 );

  } else {
    return -1;
  }

  return -(h->num < 0 || h->num > 15);
}

/* ============================= mruby/c codes ============================= */

/*! control the onboard LEDs

  leds_write( n )  # n = 0 to 0b1111 (bit mapped)
*/
static void c_leds_write(mrb_vm *vm, mrbc_value v[], int argc)
{
  int led = GET_INT_ARG(1);
  LATAbits.LATA0 = led & 0x01;
  led >>= 1;
  LATAbits.LATA1 = led & 0x01;
  led >>= 1;
  LATBbits.LATB0 = led & 0x01;
  led >>= 1;
  LATBbits.LATB1 = led & 0x01;
}

/*! read the onboard switch

  x = sw()
*/
static void c_sw(mrb_vm *vm, mrbc_value v[], int argc)
{
  SET_INT_RETURN( PORTBbits.RB7 );
}


/*! GPIO constructor

  $gpio_x = GPIO.new( num )  # num = pin number of Rboard.
  $gpio_x = GPIO.new("A0")   # PIC origined pin assignment.
*/
static void c_gpio_new(mrb_vm *vm, mrbc_value v[], int argc)
{
  if( argc == 0 ) goto ERROR_RETURN;

  mrbc_value val = mrbc_instance_new( vm, v->cls, sizeof(GPIO_HANDLE) );
  GPIO_HANDLE *h = (GPIO_HANDLE*)val.instance->data;

  if( set_gpio_handle( h, v ) != 0 ) goto ERROR_RETURN;
  SET_RETURN( val );
  return;


 ERROR_RETURN:
  mrbc_raise(vm, MRBC_CLASS(ArgumentError),0);
}


/*! GPIO setmode

  $gpio_x.setmode( 0 )	# set output
  $gpio_x.setmode( 1 )  # set input
*/
static void c_gpio_setmode(mrb_vm *vm, mrbc_value v[], int argc)
{
  GPIO_HANDLE *h = (GPIO_HANDLE *)v->instance->data;
  volatile uint32_t *ansel_x_clr = &ANSELACLR;
  volatile uint32_t *tris_x = GET_INT_ARG(1) ? &TRISASET : &TRISACLR;

  ansel_x_clr[ OFS_PORT(h->port) ] = (1 << h->num);
  tris_x[ OFS_PORT(h->port) ] = (1 << h->num);
}


/*! GPIO write

  $gpio_x.write( 0 or 1 )
*/
static void c_gpio_write(mrb_vm *vm, mrbc_value v[], int argc)
{
  GPIO_HANDLE *h = (GPIO_HANDLE *)v->instance->data;
  volatile uint32_t *lat_x = GET_INT_ARG(1) ? &LATASET : &LATACLR;

  lat_x[ OFS_PORT(h->port) ] = (1 << h->num);
}


/*! GPIO read

  x = $gpio_x.read()
*/
static void c_gpio_read(mrb_vm *vm, mrbc_value v[], int argc)
{
  GPIO_HANDLE *h = (GPIO_HANDLE *)v->instance->data;
  volatile uint32_t *port_x = &PORTA;

  SET_INT_RETURN( (port_x[ OFS_PORT(h->port) ] >> h->num) & 1 );
}


/*! GPIO internal pull up or pull down.

  (note) Rboard original function.

  $gpio_x.pull( 1 )   # pull up
  $gpio_x.pull( -1 )  # pull down
  $gpio_x.pull( 0 )   # cancel
*/
static void c_gpio_pull(mrb_vm *vm, mrbc_value v[], int argc)
{
  GPIO_HANDLE *h = (GPIO_HANDLE *)v->instance->data;
  int mode = GET_INT_ARG(1);

  if( mode > 0) {
    // pull up.
    volatile uint32_t *cnpu_x_set = &CNPUASET;
    cnpu_x_set[ OFS_PORT(h->port) ] = (1 << h->num);

  } else if( mode < 0 ) {
    // pull down.
    volatile uint32_t *cnpd_x_set = &CNPDASET;
    cnpd_x_set[ OFS_PORT(h->port) ] = (1 << h->num);

  } else {
    // cancel.
    volatile uint32_t *cnpu_x_clr = &CNPUACLR;
    volatile uint32_t *cnpd_x_clr = &CNPDACLR;

    cnpu_x_clr[ OFS_PORT(h->port) ] = (1 << h->num);
    cnpd_x_clr[ OFS_PORT(h->port) ] = (1 << h->num);
  }
}


/*! PWM constructor

  $pwm = PWM.new( num )	 # num = pin number of Rboard.
  $pwm = PWM.new("A0")	 # PIC origined pin assignment.
*/
static void c_pwm_new(mrb_vm *vm, mrbc_value v[], int argc)
{
  if( argc == 0 ) goto ERROR_RETURN;

  mrbc_value val = mrbc_instance_new( vm, v->cls, sizeof(GPIO_HANDLE) );
  GPIO_HANDLE *h = (GPIO_HANDLE*)val.instance->data;

  if( set_gpio_handle( h, v ) != 0 ) goto ERROR_RETURN;
  int oc_num = pin_to_oc_num( h->port, h->num );
  if( oc_num < 0 ) goto ERROR_RETURN;

  // set pin to output mode.
  volatile uint32_t *ansel_x_clr = &ANSELACLR;
  volatile uint32_t *tris_x_clr = &TRISACLR;

  ansel_x_clr[ OFS_PORT(h->port) ] = (1 << h->num);
  tris_x_clr[ OFS_PORT(h->port) ] = (1 << h->num);

  assign_pwm_pin( h->port, h->num, oc_num );

  // set OC module
  volatile uint32_t *oc_x_con = &OC1CON;
  volatile uint32_t *oc_x_r = &OC1R;
  volatile uint32_t *oc_x_rs = &OC1RS;

  oc_x_con[ OFS_OC(oc_num) ] = 0x0006;	// PWM mode, use Timer2.
  oc_x_r[ OFS_OC(oc_num) ] = 0;
  oc_x_rs[ OFS_OC(oc_num) ] = 0;
  oc_x_con[ OFS_OC(oc_num) ] |= 0x8000;	// ON

  SET_RETURN( val );
  return;


 ERROR_RETURN:
  mrbc_raise(vm, MRBC_CLASS(ArgumentError),0);
}


/*! PWM set frequency

  $pwm.frequency( 440 )
*/
static void c_pwm_frequency(mrb_vm *vm, mrbc_value v[], int argc)
{
  if( argc != 1 ) return;
  if( mrbc_type(v[1]) != MRBC_TT_INTEGER ) return;

  uint64_t period_us = 1000000 / mrbc_integer(v[1]);
  uint16_t pr = (period_us * (PBCLK/4) / 1000000 - 1);

  PR2 = pr;
  TMR2 = 0;
}

/*! PWM set period

  $pwm.period( 2273 )
*/
static void c_pwm_period_us(mrb_vm *vm, mrbc_value v[], int argc)
{
  if( argc != 1 ) return;
  if( mrbc_type(v[1]) != MRBC_TT_INTEGER ) return;

  uint16_t pr = ((uint64_t)mrbc_integer(v[1]) * (PBCLK/4) / 1000000 - 1);

  PR2 = pr;
  TMR2 = 0;
}

/*! PWM set duty cycle as percentage.

  parameter range is 0(0%) to 1024(100%).

  $pwm.duty( 512 )
*/
static void c_pwm_duty(mrb_vm *vm, mrbc_value v[], int argc)
{
  GPIO_HANDLE *h = (GPIO_HANDLE *)v->instance->data;

  if( argc != 1 ) return;
  if( mrbc_type(v[1]) != MRBC_TT_INTEGER ) return;

  int duty = mrbc_integer(v[1]);
  uint32_t pr = PR2;
  uint16_t rs = pr * duty / 1024;
  if( duty == 1024 ) rs++;

  int oc_num = pin_to_oc_num( h->port, h->num );
  volatile uint32_t *oc_x_rs = &OC1RS;
  oc_x_rs[ OFS_OC(oc_num) ] = rs;
}

/*! PWM set duty cycle by microseconds.

  $pwm.duty( 1000 )
*/
static void c_pwm_duty_us(mrb_vm *vm, mrbc_value v[], int argc)
{
  GPIO_HANDLE *h = (GPIO_HANDLE *)v->instance->data;

  if( argc != 1 ) return;
  if( mrbc_type(v[1]) != MRBC_TT_INTEGER ) return;

  uint16_t rs = ((uint64_t)mrbc_integer(v[1]) * (PBCLK/4) / 1000000 - 1);

  int oc_num = pin_to_oc_num( h->port, h->num );
  volatile uint32_t *oc_x_rs = &OC1RS;
  oc_x_rs[ OFS_OC(oc_num) ] = rs;
}


/*! initialize onboard devices.
*/
void mrbc_init_class_onboard(struct VM *vm)
{
  mrbc_define_method(0, mrbc_class_object, "leds_write", c_leds_write);
  mrbc_define_method(0, mrbc_class_object, "sw", c_sw);
}

/*! initialize GPIO class.
*/
void mrbc_init_class_digital(struct VM *vm)
{
  mrb_class *gpio;

  gpio = mrbc_define_class(0, "GPIO", mrbc_class_object);
  mrbc_define_method(0, gpio, "new", c_gpio_new);
  mrbc_define_method(0, gpio, "write", c_gpio_write);
  mrbc_define_method(0, gpio, "setmode", c_gpio_setmode);
  mrbc_define_method(0, gpio, "read", c_gpio_read);
  mrbc_define_method(0, gpio, "pull", c_gpio_pull);

  mrbc_set_class_const(gpio, mrbc_str_to_symid("IN"), &mrbc_integer_value(1));
  mrbc_set_class_const(gpio, mrbc_str_to_symid("OUT"), &mrbc_integer_value(0));
}

/*! initialize PWM class.
*/
void mrbc_init_class_pwm(struct VM *vm)
{
  // start timer2
  T2CON = 0x0020;	// 1:4 prescalor, 16bit
  TMR2 = 0;
  PR2 = 0xffff;
  T2CONSET = (1 << _T2CON_ON_POSITION);

  mrb_class *pwm;
  pwm = mrbc_define_class(0, "PWM", mrbc_class_object);
  mrbc_define_method(0, pwm, "new", c_pwm_new);
  mrbc_define_method(0, pwm, "frequency", c_pwm_frequency);
  mrbc_define_method(0, pwm, "period_us", c_pwm_period_us);
  mrbc_define_method(0, pwm, "duty", c_pwm_duty);
  mrbc_define_method(0, pwm, "duty_us", c_pwm_duty_us);
}
