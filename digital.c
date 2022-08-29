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

 */
/* ************************************************************************** */

#include <stdint.h>
#include <xc.h>
#include "pic32mx.h"
#include "digital.h"
#include "mrubyc.h"


/* ================================ C codes ================================ */

int set_gpio_handle( GPIO_HANDLE *h, const mrbc_value *pin )
{
  if( pin->tt == MRBC_TT_INTEGER ) {
    int ch = mrbc_integer(*pin);
    if( ch <= 4 ) {
      h->port = 1;
      h->num = ch;
    } else {
      h->port = 2;
      h->num = ch-5;
    }

  } else if( pin->tt == MRBC_TT_STRING ) {
    const char *s = mrbc_string_cstr(pin);
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
static void c_leds_write(mrbc_vm *vm, mrbc_value v[], int argc)
{
  int led = GET_INT_ARG(1);

  onboard_led( 1, led & 0x01 );
  onboard_led( 2, led & 0x02 );
  onboard_led( 3, led & 0x04 );
  onboard_led( 4, led & 0x08 );
}

/*! read the onboard switch

  x = sw()
*/
static void c_sw(mrbc_vm *vm, mrbc_value v[], int argc)
{
  SET_INT_RETURN( onboard_sw(1) );
}


/*! GPIO constructor

  $gpio_x = GPIO.new( num )  # num = pin number of Rboard.
  $gpio_x = GPIO.new("A0")   # PIC origined pin assignment.
*/
static void c_gpio_new(mrbc_vm *vm, mrbc_value v[], int argc)
{
  if( argc == 0 ) goto ERROR_RETURN;

  mrbc_value val = mrbc_instance_new( vm, v->cls, sizeof(GPIO_HANDLE) );
  GPIO_HANDLE *h = (GPIO_HANDLE*)val.instance->data;

  if( set_gpio_handle( h, &v[1] ) != 0 ) goto ERROR_RETURN;
  SET_RETURN( val );
  return;


 ERROR_RETURN:
  mrbc_raise(vm, MRBC_CLASS(ArgumentError),0);
}


/*! GPIO setmode

  $gpio_x.setmode( GPIO::OUT )	# set output
  $gpio_x.setmode( GPIO::IN )   # set input
*/
static void c_gpio_setmode(mrbc_vm *vm, mrbc_value v[], int argc)
{
  GPIO_HANDLE *h = (GPIO_HANDLE *)v->instance->data;

  if( GET_INT_ARG(1) == 0 ) {
    set_pin_to_digital_output( h->port, h->num );
  } else {
    set_pin_to_digital_input( h->port, h->num );
  }
}


/*! GPIO write

  $gpio_x.write( 0 or 1 )
*/
static void c_gpio_write(mrbc_vm *vm, mrbc_value v[], int argc)
{
  GPIO_HANDLE *h = (GPIO_HANDLE *)v->instance->data;

  if( GET_INT_ARG(1) == 0 ) {
    LATxCLR(h->port) = (1 << h->num);
  } else {
    LATxSET(h->port) = (1 << h->num);
  }
}


/*! GPIO read

  x = $gpio_x.read()
*/
static void c_gpio_read(mrbc_vm *vm, mrbc_value v[], int argc)
{
  GPIO_HANDLE *h = (GPIO_HANDLE *)v->instance->data;

  SET_INT_RETURN( (PORTx(h->port) >> h->num) & 1 );
}


/*! GPIO internal pull up or pull down.

  (note) Rboard original function.

  $gpio_x.pull( 1 )   # pull up
  $gpio_x.pull( -1 )  # pull down
  $gpio_x.pull( 0 )   # cancel
*/
static void c_gpio_pull(mrbc_vm *vm, mrbc_value v[], int argc)
{
  GPIO_HANDLE *h = (GPIO_HANDLE *)v->instance->data;
  int mode = GET_INT_ARG(1);

  if( mode > 0 ) {
    // pull up.
    CNPUxSET(h->port) = (1 << h->num);

  } else if( mode < 0 ) {
    // pull down.
    CNPDxSET(h->port) = (1 << h->num);

  } else {
    // cancel.
    CNPUxCLR(h->port) = (1 << h->num);
    CNPDxCLR(h->port) = (1 << h->num);
  }
}


/*! PWM constructor

  $pwm = PWM.new( num )	 # num = pin number of Rboard.
  $pwm = PWM.new("A0")	 # PIC origined pin assignment.
*/
static void c_pwm_new(mrbc_vm *vm, mrbc_value v[], int argc)
{
  if( argc == 0 ) {
    mrbc_raise(vm, MRBC_CLASS(ArgumentError), 0);
    return;
  }

  mrbc_value val = mrbc_instance_new( vm, v->cls, sizeof(PWM_HANDLE) );
  PWM_HANDLE *h = (PWM_HANDLE*)val.instance->data;

  if( set_gpio_handle( &(h->gpio), &v[1] ) != 0 ) {
    mrbc_raise(vm, MRBC_CLASS(ArgumentError), "PWM: Invalid pin assignment.");
    return;
  }
  h->oc_num = set_pin_to_pwm( -1, h->gpio.port, h->gpio.num );
  if( h->oc_num < 0 ) {
    mrbc_raise(vm, MRBC_CLASS(ArgumentError), "PWM: Can't assign PWM.");
    return;
  }

  // set OC module
  OCxCON(h->oc_num) = 0x0006;		// PWM mode, use Timer2.
  OCxR(h->oc_num) = 0;
  OCxRS(h->oc_num) = 0;
  OCxCON(h->oc_num) |= 0x8000;		// ON

  SET_RETURN( val );
}


/*! PWM set frequency

  $pwm.frequency( 440 )
*/
static void c_pwm_frequency(mrbc_vm *vm, mrbc_value v[], int argc)
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
static void c_pwm_period_us(mrbc_vm *vm, mrbc_value v[], int argc)
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
static void c_pwm_duty(mrbc_vm *vm, mrbc_value v[], int argc)
{
  PWM_HANDLE *h = (PWM_HANDLE *)v->instance->data;

  if( argc != 1 ) return;
  if( mrbc_type(v[1]) != MRBC_TT_INTEGER ) return;

  int duty = mrbc_integer(v[1]);
  uint32_t pr = PR2;
  uint16_t rs = pr * duty / 1024;
  if( duty == 1024 ) rs++;

  OCxRS(h->oc_num) = rs;
}

/*! PWM set duty cycle by microseconds.

  $pwm.duty( 1000 )
*/
static void c_pwm_duty_us(mrbc_vm *vm, mrbc_value v[], int argc)
{
  PWM_HANDLE *h = (PWM_HANDLE *)v->instance->data;

  if( argc != 1 ) return;
  if( mrbc_type(v[1]) != MRBC_TT_INTEGER ) return;

  uint16_t rs = ((uint64_t)mrbc_integer(v[1]) * (PBCLK/4) / 1000000 - 1);

  OCxRS(h->oc_num) = rs;
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
  mrbc_class *gpio;

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

  mrbc_class *pwm;
  pwm = mrbc_define_class(0, "PWM", mrbc_class_object);
  mrbc_define_method(0, pwm, "new", c_pwm_new);
  mrbc_define_method(0, pwm, "frequency", c_pwm_frequency);
  mrbc_define_method(0, pwm, "period_us", c_pwm_period_us);
  mrbc_define_method(0, pwm, "duty", c_pwm_duty);
  mrbc_define_method(0, pwm, "duty_us", c_pwm_duty_us);
}
