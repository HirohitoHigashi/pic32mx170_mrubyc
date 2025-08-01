/* ************************************************************************** */
/** GPIO

  @Company
    ShimaneJohoshoriCenter.inc

  @File Name
    gpio.c

  @Summary
    GPIO class processing

  @Description
    mruby/c function army
 */
/* ************************************************************************** */

#include "pic32mx.h"
#include "gpio.h"
#include "mrubyc.h"


/* ================================ C codes ================================ */

/*! PIN handle setter

  valが、ピン番号（数字）でもポート番号（e.g."B3"）でも受け付ける。

  @param  pin_handle	dist.
  @param  val		src.
  @retval 0		No error.
*/
int set_pin_handle( PIN_HANDLE *pin_handle, const mrbc_value *val )
{
  switch( val->tt ) {
  case MRBC_TT_INTEGER: {
    int ch = mrbc_integer(*val);
    if( ch <= 4 ) {		// Rboard J9,J10,J11 mapping.
      pin_handle->port = 1;
      pin_handle->num = ch;
    } else {
      pin_handle->port = 2;
      pin_handle->num = ch-5;
    }
  } break;

  case MRBC_TT_STRING: {
    const char *s = mrbc_string_cstr(val);
    if( 'A' <= s[0] && s[0] <= 'G' ) {
      pin_handle->port = s[0] - 'A' + 1;
    } else if( 'a' <= s[0] && s[0] <= 'g' ) {
      pin_handle->port = s[0] - 'a' + 1;
    } else {
      return -1;
    }

    pin_handle->num = mrbc_atoi( s+1, 10 );
  } break;

  default:
    return -1;
  }

  return -(pin_handle->num < 0 || pin_handle->num > 15);
}


/*! set (change) mode

  @param  pin	target pin.
  @param  mode	mode. Sepcified by GPIO_* constant.
  @return int	zero is no error.
*/
int gpio_setmode( const PIN_HANDLE *pin, unsigned int mode )
{
  if( mode & (GPIO_IN|GPIO_OUT|GPIO_ANALOG|GPIO_HIGH_Z) ) {
    if( mode & GPIO_ANALOG ) {
      ANSELxSET(pin->port) = (1 << pin->num);
    } else {
      ANSELxCLR(pin->port) = (1 << pin->num);
    }
    CNPUxCLR(pin->port) = (1 << pin->num);
    CNPDxCLR(pin->port) = (1 << pin->num);
    ODCxCLR(pin->port) = (1 << pin->num);
  }
  if( mode & GPIO_IN ) TRISxSET(pin->port) = (1 << pin->num);
  if( mode & GPIO_OUT ) TRISxCLR(pin->port) = (1 << pin->num);
  if( mode & GPIO_HIGH_Z ) return -1;

  if( mode & GPIO_PULL_UP ) {
    CNPDxCLR(pin->port) = (1 << pin->num);
    CNPUxSET(pin->port) = (1 << pin->num);
  }
  if( mode & GPIO_PULL_DOWN ) {
    CNPUxCLR(pin->port) = (1 << pin->num);
    CNPDxSET(pin->port) = (1 << pin->num);
  }
  if( mode & GPIO_OPEN_DRAIN ) ODCxSET(pin->port) = (1 << pin->num);

  return 0;
}


/* ============================= mruby/c codes ============================= */
/*! constructor

  gpio1 = GPIO.new(pin, GPIO::IN )
*/
static void c_gpio_new(mrbc_vm *vm, mrbc_value v[], int argc)
{
  v[0] = mrbc_instance_new(vm, v[0].cls, sizeof(PIN_HANDLE));
  mrbc_instance_call_initialize( vm, v, argc );
}


/*! initializer
*/
static void c_gpio_initialize(mrbc_vm *vm, mrbc_value v[], int argc)
{
  PIN_HANDLE *pin = MRBC_INSTANCE_DATA_PTR(v, PIN_HANDLE);

  mrbc_value *arg_pin = MRBC_ARG(1);
  int arg_modes = MRBC_ARG_I(2);
  if( mrbc_israised(vm) ) return;

  if( set_pin_handle( pin, arg_pin ) != 0 ) goto ERROR_RETURN;
  if( (arg_modes & (GPIO_IN|GPIO_OUT|GPIO_HIGH_Z)) == 0 ) goto ERROR_RETURN;
  if( gpio_setmode( pin, arg_modes ) < 0 ) goto ERROR_RETURN;
  return;

 ERROR_RETURN:
  mrbc_raise(vm, MRBC_CLASS(ArgumentError), 0);
}


/*! setmode

  GPIO.setmode( num, GPIO::IN )		# class method
  gpio1.setmode( GPIO::PULL_UP )	# instance method
*/
static void c_gpio_setmode(mrbc_vm *vm, mrbc_value v[], int argc)
{
  if( v[0].tt == MRBC_TT_CLASS ) {
    /*
      Class method mode.
    */
    PIN_HANDLE pin;

    mrbc_value *arg_pin = MRBC_ARG(1);
    int arg_modes = MRBC_ARG_I(2);
    if( mrbc_israised(vm) ) return;

    if( set_pin_handle( &pin, arg_pin ) != 0 ) goto ERROR_RETURN;
    if( gpio_setmode( &pin, arg_modes ) < 0 ) goto ERROR_RETURN;
  }
  else {
    /*
      Instance method mode.
    */
    PIN_HANDLE *pin = MRBC_INSTANCE_DATA_PTR(v, PIN_HANDLE);

    int arg_modes = MRBC_ARG_I(1);
    if( mrbc_israised(vm) ) return;

    if( gpio_setmode( pin, arg_modes ) < 0 ) goto ERROR_RETURN;
  }

  SET_NIL_RETURN();
  return;


 ERROR_RETURN:
  mrbc_raise(vm, MRBC_CLASS(ArgumentError), 0);
}


/*! read_at -> Integer

  v1 = GPIO.read_at( 1 )          # read from pin 1.
*/
static void c_gpio_read_at(mrbc_vm *vm, mrbc_value v[], int argc)
{
  PIN_HANDLE pin;

  mrbc_value *arg_pin = MRBC_ARG(1);
  if( mrbc_israised(vm) ) return;

  if( set_pin_handle( &pin, arg_pin ) != 0 ) {
    mrbc_raise(vm, MRBC_CLASS(ArgumentError), 0);
    return;
  }

  SET_INT_RETURN( (PORTx(pin.port) >> pin.num) & 1 );
}


/*! high_at? -> bool

  v1 = GPIO.high_at( 1 )          # pin 1 is high?
*/
static void c_gpio_high_at(mrbc_vm *vm, mrbc_value v[], int argc)
{
  PIN_HANDLE pin;

  mrbc_value *arg_pin = MRBC_ARG(1);
  if( mrbc_israised(vm) ) return;

  if( set_pin_handle( &pin, arg_pin ) != 0 ) {
    mrbc_raise(vm, MRBC_CLASS(ArgumentError), 0);
    return;
  }

  SET_BOOL_RETURN( (PORTx(pin.port) >> pin.num) & 1 );
}


/*! low_at? -> bool

  v1 = GPIO.row_at( 1 )          # pin 1 is row?
*/
static void c_gpio_low_at(mrbc_vm *vm, mrbc_value v[], int argc)
{
  PIN_HANDLE pin;

  mrbc_value *arg_pin = MRBC_ARG(1);
  if( mrbc_israised(vm) ) return;

  if( set_pin_handle( &pin, arg_pin ) != 0 ) {
    mrbc_raise(vm, MRBC_CLASS(ArgumentError), 0);
    return;
  }

  SET_BOOL_RETURN( ~(PORTx(pin.port) >> pin.num) & 1 );
}


/*! write_at

  v1 = GPIO.write_at( 1, 0 )      # output zero to pin 1.
*/
static void c_gpio_write_at(mrbc_vm *vm, mrbc_value v[], int argc)
{
  PIN_HANDLE pin;

  mrbc_value *arg_pin = MRBC_ARG(1);
  int arg_data = MRBC_ARG_I(2);
  if( mrbc_israised(vm) ) return;

  if( set_pin_handle( &pin, arg_pin ) != 0 ) {
    mrbc_raise(vm, MRBC_CLASS(ArgumentError), 0);
    return;
  }

  switch( arg_data ) {
  case 0:
    LATxCLR(pin.port) = (1 << pin.num);
    break;

  case 1:
    LATxSET(pin.port) = (1 << pin.num);
    break;

  default:
    mrbc_raise(vm, MRBC_CLASS(RangeError), 0);
    break;
  }
}


/*! read

  x = gpio1.read()
*/
static void c_gpio_read(mrbc_vm *vm, mrbc_value v[], int argc)
{
  PIN_HANDLE *pin = MRBC_INSTANCE_DATA_PTR(v, PIN_HANDLE);

  SET_INT_RETURN( (PORTx(pin->port) >> pin->num) & 1 );
}


/*! high?

  if gpio1.high?() ...
*/
static void c_gpio_high(mrbc_vm *vm, mrbc_value v[], int argc)
{
  PIN_HANDLE *pin = MRBC_INSTANCE_DATA_PTR(v, PIN_HANDLE);

  SET_BOOL_RETURN( (PORTx(pin->port) >> pin->num) & 1 );
}


/*! low?

  if gpio1.low?() ...
*/
static void c_gpio_low(mrbc_vm *vm, mrbc_value v[], int argc)
{
  PIN_HANDLE *pin = MRBC_INSTANCE_DATA_PTR(v, PIN_HANDLE);

  SET_BOOL_RETURN( ~(PORTx(pin->port) >> pin->num) & 1 );
}


/*! write

  gpio1.write( 0 or 1 )
*/
static void c_gpio_write(mrbc_vm *vm, mrbc_value v[], int argc)
{
  PIN_HANDLE *pin = MRBC_INSTANCE_DATA_PTR(v, PIN_HANDLE);

  int arg_data = MRBC_ARG_I(1);
  if( mrbc_israised(vm) ) return;

  switch( arg_data ) {
  case 0:
    LATxCLR(pin->port) = (1 << pin->num);
    break;

  case 1:
    LATxSET(pin->port) = (1 << pin->num);
    break;

  default:
    mrbc_raise(vm, MRBC_CLASS(RangeError), 0);
    break;
  }
}


/*! Initializer
*/
void mrbc_init_class_gpio( void )
{
  static const struct MRBC_DEFINE_METHOD_LIST method_list[] = {
    {"new", c_gpio_new},
    {"initialize", c_gpio_initialize},
    {"setmode", c_gpio_setmode},
    {"read_at", c_gpio_read_at},
    {"high_at?", c_gpio_high_at},
    {"low_at?", c_gpio_low_at},
    {"write_at", c_gpio_write_at},

    {"read", c_gpio_read},
    {"high?", c_gpio_high},
    {"low?", c_gpio_low},
    {"write", c_gpio_write},
  };

  mrbc_class *gpio = mrbc_define_class(0, "GPIO", 0);
  mrbc_define_method_list(0, gpio, method_list,
			  sizeof(method_list) / sizeof(method_list[0]));

  mrbc_set_class_const(gpio, mrbc_str_to_symid("IN"),         &mrbc_integer_value(GPIO_IN));
  mrbc_set_class_const(gpio, mrbc_str_to_symid("OUT"),        &mrbc_integer_value(GPIO_OUT));
  mrbc_set_class_const(gpio, mrbc_str_to_symid("HIGH_Z"),     &mrbc_integer_value(GPIO_HIGH_Z));
  mrbc_set_class_const(gpio, mrbc_str_to_symid("PULL_UP"),    &mrbc_integer_value(GPIO_PULL_UP));
  mrbc_set_class_const(gpio, mrbc_str_to_symid("PULL_DOWN"),  &mrbc_integer_value(GPIO_PULL_DOWN));
  mrbc_set_class_const(gpio, mrbc_str_to_symid("OPEN_DRAIN"), &mrbc_integer_value(GPIO_OPEN_DRAIN));
}
