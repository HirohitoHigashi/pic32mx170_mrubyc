/* ************************************************************************** */
/** ADC

  @Company
    ShimaneJohoshoriCenter.inc

  @File Name
    adc.c

  @Summary
    ADC processing

  @Description
    mruby/c function army
 */
/* ************************************************************************** */

#include "adc.h"


/* ================================ C codes ================================ */
void adc_init()
{
  AD1CON1 = 0x00e0;	// SSRC=111 CLRASAM=0 ASAM=0 SAMP=0
  AD1CON2 = 0x0000;
  AD1CON3 = 0x1e09;	// SAMC=1e(60us) ADCS=09(TAD=2us)
  AD1CHS  = 0x0000;
  AD1CSSL = 0x0000;

  // Enable ADC
  AD1CON1bits.ADON = 1;
}


/* ============================= mruby/c codes ============================= */

/*! ADC constructor

  $adc_x = ADC.new( num )	# num: pin number of Rboard
  $adc_x = ADC.new("A0")	# PIC origined pin assingment.
*/
void c_adc_new(mrbc_vm *vm, mrbc_value v[], int argc)
{
  if( argc == 0 ) {
    mrbc_raise(vm, MRBC_CLASS(ArgumentError), 0);
    return;
  }

  mrbc_value val = mrbc_instance_new( vm, v->cls, sizeof(ADC_HANDLE) );
  ADC_HANDLE *h = (ADC_HANDLE*)val.instance->data;

  if( set_gpio_handle( &(h->gpio), &v[1] ) != 0 ) {
    mrbc_raise(vm, MRBC_CLASS(ArgumentError), "ADC: Invalid pin assignment.");
    return;
  }
  h->channel = set_pin_for_adc( h->gpio.port, h->gpio.num );
  if( h->channel < 0 ) {
    mrbc_raise(vm, MRBC_CLASS(ArgumentError), "ADC: Not analog pin.");
    return;
  }

  SET_RETURN( val );
}


/*! ADC read

  $adc_x.read()
*/
void c_adc_read(mrbc_vm *vm, mrbc_value v[], int argc)
{
  ADC_HANDLE *h = (ADC_HANDLE *)v->instance->data;

  AD1CHSbits.CH0SA = h->channel;
  AD1CON1bits.SAMP = 1;
  while( !AD1CON1bits.DONE )
    ;
  AD1CON1bits.DONE = 0;

  SET_FLOAT_RETURN( ADC1BUF0 * 3.3 / 1023 );
}


void mrbc_init_class_adc(struct VM *vm)
{
  adc_init();

  mrbc_class *adc;
  adc = mrbc_define_class(0, "ADC", 0);
  mrbc_define_method(0, adc, "new", c_adc_new);
  mrbc_define_method(0, adc, "read", c_adc_read);
  mrbc_define_method(0, adc, "read_v", c_adc_read);
}
