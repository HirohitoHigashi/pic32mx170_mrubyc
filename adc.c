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

#include <xc.h>

#include "common.h"
#include "adc.h"


typedef struct ADC_HANDLE {
  PIN_HANDLE pin;
  uint8_t channel;	// 0..12
} ADC_HANDLE;

static const ADC_HANDLE adc_handle_[] = {
  {1, 0, 0},	// AN0=RA0
  {1, 1, 1},	// AN1=RA1
  {2, 0, 2},	// AN2=RB0
  {2, 1, 3},	// AN3=RB1
  {2, 2, 4},	// AN4=RB2
  {2, 3, 5},	// AN5=RB3
  {2,15, 9},	// AN9=RB15
  {2,14, 10},	// AN10=RB14
  {2,13, 11},	// AN11=RB13
  {2,12, 12},	// AN12=RB12
};


/* ================================ C codes ================================ */

/* ============================= mruby/c codes ============================= */

/*! constructor

  adc1 = ADC.new( num )	# num: pin number of Rboard
  adc1 = ADC.new("A0")	# PIC origined pin assingment.
*/
static void c_adc_new(mrbc_vm *vm, mrbc_value v[], int argc)
{
  if( argc != 1 ) goto ERROR_RETURN;

  // allocate instance with ADC_HANDLE table pointer.
  mrbc_value val = mrbc_instance_new(vm, v[0].cls, sizeof(ADC_HANDLE *));

  ADC_HANDLE adc_h;
  if( set_pin_handle( &adc_h.pin, &v[1] ) != 0 ) goto ERROR_RETURN;

  // find ADC channel from adc_channel_ table.
  int i;
  for( i = 0; i < sizeof(adc_handle_)/sizeof(ADC_HANDLE); i++ ) {
    if( (adc_handle_[i].pin.port == adc_h.pin.port) &&
	(adc_handle_[i].pin.num == adc_h.pin.num )) break;
  }
  if( i == sizeof(adc_handle_)/sizeof(ADC_HANDLE) ) goto ERROR_RETURN;
  *(const ADC_HANDLE **)(val.instance->data) = &adc_handle_[i];

  // set pin to analog input
  ANSELxSET(adc_h.pin.port) = (1 << adc_h.pin.num);
  TRISxSET(adc_h.pin.port) = (1 << adc_h.pin.num);
  CNPUxCLR(adc_h.pin.port) = (1 << adc_h.pin.num);
  CNPDxCLR(adc_h.pin.port) = (1 << adc_h.pin.num);
  ODCxCLR(adc_h.pin.port) = (1 << adc_h.pin.num);

  SET_RETURN( val );
  return;

 ERROR_RETURN:
  mrbc_raise(vm, MRBC_CLASS(ArgumentError), "ADC initialize.");
}

/*! read

  adc1.read() -> Float
*/
static void c_adc_read(mrbc_vm *vm, mrbc_value v[], int argc)
{
  ADC_HANDLE *h = *(ADC_HANDLE **)v[0].instance->data;

  AD1CHSbits.CH0SA = h->channel;
  AD1CON1bits.SAMP = 1;
  while( !AD1CON1bits.DONE )
    ;
  AD1CON1bits.DONE = 0;

  SET_FLOAT_RETURN( ADC1BUF0 * 3.3 / 1023 );
}


/*! min

  adc1.min() -> Float
*/
static void c_adc_min(mrbc_vm *vm, mrbc_value v[], int argc)
{
  SET_FLOAT_RETURN( 0 );
}


/*! max

  adc1.max() -> Float
*/
static void c_adc_max(mrbc_vm *vm, mrbc_value v[], int argc)
{
  SET_FLOAT_RETURN( 3.3 );
}


/*! Initializer
*/
void mrbc_init_class_adc(void)
{
  AD1CON1 = 0x00e0;	// SSRC=111 CLRASAM=0 ASAM=0 SAMP=0
  AD1CON2 = 0x0000;
  AD1CON3 = 0x1e09;	// SAMC=1e(60us) ADCS=09(TAD=2us)
  AD1CHS  = 0x0000;
  AD1CSSL = 0x0000;

  // Enable ADC
  AD1CON1bits.ADON = 1;

  mrbc_class *adc = mrbc_define_class(0, "ADC", 0);

  mrbc_define_method(0, adc, "new", c_adc_new);
  mrbc_define_method(0, adc, "read", c_adc_read);
  mrbc_define_method(0, adc, "min", c_adc_min);
  mrbc_define_method(0, adc, "max", c_adc_max);
}
