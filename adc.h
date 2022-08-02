/* ************************************************************************** */
/** ADC

  @Company
    ShimaneJohoshoriCenter.inc

  @File Name
    adc.h

  @Summary
    ADC processing

  @Description
    mruby/c function army
 */
/* ************************************************************************** */

#ifndef _ADC_H
#define _ADC_H

#include "mrubyc.h"
#include "digital.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

/* C codes */
typedef struct ADC_HANDLE {
  GPIO_HANDLE gpio;
  int8_t channel;	// AN0..12
} ADC_HANDLE;


/* mruby/c codes */
void mrbc_init_class_adc(struct VM *vm);

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _ADC_H */
