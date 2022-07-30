/* ************************************************************************** */
/** DIGITAL

  @Company
    ShimaneJohoshoriCenter.inc

  @File Name
    digital.h

  @Summary
    Digital & PWM processing

  @Description
    mruby/c function army
 */
/* ************************************************************************** */

#ifndef _DIGITAL_H
#define _DIGITAL_H

#include <stdint.h>
#include "mrubyc.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

/* C codes */
typedef struct GPIO_HANDLE {
  uint8_t port;		// A=1,B=2,..,G=7
  uint8_t num;		// 0..15
} GPIO_HANDLE;

typedef struct PWM_HANDLE {
  GPIO_HANDLE gpio;
  int8_t oc_num;	// 0..5
} PWM_HANDLE;

int set_gpio_handle( GPIO_HANDLE *h, const mrbc_value *pin );
void mrbc_init_class_onboard(struct VM *vm);
void mrbc_init_class_digital(struct VM *vm);
void mrbc_init_class_pwm(struct VM *vm);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _DIGITAL_H */
