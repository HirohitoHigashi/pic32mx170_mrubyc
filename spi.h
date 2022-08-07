/* ************************************************************************** */
/** SPI

  @Company
    ShimaneJohoshoriCenter.inc

  @File Name
    spi.h

  @Summary
    SPI processing

  @Description
    mruby/c function army
 */
/* ************************************************************************** */

#ifndef _SPI_H
#define _SPI_H

#include "mrubyc.h"
#include "digital.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

/* C codes */
typedef struct SPI_HANDLE {
  GPIO_HANDLE sdi;
  GPIO_HANDLE sdo;
  GPIO_HANDLE sck;
  int8_t spi_unit;
} SPI_HANDLE;

/* mruby/c codes */
void mrbc_init_class_spi(struct VM *vm);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _I2C_H */
