/*
  CPU related params.
*/

#include "pic32mx_common.h"

#if !defined(__32MX170F256B__) && !defined(__PIC32MX170F256B__)
# error "Change the project property, xc32-gcc Include directories to the MPU you want to use."
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define FLASH_SAVE_ADDR 0xBD032000
#define FLASH_END_ADDR  0xBD03EFFF
#define FLASH_PAGE_SIZE 1024
#define FLASH_ROW_SIZE (FLASH_PAGE_SIZE / 8)


#if !defined(_XTAL_FREQ)
#define _XTAL_FREQ  40000000UL
#endif

#if !defined(PBCLK)
#define PBCLK (_XTAL_FREQ / 4)
#endif


void pin_init( void );
void interrupt_init( void );

#ifdef __cplusplus
}
#endif
