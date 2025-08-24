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
#include "gpio.h"
#include "uart.h"
#include "mrubyc.h"

#include "model_dependent.c"  // include system (CPU) related functions.

// mruby/c heap
#if !defined(MRBC_MEMORY_SIZE)
#define MRBC_MEMORY_SIZE (1024*40)
#endif
uint8_t memory_pool[MRBC_MEMORY_SIZE];


// function prototypes.
void tick_timer_init( void );
void mrbc_init_class_adc(void);
void mrbc_init_class_pwm(void);
void mrbc_init_class_i2c(void);
void mrbc_init_class_spi(void);

extern const uint8_t mrbwrite_bytecode[];


//================================================================
/*
  Return the mrb file size.
 */
static uint32_t mrb_file_size( const void *mrb_file )
{
  const uint8_t *p = mrb_file;
  uint32_t size = 0;

  for( int i = 0; i < 4; i++ ) {
    size = (size << 8) | p[8 + i];
  }

  return size;
}


//================================================================
/*
  HAL functions.
*/
int hal_write(int fd, const void *buf, int nbytes) {
  return uart_write( UART_HANDLE_CONSOLE, buf, nbytes );
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


//================================================================
// on board devices
/*!
  control the onboard LEDs

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

/*!
  read the onboard switch

  x = sw()
*/
static void c_sw(mrbc_vm *vm, mrbc_value v[], int argc)
{
  SET_INT_RETURN( onboard_sw(1) );
}


//================================================================
// for mrbwrite protocols.
/*
  Clear the FLASH memory.
*/
static void c_clear(mrbc_vm *vm, mrbc_value v[], int argc)
{
  uint8_t *addr = (uint8_t*)FLASH_SAVE_ADDR;

  SET_BOOL_RETURN( flash_erase_page( addr ) == 0 );
}


/*
  Write to FLASH memory.
*/
static void c_write(mrbc_vm *vm, mrbc_value v[], int argc)
{
  static const char RITE[4] = "RITE";

  char *bytecode = mrbc_string_cstr(&v[1]);
  unsigned int size = mrbc_string_size(&v[1]);

  // search write point.
  uint8_t *addr = (uint8_t *)FLASH_SAVE_ADDR;
  while( strncmp( (const char *)addr, RITE, sizeof(RITE)) == 0 ) {
    addr += FLASH_ALIGN_ROW_SIZE( mrb_file_size(addr) );
  }

  if( addr + size > (uint8_t *)(FLASH_END_ADDR+1) ) {
    mrbc_raise(vm, 0, "Total bytecode size overflow.");
    return;
  }

  // erase required amount of PAGE
  uint8_t *page_top = (uint8_t *)(((uintptr_t)addr + FLASH_PAGE_SIZE) & (1-FLASH_PAGE_SIZE));
  uint8_t *prog_end_row = addr + FLASH_ALIGN_ROW_SIZE(size);

  while( page_top < prog_end_row ) {
    if( flash_erase_page( page_top ) != 0 ) {
      mrbc_raise(vm, 0, "-ERR Flash erase error.");
      return;
    }
    page_top += FLASH_PAGE_SIZE;
  }

  // Check if magic word "RITE" remains on the next page top.
  if( (page_top < (uint8_t *)(FLASH_END_ADDR+1)) &&
      memcmp( (const char *)page_top, RITE, sizeof(RITE)) == 0 ) {
    flash_erase_page( page_top );
  }

  // Write bytecode to FLASH. segmented by ROW size.
  while( addr < prog_end_row ) {
    if( flash_write_row( addr, bytecode ) != 0 ) {
      mrbc_raise(vm, 0, "Flash write error.");
      return;
    }
    addr += FLASH_ROW_SIZE;
    bytecode += FLASH_ROW_SIZE;
  }
}


/*
  Show the programs.
*/
static void c_showprog(mrbc_vm *vm, mrbc_value v[], int argc)
{
  static const char RITE[4] = "RITE";
  const uint8_t *addr = (const uint8_t *)FLASH_SAVE_ADDR;
  int n = 0;
  char buf[80];

  uart_puts(UART_HANDLE_CONSOLE, "idx size offset\r\n");
  while( strncmp( (const char *)addr, RITE, sizeof(RITE)) == 0 ) {
    uint32_t size = mrb_file_size(addr);

    mrbc_snprintf(buf, sizeof(buf), " %d  %-4d %p\r\n", n++, size, addr);
    uart_puts(UART_HANDLE_CONSOLE, buf);

    addr += FLASH_ALIGN_ROW_SIZE( size );
  }

  int total = (FLASH_END_ADDR - FLASH_SAVE_ADDR + 1);
  int used = addr - (uint8_t *)FLASH_SAVE_ADDR;
  int percent = 100 * used / total;
  mrbc_snprintf(buf, sizeof(buf), "total %d / %d (%d%%)\r\n", used, total, percent);
  uart_puts(UART_HANDLE_CONSOLE, buf);
}


/*
  Software reset
*/
static void c_reset(mrbc_vm *vm, mrbc_value v[], int argc)
{
  __builtin_disable_interrupts();
  system_reset();
}


//================================================================
/*! pick up a task
*/
void * pickup_task( void *task )
{
  static const char RITE[4] = "RITE";
  uint8_t *addr;

  if( task ) {
    if( strncmp( task, RITE, sizeof(RITE)) != 0 ) return 0;
    addr = (uint8_t *)task + FLASH_ALIGN_ROW_SIZE( mrb_file_size(task) );
  } else {
    addr = (uint8_t *)FLASH_SAVE_ADDR;
  }

  if( strncmp( (const char *)addr, RITE, sizeof(RITE)) == 0 ) {
    return addr;
  }

  return 0;
}


//================================================================
/*!
  Choose to enter programming mode or run mode.
*/
static int check_timeout( void )
{
  for( int i = 0; i < 50; i++ ) {
    onboard_led( 1, 1 );
    __delay_ms( 30 );
    onboard_led( 1, 0 );
    __delay_ms( 30 );
    if( uart_can_read_line( UART_HANDLE_CONSOLE ) ) return 1;
  }
  return 0;
}


//================================================================
/*!
  Initialize RBoard specific class.
*/
static void init_additional_class( void )
{
  mrbc_init_class_gpio();
  mrbc_init_class_uart();
  mrbc_init_class_adc();
  mrbc_init_class_pwm();
  mrbc_init_class_i2c();
  mrbc_init_class_spi();

  mrbc_define_method(0, 0, "leds_write", c_leds_write);
  mrbc_define_method(0, 0, "sw", c_sw);

  tick_timer_init();
}


//================================================================
/*!
  main function
*/
int main(void)
{
  /* module init */
  system_init();
  uart_init();
  mrbc_init(memory_pool, MRBC_MEMORY_SIZE);
  init_additional_class();
  mrbc_printf("\r\n\x1b(B\x1b)B\x1b[0m\x1b[2JRboard v2.1.0, mruby/c v3.4 start.\n");

#if 1
  if( check_timeout() ) {
    /* communicate to host using mrbwrite protocol */
    static const struct MRBC_DEFINE_METHOD_LIST method_list[] = {
      {"c_clear", c_clear},
      {"c_write", c_write},
      {"c_showprog", c_showprog},
      {"c_reset", c_reset},
    };
    mrbc_define_method_list(0,0, method_list, sizeof(method_list)/sizeof(method_list[0]));
    mrbc_create_task(mrbwrite_bytecode, 0);
    mrbc_run();

    mrbc_cleanup();
    init_additional_class();
  }

  /* start user mruby/c code */
  mrbc_printf("start mruby/c\n");
  void *task = 0;
  while( 1 ) {
    task = pickup_task( task );
    if( task == 0 ) break;

    mrbc_create_task( task, 0 );
  }
  mrbc_run();


#else
  /* Or run the prepared bytecode.

     How to create "prepared_bytecode.c"
       mrbc --remove-lv -B bytecode -o prepared_bytecode.c *.rb
  */
  #include "prepared_bytecode.c"
  mrbc_printf("prepared bytecode executing.\n");
  mrbc_create_task(bytecode, 0);
  mrbc_run();
#endif

  return 0;
}
