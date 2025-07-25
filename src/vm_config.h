/*! @file
  @brief
  Global configuration of mruby/c VM's

  <pre>
  Copyright (C) 2015- Kyushu Institute of Technology.
  Copyright (C) 2015- Shimane IT Open-Innovation Center.

  This file is distributed under BSD 3-Clause License.
  </pre>
*/

#ifndef MRBC_SRC_VM_CONFIG_H_
#define MRBC_SRC_VM_CONFIG_H_

// maximum number of VMs
#if !defined(MAX_VM_COUNT)
#define MAX_VM_COUNT 5
#endif

// maximum size of registers
#if !defined(MAX_REGS_SIZE)
#define MAX_REGS_SIZE 110
#endif

// maximum number of symbols
#if !defined(MAX_SYMBOLS_COUNT)
#define MAX_SYMBOLS_COUNT 255
#endif


// memory management
//  MRBC_ALLOC_16BIT or MRBC_ALLOC_24BIT
#define MRBC_ALLOC_24BIT


/* USE Float. Support Float class.
   0: NOT USE
   1: USE float
   2: USE double
*/
#if !defined(MRBC_USE_FLOAT)
#define MRBC_USE_FLOAT 2
#endif

// Use math. Support Math class.
#if !defined(MRBC_USE_MATH)
#define MRBC_USE_MATH 1		/* CHANGED */
#endif
/* (NOTE)
   maybe you need
   $ export LDFLAGS=-lm
   $ make

   on ubuntu
   $ export LDFLAGS="-Wl,--no-as-needed -lm"
   $ make
*/

// USE String. Support String class.
#if !defined(MRBC_USE_STRING)
#define MRBC_USE_STRING 1
#endif


/* Hardware dependent flags

  Use the MRBC_BIG_ENDIAN, MRBC_LITTLE_ENDIAN and MRBC_REQUIRE_*BIT_ALIGNMENT
  macros.
  for conversion functions from binary (byte array) to each data type.

  (each cases)
  Little endian, no alignment.
   MRBC_LITTLE_ENDIAN && !MRBC_REQUIRE_32BIT_ALIGNMENT
   (e.g.) ARM Cortex-M4, Intel x86

  Big endian, no alignment.
   MRBC_BIG_ENDIAN && !MRBC_REQUIRE_32BIT_ALIGNMENT
   (e.g.) IBM PPC405

  Little endian, 32bit alignment required.
   MRBC_LITTLE_ENDIAN && MRBC_REQUIRE_32BIT_ALIGNMENT
   (e.g.) ARM Cortex-M0

  Big endian, 32bit alignment required.
   MRBC_BIG_ENDIAN) && MRBC_REQUIRE_32BIT_ALIGNMENT
   (e.g.) OpenRISC
*/

/* Endian
   Define either MRBC_BIG_ENDIAN or MRBC_LITTLE_ENDIAN.
*/
#if !defined(MRBC_BIG_ENDIAN) && !defined(MRBC_LITTLE_ENDIAN)
# define MRBC_LITTLE_ENDIAN
#endif

/* Word alignment
   If 32bit and/or 64bit alignment is required, enable the following line.
*/
#define MRBC_REQUIRE_32BIT_ALIGNMENT	/* CHANGED */
#define MRBC_REQUIRE_64BIT_ALIGNMENT


/* Others */

// Compile with debug code.
#if !defined(NDEBUG)
#define MRBC_DEBUG
#endif

// #define MRBC_NO_TIMER

// Console new-line mode.
// If you need to convert LF to CRLF in console output, enable the following:
// #define MRBC_CONVERT_CRLF

// If you need 64bit integer.
// #define MRBC_INT64

// If you get exception with message "Not support op_ext..." when runtime.
// #define MRBC_SUPPORT_OP_EXT

// If you use LIBC malloc instead of mruby/c malloc
// #define MRBC_ALLOC_LIBC

// Nesting level for exception printing (default 8)
// #define MRBC_EXCEPTION_CALL_NEST_LEVEL 8

// Examples of override actions when some fatal errors.
// #define MRBC_OUT_OF_MEMORY() mrbc_alloc_print_memory_pool(); hal_abort(0)
// #define MRBC_ABORT_BY_EXCEPTION(vm) mrbc_p( &vm->exception ); hal_abort(0)

void hal_abort(const char *s);
#define MRBC_OUT_OF_MEMORY() hal_abort("Fatal error: Out of memory.\n")
#define MRBC_ABORT_BY_EXCEPTION(vm) mrbc_print_vm_exception( vm ); hal_abort(0)

#endif // MRBC_SRC_VM_CONFIG_H_
