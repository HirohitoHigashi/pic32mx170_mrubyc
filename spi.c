/* ************************************************************************** */
/** SPI

  @Company
    ShimaneJohoshoriCenter.inc

  @File Name
    spi.c

  @Summary
    SPI processing

  @Description
    mruby/c function army
 */
/* ************************************************************************** */

#include "pic32mx.h"
#include "spi.h"


/* ================================ C codes ================================ */

static int spi_set_frequency( int unit, int freq )
{
  uint32_t brg = (uint32_t)PBCLK / 2 / freq - 1;
  if( brg >= 0x2000 ) return -1;
  SPIxBRG(unit) = brg;
}


static int spi_transfer( int unit, const void *send_buf, int send_size,
                         void *recv_buf, int recv_size, int flag_include )
{
  // force empty FIFO.
  while( !(SPIxSTAT(unit) & _SPI1STAT_SPITBE_MASK) ) {
  }
  while( !(SPIxSTAT(unit) & _SPI1STAT_SPIRBE_MASK) ) {
    int dummy = SPIxBUF(unit);
  }

  const uint8_t *p_send = send_buf;
  uint8_t *p_recv = recv_buf;
  int s_count = send_size;
  int r_count = send_size;

  // data send process.
  while( r_count > 0 ) {
    // write data to FIFO
    if( s_count > 0 && !(SPIxSTAT(unit) & _SPI1STAT_SPITBF_MASK)) {
      SPIxBUF(unit) = *p_send++;
      s_count--;
    }

    // read data from FIFO
    while( !(SPIxSTAT(unit) & _SPI1STAT_SPIRBE_MASK)) {
      uint8_t data = SPIxBUF(unit);
      if( flag_include ) *p_recv++ = data;
      r_count--;
    }
  }

  // data receive process.
  s_count = recv_size;
  r_count = recv_size;
  while( r_count > 0 ) {
    // write dummy data to FIFO
    if( s_count > 0 && !(SPIxSTAT(unit) & _SPI1STAT_SPITBF_MASK)) {
      SPIxBUF(unit) = 0;
      s_count--;
    }

    // read data from FIFO
    while( !(SPIxSTAT(unit) & _SPI1STAT_SPIRBE_MASK)) {
      *p_recv++ = SPIxBUF(unit);
      r_count--;
    }
  }

  return 0;
}


/*! SPI constructor

  $spi = SPI.new()  # all default.
                    # mode0 1MHz SPI1 SDI=10(B5) SDO=11(B6) SCK=19(B14)

  $spi = SPI.new( params )
                    # mode: 0..3
		    # freq: 9770..5000000 (9.77kHz - 5MHz)
                    # unit: 1 or 2
		    # sdi_pin: SPI1: 1(A1) 6(B1) 10(B5) 13(B8) 16(B11)
		    #          SPI2: 2(A2) 7(B2) 11(B6) 18(B13)
		    # sdo_pin:  (Any of the above pins)
  (note)
    SCK: SPI1: 19(B14)
         SPI2: 20(B15)
*/
static void c_spi_new(mrbc_vm *vm, mrbc_value v[], int argc)
{
  mrbc_value val = mrbc_instance_new( vm, v->cls, sizeof(SPI_HANDLE) );
  SPI_HANDLE *h = (SPI_HANDLE *)val.instance->data;
  int mode;
  uint32_t freq;
  int flag_error = 0;

  // set default.
  mode = 0;
  freq = 1000000;
  h->spi_num = 1;
  h->sdi.port = 2; // "B5"
  h->sdi.num = 5;
  h->sdo.port = 2; // "B6"
  h->sdo.num = 6;

  // parse argument
  if( argc == 0 ) goto SET_SCK_PIN;
  if( argc != 1 || v[1].tt != MRBC_TT_HASH ) goto ERROR_PARAM;

  mrbc_hash_iterator ite = mrbc_hash_iterator_new( &v[1] );
  while( mrbc_hash_i_has_next(&ite) ) {
    mrbc_value *kv = mrbc_hash_i_next(&ite);
    const char *key;
    if( mrbc_type(kv[0]) == MRBC_TT_STRING ) {
      key = mrbc_string_cstr(&kv[0]);
    } else if( mrbc_type(kv[0]) == MRBC_TT_SYMBOL ) {
      key = mrbc_symbol_cstr(&kv[0]);
    } else {
      goto ERROR_PARAM;
    }

    if( strcmp("mode", key) == 0 ) {
      if( mrbc_type(kv[1]) != MRBC_TT_INTEGER ) goto ERROR_PARAM;
      mode = mrbc_integer(kv[1]);

    } else if( strcmp("freq", key) == 0 ) {
      switch( mrbc_type(kv[1]) ) {
      case MRBC_TT_INTEGER:
	freq = mrbc_integer(kv[1]);
	break;
      case MRBC_TT_FLOAT:
	freq = mrbc_float(kv[1]);
	break;
      default:
	goto ERROR_PARAM;
      }

    } else if( strcmp("unit", key) == 0 ) {
      if( mrbc_type(kv[1]) != MRBC_TT_INTEGER ) goto ERROR_PARAM;
      h->spi_num = mrbc_integer(kv[1]);

    } else if( strcmp("sdi_pin", key) == 0 ) {
      flag_error |= set_gpio_handle( &(h->sdi), &kv[1] );

    } else if( strcmp("sdo_pin", key) == 0 ) {
      flag_error |= set_gpio_handle( &(h->sdo), &kv[1] );

    } else {
      goto ERROR_PARAM;
    }
  } // to next param

 SET_SCK_PIN:
  h->sck.port = 2;
  switch( h->spi_num ) {
  case 1: h->sck.num = 14; break;
  case 2: h->sck.num = 15; break;
  default:
    goto ERROR_PARAM;
  }

  flag_error |= set_pin_for_spi( h->spi_num,
				 h->sdi.port, h->sdi.num,
				 h->sdo.port, h->sdo.num,
				 h->sck.port, h->sck.num );
  if( flag_error ) goto ERROR_PARAM;

  // Initialize SPI registers.
  SPIxSTAT(h->spi_num) = 0;
  spi_set_frequency( h->spi_num, freq );
  SPIxCON2(h->spi_num) = 0;

  static const uint32_t SPIxCON_MODES[] = {
    0x00018120,		// 0: CKP(CPOL)=0,CKE(/CPHA)=1
    0x00018020,	        // 1: CKP(CPOL)=0,CKE(/CPHA)=0
    0x00018160,		// 2: CKP(CPOL)=1,CKE(/CPHA)=1
    0x00018060,		// 3: CKP(CPOL)=1,CKE(/CPHA)=0
  };
  SPIxCON(h->spi_num) = SPIxCON_MODES[ mode ];

  SET_RETURN( val );
  return;


 ERROR_PARAM:
   mrbc_raise(vm, MRBC_CLASS(ArgumentError), "SPI: Constructor parameter error.");
}


/*! SPI read

  s = $spi.read(n)
  @param  n		Number of bytes receive.
  @return String	Received data.
*/
static void c_spi_read(mrbc_vm *vm, mrbc_value v[], int argc)
{
  mrbc_value ret;
  SPI_HANDLE *h = (SPI_HANDLE *)v->instance->data;
  int recv_len = GET_INT_ARG(1);
  uint8_t *buf = mrbc_alloc( vm, recv_len+1 );
  if( !buf ) {			// ENOMEM
    ret = mrbc_nil_value();
    goto DONE;
  }

  spi_transfer(h->spi_num, 0, 0, buf, recv_len, 0);
  buf[recv_len] = 0;
  ret = mrbc_string_new_alloc( vm, buf, recv_len );

 DONE:
  SET_RETURN(ret);
}


/*! SPI write

  $spi.write( str )
  $spi.write( d1, d2, ...)
*/
static void c_spi_write(mrbc_vm *vm, mrbc_value v[], int argc)
{
  SPI_HANDLE *h = (SPI_HANDLE *)v->instance->data;

  if( v[1].tt == MRBC_TT_STRING ) {
    spi_transfer( h->spi_num, mrbc_string_cstr(&v[1]), mrbc_string_size(&v[1]),
		  0, 0, 0 );
    goto DONE;
  }

  if( v[1].tt == MRBC_TT_INTEGER ) {
    uint8_t *buf = mrbc_alloc( vm, argc );
    if( !buf ) goto DONE;	// ENOMEM
    int i;
    for( i = 0; i < argc; i++ ) {
      buf[i] = GET_INT_ARG(i+1);
    }
    spi_transfer( h->spi_num, buf, argc, 0, 0, 0 );
    mrbc_free( vm, buf );
    goto DONE;
  }

  // else TypeError. raise?


 DONE:
  SET_NIL_RETURN();
}


/*! SPI transfer

  $spi.transfer( s, recv_size )
  $spi.transfer( [d1, d2,...], recv_size )
*/
static void c_spi_transfer(mrbc_vm *vm, mrbc_value v[], int argc)
{
  mrbc_value ret;
  SPI_HANDLE *h = (SPI_HANDLE *)v->instance->data;

  if( v[1].tt == MRBC_TT_STRING && v[2].tt == MRBC_TT_INTEGER ) {
    int recv_len = GET_INT_ARG(2);
    uint8_t *buf = mrbc_alloc( vm, recv_len+1 );
    if( !buf ) goto ERROR_RETURN;		// ENOMEM
    spi_transfer( h->spi_num, mrbc_string_cstr(&v[1]), mrbc_string_size(&v[1]),
		  buf, recv_len, 0 );
    buf[recv_len] = 0;
    ret = mrbc_string_new_alloc( vm, buf, recv_len );
    goto DONE;
  }


  if( v[1].tt == MRBC_TT_ARRAY && v[2].tt == MRBC_TT_INTEGER ) {
    int send_len = mrbc_array_size( &v[1] );
    int recv_len = GET_INT_ARG(2);
    uint8_t *buf = mrbc_raw_alloc( send_len > recv_len ? send_len : recv_len );
    if( !buf ) goto ERROR_RETURN;		// ENOMEM

    int i;
    for( i = 0; i < send_len; i++ ) {
      mrbc_value v1 = mrbc_array_get( &v[1], i );
      if( v1.tt != MRBC_TT_INTEGER ) {		// TypeError. raise?
	mrbc_raw_free( buf );
	goto ERROR_RETURN;
      }
      buf[i] = v1.i;
    }

    spi_transfer( h->spi_num, buf, send_len, buf, recv_len, 0 );
    buf[recv_len] = 0;
    ret = mrbc_string_new_alloc( vm, buf, recv_len );
    goto DONE;
  }

  // else TypeError. raise?


 ERROR_RETURN:
  ret = mrbc_nil_value();

 DONE:
  SET_RETURN(ret);
}


void mrbc_init_class_spi(struct VM *vm)
{
  mrbc_class *spi;

  spi = mrbc_define_class(0, "SPI", mrbc_class_object);
  mrbc_define_method(0, spi, "new", c_spi_new);
  mrbc_define_method(0, spi, "read", c_spi_read);
  mrbc_define_method(0, spi, "write", c_spi_write);
  mrbc_define_method(0, spi, "transfer", c_spi_transfer);
}
