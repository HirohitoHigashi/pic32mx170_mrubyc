/*
  CPU related params.
*/

#if !defined(__32MX170F256B__) && !defined(__PIC32MX170F256B__)
# error "Change the project property, xc32-gcc Include directories to the MPU you want to use."
#endif

#include <xc.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! get offset of port address.

    以下のレジスタが、番号ごとに一定番地離れている事に依存しています。
      * GPIO 関連のレジスタが、ポートごとに 0x100番地
      * OC(PWM) 関連のレジスタが、0x200番地
    そうでないプロセッサに対応が必要になった場合は、戦略の変更が必要です。
*/
// Pin settings
#define ANSELxSET(x)	*(&ANSELASET + (0x100 / sizeof(uint32_t)) * ((x)-1))
#define ANSELxCLR(x)	*(&ANSELACLR + (0x100 / sizeof(uint32_t)) * ((x)-1))
#define TRISxSET(x)	*(&TRISASET  + (0x100 / sizeof(uint32_t)) * ((x)-1))
#define TRISxCLR(x)	*(&TRISACLR  + (0x100 / sizeof(uint32_t)) * ((x)-1))
#define PORTx(x)	*(&PORTA     + (0x100 / sizeof(uint32_t)) * ((x)-1))
#define LATxSET(x)	*(&LATASET   + (0x100 / sizeof(uint32_t)) * ((x)-1))
#define LATxCLR(x)	*(&LATACLR   + (0x100 / sizeof(uint32_t)) * ((x)-1))
#define CNPUxSET(x)	*(&CNPUASET  + (0x100 / sizeof(uint32_t)) * ((x)-1))
#define CNPUxCLR(x)	*(&CNPUACLR  + (0x100 / sizeof(uint32_t)) * ((x)-1))
#define CNPDxSET(x)	*(&CNPDASET  + (0x100 / sizeof(uint32_t)) * ((x)-1))
#define CNPDxCLR(x)	*(&CNPDACLR  + (0x100 / sizeof(uint32_t)) * ((x)-1))

// Output comparator
#define OCxCON(x)	*(&OC1CON    + (0x200 / sizeof(uint32_t)) * ((x)-1))
#define OCxR(x)		*(&OC1R      + (0x200 / sizeof(uint32_t)) * ((x)-1))
#define OCxRS(x)	*(&OC1RS     + (0x200 / sizeof(uint32_t)) * ((x)-1))

// SPI
#define SPIxCON(x)	*(&SPI1CON   + (0x200 / sizeof(uint32_t)) * ((x)-1))
#define SPIxSTAT(x)	*(&SPI1STAT  + (0x200 / sizeof(uint32_t)) * ((x)-1))
#define SPIxBUF(x)	*(&SPI1BUF   + (0x200 / sizeof(uint32_t)) * ((x)-1))
#define SPIxBRG(x)	*(&SPI1BRG   + (0x200 / sizeof(uint32_t)) * ((x)-1))
#define SPIxCON2(x)	*(&SPI1CON2  + (0x200 / sizeof(uint32_t)) * ((x)-1))
#define SDIxR(x)	*(&SDI1R     + (0x00c / sizeof(uint32_t)) * ((x)-1))

// UART
#define UxMODE(x)	*(&U1MODE    + (0x200 / sizeof(uint32_t)) * ((x)-1))
#define UxMODESET(x)	*(&U1MODESET + (0x200 / sizeof(uint32_t)) * ((x)-1))
#define UxMODECLR(x)	*(&U1MODECLR + (0x200 / sizeof(uint32_t)) * ((x)-1))
#define UxSTA(x)	*(&U1STA     + (0x200 / sizeof(uint32_t)) * ((x)-1))
#define UxSTASET(x)	*(&U1STASET  + (0x200 / sizeof(uint32_t)) * ((x)-1))
#define UxSTACLR(x)	*(&U1STACLR  + (0x200 / sizeof(uint32_t)) * ((x)-1))
#define UxTXREG(x)	*(&U1TXREG   + (0x200 / sizeof(uint32_t)) * ((x)-1))
#define UxRXREG(x)	*(&U1RXREG   + (0x200 / sizeof(uint32_t)) * ((x)-1))
#define UxBRG(x)	*(&U1BRG     + (0x200 / sizeof(uint32_t)) * ((x)-1))
#define UxRXR(x)	*(&U1RXR     + (0x008 / sizeof(uint32_t)) * ((x)-1))

// UART IFS,IEC,IPC
#define IFS_U1RXIF_read()	(IFS1bits.U1RXIF)
#define IFS_U2RXIF_read()	(IFS1bits.U2RXIF)
#define IFS_U1RXIF_clear()	(IFS1CLR = (1 << _IFS1_U1RXIF_POSITION))
#define IFS_U2RXIF_clear()	(IFS1CLR = (1 << _IFS1_U2RXIF_POSITION))
#define IEC_U1RXIE_set(f)	(IEC1bits.U1RXIE = (f))
#define IEC_U2RXIE_set(f)	(IEC1bits.U2RXIE = (f))
#define IPC_U1IPIS_set(ip,is)	(IPC8bits.U1IP = (ip), IPC8bits.U1IS = (is))
#define IPC_U2IPIS_set(ip,is)	(IPC9bits.U2IP = (ip), IPC9bits.U2IS = (is))

// TIMER IFS,IEC,IPC
#define IFS_T1IF_clear()	(IFS0CLR= 1 << _IFS0_T1IF_POSITION)
#define IEC_T1IE_set(f)		(IEC0bits.T1IE = (f))
#define IPC_T1IPIS(ip,is)	(IPC1bits.T1IP = (ip), IPC1bits.T1IS = (is))

// Output pin selection.
#define RPxnR(x,n)	(TBL_RPxnR[(x)-1][n])
extern volatile uint32_t *TBL_RPxnR[];


/*! default pin assign.
*/
// UART
#define UART1_TXD	2,4	// B4
#define UART1_RXD	1,4	// A4
#define UART2_TXD	2,9	// B9
#define UART2_RXD	2,8	// B8

// I2C
#define I2C2_SCL	2,3	// B3
#define I2C2_SDA	2,2	// B2


// flash memory address and parameter.
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


void system_init(void);
void pin_init(void);
int set_pin_to_digital_input(int port, int num);
int set_pin_to_digital_output(int port, int num);
int set_pin_to_analog_input(int port, int num);
int set_pin_to_pwm(int unit, int port, int num);
int set_pin_to_spi(int unit, int sdi_p, int sdi_n, int sdo_p, int sdo_n, int sck_p, int sck_n);
int set_pin_to_uart(int unit, int txd_p, int txd_n, int rxd_p, int rxd_n);
int release_pin_from_peripheral(int port, int num);
void onboard_led(int num, int on_off);
int onboard_sw(int num);


#ifdef __cplusplus
}
#endif
