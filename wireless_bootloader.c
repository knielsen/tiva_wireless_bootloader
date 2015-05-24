#include <inttypes.h>
#include <string.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_ssi.h"
#include "inc/hw_uart.h"
#include "inc/hw_timer.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/ssi.h"
#include "driverlib/systick.h"
#include "driverlib/flash.h"

#include "nrf24l01p.h"


/*
  nRF24L01 pinout:

  Tx:
    PF2  SCK        GND *1 2. VCC
    PF3  CSN        PB3 .3 4. PF3
    PF0  MISO       PF2 .5 6. PF1
    PF1  MOSI       PF0 .7 8. PB0
    PB0  IRQ
    PB3  CE
*/


/* Start offset for target main program. */
static const uint32_t TARGET_START = 0x1000;
static const uint32_t TARGET_END = 0x100000;

/*
  Note that SSI pin assignments need changing in config_ssi_gpio to change
  to something other than SSI1.
*/
#define NRF_SSI_PERIPH SYSCTL_PERIPH_SSI1
#define NRF_SSI_GPIO_PERIPH SYSCTL_PERIPH_GPIOF
#define NRF_SSI_BASE SSI1_BASE
#define NRF_CSN_PERIPH SYSCTL_PERIPH_GPIOF
#define NRF_CSN_BASE GPIO_PORTF_BASE
#define NRF_CSN_PIN GPIO_PIN_3
#define NRF_CE_PERIPH SYSCTL_PERIPH_GPIOB
#define NRF_CE_BASE GPIO_PORTB_BASE
#define NRF_CE_PIN GPIO_PIN_3
#define NRF_IRQ_PERIPH SYSCTL_PERIPH_GPIOB
#define NRF_IRQ_BASE GPIO_PORTB_BASE
#define NRF_IRQ_PIN GPIO_PIN_0


/* Communications protocol. */
#define POV_CMD_DEBUG 254
#define POV_SUBCMD_RESET_TO_BOOTLOADER 255
#define POV_SUBCMD_ENTER_BOOTLOADER 254
#define POV_SUBCMD_RESET_TO_APP 253
#define POV_SUBCMD_FLASH_BUFFER 252
#define POV_SUBCMD_EXIT_DEBUG   251
#define POV_SUBCMD_STATUS_REPLY 240


/* To change this, must fix clock setup in the code. */
#define MCU_HZ 80000000


static void
serial_output_hexdig(uint32_t dig)
{
  ROM_UARTCharPut(UART0_BASE, (dig >= 10 ? 'A' - 10 + dig : '0' + dig));
}


 __attribute__ ((unused))
static void
serial_output_hexbyte(uint8_t byte)
{
  serial_output_hexdig(byte >> 4);
  serial_output_hexdig(byte & 0xf);
}


static void
serial_output_str(const char *str)
{
  char c;

  while ((c = *str++))
    ROM_UARTCharPut(UART0_BASE, c);
}


 __attribute__ ((unused))
static void
println_uint32(uint32_t val)
{
  char buf[12];
  char *p = buf;
  uint32_t l, d;

  l = 1000000000UL;
  while (l > val && l > 1)
    l /= 10;

  do
  {
    d = val / l;
    *p++ = '0' + d;
    val -= d*l;
    l /= 10;
  } while (l > 0);

  *p++ = '\n';
  *p = '\0';
  serial_output_str(buf);
}


static void
config_led(void)
{
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_7);
}


__attribute__ ((unused))
static void
led_on(void)
{
  GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
}


__attribute__ ((unused))
static void
led_off(void)
{
  GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
}


static void
config_ssi_gpio(void)
{
  /* Config Tx on SSI1, PF0-PF3 + PB0/PB3. */
  ROM_SysCtlPeripheralEnable(NRF_SSI_PERIPH);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

  /* PF0 is special (NMI), needs unlock to be re-assigned to SSI1. */
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY_DD;
  HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

  ROM_GPIOPinConfigure(GPIO_PF2_SSI1CLK);
  ROM_GPIOPinConfigure(GPIO_PF0_SSI1RX);
  ROM_GPIOPinConfigure(GPIO_PF1_SSI1TX);
  ROM_GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
  /* CSN pin, high initially */
  ROM_SysCtlPeripheralEnable(NRF_CSN_PERIPH);
  ROM_GPIOPinTypeGPIOOutput(NRF_CSN_BASE, NRF_CSN_PIN);
  ROM_GPIOPinWrite(NRF_CSN_BASE, NRF_CSN_PIN, NRF_CSN_PIN);
  /* CE pin, low initially */
  ROM_SysCtlPeripheralEnable(NRF_CE_PERIPH);
  ROM_GPIOPinTypeGPIOOutput(NRF_CE_BASE, NRF_CE_PIN);
  ROM_GPIOPinWrite(NRF_CE_BASE, NRF_CE_PIN, 0);
  /* IRQ pin as input. */
  ROM_SysCtlPeripheralEnable(NRF_IRQ_PERIPH);
  ROM_GPIOPinTypeGPIOInput(NRF_IRQ_BASE, NRF_IRQ_PIN);
}


static void
config_spi(uint32_t base)
{
  /*
    Configure the SPI for correct mode to read from nRF24L01+.

    We need CLK inactive low, so SPO=0.
    We need to setup and sample on the leading, rising CLK edge, so SPH=0.

    The datasheet says up to 10MHz SPI is possible, depending on load
    capacitance. Let's go with a slightly cautious 8MHz, which should be
    aplenty.
  */

  ROM_SSIDisable(base);
  ROM_SSIConfigSetExpClk(base, ROM_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                         SSI_MODE_MASTER, 8000000, 8);
  ROM_SSIEnable(base);
}


static void
bzero(uint8_t *buf, uint32_t len)
{
  while (len > 0)
  {
    *buf++ = 0;
    --len;
  }
}


static void
my_memset(uint8_t *dst, uint8_t val, uint32_t size)
{
  while (size > 0)
  {
    *dst++ = val;
    --size;
  }
}


static inline void
csn_low(uint32_t csn_base, uint32_t csn_pin)
{
  ROM_GPIOPinWrite(csn_base, csn_pin, 0);
}


static inline void
csn_high(uint32_t csn_base, uint32_t csn_pin)
{
  ROM_GPIOPinWrite(csn_base, csn_pin, csn_pin);
}


static inline void
ce_low(uint32_t ce_base, uint32_t ce_pin)
{
  ROM_GPIOPinWrite(ce_base, ce_pin, 0);
}


static inline void
ce_high(uint32_t ce_base, uint32_t ce_pin)
{
  ROM_GPIOPinWrite(ce_base, ce_pin, ce_pin);
}


static void
ssi_cmd(uint8_t *recvbuf, const uint8_t *sendbuf, uint32_t len,
        uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint32_t i;
  uint32_t data;

  /* Take CSN low to initiate transfer. */
  csn_low(csn_base, csn_pin);

  for (i = 0; i < len; ++i)
  {
    ROM_SSIDataPut(ssi_base, sendbuf[i]);
    while (ROM_SSIBusy(ssi_base))
      ;
    ROM_SSIDataGet(ssi_base, &data);
    recvbuf[i] = data;
  }

  /* Take CSN high to complete transfer. */
  csn_high(csn_base, csn_pin);
}


static void
nrf_rx(uint8_t *data, uint32_t len,
       uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[33], recvbuf[33];

  if (len > 32)
    len = 32;
  sendbuf[0] = nRF_R_RX_PAYLOAD;
  bzero(&sendbuf[1], len);
  ssi_cmd(recvbuf, sendbuf, len+1, ssi_base, csn_base, csn_pin);
  memcpy(data, &recvbuf[1], len);
}


static void
nrf_tx(uint8_t *data, uint32_t len,
       uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[33], recvbuf[33];

  if (len > 32)
    len = 32;
  sendbuf[0] = nRF_W_TX_PAYLOAD;
  memcpy(&sendbuf[1], data, len);
  ssi_cmd(recvbuf, sendbuf, len+1, ssi_base, csn_base, csn_pin);
}


static void
nrf_flush_tx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t cmd = nRF_FLUSH_TX;
  uint8_t status;
  ssi_cmd(&status, &cmd, 1, ssi_base, csn_base, csn_pin);
}


static void
nrf_flush_rx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t cmd = nRF_FLUSH_RX;
  uint8_t status;
  ssi_cmd(&status, &cmd, 1, ssi_base, csn_base, csn_pin);
}


static void
nrf_write_reg_n(uint8_t reg, const uint8_t *data, uint32_t len,
                uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[6], recvbuf[6];
  if (len > 5)
    len = 5;
  sendbuf[0] = nRF_W_REGISTER | reg;
  memcpy(&sendbuf[1], data, len);
  ssi_cmd(recvbuf, sendbuf, len+1, ssi_base, csn_base, csn_pin);
}


static void
nrf_write_reg(uint8_t reg, uint8_t val,
              uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  nrf_write_reg_n(reg, &val, 1, ssi_base, csn_base, csn_pin);
}


static void
nrf_read_reg_n(uint8_t reg, uint8_t *out, uint32_t len,
               uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t sendbuf[6];
  if (len > 5)
    len = 5;
  sendbuf[0] = nRF_R_REGISTER | reg;
  bzero(&sendbuf[1], len);
  ssi_cmd(out, sendbuf, len+1, ssi_base, csn_base, csn_pin);
}


static uint8_t
nrf_read_reg(uint8_t reg, uint8_t *status_ptr,
             uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t recvbuf[2];
  nrf_read_reg_n(reg, recvbuf, 2, ssi_base, csn_base, csn_pin);
  if (status_ptr)
    *status_ptr = recvbuf[0];
  return recvbuf[1];
}


static const uint8_t nrf_addr[3] = { 0xe7, 0xe7, 0xe7 };

/*
  Configure nRF24L01+ as Rx or Tx.
    channel - radio frequency channel to use, 0 <= channel <= 127.
    power - nRF_RF_PWR_<X>DBM, <X> is 0, 6, 12, 18 dBm.
*/
static void
nrf_init_config(uint8_t is_rx, uint32_t channel, uint32_t power,
                uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  if (is_rx)
    nrf_write_reg(nRF_CONFIG, nRF_PRIM_RX | nRF_MASK_TX_DS |
                  nRF_MASK_MAX_RT|nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP,
                  ssi_base, csn_base, csn_pin);
  else
    nrf_write_reg(nRF_CONFIG, nRF_MASK_RX_DR |
                  nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP,
                  ssi_base, csn_base, csn_pin);
  /* Enable auto-ack. */
  nrf_write_reg(nRF_EN_AA, nRF_ENAA_P0|nRF_ENAA_P1|nRF_ENAA_P2|
                           nRF_ENAA_P3|nRF_ENAA_P4|nRF_ENAA_P5,
                ssi_base, csn_base, csn_pin);
  /* Enable only pipe 0. */
  nrf_write_reg(nRF_EN_RXADDR, nRF_ERX_P0, ssi_base, csn_base, csn_pin);
  /* 3 byte adresses. */
  nrf_write_reg(nRF_SETUP_AW, nRF_AW_3BYTES, ssi_base, csn_base, csn_pin);
  /* Enable auto retransmit. */
  nrf_write_reg(nRF_SETUP_RETR, (1 << nRF_ARD_SHIFT) | 15,
                ssi_base, csn_base, csn_pin);
  nrf_write_reg(nRF_RF_CH, channel, ssi_base, csn_base, csn_pin);
  /* Use 2Mbps, and set transmit power. */
  nrf_write_reg(nRF_RF_SETUP, nRF_RF_DR_HIGH | power,
                ssi_base, csn_base, csn_pin);
  nrf_write_reg_n(nRF_RX_ADDR_P0, nrf_addr, 3, ssi_base, csn_base, csn_pin);
  nrf_write_reg_n(nRF_TX_ADDR, nrf_addr, 3, ssi_base, csn_base, csn_pin);
  /* Set payload size for pipe 0. */
  nrf_write_reg(nRF_RX_PW_P0, 32, ssi_base, csn_base, csn_pin);
  /* Disable pipe 1-5. */
  nrf_write_reg(nRF_RX_PW_P1, 0, ssi_base, csn_base, csn_pin);
  /* Disable dynamic payload length. */
  nrf_write_reg(nRF_DYNDP, 0, ssi_base, csn_base, csn_pin);
  /* Allow disabling acks. */
  nrf_write_reg(nRF_FEATURE, nRF_EN_DYN_ACK, ssi_base, csn_base, csn_pin);

  /* Clear out all FIFOs. */
  nrf_flush_tx(ssi_base, csn_base, csn_pin);
  nrf_flush_rx(ssi_base, csn_base, csn_pin);
  /* Clear the IRQ bits in STATUS register. */
  nrf_write_reg(nRF_STATUS, nRF_RX_DR|nRF_TX_DS|nRF_MAX_RT,
                ssi_base, csn_base, csn_pin);
}


/*
  Configure nRF24L01+ as Tx for replying status from the bootloader.

  The nRF24L01+ is configured as transmitter, with auto-ack and
  retransmission enabled.
*/
static void
nrf_config_bootload_tx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  nrf_write_reg(nRF_CONFIG, nRF_MASK_RX_DR |
                  nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP,
                  ssi_base, csn_base, csn_pin);
  /* Enable auto-ack. */
  nrf_write_reg(nRF_EN_AA, nRF_ENAA_P0|nRF_ENAA_P1|nRF_ENAA_P2|
                           nRF_ENAA_P3|nRF_ENAA_P4|nRF_ENAA_P5,
                ssi_base, csn_base, csn_pin);
  /* Enable auto retransmit. */
  nrf_write_reg(nRF_SETUP_RETR, (1 << nRF_ARD_SHIFT) | 15,
                ssi_base, csn_base, csn_pin);

  /* Clear out all FIFOs. */
  nrf_flush_tx(ssi_base, csn_base, csn_pin);
  nrf_flush_rx(ssi_base, csn_base, csn_pin);
  /* Clear the IRQ bits in STATUS register. */
  nrf_write_reg(nRF_STATUS, nRF_RX_DR|nRF_TX_DS|nRF_MAX_RT,
                ssi_base, csn_base, csn_pin);
}


/*
  Configure nRF24L01+ as Rx for getting commands for the bootloader.

  The nRF24L01+ is configured as receiver, with auto-ack and
  retransmission enabled.
*/
static void
nrf_config_bootload_rx(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  nrf_write_reg(nRF_CONFIG, nRF_PRIM_RX | nRF_MASK_TX_DS |
                  nRF_MASK_MAX_RT|nRF_EN_CRC|nRF_CRCO|nRF_PWR_UP,
                  ssi_base, csn_base, csn_pin);
  /* Enable auto-ack. */
  nrf_write_reg(nRF_EN_AA, nRF_ENAA_P0|nRF_ENAA_P1|nRF_ENAA_P2|
                           nRF_ENAA_P3|nRF_ENAA_P4|nRF_ENAA_P5,
                ssi_base, csn_base, csn_pin);
  /* Enable auto retransmit. */
  nrf_write_reg(nRF_SETUP_RETR, (1 << nRF_ARD_SHIFT) | 15,
                ssi_base, csn_base, csn_pin);

  /* Clear out all FIFOs. */
  nrf_flush_tx(ssi_base, csn_base, csn_pin);
  nrf_flush_rx(ssi_base, csn_base, csn_pin);
  /* Clear the IRQ bits in STATUS register. */
  nrf_write_reg(nRF_STATUS, nRF_RX_DR|nRF_TX_DS|nRF_MAX_RT,
                ssi_base, csn_base, csn_pin);
}


static void
setup_systick(void)
{
  ROM_SysTickPeriodSet(0xffffff+1);
  /* Force reload. */
  HWREG(NVIC_ST_CURRENT) = 0;
  ROM_SysTickEnable();
}


static inline uint32_t
get_time(void)
{
  return HWREG(NVIC_ST_CURRENT);
}


static inline uint32_t
calc_time_from_val(uint32_t start, uint32_t stop)
{
  return (start - stop) & 0xffffff;
}


static inline uint32_t
calc_time(uint32_t start)
{
  uint32_t stop = HWREG(NVIC_ST_CURRENT);
  return calc_time_from_val(start, stop);
}


static inline uint32_t
dec_time(uint32_t val, uint32_t inc)
{
  return (val - inc) & 0xffffff;
}


/*
  Delay until specified amount of systicks have passed.

  As systick is a 24-bit counter, the amount cannot exceed 0xffffff, or a bit
  more than 16000000.
*/
static void
delay_systicks(uint32_t cycles)
{
  uint32_t start = get_time();

  while (calc_time(start) < cycles)
    ;
}


static void
delay_us(uint32_t us)
{
  /* This assumes that MCU_HZ is divisible by 1000000. */
  uint32_t cycles = (MCU_HZ/1000000)*us;
#if (MCU_HZ % 1000000)
#error delay_us() computes delay incorrectly if MCU_HZ is not a multiple of 1000000
#endif

  while (cycles > 0xffffff)
  {
    delay_systicks(0xffffff);
    cycles -= 0xffffff;
  }
  delay_systicks(cycles);
}


/*
  Read both the normal and FIFO status registers.
  Returns normal status or'ed with (fifo status left-shifted 8).
*/
static uint32_t
nrf_get_status(uint32_t ssi_base, uint32_t csn_base, uint32_t csn_pin)
{
  uint8_t status;
  uint32_t fifo_status;

  fifo_status =
    nrf_read_reg(nRF_FIFO_STATUS, &status, ssi_base, csn_base, csn_pin);
  return (fifo_status << 8) | status;
}


static uint32_t
nrf_transmit_packet(uint8_t *packet)
{
  uint32_t start_time = get_time();

  nrf_tx(packet, 32, NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
  ce_high(NRF_CE_BASE, NRF_CE_PIN);
  delay_us(10);
  ce_low(NRF_CE_BASE, NRF_CE_PIN);

  for (;;)
  {
    uint32_t status = nrf_get_status(NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
    if (status & nRF_MAX_RT)
    {
      serial_output_str("No ack from receiver\r\n");
      return 1;
    }
    if (status & nRF_TX_DS)
      return 0;
    if (calc_time(start_time) > MCU_HZ/5)
    {
      serial_output_str("Timeout from nRF24L01+ waiting for transmit\r\n");
      return 1;
    }
  }
}


/*
  Invoke the real target application.

  We need to relocate the interrupt vector, load the stack pointer from
  vector 0, and then jump to vector 1.
*/
static void
jump_to_target(uint32_t target_start)
{
  uint32_t stack = ((uint32_t *)target_start)[0];
  uint32_t start = ((uint32_t *)target_start)[1];

  /* De-initialise the nRF24L01+, powering it down. */
  nrf_write_reg(nRF_CONFIG, nRF_PRIM_RX | nRF_MASK_RX_DR | nRF_MASK_TX_DS |
                nRF_MASK_MAX_RT|nRF_EN_CRC|nRF_CRCO,
                NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
  csn_high(NRF_CSN_BASE, NRF_CSN_PIN);
  ce_low(NRF_CE_BASE, NRF_CE_PIN);

  /* ToDo: maybe de-initialise some stuff:
      - systicks
      - system clock
      - GPIOs B and F
      - SSI
  */

  serial_output_str("Boot to target\r\n");

  HWREG(NVIC_VTABLE) = target_start;
  __asm__ __volatile__
    ("mov  sp, %0\n\t"
     "bx   %1\n"
     :
     : "r" (stack), "r" (start)
     );
}


static uint32_t flash_buffer[1024/sizeof(uint32_t)];


static void
nrf_send_status_reply(uint8_t *packet_buf, uint8_t status)
{
  ce_low(NRF_CE_BASE, NRF_CE_PIN);
  nrf_config_bootload_tx(NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
  bzero(packet_buf, 32);
  packet_buf[0] = POV_CMD_DEBUG;
  packet_buf[1] = POV_SUBCMD_STATUS_REPLY;
  packet_buf[2] = status;
  if (nrf_transmit_packet(packet_buf))
    jump_to_target(TARGET_START);
  nrf_config_bootload_rx(NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
  ce_high(NRF_CE_BASE, NRF_CE_PIN);
}


/*
  Write one 1024-byte block of flash memory.

  First check if we need to erase, and if we need to write at all.
  (We do not need to write identical data, and we only need to erase to
  change a "0" bit to a "1").
*/
static void
check_erase_and_flash(uint32_t address, uint32_t *data)
{
  uint32_t i, *ptr;
  ptr = (uint32_t *)address;
  uint32_t do_write = 0, do_erase = 0;

  for (i = 0; i < 1024/sizeof(uint32_t); ++i)
  {
    uint32_t old = ptr[i];
    uint32_t new = data[i];
    if (~old & new)
      do_erase = 1;
    if (old != new)
      do_write = 1;
  }
  if (do_erase && ROM_FlashErase(address))
    serial_output_str("Error during flash erase\r\n");
  if (do_write && ROM_FlashProgram(data, address, 1024))
    serial_output_str("Error during flash write\r\n");
}


int main()
{
  uint32_t start_time, wait_counter;
  uint8_t status;
  uint8_t val;
  uint8_t packet_buf[32];

  /* Use a 16 MHz system clock using the internal oscillator and PLL. */
  //ROM_SysCtlClockSet(SYSCTL_SYSDIV_12_5 | SYSCTL_USE_PLL | SYSCTL_OSC_INT);
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
                     SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  ROM_FPULazyStackingEnable();
  setup_systick();

  /* Configure serial. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
  ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
  ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  ROM_UARTConfigSetExpClk(UART0_BASE, (ROM_SysCtlClockGet()), 500000,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));
  serial_output_str("Wireless bootloader starting...\r\n");

  config_ssi_gpio();
  config_spi(NRF_SSI_BASE);
  config_led();
  /* nRF24L01+ datasheet says to wait 100msec for bootup. */
  ROM_SysCtlDelay(MCU_HZ/3/10);

  nrf_init_config(0 /* Tx */, 2, nRF_RF_PWR_0DBM,
                  NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
  serial_output_str("Tx: Read CONFIG=0x");
  val = nrf_read_reg(nRF_CONFIG, &status,
                     NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
  serial_output_hexbyte(val);
  serial_output_str(" status=0x");
  serial_output_hexbyte(status);
  serial_output_str("\r\n");

  serial_output_str("Wireless bootloader started\r\n");

  nrf_config_bootload_rx(NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
  /* Assert CE to start receiving. */
  ce_high(NRF_CE_BASE, NRF_CE_PIN);

  start_time = get_time();
  /*
    Systick can only count up to about 0.2 seconds. We want to wait for longer
    for pov_sender to contact us, so count in 0.1-seconds intervals.

    (I prefer to keep the bootloader as simple as possible, thus no timer
    interrupt).
  */
  wait_counter = 20;  /* 2 seconds */
  while (wait_counter > 0)
  {
    uint32_t now_time;
    uint32_t status = nrf_get_status(NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
    if (status & nRF_RX_DR)
      break;                                    /* Data ready. */
    now_time = get_time();
    if (calc_time_from_val(start_time, now_time) > MCU_HZ/10)
    {
      --wait_counter;
      start_time = dec_time(start_time, MCU_HZ/10);
      serial_output_str(".");
    }
  }
  serial_output_str("\r\n");

  /* If no packet received within 2 seconds, proceed to boot target app. */
  if (!wait_counter)
  {
    serial_output_str("No programmer found\r\n");
    jump_to_target(TARGET_START);
  }

  /*
    Check if we got a "hello" packet. If we did, send a reply and start
    processing bootloader commands. But if not, just boot to the target
    application.
  */
  nrf_rx(packet_buf, 32, NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
  if (packet_buf[0] != POV_CMD_DEBUG ||
      packet_buf[1] != POV_SUBCMD_ENTER_BOOTLOADER)
  {
    serial_output_str("Got unexpected packet\r\n");
    jump_to_target(TARGET_START);
  }

  /* Now reply with a status packet so sender knows we are here. */
  serial_output_str("Got request from programmer, sending ack\r\n");
  /*
    For some reason this delay is needed. Else we time out waiting for the
    TX_DS flag.
  */
  delay_us(100);
  nrf_send_status_reply(packet_buf, 0);
  serial_output_str("Ack send, starting to process cmds\r\n");

  my_memset((uint8_t *)flash_buffer, 0xff, sizeof(flash_buffer));
  /* Now loop, processing received commands. */
  for (;;)
  {
    /* Clear the "data ready flag", then check fifo to avoid races. */
    nrf_write_reg(nRF_STATUS, nRF_RX_DR,
                  NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
    /* Read any packets in the Rx fifo. */
    while (!(nRF_RX_EMPTY & nrf_read_reg(nRF_FIFO_STATUS, NULL, NRF_SSI_BASE,
                                         NRF_CSN_BASE, NRF_CSN_PIN)))
    {
      uint32_t do_reply;
      uint8_t reply_status;
      uint8_t subcmd;

      nrf_rx(packet_buf, 32, NRF_SSI_BASE, NRF_CSN_BASE, NRF_CSN_PIN);
      if (packet_buf[0] != POV_CMD_DEBUG)
      {
        serial_output_str("Got non-debug packet\r\n");
        jump_to_target(TARGET_START);
      }

      subcmd = packet_buf[1];
      //serial_output_str("P:");
      //println_uint32(subcmd);
      if (subcmd >= 0 && subcmd <= 34)
      {
        /* Load flash data. */
        uint32_t start = subcmd * 30;
        uint32_t len = 30;
        if (start + len > 1024)
          len = 1024 - start;
        memcpy((uint8_t *)flash_buffer + start, &packet_buf[2], len);
        do_reply = 0;
      }
      else if (subcmd == POV_SUBCMD_FLASH_BUFFER)
      {
        uint32_t block;
        block = packet_buf[2] |
          ((uint32_t)packet_buf[3] << 8) |
          ((uint32_t)packet_buf[4] << 16) |
          ((uint32_t)packet_buf[5] << 24);
        if (block < TARGET_START/1024 || block >= TARGET_END/1024)
        {
          serial_output_str("Invalid block to flash\r\n");
          reply_status = 1;
        }
        else
        {
          check_erase_and_flash(block*1024, flash_buffer);
          reply_status = 0;
        }
        /* Clear the buffer in case of partial load of next flash data. */
        my_memset((uint8_t *)flash_buffer, 0xff, sizeof(flash_buffer));
        do_reply = 1;
      }
      else if (subcmd == POV_SUBCMD_RESET_TO_APP ||
               subcmd == POV_SUBCMD_EXIT_DEBUG)
      {
        serial_output_str("Remote requested boot of target app\r\n");
        jump_to_target(TARGET_START);
        /* NotReached. */
        do_reply = 0;
      }
      else if (subcmd == POV_SUBCMD_ENTER_BOOTLOADER)
      {
        /* Apparently flasher started over, let's send an ack and proceed. */
        my_memset((uint8_t *)flash_buffer, 0xff, sizeof(flash_buffer));
        reply_status = 0;
        do_reply = 1;
      }
      else
      {
        do_reply = 0;
      }

      if (do_reply)
      {
        nrf_send_status_reply(packet_buf, reply_status);
        serial_output_str("*");
      }
    }
    /* Wait for more data to arrive. */
    while (!(nRF_RX_DR & nrf_get_status(NRF_SSI_BASE,
                                        NRF_CSN_BASE, NRF_CSN_PIN)))
      ;
  }
}
