#include <inttypes.h>
#include <string.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_uart.h"
#include "inc/hw_timer.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"


/* Start offset for target main program. */
static const uint32_t TARGET_START = 0x1000;

#define LED_RED GPIO_PIN_1
#define LED_BLUE GPIO_PIN_2
#define LED_GREEN GPIO_PIN_3

/* To change this, must fix clock setup in the code. */
#define MCU_HZ 16000000


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
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE);
}


__attribute__ ((unused))
static void
led_red_on(void)
{
  ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED, LED_RED);
}


__attribute__ ((unused))
static void
led_red_off(void)
{
  ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED, 0);
}


__attribute__ ((unused))
static void
led_green_on(void)
{
  ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_GREEN, LED_GREEN);
}


__attribute__ ((unused))
static void
led_green_off(void)
{
  ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_GREEN, 0);
}


__attribute__ ((unused))
static void
led_blue_on(void)
{
  ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_BLUE, LED_BLUE);
}


__attribute__ ((unused))
static void
led_blue_off(void)
{
  ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_BLUE, 0);
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

  HWREG(NVIC_VTABLE) = target_start;
  __asm__ __volatile__
    ("mov  sp, %0\n\t"
     "bx   %1\n"
     :
     : "r" (stack), "r" (start)
     );
}


int main()
{
  uint32_t led_status;

  /* Use a 16 MHz system clock using the internal oscillator and PLL. */
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_12_5 | SYSCTL_USE_PLL | SYSCTL_OSC_INT);
  ROM_FPULazyStackingEnable();

  /* Configure serial. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
  ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
  ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  ROM_UARTConfigSetExpClk(UART0_BASE, (ROM_SysCtlClockGet()), 115200,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));
  serial_output_str("Wireless bootloader starting...\r\n");

  config_led();
  led_red_on();
  ROM_SysCtlDelay(MCU_HZ/3/2);
  led_red_off();
  led_green_on();
  serial_output_str("Wireless bootloader started\r\n");

  led_status = 0;
  for (;;)
  {
    if (!(led_status & 7))
      serial_output_str(".");
    if (led_status & 1)
      led_red_on();
    else
      led_red_off();
    if (led_status & 2)
      led_green_on();
    else
      led_green_off();
    if (led_status & 4)
      led_blue_on();
    else
      led_blue_off();
    ROM_SysCtlDelay(MCU_HZ/3/5);
    ++led_status;

    if (led_status >= 32)
      break;
  }

  jump_to_target(TARGET_START);
}
