#include <stdarg.h>
#include <stdio.h>

#include "nordic_common.h"
#include "nrf.h"
#include "boards.h"
#include "app_uart.h"
#include "nrf_assert.h"
#include "nrf_delay.h"


#define UART_RX_BUF_SIZE        2
#define UART_TX_BUF_SIZE        256


void uart_event_handle(app_uart_evt_t * p_event)
{
  switch (p_event->evt_type)
  {
  case APP_UART_DATA_READY:
    break;
  case APP_UART_FIFO_ERROR:
    break;
  case APP_UART_COMMUNICATION_ERROR:
    break;
  case APP_UART_TX_EMPTY:
    break;
  case APP_UART_DATA:
    break;
  default:
    break;
  }
  return;
}

/**@brief Initialize UART.
 */
void uart_init(void)
{
  int status = NRF_SUCCESS;
  const app_uart_comm_params_t uart_params = {
    .rx_pin_no    = RX_PIN_NUMBER,
    .tx_pin_no    = TX_PIN_NUMBER,
    .rts_pin_no   = RTS_PIN_NUMBER,
    .cts_pin_no   = CTS_PIN_NUMBER,
    .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
    .use_parity   = false,
    .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud38400
  };
  APP_UART_FIFO_INIT(&uart_params,
                     UART_RX_BUF_SIZE,
                     UART_TX_BUF_SIZE,
                     uart_event_handle,
                     APP_IRQ_PRIORITY_LOW,
                     status);
  if (status != NRF_SUCCESS)
  {
    while (true)
    {
      LEDS_INVERT(1 << LED_0);
      nrf_delay_ms(500);
    }
  }
}

/**@brief Writes null terminated string to the UART.
 */
void simple_uart_putstring(const uint8_t * str)
{
  uint_fast8_t i  = 0;
  uint8_t      ch = str[i++];
  while (ch != '\0')
  {
    while(NRF_SUCCESS != app_uart_put(ch));
    ch = str[i++];
  }
}

/**@brief Logging function, used for formated output on the UART.
 */
void test_logf(const char *fmt, ...)
{
  int16_t res = 0;
  (void)res; // silence compiler
  static uint8_t buf[60];

  va_list args;
  va_start(args, fmt);

  res = vsnprintf((char*) buf, sizeof(buf), fmt,  args);
  ASSERT(res >= 0 && res <= (sizeof buf) - 1);

  simple_uart_putstring(buf);

  va_end(args);
  /**/
}
