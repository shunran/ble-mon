/* Copyright (c) Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of other
 *   contributors to this software may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 *   4. This software must only be used in a processor manufactured by Nordic
 *   Semiconductor ASA, or in a processor manufactured by a third party that
 *   is used in combination with a processor manufactured by Nordic Semiconductor.
 *
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "boards.h"

#include "btle.h"
#include "core_cmInstr.h"
//#include "nrf_adv_conn.h"
#include "nrf_scan.h"

#include "boards.h"

#include "ble.h"
#include "nrf51.h"
#include "nrf51_bitfields.h"
#include "nrf_assert.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_sdm.h"
#include "nrf_delay.h"
#include "app_uart.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UART_RX_BUF_SIZE        2
#define UART_TX_BUF_SIZE        256

/*****************************************************************************
* Local definitions
*****************************************************************************/

/**@brief Disable logging to UART by commenting out this line. It is recomended to do this if
 * if you want to study timing in the example using a logic analyzer.
 */
#define USE_UART_LOGGING

/**@brief Macro defined to output log data on the UART or not, based on the USE_UART_LOGGING flag.
 * If logging is disabled, it will just yield a NOP instruction.
 */
#ifdef USE_UART_LOGGING
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define __LOG(F, ...) (test_logf("TIMESLOT_TEST_LOG: %s: %d: " #F "\r\n", __FILENAME__, __LINE__, ##__VA_ARGS__))
#else
  #define __LOG(F, ...) (void)__NOP()
#endif

#define TIMESLOT_LENGTH_US 10000
#define TIMESLOT_DISTANCE_US 20000


/*****************************************************************************
* Static Globals
*****************************************************************************/

/**@brief Global variables used for storing assert information from the SoftDevice.
 */
static uint32_t g_sd_assert_line_num;
static uint32_t g_sd_assert_pc;
static uint8_t  g_sd_assert_file_name[100];

/**@brief Global variables used for storing assert information from the nRF51 SDK.
 */
static uint32_t g_nrf_assert_line_num;
static uint8_t  g_nrf_assert_file_name[100];

/**@brief Global variables used for storing assert information from the timeslot event handler.
 */
static uint32_t g_evt;

/**@brief Global variables for the scanner
 */

/* These are the parameters for the scanner running in the timeslot */
static btle_cmd_param_le_write_scan_parameters_t scan_param = {
  BTLE_SCAN_TYPE_ACTIVE,          /* Active scanning. SCAN_REQ packets may be sent */
  TIMESLOT_DISTANCE_US,           /* Time from controller starts its last scan until it begins the next scan */
  TIMESLOT_LENGTH_US,             /* Duration of the scan */
  BTLE_ADDR_TYPE_PUBLIC,          /* Use public address type */
  BTLE_SCAN_FILTER_ACCEPT_ANY     /* Accept anyone (whitelist unsupported for now) */
};

static btle_cmd_param_le_write_scan_enable_t scan_enable = {
  BTLE_SCAN_MODE_ENABLE,              /* Enable scanner */
  BTLE_SCAN_DUPLICATE_FILTER_DISABLE  /* Do not filter duplicates */
};

volatile bool sw_interrupt = false;

/*****************************************************************************
* Static Functions
*****************************************************************************/

/**@brief Callback handlers
 */
static void sd_assert_cb(uint32_t pc, uint16_t line_num, const uint8_t *file_name);

/**@brief Local function prototypes.
 */
static void test_logf(const char *fmt, ...);

static void initialize_uart(void);
void simple_uart_putstring(const uint8_t * str);

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

int initialize_observer(void)
{
  uint8_t err_code = NRF_SUCCESS;
  nrf_report_t report;
  btle_status_codes_t btle_err_code = BTLE_STATUS_CODE_SUCCESS;

  /* Silence the compiler */
  (void) g_sd_assert_pc;
  (void) g_evt;
  (void) err_code;
  (void) btle_err_code;

// nrf_gpio_cfg_output (BSP_LED_0);
//  nrf_gpio_cfg_output (BSP_LED_1);
  nrf_gpio_range_cfg_output (0, 7);

  nrf_gpio_pin_set (LED_0);

  /* Setup UART */
  initialize_uart();

  __LOG("Program init", __FUNCTION__);

  err_code = sd_softdevice_enable ((uint32_t) NRF_CLOCK_LFCLKSRC_XTAL_75_PPM, sd_assert_cb);
  ASSERT (err_code == NRF_SUCCESS);
  __LOG ("Softdevice enabled");

  err_code = sd_nvic_EnableIRQ(SD_EVT_IRQn);
  ASSERT (err_code == NRF_SUCCESS);

  err_code = sd_nvic_SetPriority(SWI0_IRQn, NRF_APP_PRIORITY_LOW);
  ASSERT (err_code == NRF_SUCCESS);

  err_code = sd_nvic_EnableIRQ(SWI0_IRQn);
  ASSERT (err_code == NRF_SUCCESS);

  __LOG ("Interrupts enabled");

  btle_err_code = btle_scan_init (SWI0_IRQn);
  ASSERT (btle_err_code == BTLE_STATUS_CODE_SUCCESS);
  __LOG ("Scanner initialized");

  btle_err_code = btle_scan_param_set (scan_param);
  ASSERT (btle_err_code == BTLE_STATUS_CODE_SUCCESS);
  __LOG ("Scanner parameters set");

  btle_err_code = btle_scan_enable_set (scan_enable);
  ASSERT (btle_err_code == BTLE_STATUS_CODE_SUCCESS);
  __LOG ("Scanner enabled");

  nrf_adv_conn_init ();

  while (true)
  {
    if (sw_interrupt)
    {
      while (btle_scan_ev_get (&report) != BTLE_STATUS_CODE_COMMAND_DISALLOWED)
      {
        __LOG("Type: %X, Addr: %X:%X:%X:%X:%X:%X, RSSI: %i",
          report.event.params.le_advertising_report_event.event_type,
          report.event.params.le_advertising_report_event.address[5],
          report.event.params.le_advertising_report_event.address[4],
          report.event.params.le_advertising_report_event.address[3],
          report.event.params.le_advertising_report_event.address[2],
          report.event.params.le_advertising_report_event.address[1],
          report.event.params.le_advertising_report_event.address[0],
          report.event.params.le_advertising_report_event.rssi);
      }

      sw_interrupt = false;
    }
  }
}

/**@brief Assert callback handler for SoftDevice asserts. */
void sd_assert_cb (uint32_t pc, uint16_t line_num, const uint8_t *file_name)
{
  g_sd_assert_line_num = line_num;
  g_sd_assert_pc = pc;
  memset ((void*)g_sd_assert_file_name, 0x00, sizeof(g_sd_assert_file_name));
  (void) strncpy ((char*) g_sd_assert_file_name, (const char*) file_name, sizeof(g_sd_assert_file_name) - 1);

  nrf_gpio_pin_set(LED_0);

   __LOG("%s: SOFTDEVICE ASSERT: line = %d file = %s", __FUNCTION__, g_sd_assert_line_num, g_sd_assert_file_name);

  while(1);
}

void assert_nrf_callback(uint16_t line_num, const uint8_t *file_name)
{
  g_nrf_assert_line_num = line_num;
  memset((void*)g_nrf_assert_file_name, 0x00, sizeof (g_nrf_assert_file_name));
  (void) strncpy ((char*) g_nrf_assert_file_name, (const char*) file_name, sizeof (g_nrf_assert_file_name) - 1);

  nrf_gpio_pin_set(LED_1);

   __LOG("%s: NRF ASSERT: line = %d file = %s", __FUNCTION__, g_nrf_assert_line_num, g_nrf_assert_file_name);

  while (1);
}

/**@brief BLE Stack event interrupt
 *        Triggered whenever an event is ready to be pulled
 */
void SD_EVT_IRQHandler (void)
{
  uint32_t evt;
  ble_evt_t ble_evt;
  uint16_t len;

  while (sd_evt_get(&evt) == NRF_SUCCESS)
  {
    g_evt = evt;

    switch (evt)
    {
      case NRF_EVT_RADIO_SESSION_IDLE:
      case NRF_EVT_RADIO_BLOCKED:
        /* Request a new timeslot */
        ASSERT (btle_scan_enable_set (scan_enable) == BTLE_STATUS_CODE_SUCCESS);
        break;

      case NRF_EVT_RADIO_SESSION_CLOSED:
        break;

      case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
        ASSERT(false);
        break;

      case NRF_EVT_RADIO_CANCELED:
        ASSERT (btle_scan_enable_set (scan_enable) == BTLE_STATUS_CODE_SUCCESS);
        break;

      default:
        /* This should not happen */
        __LOG ("%s: Program failure, undefined event = %d", __FUNCTION__, evt);
        ASSERT(false);
    }
  }

  while (sd_ble_evt_get((uint8_t *) &evt, &len) == NRF_SUCCESS)
  {
    nrf_adv_conn_evt_handler(&ble_evt);
  }
}

/**@brief Timeslot event interrupt
 *        Triggered whenever an event is ready to be pulled
 */
void SWI0_IRQHandler(void)
{
  sw_interrupt = true;
}

/**@brief Logging function, used for formated output on the UART.
 */
void test_logf(const char *fmt, ...)
{
  int16_t res = 0;
  (void)res; /* silence compiler */
  static uint8_t buf[150];

  va_list args;
  va_start(args, fmt);

  res = vsnprintf((char*) buf, sizeof(buf), fmt,  args);
  ASSERT(res >= 0 && res <= (sizeof buf) - 1);

  simple_uart_putstring(buf);

  va_end(args);
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

/**@brief Initialize UART.
 */
static void initialize_uart(void)
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
      LEDS_INVERT(1 << LED_1);
      nrf_delay_ms(500);
    }
  }
}



