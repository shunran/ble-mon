/*
 * BLE MON
 */

// TODO: implement advertiser. OK.
// TODO: implement observer. OK.
// TODO: implement time. OK.
// TODO: implement storage
// TODO: implement connection and data download
// TODO: add rssi field to advertiser if needed
// TODO: whitelist in scanning and advertising only others and admin phone.

//#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include "nordic_common.h"
#include <nrf.h>

#include "nrf_soc.h"
//#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_nus.h"
//#include "ble_advertising.h"

#include "boards.h"
//#include "device_manager.h"
//#include "app_trace.h"
//#include "bsp.h"
//#include "bsp_btn_ble.h"

#include "uart.h"
#include "timer.h"
#include "contact.h"
#include "radio.h"
#include "storage.h"

#define DEVICE_NAME                      "sensor"                               /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "NordicSemiconductor"                      /**< Manufacturer. Will be passed to Device Information Service. */


#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//static dm_application_instance_t        m_app_handle;                               /**< Application identifier allocated by device manager */

#define TIMESLOT_LENGTH_US 			10000
#define TIMESLOT_DISTANCE_US		20000


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 /
static void sleep_mode_enter(void)
{
    uint32_t err_code = NRF_SUCCESS;
    //TODO: no bsp
    //bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    //TODO: no bsp
    err_code = NRF_SUCCESS;
    //bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}*/


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
/*
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    // Not used as we dont have bsp
	//uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            // NO bsp, TODO: rewrite for ble400 board
           //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            //APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}*/

/*
static pstorage_handle_t m_p_storage_id;

static void storage_init()
{
    uint32_t err_code;
    err_code = UINT32_MAX;

    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    pstorage_module_param_t p_storage_param;

    // Setup pstorage with 60blocks of 16byte in each.
    p_storage_param.block_size  = 16; // recommended to be >= 4 byte
    p_storage_param.block_count = 128;
    //p_storage_param.cb          = NULL;
    //130 too much, 128 ok. = 2k bytes
    err_code = pstorage_register (&p_storage_param, &m_p_storage_id);
    APP_ERROR_CHECK(err_code);
}*/

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(void)
{
	nrf_gpio_range_cfg_output(LED_0, LED_4);
	const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;
	for (int i = 0; i < LEDS_NUMBER; i++)
	        {
	            LEDS_INVERT(1 << leds_list[i]);
	            nrf_delay_ms(100);
	        }
	        for (int i = 0; i < LEDS_NUMBER; i++)
	        {
	            LEDS_INVERT(1 << leds_list[i]);
	            nrf_delay_ms(100);
	        }
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/*int initialize_observer(void)
{
  uint8_t err_code = NRF_SUCCESS;
  nrf_report_t report;
  btle_status_codes_t btle_err_code = BTLE_STATUS_CODE_SUCCESS;

  / Silence the compiler /
  (void) g_sd_assert_pc;
  (void) g_evt;
  (void) err_code;
  (void) btle_err_code;

// nrf_gpio_cfg_output (BSP_LED_0);
//  nrf_gpio_cfg_output (BSP_LED_1);
  nrf_gpio_range_cfg_output (0, 7);
  nrf_gpio_pin_set (LED_0);

  / Setup UART *
  //initialize_uart();

  __LOG("Program init", __FUNCTION__);

  //err_code = sd_softdevice_enable ((uint32_t) NRF_CLOCK_LFCLKSRC_XTAL_75_PPM, sd_assert_cb);
  //ASSERT (err_code == NRF_SUCCESS);
  //__LOG ("Softdevice enabled");

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
}*/

/**@brief Function for initializing the UART.
 */
/*
static void uart_init(void)
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
      {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud38400
      };
//#define RX_PIN_NUMBER  11
//#define TX_PIN_NUMBER  9
//#define CTS_PIN_NUMBER 10
//#define RTS_PIN_NUMBER 8

    APP_UART_FIFO_INIT(&comm_params,
                        UART_RX_BUF_SIZE,
                        UART_TX_BUF_SIZE,
                        uart_event_handle,
                        APP_IRQ_PRIORITY_LOW,
                        err_code);

    APP_ERROR_CHECK(err_code);
}*/


/**@brief Function for application main entry.
 */

int main(void)
{
	//uint32_t err_code;
	//nrf_report_t report;
	//btle_status_codes_t btle_err_code;
	uart_init();
   	__LOG("UART initialized");
	buttons_leds_init();
   	__LOG("LEDS GPIO initialized");
    timers_init();
   	__LOG("TIMERS initialized");
    // NO BSP, set erase bonds manually:
    //
    //bool erase_bonds = true;
    ble_stack_init();
   	__LOG("Bluetooth stack initialized");
    //device_manager_init(erase_bonds);
    gap_params_init();
    services_init();
    advertising_init();
   	__LOG("ADV initialized");
    conn_params_init();
    __LOG("Connection params set");
   	storage_init();

   	contacts_init();
   	__LOG("Contacts initialized");

    // Start execution.
    timers_start();
    advertising_start();
   	__LOG("ADV Started");
   	scan_start();
   	__LOG("Scan Started");

    // Enter main loop.
    for (;;)
    {
        power_manage();
    }
}

/**
 * @}
 */
