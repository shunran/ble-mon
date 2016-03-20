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
#include "pstorage.h"
#include "pstorage_platform.h"
//#include "app_trace.h"
//#include "bsp.h"
//#include "bsp_btn_ble.h"

#include "uart.h"
#include "timer.h"
#include "contact.h"
#include "radio.h"

// Scanner
//#include "btle.h"

#define DEVICE_NAME                      "sensor"                               /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "NordicSemiconductor"                      /**< Manufacturer. Will be passed to Device Information Service. */




//#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
//#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

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


//#define PSTORAGE_MIN_BLOCK_SIZE     4



// YOUR_JOB: Use UUIDs for service(s) used in your application.
//static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */

                                   
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



/**@brief Function for handling the YYY Service events. 
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
static void on_yys_evt(ble_yy_service_t     * p_yy_service, 
                       ble_yy_service_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. \r\n", p_evt->params.char_xx.value.p_str);
            break;
        
        default:
            // No implementation needed.
            break;
    }
}*/

/**@brief Function for initializing services that will be used by the application.
 */
/*static void services_init(void)
{
    / YOUR_JOB: Add code to initialize the services used by the application.
    uint32_t                           err_code;
    ble_xxs_init_t                     xxs_init;
    ble_yys_init_t                     yys_init;

    // Initialize XXX Service.
    memset(&xxs_init, 0, sizeof(xxs_init));

    xxs_init.evt_handler                = NULL;
    xxs_init.is_xxx_notify_supported    = true;
    xxs_init.ble_xx_initial_value.level = 100; 
    
    err_code = ble_bas_init(&m_xxs, &xxs_init);
    APP_ERROR_CHECK(err_code);

    // Initialize YYY Service.
    memset(&yys_init, 0, sizeof(yys_init));
    yys_init.evt_handler                  = on_yys_evt;
    yys_init.ble_yy_initial_value.counter = 0;

    err_code = ble_yy_service_init(&yys_init, &yy_init);
    APP_ERROR_CHECK(err_code);
    /
}*/


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 *
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}*/


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
/*
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}*/


/**@brief Function for initializing the Connection Parameters module.
 */
/*
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}*/





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
}

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
	buttons_leds_init();

	initialize_uart();

    timers_init();

    // NO BSP, set erase bonds manually:
    //
    //bool erase_bonds = true;
    ble_stack_init();
   	__LOG("Bluetooth stack initialized");
    //device_manager_init(erase_bonds);
    gap_params_init();
    advertising_init();
   	__LOG("ADV initialized");

   	storage_init();

    //services_init();
    //conn_params_init();

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
        //sd_ble_opt_set();
    }
}

/**
 * @}
 */
