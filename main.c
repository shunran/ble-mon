/*
 * BLE MON
 */

// TODO: implement advertiser.
// TODO: implement observer
// TODO: implement connection and data download
// TODO: implement time
// TODO: whitelist only others and admin phone
// TODO: implement storage
// TODO: add rssi field to advertiser if needed

//#include <stdint.h>
#include <inttypes.h>
#include <string.h>
#include "nordic_common.h"
#include <nrf.h>

#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
//#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "softdevice_handler.h"
//#include "device_manager.h"
//#include "pstorage.h"
//#include "app_trace.h"
//#include "bsp.h"
//#include "bsp_btn_ble.h"

#include "uart.h"
#include "timer.h"

// Scanner
//#include "btle.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define DEVICE_NAME                      "sensor"                               /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "NordicSemiconductor"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                 300                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT       0                                         /**< The advertising timeout in units of seconds. */

//#define SLOW_APP_ADV_INTERVAL			160
#define APP_GAP_TX_POWER -30		/** Radio transmit power in dBm (accepted values are -40, -30, -20, -16, -12, -8, -4, 0, and 4 dBm). */


#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(100, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(200, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                    0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

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

static uint16_t                          m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */

static ble_gap_adv_params_t m_adv_params;

#define TIMESLOT_LENGTH_US 10000
#define TIMESLOT_DISTANCE_US 20000

// These are the parameters for the scanner running in the timeslot
//static btle_cmd_param_le_write_scan_parameters_t scan_param = {
//  BTLE_SCAN_TYPE_PASSIVE,          /* Active scanning. SCAN_REQ packets may be sent */
//  20000,           /* Time from controller starts its last scan until it begins the next scan */
//  10000,             /* Duration of the scan */
//  BTLE_ADDR_TYPE_PUBLIC,          /* Use public address type */
//  BTLE_SCAN_FILTER_ACCEPT_ANY     /* Accept anyone (whitelist unsupported for now) */
//};

//static btle_cmd_param_le_write_scan_enable_t scan_enable = {
//  BTLE_SCAN_MODE_ENABLE,              /* Enable scanner */
//  BTLE_SCAN_DUPLICATE_FILTER_DISABLE  /* Do not filter duplicates */
//};

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

void led_stripe() {
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


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);


    sd_ble_gap_tx_power_set(APP_GAP_TX_POWER);


    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);

    ble_gap_addr_t gap_address;
    memset(&gap_address, 0, sizeof(gap_address));
    //TODO: BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE & IRK
    gap_address.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
    gap_address.addr[5] = 0xC0;
    gap_address.addr[4] = 0xFF;
    gap_address.addr[3] = 0xEE;
    gap_address.addr[2] = 0x00;
    gap_address.addr[1] = 0x00;
    gap_address.addr[0] = 0x01;
    	//	FFEE; // 48-bit address, LSB format
    //TODO: BLE_GAP_ADDR_CYCLE_MODE_AUTO
    err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &gap_address);
    APP_ERROR_CHECK(err_code);// Check for errors
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
static void services_init(void)
{
    /* YOUR_JOB: Add code to initialize the services used by the application.
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
    */
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


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
 */
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
}


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


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
            {
        case BLE_GAP_EVT_CONNECTED:
            err_code = NRF_SUCCESS;
            // TODO: no bsp, rewrite
            //bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
/*
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}*/


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

#if defined(S110) || defined(S130) || defined(S132)
    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#if (defined(S130) || defined(S132))
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
#endif
}



/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_NO_NAME; //BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags                   = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED; //BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    //advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    //advdata.uuids_complete.p_uuids  = m_adv_uuids;
    /*BLE_GAP_ADV_TYPE_ADV_NONCONN_IND
    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;
    */

    /*
    bool     ble_adv_slow_enabled;            /< Enable or disable slow advertising mode. /
    uint32_t ble_adv_slow_interval;           /< Advertising interval for slow advertising. /
    uint32_t ble_adv_slow_timeout;            /< Time-out (in seconds) for slow advertising. *

    */

    //options.ble_ad
    err_code = ble_advdata_set(&advdata, NULL);
    //err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT;
}

static void advertising_start(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);

    //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
    /**
	err_code = (BLE_ADV_MODE_FAST);
	APP_ERROR_CHECK(err_code);
	*/

}



/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    //TODO: no bsp
   // bsp_event_t startup_event;
	/*
    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), 
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);
	 */
    //TODO: no bsp
    /*err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    */
    //p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
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
	nrf_gpio_range_cfg_output(LED_0, LED_4);

	initialize_uart();

    timers_init();

    // NO BSP, set erase bonds manually:
    //buttons_leds_init(&erase_bonds);
    //bool erase_bonds = true;
    ble_stack_init();
   	__LOG("Bluetooth stack initialized");
    //device_manager_init(erase_bonds);
    gap_params_init();
    advertising_init();
   	__LOG("ADV initialized");
    //services_init();
    //conn_params_init();

    // Start execution.
    timers_start();
    advertising_start();
   	__LOG("Started");

    //led_stripe();

    // Enter main loop.
    for (;;)
    {
        //LEDS_ON(LED_0_MASK);
        //NRF_GPIO->OUTSET(LED_0_MASK);
        //nrf_gpio_pin_set(LED_0);
        //nrf_delay_ms(100);
        //nrf_gpio_pin_clear(LED_0);
       // NRF_GPIO->OUTCLR(LED_0_MASK);

        //LEDS_OFF(LED_0_MASK);
        power_manage();
    }
}

/**
 * @}
 */
