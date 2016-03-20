/*
 * radio.c
 *
 *  Created on: 20. märts 2016
 *      Author: lauri
 */
#include "radio.h"

#include <inttypes.h>
#include <string.h>
#include "nordic_common.h"
#include <nrf.h>

#include "ble.h"
#include "ble_gap.h"
#include "ble_gatts.h"
#include "nrf_error.h"
#include "ble_advdata.h"
#include "softdevice_handler.h"
#include "ble_conn_params.h"

#include "uart.h"

#define APP_GAP_TX_POWER	-30	/** Radio transmit power in dBm (accepted values are -40, -30, -20, -16, -12, -8, -4, 0, and 4 dBm). */
#define MIN_CONN_INTERVAL	MSEC_TO_UNITS(100, UNIT_1_25_MS)	/**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL	MSEC_TO_UNITS(200, UNIT_1_25_MS)	/**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY	0	/**< Slave latency. */
#define CONN_SUP_TIMEOUT	MSEC_TO_UNITS(4000, UNIT_10_MS)	/**< Connection supervisory timeout (4 seconds). */

#define IS_SRVC_CHANGED_CHARACT_PRESENT	1	/**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define APP_ADV_INTERVAL                 300                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT       0                                         /**< The advertising timeout in units of seconds. */

#define SCAN_INTERVAL               0x00A0                             /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                 0x0050                             /**< Determines scan window in units of 0.625 millisecond. */

static uint16_t	m_conn_handle = BLE_CONN_HANDLE_INVALID;	/**< Handle of the current connection. */
static ble_gap_adv_params_t m_adv_params;

/**
 * @brief Scan parameters requested for scanning and connection. */
static const ble_gap_scan_params_t m_scan_param =
{
    0,              // Active scanning not set.
    0,              // Selective scanning not set.
    NULL,           // No whitelist provided.
    SCAN_INTERVAL,
    SCAN_WINDOW,
    0x0000          // No timeout.
};

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */

static void sys_evt_dispatch(uint32_t sys_evt)
{
    //pstorage_sys_event_handler(sys_evt);
    //ble_advertising_on_sys_evt(sys_evt);
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    //dm_ble_evt_handler(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    //TODO: no bsp
    //bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    //ble_advertising_on_ble_evt(p_ble_evt);
    /*YOUR_JOB add calls to _on_ble_evt functions from each service your application is using
    ble_xxs_on_ble_evt(&m_xxs, p_ble_evt);
    ble_yys_on_ble_evt(&m_yys, p_ble_evt);
    */
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));

    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

// Register with the SoftDevice handler module for BLE events.
err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
APP_ERROR_CHECK(err_code);

// Register with the SoftDevice handler module for System events.
err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
void gap_params_init(void)
{
    uint32_t                err_code = NRF_SUCCESS;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);


    sd_ble_gap_tx_power_set(APP_GAP_TX_POWER);

    /*

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    */
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


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
            {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            const ble_gap_evt_adv_report_t *p_adv_report = &p_gap_evt->params.adv_report;
           //test_logf("Found target");
		   //(unsigned int) NRF_RTC1->COUNTER,
           __LOG("Target %02x%02x%02x%02x%02x%02x %ddBm",
                     p_adv_report->peer_addr.addr[5],
                     p_adv_report->peer_addr.addr[4],
                     p_adv_report->peer_addr.addr[3],
                     p_adv_report->peer_addr.addr[2],
                     p_adv_report->peer_addr.addr[1],
                     p_adv_report->peer_addr.addr[0],
					 p_adv_report->rssi
                     );
           break;
        }
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

/**@brief Function for initializing the Advertising functionality.
 */
void advertising_init(void)
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

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND; //BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = APP_ADV_INTERVAL;
    m_adv_params.timeout     = APP_ADV_TIMEOUT;
}

/**@brief Function to start scanning.
 */
void scan_start(void)
{
	uint32_t err_code;

    sd_ble_gap_scan_stop();

    err_code = sd_ble_gap_scan_start(&m_scan_param);
    // It is okay to ignore this error since we are stopping the scan anyway.
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }
}

void advertising_start(void)
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
