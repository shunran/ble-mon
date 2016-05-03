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
#include "ble_advertising.h"
#include "softdevice_handler.h"
#include "ble_conn_params.h"
#include "ble_nus.h"
#include "ble_hci.h"
#include "app_timer.h"
#include "pstorage.h"
#include "app_uart.h"
#include "nrf_delay.h"

#include "uart.h"
#include "timer.h"


#include "contact.h"
#include "storage.h"

#define ADDRESS_OF_DEVICE	{0xC0, 0xFF, 0xEE, 0x00, 0x00, 0x13}

#define INITIAL_APP_GAP_TX_POWER	-40	/** Radio transmit power in dBm (accepted values are -40, -30, -20, -16, -12, -8, -4, 0, and 4 dBm). */
#define MIN_CONN_INTERVAL	MSEC_TO_UNITS(20, UNIT_1_25_MS)	/**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL	MSEC_TO_UNITS(75, UNIT_1_25_MS)	/**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY	0	/**< Slave latency. */
#define CONN_SUP_TIMEOUT	MSEC_TO_UNITS(4000, UNIT_10_MS)	/**< Connection supervisory timeout (4 seconds). */

#define IS_SRVC_CHANGED_CHARACT_PRESENT	1	/**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define APP_ADV_INTERVAL                 160                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT       0                                         /**< The advertising timeout in units of seconds. */

#define SCAN_INTERVAL               0x00A0                             /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW                 0x0050                             /**< Determines scan window in units of 0.625 millisecond. */

#define NUS_SERVICE_UUID_TYPE	BLE_UUID_TYPE_VENDOR_BEGIN	/**< UUID type for the Nordic UART Service (vendor specific). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3      /**< Number of attempts before giving up the connection parameter negotiation. */


static ble_nus_t	m_nus;	/**< Structure to identify the Nordic UART Service. */
static uint16_t	m_conn_handle = BLE_CONN_HANDLE_INVALID;	/**< Handle of the current connection. */

static ble_uuid_t	m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};	/**< Universally unique service identifier. */

static const uint8_t l_device_address[] = ADDRESS_OF_DEVICE;
static volatile bool	m_file_in_transit = false;
static uint8_t * m_nus_data;
static uint8_t m_data_length = 0;
static int8_t gap_tx_power = INITIAL_APP_GAP_TX_POWER;


//48bit addresses of whitelist
//static int32_t known_devices[NUMBER_OF_DEVICES] = {0xC0FFEE0000, 0xC0FFEE0001};
//static ble_gap_adv_params_t m_adv_params;

//inline function declarations
static void ble_nus_data_transfer(void);
static void sys_evt_dispatch(uint32_t);
static void ble_evt_dispatch(ble_evt_t*);
static void on_ble_evt(ble_evt_t*);
static void on_adv_evt(ble_adv_evt_t);
static void nus_data_handler(ble_nus_t*, uint8_t*, uint16_t);
static void on_conn_params_evt(ble_conn_params_evt_t*);
static void conn_params_error_handler(uint32_t);
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


static void on_ble_evt(ble_evt_t * p_ble_evt);
static void conn_params_error_handler(uint32_t nrf_error);
static void on_adv_evt(ble_adv_evt_t ble_adv_evt);

/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */

static void sys_evt_dispatch(uint32_t sys_evt)
{
    ble_advertising_on_sys_evt(sys_evt);
    pstorage_sys_event_handler(sys_evt);
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
    on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    //TODO: no bsp
    //bsp_btn_ble_on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
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
    // TODO: use scheduler, maybe?

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

    sd_ble_gap_tx_power_set(gap_tx_power);

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
    for (uint8_t i = 0; i < BLE_GAP_ADDR_LEN; i++) //Only last byte differs.
    {
    	gap_address.addr[BLE_GAP_ADDR_LEN - 1 - i] = l_device_address[i];
    }
    //TODO: BLE_GAP_ADDR_CYCLE_MODE_AUTO
    err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &gap_address);
    APP_ERROR_CHECK(err_code);// Check for errors
}

bool known_device(uint32_t device_address)
{
	bool found = false;
	for (uint8_t i = 0; i < NUMBER_OF_DEVICES; i++) {

	}
	return found;
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
             //TODO: implement whitelist
            bool friend = true;
            // Whitelist substitute
            for (int i = 0; i < BLE_GAP_ADDR_LEN - 1; i++)
            	// Last number of address doesnt count. Others should be same as ours.
            {
            	if (p_adv_report->peer_addr.addr[BLE_GAP_ADDR_LEN - 1 - i] != l_device_address[i])
            	{
            		friend = false;
            		break;
            	}
            }

            if (friend)
            {
                make_report(NRF_RTC1->COUNTER, timer_epoch, p_adv_report->rssi, p_adv_report->peer_addr.addr[0]);
            	//make_contact(p_adv_report->peer_addr.addr[0]);
                /*__LOG("Target %02x %ddBm",
                           p_adv_report->peer_addr.addr[0],
      					 p_adv_report->rssi
                           );*/
            }
           break;
        }
        case BLE_GAP_EVT_CONNECTED:
            err_code = NRF_SUCCESS;
            // TODO: no bsp, rewrite
            //bsp_indication_set(BSP_INDICATE_CONNECTED);
            sd_ble_gap_scan_stop();
            __LOG("conn. scan stopped");
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            //sd_ble_gap_tx_power_set(0);
            //scan_stop(); ????
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            __LOG("disco. scan start");
            scan_start();
            //initialize advertising again in case power level changed.
            //advertised power is not by default tx power
            advertising_init();
            advertising_start();
            break;

        case BLE_GAP_EVT_TIMEOUT:
        	//advertising starts itself.
        	scan_start();
        	__LOG("timeout. scan start");
        	break;

        case BLE_GATTS_EVT_TIMEOUT:
            if (p_ble_evt->evt.gatts_evt.params.timeout.src == BLE_GATT_TIMEOUT_SRC_PROTOCOL)
            {
                err_code = sd_ble_gap_disconnect(m_conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
            //TODO: manage timeout.
        	__LOG("GATTS timeout.");
            break;
        case BLE_EVT_TX_COMPLETE:
            if (m_file_in_transit) ble_nus_data_transfer();
            break;
        default:
            // No implementation needed.
            break;
    }
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
	//uint32_t err_code;
    //TODO: NO bsp, rewrite for ble400 board
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_SLOW: // 4
           //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            //APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE: // 0
            //sleep_mode_enter();
            break;
        default:
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
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;
    advdata.p_tx_power_level        = &gap_tx_power;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_slow_enabled  = BLE_ADV_SLOW_ENABLED;
    options.ble_adv_slow_interval = APP_ADV_INTERVAL;
    options.ble_adv_slow_timeout  = APP_ADV_TIMEOUT;

    //err_code = ble_advdata_set(&advdata, NULL, options, on_adv_event);
    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
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

    err_code = ble_advertising_start(BLE_ADV_MODE_SLOW);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
	uint32_t        err_code;
	 if (length==1) //< Control Command
	    {
	        switch(p_data[0])
	        {
	        /** Radio transmit power in dBm
	         * (accepted values are -40, -30, -20, -16, -12, -8, -4, 0, and 4 dBm). */
            case '1':
            	err_code = ble_nus_string_send(p_nus, (uint8_t *) "Power set to -40", 16);
            	APP_ERROR_CHECK(err_code);
            	gap_tx_power = -40;
            	err_code = sd_ble_gap_tx_power_set(gap_tx_power);
            	APP_ERROR_CHECK(err_code);
                break;
            case '2':
            	err_code = ble_nus_string_send(p_nus, (uint8_t *) "Power set to -20", 16);
            	APP_ERROR_CHECK(err_code);
            	gap_tx_power = -20;
            	err_code = sd_ble_gap_tx_power_set(gap_tx_power);
            	APP_ERROR_CHECK(err_code);
            	break;
            case '3':
            	err_code = ble_nus_string_send(p_nus, (uint8_t *) "Power set to 0", 14);
            	APP_ERROR_CHECK(err_code);
            	gap_tx_power = 0;
            	err_code = sd_ble_gap_tx_power_set(gap_tx_power);
            	APP_ERROR_CHECK(err_code);
            	break;
            case 'c':
            	;
            	storage_clear();
            	err_code = ble_nus_string_send(p_nus, (uint8_t *) "Storage cleared.", 16);
            	APP_ERROR_CHECK(err_code);
            	break;
            case 'l':
            	;
            	char str_l[17];
            	sprintf(str_l, "Toggled lock to %d", storage_toggle_lock());
            	ble_nus_string_send(p_nus, (uint8_t *) str_l, 18);
            	break;
            case 'r':
            	store_read_init();
            	read_store_data(m_nus_data, &m_data_length);
            	nrf_delay_ms(MAX_CONN_INTERVAL * 2);
            	m_file_in_transit = true;
            	err_code = ble_nus_string_send(p_nus, (uint8_t *) "***DATA TRANSFER***", 19);
            	APP_ERROR_CHECK(err_code);

            	break;
            case 't':
            	;
            	char str_t[20];
            	uint8_t c;
            	uint32_t ts = NRF_RTC1->COUNTER;
            	c = sprintf(str_t, "ts:%lu epoch:%d", ts, timer_epoch);
            	err_code = ble_nus_string_send(p_nus, (uint8_t *) str_t, c);
            	 APP_ERROR_CHECK(err_code);
            	break;
	        }
	    }
	__LOG("NUSCON:%s", p_data);
}


/**@brief Function for initializing services that will be used by the application.
 */
void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);

    m_nus_data = malloc(sizeof(uint8_t) * BLE_NUS_MAX_DATA_LEN);
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
void conn_params_init(void)
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
}


/**@brief Send data through BLE UART service with maximum throughput. */
static void ble_nus_data_transfer()
{
    uint32_t        err_code;

    if (m_data_length == 0)    //< All data is sent.
    {
        m_file_in_transit = false;
        err_code = ble_nus_string_send(&m_nus, (uint8_t *) "**END**", 7);     //< End indicator
        APP_ERROR_CHECK(err_code);
        return;
    }

    /** @note    Maximize BLE throughput
      *          https://devzone.nordicsemi.com/question/1741/dealing-large-data-packets-through-ble/
      */
    while (1)
    {
    	if (m_data_length == 0) break;
        err_code = ble_nus_string_send(&m_nus, m_nus_data, m_data_length);
        //test_logf("First nus send with len %d and err %d", m_data_length, err_code);
        if (err_code == BLE_ERROR_NO_TX_BUFFERS ||
            err_code == NRF_ERROR_INVALID_STATE ||
            err_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        {
            break;
        }
        else if (err_code != NRF_SUCCESS)
        {
            APP_ERROR_CHECK(err_code);
        }
        //__LOG("Getting data with len: %d", m_data_length);
        read_store_data(m_nus_data, &m_data_length);
        //back_data_ble_nus_fill(m_data, &m_data_length);
    }
}
