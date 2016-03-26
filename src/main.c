/*
 * BLE MON
 *
 * TODO: whitelist in scanning and advertising only others and admin phone.
 * TODO: calibrating via nus.
 */

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
#include "gpio.h"
#include "timer.h"
#include "contact.h"
#include "radio.h"
#include "storage.h"


#define DEVICE_NAME                      "sensor"                               /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "NordicSemiconductor"                      /**< Manufacturer. Will be passed to Device Information Service. */


//#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
//#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
//#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
//#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
//#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
//#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */


#define DEAD_BEEF                        0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//static dm_application_instance_t        m_app_handle;                               /**< Application identifier allocated by device manager */


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


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */

int main(void)
{
	gpio_init();
	uart_init();
    timers_init();
    __LOG("Timers initialized.");

    ble_stack_init();
    //device_manager_init(erase_bonds);
    gap_params_init();
    services_init();
    advertising_init();
   	__LOG("Radio initialized.");
    conn_params_init();

   	storage_init();
   	contacts_init();
   	__LOG("Application modules initialized");

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
