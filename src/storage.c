/*
 * storage.c
 *
 *  Created on: 21. märts 2016
 *      Author: lauri
 */
#include <inttypes.h>
#include <string.h>
#include <stdlib.h>

#include "pstorage.h"
#include "pstorage_platform.h"
#include "app_error.h"
#include "nrf_soc.h"
#include "ble_nus.h"

#include "contact.h"
#include "storage.h"
#include "uart.h"
#include "timer.h"
#include "gpio.h"

//#undef PSTORAGE_MIN_BLOCK_SIZE
//#define PSTORAGE_MIN_BLOCK_SIZE     4 // library min is 16
#define STORE_BLOCK_SIZE	40
#define STORE_BLOCK_COUNT	170

static pstorage_handle_t m_p_storage_id;

static uint8_t * write_cache; //[STORE_BLOCK_SIZE]; // 40 bytes
static uint8_t * read_cache; //[STORE_BLOCK_SIZE]; // 40 bytes

static uint8_t w_cache_curr_idx = 0;
static uint16_t w_blix = 0;

static uint8_t r_cache_curr_idx = STORE_BLOCK_SIZE; // index for copying to nus.
static uint16_t r_blix = 0; // begin reading from first block;

static bool operation_lock = false; /** operation in progress and storage cannot be accessed. */
static bool manual_lock = true; /** initially the storage is locked to prevent accidental overwriting */
static bool storage_full = false; /** initially the storage is locked to prevent accidental overwriting */

// internal function declarations
static uint32_t initiate_write();
static void fill_data(uint8_t *, uint8_t *);


//static void storage_callback(pstorage_handle_t  *, uint8_t,  uint32_t, uint8_t*,uint32_t);
static void storage_callback(pstorage_handle_t  * handle,
	                               uint8_t              op_code,
	                               uint32_t             result,
	                               uint8_t            * p_data,
	                               uint32_t             data_len)
{
	    switch(op_code)
	    {
	       case PSTORAGE_LOAD_OP_CODE:
	           if (result == NRF_SUCCESS)
	           {

	               //Store load operation successful.
	        	   //Not good to debug log here,
	        	   //NUS has tight timing.
	           }
	           else
	           {
	        	   	__LOG("load operation failed with %d", result);
	        	   // APP_ERROR_CHECK(result);
	               // Store operation failed.
	           }
	            //memset(read_cache, 0, sizeof(uint8_t) * STORE_BLOCK_SIZE);
				r_blix++;
				r_cache_curr_idx = 0;
				operation_lock = false;
	           break;
	       case PSTORAGE_STORE_OP_CODE:
	           if (result == NRF_SUCCESS)
	           {
	        	   __LOG("successful write to %d.", w_blix);
	               // Store operation successful.
	           }
	           else
	           {
	               // Store operation failed.
	        	    APP_ERROR_CHECK(result);
	           }

        	   if (w_blix + 1 < STORE_BLOCK_COUNT)
        	   {
        		   w_blix++;
        	   }
        	   else
        	   {
        		   storage_full = true;
        		   timer_event_indication = indicate_storage_problem;
        	   }
        	   memset(write_cache, 0, sizeof(uint8_t) * STORE_BLOCK_SIZE);
        	   w_cache_curr_idx = 0;
        	   operation_lock = false;
	           break;
	       case PSTORAGE_CLEAR_OP_CODE:
	           if (result == NRF_SUCCESS)
	           {
	               __LOG("successful clear");
				   // Store operation successful.
	           }
	           else
	           {
	               // Store operation failed.
	        	    APP_ERROR_CHECK(result);
	           }
	           memset(write_cache, 0, sizeof(uint8_t) * STORE_BLOCK_SIZE);
	           w_cache_curr_idx = 0;
	           w_blix = 0;
	           storage_full = false;
               operation_lock = false;
	           break;
	       case PSTORAGE_UPDATE_OP_CODE:
	           if (result == NRF_SUCCESS)
	           {
	               // Store operation successful.
	           }
	           else
	           {
	        	    APP_ERROR_CHECK(result);
	        	    // Store operation failed.
	           }
	           break;
	}
}

void storage_init()
{
	/**
	 * @brief Initialize flash storage.
	 */
	uint32_t err_code = NRF_SUCCESS;
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);
    pstorage_module_param_t p_storage_param;

    p_storage_param.block_size  = STORE_BLOCK_SIZE; // recommended to be >= 4 byte
    p_storage_param.block_count = STORE_BLOCK_COUNT;
    p_storage_param.cb          = storage_callback;
    err_code = pstorage_register (&p_storage_param, &m_p_storage_id);
    APP_ERROR_CHECK(err_code);

    write_cache = malloc(sizeof(uint8_t) * STORE_BLOCK_SIZE); // 40 bytes
    read_cache = malloc(sizeof(uint8_t) * STORE_BLOCK_SIZE); // 40 bytes
    memset(write_cache, 0, sizeof(uint8_t) * STORE_BLOCK_SIZE);
    memset(read_cache, 0, sizeof(uint8_t) * STORE_BLOCK_SIZE);
    //memset()
}

//uint32_t store_contact(contact row) {
//	uint32_t err_code = NRF_SUCCESS;
//
//	if (operation_lock || manual_lock) {
//		//TODO: check it from contact side and dont clear the contact.
//		//Write or read in progress, dont do anything.
//		__LOG("write was busy");
//		err_code = NRF_ERROR_BUSY;
//		return err_code;
//	}
//	write_cache[w_cache_curr_idx++] = row.address;
//	write_cache[w_cache_curr_idx++] = row.first_epoch;
//	write_cache[w_cache_curr_idx++] = row.first_time_seen;
//	write_cache[w_cache_curr_idx++] = row.last_epoch;
//	write_cache[w_cache_curr_idx++] = row.last_time_seen;
//	if (w_cache_curr_idx == STORE_BLOCK_SIZE) {
//		if (!operation_lock && !manual_lock)
//			err_code = initiate_write();
//	};
//	return err_code;
//}


uint32_t store_report(uint8_t ts, uint8_t epoch, int8_t rssi, uint8_t addr) {
	uint32_t err_code = NRF_SUCCESS;
	if (operation_lock || manual_lock || storage_full) {
		//TODO: check it from contact side and dont clear the contact.
		//Write or read in progress, dont do anything.
		//__LOG("write was busy");
		timer_event_indication = indicate_storage_problem;
		err_code = NRF_ERROR_BUSY;
		return err_code;
	}
	write_cache[w_cache_curr_idx++] = ts;
	write_cache[w_cache_curr_idx++] = epoch;
	write_cache[w_cache_curr_idx++] = rssi;
	write_cache[w_cache_curr_idx++] = addr;
	if (w_cache_curr_idx >= STORE_BLOCK_SIZE) {
		err_code = initiate_write();
	};
	//__LOG("Write cache idx %d", write_cache_curr_idx);
	/*while (storage_locked) { // seems to lock, doesnt work.
		err_code = sd_app_evt_wait();
		APP_ERROR_CHECK(err_code);
	}*/
	return err_code;
}


/**
 *
 */
void read_store_data(uint8_t * p_row, uint8_t * size) {
	if (operation_lock) {
		//TODO: check it from reading side and dont clear the contact.
		//Write or read in progress, dont do anything.
		__LOG("Read locked.");
		return;
	}
	fill_data(p_row, size);
}

static uint32_t initiate_write() {
	operation_lock = true; // cb will unlock this.
	uint32_t err_code = NRF_SUCCESS;
	pstorage_handle_t block_handle;
    err_code = pstorage_block_identifier_get(&m_p_storage_id, w_blix, &block_handle);
    APP_ERROR_CHECK(err_code);
	//__LOG("Initiated flash write.");
	err_code = pstorage_store(&block_handle, write_cache, STORE_BLOCK_SIZE, 0);
	APP_ERROR_CHECK(err_code);
	return err_code;
}

/*
static uint32_t initiate_read()
{
	uint32_t err_code = NRF_SUCCESS;
	pstorage_handle_t block_handle;
    err_code = pstorage_block_identifier_get(&m_p_storage_id, 0, &block_handle);
    APP_ERROR_CHECK(err_code);
	//err_code = pstorage_load(row, &block_handle, STORE_BLOCK_SIZE, 0);
    APP_ERROR_CHECK(err_code);
	//pstorage_handle_t block_handle;
	__LOG("Initiated flash write.");
	operation_lock = true; // cb will unlock this.
	//err_code = pstorage_store(&block_handle, storage_cache, STORE_BLOCK_SIZE, 0);
	APP_ERROR_CHECK(err_code);
	return err_code;
}*/

void store_read_init(void)
{
    // Initialize data transfer
    r_cache_curr_idx = STORE_BLOCK_SIZE;
    r_blix = 0;
}

static void fill_data(uint8_t * nus_storage, uint8_t * nus_length )
{
	//TODO: method to reset data_idx and blix after successful nus receive.
	//__LOG("filling data for nus");
	pstorage_handle_t block_handle;
	memset(nus_storage, 0, BLE_NUS_MAX_DATA_LEN);
	*nus_length = 0;

    //uint8_t data_len = STORE_BLOCK_SIZE;
	uint32_t err_code = NRF_SUCCESS;

    while(*nus_length < BLE_NUS_MAX_DATA_LEN )	//nus string has room.
    {
    	if (r_cache_curr_idx < STORE_BLOCK_SIZE)	//there is data
    	{
    		nus_storage[*nus_length] = read_cache[r_cache_curr_idx];
    		r_cache_curr_idx++;
    		(*nus_length)++;
    	} else {
    		if (r_blix < STORE_BLOCK_COUNT)	//there are still blocks;
    		{
    			//__LOG("trying to get block id %d", blix);
    	        err_code = pstorage_block_identifier_get(&m_p_storage_id, r_blix, &block_handle);
    			//__LOG("got block id %d with err %d", blix, err_code);
    	        APP_ERROR_CHECK(err_code);
    	        operation_lock = true;
    	    	err_code = pstorage_load(read_cache, &block_handle, STORE_BLOCK_SIZE, 0);
    	        APP_ERROR_CHECK(err_code);
    		} else {
    			//loaded all blocks
    			//blix = 0;
    			//__LOG("%x", write_cache[39]);
    			break; //nothing more to send
    		}
    	}
    }
}

void storage_clear()
{
	operation_lock = true;
    pstorage_clear(&m_p_storage_id, STORE_BLOCK_SIZE * STORE_BLOCK_COUNT);

}

bool storage_toggle_lock()
{
	manual_lock = (manual_lock ? false : true);
	__LOG("Manual lock is now %d", manual_lock);
	return manual_lock;
}

/**@brief Wait if there is any flash access pending
*
void wait_flash_op(void)
{
    uint32_t    err_code;
    uint32_t    count;

    do
    {
        app_sched_execute();
        err_code = pstorage_access_status_get(&count);
        APP_ERROR_CHECK(err_code);
    }
    while (count);
}
pstorage_access_status_get()*/
