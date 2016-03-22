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

#include "contact.h"
#include "storage.h"
#include "uart.h"

//#undef PSTORAGE_MIN_BLOCK_SIZE
//#define PSTORAGE_MIN_BLOCK_SIZE     4 // library min is 16
#define STORE_BLOCK_SIZE	40
#define STORE_BLOCK_COUNT	50

static pstorage_handle_t m_p_storage_id;
static pstorage_handle_t block_handle;

static uint8_t storage_cache[STORE_BLOCK_SIZE]; // 40 bytes

static uint8_t cache_curr_idx = 0;
static uint8_t block_curr_idx = 0;

static bool operation_lock = false; /** operation in progress and storage cannot be accessed. */
static bool manual_lock = true; /** initially the storage is locked to prevent accidental overwriting */

static uint32_t initiate_write();
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
	        	   operation_lock = false;
	        	   __LOG("successful read.");
	               // Store operation successful.
	           }
	           else
	           {
	        	    APP_ERROR_CHECK(result);
	               // Store operation failed.
	           }
	           break;
	       case PSTORAGE_STORE_OP_CODE:
	           if (result == NRF_SUCCESS)
	           {
	        	   __LOG("successful write to %d.", block_curr_idx);
	        	   memset(&storage_cache, 0, sizeof(storage_cache));
	        	   cache_curr_idx = 0;
	        	   block_curr_idx++;
	        	   operation_lock = false;
	               // Store operation successful.
	           }
	           else
	           {
	               // Store operation failed.
	        	    APP_ERROR_CHECK(result);
	           }
	           break;
	       case PSTORAGE_CLEAR_OP_CODE:
	           if (result == NRF_SUCCESS)
	           {
	               // Store operation successful.
	           }
	           else
	           {
	               // Store operation failed.
	        	    APP_ERROR_CHECK(result);
	           }
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
    err_code = pstorage_block_identifier_get(&m_p_storage_id, block_curr_idx, &block_handle);
    APP_ERROR_CHECK(err_code);
}

uint32_t store_contact(contact row) {
	uint32_t err_code = NRF_SUCCESS;

	if (operation_lock || manual_lock) {
		//TODO: check it from contact side and dont clear the contact.
		//Write or read in progress, dont do anything.
		err_code = NRF_ERROR_BUSY;
		return err_code;
	}
	storage_cache[cache_curr_idx++] = row.address;
	storage_cache[cache_curr_idx++] = row.first_epoch;
	storage_cache[cache_curr_idx++] = row.first_time_seen;
	storage_cache[cache_curr_idx++] = row.last_epoch;
	storage_cache[cache_curr_idx++] = row.last_time_seen;
	if (cache_curr_idx == STORE_BLOCK_SIZE - 1 ) {
		if (false)
			err_code = initiate_write();
	};
	/*while (storage_locked) { // seems to lock, doesnt work.
		err_code = sd_app_evt_wait();
		APP_ERROR_CHECK(err_code);
	}*/
	return err_code;
}

/**
 *
 */
uint8_t read_contact_store(uint8_t * p_row, uint8_t size) {
	//TODO write directly to p_row or free row after use.
	if (operation_lock || manual_lock) {
		//TODO: check it from contact side and dont clear the contact.
		//Write or read in progress, dont do anything.
		return 0;
	}
	uint32_t err_code = NRF_SUCCESS;
	uint8_t * row;
	row = (uint8_t *) malloc(sizeof(uint8_t) * STORE_BLOCK_SIZE);
	memset(&storage_cache, 0, sizeof(storage_cache));
	operation_lock = true;
	err_code = pstorage_load(row, &block_handle, STORE_BLOCK_SIZE, 0);
	while (operation_lock) {
	    err_code = sd_app_evt_wait();
	    APP_ERROR_CHECK(err_code);
	}
	memcpy (p_row, row, size);
	return 0;
//	row[0] = storage_cache[0]; //address
//	row[1] = storage_cache[1]; //first epoch
//	row[2] = storage_cache[2]; //first time seen
//	p_contact->first_epoch = storage_cache[0];
//	p_contact->first_time_seen =
//	p_contact->last_epoch = storage_cache[2];
//	p_contact->last_time_seen = storage_cache[3];
//	//return p_contact;
//	//pstorage_handle_t block_handle;
//	//uint8_t           dest_data[4];
//	//uint32_t          err_code;
//	// Request to read 4 bytes from block at an offset of 12 bytes.
//	APP_ERROR_CHECK(err_code);
//	return p_contact;
}

static uint32_t initiate_write() {
	uint32_t err_code = NRF_SUCCESS;
	__LOG("Initiated flash write.");
	operation_lock = true; // cb will unlock this.
	//pstorage_handle_t block_handle;
	//uint8_t           source_data[4];
	//uint32_t          err_code;
	// Request to write 8 bytes to block at an offset of 20 bytes.
	err_code = pstorage_store(&block_handle, storage_cache, STORE_BLOCK_SIZE, block_curr_idx);
	APP_ERROR_CHECK(err_code);
	return err_code;
}

void storage_clear()
{	//TODO: make this a nus command
    pstorage_clear(&block_handle, STORE_BLOCK_SIZE * STORE_BLOCK_COUNT);
    memset(&storage_cache, 0, sizeof(storage_cache));
    cache_curr_idx = 0;
    block_curr_idx = 0;
}

void storage_toggle_lock()
{
	manual_lock = (manual_lock ? false : true);
	__LOG("Manual lock is now %d", manual_lock);
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
