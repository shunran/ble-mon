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
#define STORE_BLOCK_COUNT	1500

static pstorage_handle_t m_p_storage_id;
static pstorage_handle_t block_handle;

static uint8_t storage_cache[STORE_BLOCK_SIZE]; // 40 bytes

static uint8_t cache_current_idx = 0;
static uint8_t block_curr_idx = 0;

static bool storage_locked = false;

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
	        	   storage_locked = false;
	        	   __LOG("successful read.");
	               // Store operation successful.
	           }
	           else
	           {
	        	    APP_ERROR_CHECK(result);
	               // Store operation failed.
	           }
	           // Source memory can now be reused or freed.
	           break;
	       case PSTORAGE_STORE_OP_CODE:
	           if (result == NRF_SUCCESS)
	           {
	        	   __LOG("successful write.");
	        	   memset(&storage_cache, 0, sizeof(storage_cache));
	        	   storage_locked = false;
	               // Store operation successful.
	           }
	           else
	           {
	               // Store operation failed.
	        	    APP_ERROR_CHECK(result);
	           }
	           // Source memory can now be reused or freed.
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
	           // Source memory can now be reused or freed.
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
	           // Source memory can now be reused or freed.
	           break;
	}
}

void storage_init()
{
	static uint32_t err_code = NRF_SUCCESS;
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    pstorage_module_param_t p_storage_param;

    // Setup pstorage with 60blocks of 16byte in each.
    p_storage_param.block_size  = STORE_BLOCK_SIZE; // recommended to be >= 4 byte
    p_storage_param.block_count = STORE_BLOCK_COUNT;
    p_storage_param.cb          = storage_callback; //NULL; //
    //130 too much, 128 ok. = 2k bytes
    err_code = pstorage_register (&p_storage_param, &m_p_storage_id);
    APP_ERROR_CHECK(err_code);
    err_code = pstorage_block_identifier_get(&m_p_storage_id, block_curr_idx, &block_handle);
    APP_ERROR_CHECK(err_code);
}

uint32_t store_contact(contact row) {
	static uint32_t err_code = NRF_SUCCESS;
	//uint32_t err_code = NRF_SUCCESS;
	//if (cache_current_idx > 35 ) {
	//	err_code = initiate_write();
	//} else {
		//TODO: check it and dont clear the contact
		if (storage_locked) {
			return NRF_ERROR_BUSY;
		}
		storage_cache[0] = row.address;
		//storage_cache[cache_current_pos++] = row.first_epoch;
		storage_cache[1] = row.first_time_seen;
		storage_cache[2] = row.last_epoch;
		storage_cache[3] = row.last_time_seen;
		(void) cache_current_idx;
		__LOG("Writing: %x:%x:%x:%x", storage_cache[0],
				storage_cache[1],
				storage_cache[2],
				storage_cache[3]);
		err_code = initiate_write();
		/*while (storage_locked) {
		    err_code = sd_app_evt_wait();
		    APP_ERROR_CHECK(err_code);
		}*/
		//memset(&storage_cache, 0, sizeof(storage_cache));
		return err_code;
}

uint32_t read_contact_store(uint8_t * p_row) {
	static uint32_t err_code = NRF_SUCCESS;
	uint8_t * row;
	row = (uint8_t *) malloc(sizeof(uint8_t)*4);
	memset(&storage_cache, 0, sizeof(storage_cache));
	storage_locked = true;
	err_code = pstorage_load(row, &block_handle, STORE_BLOCK_SIZE, 0);
	while (storage_locked) {
	    err_code = sd_app_evt_wait();
	    APP_ERROR_CHECK(err_code);
	}
	memcpy (p_row, row, sizeof(uint8_t) * 4);
	return NRF_SUCCESS;
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
	static uint32_t err_code = NRF_SUCCESS;
	storage_locked = true; // cb will unlock this.
	//pstorage_handle_t block_handle;
	//uint8_t           source_data[4];
	//uint32_t          err_code;
	// Request to write 8 bytes to block at an offset of 20 bytes.
	err_code = pstorage_store(&block_handle, storage_cache, STORE_BLOCK_SIZE, 0);
	APP_ERROR_CHECK(err_code);
	return err_code;
}

void storage_clear()
{	//TODO: make this a nus command
    pstorage_clear(&block_handle, STORE_BLOCK_SIZE * STORE_BLOCK_COUNT);
    memset(&storage_cache, 0, sizeof(storage_cache));
    cache_current_idx = 0;
    block_curr_idx = 0;
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
