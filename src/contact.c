/*
 * contact.c
 *
 *  Created on: 20. märts 2016
 *      Author: lauri
 */

/**@brief Manipulating contact structure. */
#include "contact.h"
#include <stddef.h>
#include <inttypes.h>
#include <stdlib.h>
#include "nrf51.h"
#include "string.h"

#include "timer.h"
#include "uart.h"
#include "storage.h"
#include "gpio.h"

#define CONTACT_MIN_RSSI		-98
//contact contacts[NUMBER_OF_DEVICES];

adv_report * reports;
uint8_t reports_idx = 0;

static uint32_t divisor;
//static uint8_t calc_current_ts(void);
static void clear_contact(uint8_t address);

void contacts_init() {
	divisor = 0xFFFFFF / (APP_TIMER_PRESCALER + 1) / 0xFF;
	//UINT16_MAX = 65535 = 2 bytes
    /*memset(&contacts, 0, sizeof(contacts));
	//contacts[NUMBER_OF_CONTACTS] = NULL;
    for (uint8_t i = 0; i < NUMBER_OF_DEVICES; i++)
    {
    	contacts[i].address = i;
    }
    */
	reports = malloc(sizeof(adv_report) * 10);
	//memset(reports, 0, sizeof(reports));
    return;
}

void make_contact(uint8_t address) {
//	uint8_t curr_ts;
//	curr_ts = calc_current_ts();
//	if (contacts[address].first_time_seen == 0) {
//		timer_event_indication = indicate_proximity;
//		contacts[address].first_time_seen = curr_ts;
//		contacts[address].first_epoch = timer_epoch;
//	}
//	contacts[address].last_time_seen = curr_ts;
//	contacts[address].last_epoch = timer_epoch;
}

void close_contacts() {
	// check if writing is not in progress
//	uint8_t ts = calc_current_ts();
//	for (uint8_t i = 0; i < NUMBER_OF_DEVICES; i++)
//			{
//				if (contacts[i].last_time_seen == 0) continue;
//				//uint8_array_t;
//				int16_t counter_diff = ts - contacts[i].last_time_seen; // can be negative, so 16 is needed.
//				uint8_t epoch_diff = timer_epoch - contacts[i].last_epoch; // cannot be negative
//				uint16_t total_diff = UINT8_MAX * epoch_diff + counter_diff;
//				if (total_diff > CONTACT_CLOSE_INTERVAL) {
//					timer_event_indication = indicate_proximity_lost;
//					store_contact(contacts[i]);
//					clear_contact(contacts[i].address);
//					__LOG("lost contact.");
//					//(void) __NOP;
//					//TODO: put it into store array, if store arr full, initiate store write and wait
//					//TODO: clear last_seen
//				}
//				//if (time_diff > CONTACT_CLOSE_INTERVAL && last_epoch <= timer_epoch) {
//				//}
//				//contacts[i].address = i;
//			}
}

//contact get_contact(uint8_t address) {
////	return contacts[address];
//}

void clear_contacts()
{
//	memset(&contacts, 0, sizeof(contacts));
    for (uint8_t i = 0; i < NUMBER_OF_DEVICES; i++)
    {
    	clear_contact(i);
    }
    return;
}

static void clear_contact(uint8_t address)
{
//	memset(&contacts[address], 0, sizeof(contact));
//	contacts[address].address = address;
}

/**
 *
 */
static uint8_t get_ts(uint32_t cnt) {
	uint8_t result = cnt / divisor;
	return result;
}


void make_report(uint32_t counter, uint8_t epoch, int8_t rssi, uint8_t addr) {
	//reports[reports_idx];
	//adv_report rep;
	//rep.addr = addr;
//	(reports + reports_idx)->ts = ts;
//	(reports + reports_idx)->epoch = epoch;
//	(reports + reports_idx)->rssi = rssi;
//	(reports + reports_idx++)->addr = addr;
	//(reports + reports_idx) = &s_rep;
//	if (reports_idx == 10) {
	if (rssi >= CONTACT_MIN_RSSI)
		{
			timer_event_indication = indicate_proximity;
			store_report(get_ts(counter), epoch, rssi, addr);
			__LOG("Contact from addr:%d rssi:%d", addr, rssi);
		}

//	}
//	uint8_t curr_ts;
//	curr_ts = calc_current_ts();
//	if (contacts[address].first_time_seen == 0) {
//		timer_event_indication = indicate_proximity;
//		contacts[address].first_time_seen = curr_ts;
//		contacts[address].first_epoch = timer_epoch;
//	}
//	contacts[address].last_time_seen = curr_ts;
//	contacts[address].last_epoch = timer_epoch;
}
