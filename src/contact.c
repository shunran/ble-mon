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
static void clear_contact(uint8_t address);

void contacts_init() {
	divisor = 0xFFFFFF / (APP_TIMER_PRESCALER + 1) / 0xFF;
	reports = malloc(sizeof(adv_report) * 10);
    return;
}

void make_contact(uint8_t address)
{

}

void close_contacts()
{

}

void clear_contacts()
{
    for (uint8_t i = 0; i < NUMBER_OF_DEVICES; i++)
    {
    	clear_contact(i);
    }
    return;
}

static void clear_contact(uint8_t address)
{

}

static uint8_t get_ts(uint32_t cnt) {
	uint8_t result = cnt / divisor;
	return result;
}


void make_report(uint32_t counter, uint8_t epoch, int8_t rssi, uint8_t addr) {
	if (rssi >= CONTACT_MIN_RSSI)
		{
			timer_event_indication = indicate_proximity;
			store_report(get_ts(counter), epoch, rssi, addr);
			__LOG("Contact from addr:%d rssi:%d", addr, rssi);
		}
}
