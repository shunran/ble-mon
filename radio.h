/*
 * radio.h
 *
 *  Created on: 20. märts 2016
 *      Author: lauri
 */

#ifndef RADIO_H_
#define RADIO_H_

#include "ble.h"
#include "ble_advertising.h"

void gap_params_init(void);
void advertising_init(void);
void ble_stack_init(void);
void scan_start(void);
void advertising_start(void);
void services_init(void);
void conn_params_init(void);

static void on_ble_evt(ble_evt_t * p_ble_evt);
static void conn_params_error_handler(uint32_t nrf_error);
static void on_adv_evt(ble_adv_evt_t ble_adv_evt);

#endif /* RADIO_H_ */
