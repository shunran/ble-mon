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

#endif /* RADIO_H_ */
