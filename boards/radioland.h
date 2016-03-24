/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef RADIOLAND_H
#define RADIOLAND_H

#include "nrf_gpio.h"

#define LED_START      29
#define LED_0          29
#define LED_STOP       29
/*
#define BUTTON_START   16
#define BUTTON_0       16
#define BUTTON_1       17
#define BUTTON_STOP    17
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP
*/
#define LED_0_MASK    (1<<LED_0)
#define LEDS_INV_MASK  0

#define LEDS_MASK      (LED_0_MASK)
#define LEDS_NUMBER    1
#define LEDS_LIST { LED_0 }

//Module
#define RX_PIN_NUMBER  5
#define TX_PIN_NUMBER  6
#define CTS_PIN_NUMBER 7
#define RTS_PIN_NUMBER 12
//#define HWFC           true
#define HWFC           false

#endif
