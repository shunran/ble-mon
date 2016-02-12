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
#ifndef BLE400_H
#define BLE400_H

#include "nrf_gpio.h"

#define LED_START      18
#define LED_0          18
#define LED_1          19
#define LED_2          20
#define LED_3          21
#define LED_4          22
#define LED_STOP       22

//#define BUTTON_START   16
//#define BUTTON_0       16
//#define BUTTON_1       17
//#define BUTTON_STOP    17
//#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define LED_0_MASK    (1<<LED_0)
#define LED_1_MASK    (1<<LED_1)
#define LED_2_MASK    (1<<LED_2)
#define LED_3_MASK    (1<<LED_3)
#define LED_4_MASK    (1<<LED_4)

#define LEDS_MASK      (LED_0_MASK | LED_1_MASK | LED_2_MASK | LED_3_MASK | LED_4_MASK)

#define BUTTONS_NUMBER 2
#define LEDS_NUMBER    5
#define LEDS_LIST { LED_0, LED_1, LED_2, LED_3, LED_4 }

//#define LEDS_MASK		0xcc0000000

//Board
#define RX_PIN_NUMBER  11
#define TX_PIN_NUMBER  9
#define CTS_PIN_NUMBER 10
#define RTS_PIN_NUMBER 8
//Module
//#define RX_PIN_NUMBER  5
//#define TX_PIN_NUMBER  6
//#define CTS_PIN_NUMBER 7
//#define RTS_PIN_NUMBER 12
//#define HWFC           true
#define HWFC           false

#endif
