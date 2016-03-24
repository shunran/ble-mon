/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
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

#ifndef NRF_DRV_CONFIG_H
#define NRF_DRV_CONFIG_H

/* GPIOTE */
#define GPIOTE_ENABLED 1

#if (GPIOTE_ENABLED == 1)
#define GPIOTE_CONFIG_USE_SWI_EGU false
#define GPIOTE_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW
#define GPIOTE_CONFIG_NUM_OF_LOW_POWER_EVENTS 4
#endif


/* UART */
#define UART0_ENABLED 1

#if (UART0_ENABLED == 1)
#define UART0_CONFIG_HWFC         NRF_UART_HWFC_DISABLED
#define UART0_CONFIG_PARITY       NRF_UART_PARITY_EXCLUDED
#define UART0_CONFIG_BAUDRATE     NRF_UART_BAUDRATE_38400
#define UART0_CONFIG_PSEL_TXD 9
#define UART0_CONFIG_PSEL_RXD 11
#define UART0_CONFIG_PSEL_CTS 10
#define UART0_CONFIG_PSEL_RTS 8
#define UART0_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW
#endif

#include "nrf_drv_config_validation.h"

#endif // NRF_DRV_CONFIG_H
