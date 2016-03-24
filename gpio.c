/*
 * led.c
 *
 *  Created on: 21. märts 2016
 *      Author: lauri
 */
#include "gpio.h"
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"

#define DISABLE_INDICATION	0
#define DELAY_MS	100


void gpio_init() {
	//nrf_gpio_range_cfg_output(LED_START, LED_STOP);
	LEDS_CONFIGURE(LEDS_MASK);
	//nrf_gpio_cfg_output(33);
	indicate_init();
}
/**
 * Indicate device detection in proximity.
 * Show 2 blinks.
 */
void indicate_proximity() {
	if (DISABLE_INDICATION) return;
	LEDS_OFF(LEDS_MASK);
	for (int i = 0; i < 4; i++) {
		LEDS_INVERT(LEDS_MASK);
		nrf_delay_ms(DELAY_MS);
	}
}

/**
 * Indicate device proximity lost.
 * 	Show 1 blink.
 */
void indicate_proximity_lost() {
	if (DISABLE_INDICATION) return;
	LEDS_ON(LEDS_MASK);
	nrf_delay_ms(DELAY_MS);
	LEDS_OFF(LEDS_MASK);
}

/**
 * Indicate intialization of the buttons and leds.
 * Show a stripe or 5 blinks depending on leds count.
 */
void indicate_init() {
	if (DISABLE_INDICATION) return;
	const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;
	if (LEDS_NUMBER < 2)
	{
		for (int i = 0; i < 10; i++)
		{
			LEDS_INVERT(LEDS_MASK);
			nrf_delay_ms(DELAY_MS * 2);
		}
	} else {
		for (int i = 0; i < LEDS_NUMBER; i++)
			{
				LEDS_INVERT(1 << leds_list[i]);
				nrf_delay_ms(DELAY_MS * 2);
			}
		for (int i = 0; i < LEDS_NUMBER; i++)
		{
			LEDS_INVERT(1 << leds_list[i]);
			nrf_delay_ms(DELAY_MS * 2);
		}
	}
}

void indicate_connection() {
	//5 plinki
}

void indicate_connection_lost() {
	//5 plinki ?
}

void indicate_storage_full() {
	//10 plinki
}

void indicate_advertising() {
	// ??
}

void indicate_timer_event() {
	// ??
}

void indicate_battery_low() {
	// ??
}
