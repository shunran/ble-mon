/*
 * timer.c
 *
 *  Created on: 19. märts 2016
 *      Author: lauri
 */
/**@brief Function for starting timers.
*/
#include <nrf.h>
#include "nrf_delay.h"
#include <inttypes.h>
#include <string.h>
#include "boards.h"
#include "uart.h"
#include "contact.h"
#include "app_timer.h"

#include "timer.h"

APP_TIMER_DEF(m_app_timer_id);

void timers_start(void)
{
	uint32_t timeout_ticks = 100000; //3 * 32768;
    uint32_t err_code;
    err_code = app_timer_start(m_app_timer_id, timeout_ticks, NULL);
    APP_ERROR_CHECK(err_code);
}

/**
 * Timeout handler for appl timer.
 */
void timeout_handler(void * p_context)
{
	const uint32_t this_time = NRF_RTC1->COUNTER;
	static uint32_t last_time = 0;
	static uint8_t epoch = 0;
	// First call, only constant can be assigned in static declaration.
	if (last_time == 0) last_time = this_time;
	// OVERFLW event has happened.
	if (last_time > this_time) ++epoch;

	last_time = this_time;
    nrf_gpio_pin_set(LED_3);
    nrf_delay_ms(20);
    nrf_gpio_pin_clear(LED_3);
    __LOG("epoch %d and clock %u.", epoch, this_time);
    //__LOG("original %x.", this_time);
	//const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;

  //  UNUSED_PARAMETER(p_context);
  //stop the timer(s):
  //uint32_t err_code;
  //err_code = app_timer_stop(m_app_timer_id);
  //APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
void timers_init(void)
{
	uint32_t err_code;
//	 uint32_t err_code = nrf_drv_clock_init(NULL);
//	 APP_ERROR_CHECK(err_code);
//	 nrf_drv_clock_lfclk_request();

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    // Create timers.ˇ

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one. */
    err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timeout_handler);
    APP_ERROR_CHECK(err_code); /**/
}
