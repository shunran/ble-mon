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
#include "contact.h"

APP_TIMER_DEF(m_app_timer_id);

uint8_t timer_epoch;
#define TIMER_TIMEOUT_TICKS	65536 /** 2 * 32768; */

void timers_start(void)
{
	uint32_t timeout_ticks = TIMER_TIMEOUT_TICKS;
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

	// First call, only constant can be assigned in static declaration.
	if (last_time == 0) last_time = this_time;
	// OVERFLW event has happened.
	if (last_time > this_time) ++timer_epoch;

	//contact_get_current_time();
	//init_contacts();
	close_contacts();
	if (timer_event_indication) {
		(*timer_event_indication)();
		//void * s = &timer_event_indication;
		timer_event_indication = NULL;
		//(*timer_event_indication)(void) = NULL;
	}

	last_time = this_time;


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

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timer.ˇ
    err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timeout_handler);
    APP_ERROR_CHECK(err_code); /**/

    timer_epoch = 0;
}
