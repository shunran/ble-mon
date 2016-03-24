/*
 * timer.h
 *
 *  Created on: 19. märts 2016
 *      Author: lauri
 */

#ifndef TIMER_H_
#define TIMER_H_

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */

extern uint8_t timer_epoch;

void (*timer_event_indication)(void); /** this is executed by timer event handler if its not null and can contain any void func(void) */
void timers_init(void); /** initialization of timer module */
void timeout_handler(void * p_context); /** timer event handler */
void timers_start(void); /** start the timers */



#endif /* TIMER_H_ */
