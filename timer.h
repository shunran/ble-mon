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

void timers_init(void);
void timeout_handler(void * p_context);
void timers_start(void);



#endif /* TIMER_H_ */
