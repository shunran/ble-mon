/*
 * timer.h
 *
 *  Created on: 19. märts 2016
 *      Author: lauri
 */

#ifndef TIMER_H_
#define TIMER_H_

void timers_init(void);
void timeout_handler(void * p_context);
void timers_start(void);



#endif /* TIMER_H_ */
