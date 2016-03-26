/**
 * gpio.h
 */

#ifndef GPIO_H_
#define GPIO_H_

void gpio_init(void);
void indicate_init(void); //Show a stripe or 5 blinks depending on leds count.
void indicate_proximity(void); // Show 1 blink.
void indicate_proximity_lost(void); //Show 2 blinks.
void indicate_connection(void);
void indicate_connection_lost(void);
void indicate_storage_problem(void); 	//3 quick blinks
void indicate_advertising(void);
void indicate_timer_event(void);
void indicate_battery_low(void);

#endif /* GPIO_H_ */
