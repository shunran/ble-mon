/*
 * led.c
 *
 *  Created on: 21. märts 2016
 *      Author: lauri
 */
#include "gpio.h"

#include "boards.h"
/**
 * See indication võiks siin olla vastavalt boardidele.
 */
void indicate_proximity() {
	//2 plinki
}

void indicate_proximity_lost() {
	//1 plink
}

void indicate_init() {
	//triip või 3 plinki
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

void indicate_timer() {
	// ??
}

void indicate_battery_low() {
	// ??
}
