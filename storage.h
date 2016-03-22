/*
 * storage.h
 *
 *  Created on: 21. märts 2016
 *      Author: lauri
 */

#ifndef STORAGE_H_
#define STORAGE_H_
#include <stdbool.h>

void read_contacts();
void storage_init();
uint32_t store_contact(contact);
uint8_t read_contact_store(uint8_t *, uint8_t); // 1 - wait for more, 0 - that's all folks
void storage_toggle_lock();
void storage_clear(void);

#endif /* STORAGE_H_ */
