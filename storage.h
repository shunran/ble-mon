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
void read_contact_store(uint8_t *, uint8_t *);
bool storage_toggle_lock(); // returns status after operation
void storage_clear(void);
void store_read_init(void);

#endif /* STORAGE_H_ */
