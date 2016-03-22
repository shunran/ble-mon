/*
 * storage.h
 *
 *  Created on: 21. märts 2016
 *      Author: lauri
 */

#ifndef STORAGE_H_
#define STORAGE_H_

void read_contacts();
void storage_init();
uint32_t store_contact(contact);
uint32_t read_contact_store(uint8_t *);

#endif /* STORAGE_H_ */
