/*
 * contact.h
 *  Created on: 20. märts 2016
 *      Author: lauri
 */

#ifndef CONTACT_H_
#define CONTACT_H_

#define NUMBER_OF_DEVICES		5 // Number of existing devices.
#define CONTACT_CLOSE_INTERVAL 2 // Not present interval to close contact. In units of 2.007843 seconds.

#include <inttypes.h>

typedef struct
{
    uint8_t                    first_time_seen; /**< Number of RTC1 ticks at contact start */
    uint8_t                    first_epoch; /**< Timer Epoch at contact start. */
    uint8_t                    last_time_seen;   /**< Number of RTC1 ticks at contact end. */
    uint8_t                    last_epoch;   /**< Timer Epoch at contact end. */
    uint8_t                    address;        /**< Address of detected party. */
} contact;

extern contact contacts[NUMBER_OF_DEVICES];

void make_contact(uint8_t);
void contacts_init(void);
void close_contacts(void);
void clear_contacts();
contact get_contact(uint8_t);

#endif /* CONTACT_H_ */
