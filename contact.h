/*
 * contact.h
 *  Created on: 20. märts 2016
 *      Author: lauri
 */

#ifndef CONTACT_H_
#define CONTACT_H_

#define NUMBER_OF_CONTACTS			5

typedef struct
{
    uint8_t                    first_time_seen; /**< Number of RTC1 ticks at contact start */
    uint8_t                    first_epoch; /**< Timer Epoch at contact start. */
    uint8_t                    last_time_seen;   /**< Number of RTC1 ticks at contact end. */
    uint8_t                    last_epoch;   /**< Timer Epoch at contact end. */
    uint8_t                    address;        /**< Address of detected party. */
} contact;

contact contacts[NUMBER_OF_CONTACTS];
#endif /* CONTACT_H_ */
