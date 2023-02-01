/*
 * misc.h
 *
 *  Created on: Dec 2, 2021
 *      Author: zlf
 */

#ifndef LIB_MISC_MISC_H_
#define LIB_MISC_MISC_H_

#include <stdbool.h>

extern int gpio_notify_func(int gpio, int val);

extern bool get_headset_insert_status(void);
extern bool get_handle_insert_status(void);

#endif /* LIB_MISC_MISC_H_ */
