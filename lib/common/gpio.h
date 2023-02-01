/*
 * gpio.h
 *
 *  Created on: Nov 26, 2021
 *      Author: zlf
 */

#ifndef LIB_COMMON_GPIO_H_
#define LIB_COMMON_GPIO_H_

#include <stdbool.h>

typedef enum {
	GPIO_INPUT_EDGE_NONE,
	GPIO_INPUT_EDGE_RISING,
	GPIO_INPUT_EDGE_FALLING,
	GPIO_INPUT_EDGE_BOTH,
} GPIO_INPUT_EDGE_E;

extern int gpio_init(int gpio_num, bool output, GPIO_INPUT_EDGE_E edge);
extern int gpio_exit(int gpio_num);

extern int gpio_get_value(int gpio_num, int *value);
extern int gpio_set_value(int gpio_num, int value);

#endif /* LIB_COMMON_GPIO_H_ */
