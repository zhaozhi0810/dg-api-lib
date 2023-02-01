/*
 * gpio.c
 *
 *  Created on: Nov 26, 2021
 *      Author: zlf
 */


#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#include "debug.h"
#include "gpio.h"

int gpio_get_value(int gpio_num, int *value) {
	CHECK(gpio_num >= 0, -1, "Error gpio_num is %d", gpio_num);
	CHECK(value, -1, "Error value is null");

	char data[4] = {0};
	char fd_name[64] = "\0";
	int fd = 0;

	snprintf(fd_name, sizeof(fd_name), "/sys/class/gpio/gpio%d/value", gpio_num);
	CHECK((fd = open(fd_name, O_RDONLY, 0)) > 0, -1, "Error open with %d: %s", errno, strerror(errno));

	if(read(fd, data, sizeof(data)) <= 0) {
		ERR("Error read with %d: %s", errno, strerror(errno));
		close(fd);
		return -1;
	}
	CHECK(!close(fd), -1, "Error close with %d: %s", errno, strerror(errno));
	DBG("data = %s", data);
	*value = atoi(data);
	return 0;
}

int gpio_set_value(int gpio_num, int value) {
	CHECK(gpio_num >= 0, -1, "Error gpio_num is %d", gpio_num);
	CHECK(value == 0 || value == 1, -1, "Error value out of range!");
	char cmd[64] = "\0";
	snprintf(cmd, sizeof(cmd), "echo %d > /sys/class/gpio/gpio%d/value", value? 1:0, gpio_num);
	CHECK(!system(cmd), -1, "Error system with %d: %s", errno, strerror(errno));
	return 0;
}

int gpio_exit(int gpio_num) {
	CHECK(gpio_num >= 0, -1, "Error gpio_num is %d", gpio_num);
	char cmd[64] = "\0";
	snprintf(cmd, sizeof(cmd), "/sys/class/gpio/gpio%d", gpio_num);
	if(access(cmd, F_OK)) {
		snprintf(cmd, sizeof(cmd), "echo %d > /sys/class/gpio/unexport", gpio_num);
		CHECK(!system(cmd), -1, "Error system with %d: %s", errno, strerror(errno));
	}
	return 0;
}

int gpio_init(int gpio_num, bool output, GPIO_INPUT_EDGE_E edge) {
	CHECK(gpio_num >= 0, -1, "Error gpio_num is %d", gpio_num);

	char cmd[64] = "\0";

	snprintf(cmd, sizeof(cmd), "/sys/class/gpio/gpio%d", gpio_num);
	if(access(cmd, F_OK)) {
		snprintf(cmd, sizeof(cmd), "echo %d > /sys/class/gpio/export", gpio_num);
		CHECK(!system(cmd), -1, "Error system with %d: %s", errno, strerror(errno));
	}

	snprintf(cmd, sizeof(cmd), "echo %s > /sys/class/gpio/gpio%d/direction", output? "out":"in", gpio_num);
	if(system(cmd)) {
		ERR("Error system with %d: %s", errno, strerror(errno));
		gpio_exit(gpio_num);
		return -1;
	}

	if(!output) {
		memset(cmd, '\0', sizeof(cmd));
		switch(edge) {
		case GPIO_INPUT_EDGE_NONE:
			snprintf(cmd, sizeof(cmd), "echo none > /sys/class/gpio/gpio%d/edge", gpio_num);
			break;
		case GPIO_INPUT_EDGE_RISING:
			snprintf(cmd, sizeof(cmd), "echo rising > /sys/class/gpio/gpio%d/edge", gpio_num);
			break;
		case GPIO_INPUT_EDGE_FALLING:
			snprintf(cmd, sizeof(cmd), "echo falling > /sys/class/gpio/gpio%d/edge", gpio_num);
			break;
		case GPIO_INPUT_EDGE_BOTH:
			snprintf(cmd, sizeof(cmd), "echo both > /sys/class/gpio/gpio%d/edge", gpio_num);
			break;
		default:
			ERR("Error non-supported edge %d", edge);
			gpio_exit(gpio_num);
			return -1;
		}
		if(system(cmd)) {
			ERR("Error system with %d: %s", errno, strerror(errno));
			gpio_exit(gpio_num);
			return -1;
		}
	}
	return 0;
}
