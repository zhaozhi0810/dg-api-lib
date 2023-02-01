/*
 * fs.c
 *
 *  Created on: Jan 6, 2022
 *      Author: zlf
 */



#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include "debug.h"

#define PROC_INPUT_DEVICES		"/proc/bus/input/devices"

int get_event_dev(char *name) {
	CHECK(name, -1, "Error name is null!");
	CHECK(strlen(name), -1, "Error name lenth is 0");

	FILE *input_device_fd = NULL;
	char *lineptr = NULL;
	int input_device_num = -1;
	size_t n = 0;

	CHECK(input_device_fd = fopen(PROC_INPUT_DEVICES, "r"), -1, "Error fopen with %d: %s", errno, strerror(errno));
	while(getline(&lineptr, &n, input_device_fd) != EOF) {
		if(strstr(lineptr, "Name") && strstr(lineptr, name)) {
			char *strstrp = NULL;
			while(getline(&lineptr, &n, input_device_fd) != EOF) {
				if((strstrp = strstr(lineptr, "Handlers"))) {
					if((strstrp = strstr(strstrp, "event"))) {
						strstrp += strlen("event");
						input_device_num = atoi(strstrp);
					}
					break;
				}
			}
			break;
		}
	}
	if(lineptr) {
		free(lineptr);
		lineptr = NULL;
	}
	CHECK(!fclose(input_device_fd), -1, "Error fclose with %d: %s", errno, strerror(errno));
	return input_device_num;
}
