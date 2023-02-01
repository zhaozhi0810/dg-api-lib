/*
 * touchscreen.c
 *
 *  Created on: Dec 2, 2021
 *      Author: zlf
 */

#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>

#include "debug.h"
#include "touchscreen.h"

static int s_touchscreen_control(bool enable) {
	int fd = 0;
	char cmd[16] = "\0";
	CHECK((fd = open(TOUCHSCREEN_ILITEK_CONTROL_FILE, O_WRONLY)) > 0, -1, "Error open with %d: %s", errno, strerror(errno));
	strncpy(cmd, enable? TOUCHSCREEN_ILITEK_CONTROL_ENABLE:TOUCHSCREEN_ILITEK_CONTROL_DISABLE, sizeof(cmd));
	if(write(fd, cmd, strlen(cmd) + 1) != strlen(cmd) + 1) {
		ERR("Error write with %d: %s", errno, strerror(errno));
		close(fd);
		return -1;
	}
	close(fd);
	return 0;
}

void drvEnableTouchModule(void) {
	CHECK(!s_touchscreen_control(true), , "Error s_touchscreen_write_cmd!");
}

void drvDisableTouchModule(void) {
	CHECK(!s_touchscreen_control(false), , "Error s_touchscreen_write_cmd!");
}
