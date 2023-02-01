/*
 * keyboard.c
 *
 *  Created on: Dec 2, 2021
 *      Author: zlf
 */

#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <linux/input.h>

#include "debug.h"
#include "drv722.h"
#include "fs.h"
#include "keyboard_cmd.h"

#define JC_KEYBOARD_DRIVER_NAME		"jc_keyboard"
#define JC_KEYBOARD_BRIGHTNESS_MIN	(0)
#define JC_KEYBOARD_BRIGHTNESS_MAX	(100)
#define ALL_KEY_INDEX 				(40)  /*dazhi modify 2023-01-05,lsr add 20220505*/

typedef struct {
	char event_dev_name[32];
	char keyboard_dev_name[32];
	char model;
	char version;
} KEYBOARD_INFO_S;

static bool s_recv_event_thread_exit = false;
static pthread_t s_recv_key_event_thread_id = 0;
static GPIO_NOTIFY_KEY_FUNC s_key_notify_func = NULL;
static KEYBOARD_INFO_S s_keyboard_info;
static int key_inited = 0;   //0表示未初始化，1非0表示初始化  2022-11-24



static void *s_recv_event_thread(void *arg) {
	fd_set readfds;
	int event_dev_fd = 0;
	struct input_event ts;
	CHECK((event_dev_fd = open(s_keyboard_info.event_dev_name, O_RDONLY, 0)) > 0, NULL, "Error open with %d: %s", errno, strerror(errno));
	INFO("Enter %s", __func__);
	while(!s_recv_event_thread_exit) {
		struct timeval timeout = {1, 0};
		FD_ZERO(&readfds);
		FD_SET(event_dev_fd, &readfds);
		if(select(event_dev_fd + 1, &readfds, NULL, NULL, &timeout) < 0) {
			ERR("Error select with %d: %s", errno, strerror(errno));
			s_recv_event_thread_exit = true;
		}
		else if(FD_ISSET(event_dev_fd, &readfds)) {
			bzero(&ts, sizeof(struct input_event));
			if(read(event_dev_fd, &ts, sizeof(struct input_event)) != sizeof(struct input_event)) {
				ERR("Error read with %d: %s", errno, strerror(errno));
				continue;
			}
			if(s_key_notify_func) {				
				if(ts.type == EV_KEY) {
					s_key_notify_func(ts.code, ts.value);
				}
			}
		}
	}
	CHECK(!close(event_dev_fd), NULL, "Error close with %d: %s", errno, strerror(errno));
	INFO("Exit %s", __func__);
	return NULL;
}

int keyboard_init(void) {
	int input_device_num = 0;
	memset(&s_keyboard_info, -1, sizeof(KEYBOARD_INFO_S));
	input_device_num = get_event_dev(JC_KEYBOARD_DRIVER_NAME);
	printf("keyboard_init input_device_num = %d\n",input_device_num);
	CHECK((input_device_num >= 0), -1, "Error get_event_dev!");
	printf("keyboard_init xxxxxxx\n");

	snprintf(s_keyboard_info.event_dev_name, sizeof(s_keyboard_info.event_dev_name), "/dev/input/event%d", input_device_num);
	snprintf(s_keyboard_info.keyboard_dev_name, sizeof(s_keyboard_info.keyboard_dev_name), "/dev/%s", JC_KEYBOARD_DRIVER_NAME);
	CHECK(!pthread_create(&s_recv_key_event_thread_id, NULL, s_recv_event_thread, NULL), -1, "Error pthread_create with %d: %s", errno, strerror(errno));
	
	key_inited = 1;
	return 0;
}

int keyboard_exit(void) {
	if(!key_inited)
	{
		return -1;
	}
	s_recv_event_thread_exit = true;
	pthread_cancel(s_recv_key_event_thread_id);   //线程取消
	CHECK(!pthread_join(s_recv_key_event_thread_id, NULL), -1, "Error pthread_join with %d: %s", errno, strerror(errno));
	return 0;
}

static int s_ioctl_cmd(int cmd, char *val) {
	int dev_fd = 0;
	if(!key_inited)
	{
		return -1;
	}
	CHECK((dev_fd = open(s_keyboard_info.keyboard_dev_name, O_RDWR, 0)) > 0, -1, "Error open with %d: %s", errno, strerror(errno));
	if(ioctl(dev_fd, cmd, val)) {
		ERR("Error ioctl with %d: %s", errno, strerror(errno));
		CHECK(!close(dev_fd), -1, "Error close with %d: %s", errno, strerror(errno));
		return -1;
	}
	CHECK(!close(dev_fd), -1, "Error close with %d: %s", errno, strerror(errno));
	return 0;
}

int getKeyboardType(void) {
	int keyboard_model = 0;
	if(!key_inited)
	{
		return -1;
	}
	CHECK(s_ioctl_cmd(KEYBOARD_IOC_GET_PANEL_MODEL, &s_keyboard_info.model)>=0, -1, "Error getKeyboardType s_ioctl_cmd!");
	printf("getKeyboardType s_keyboard_info.model = %d\n",s_keyboard_info.model);
	switch(s_keyboard_info.model){ /*lsr 20220510 add*/
			case 0x00:
			case 0x01:
			case 0x02:
			case 0x03:
			case 0x04:
				keyboard_model = 0x04;  /*lsr modify 20220613*/
				break;
			case 0x05:
				keyboard_model = 0x06;
				break;
			case 0x06:
				keyboard_model = 0x01;
			default:
				break;
	}
	return keyboard_model;
}

void drvSetGpioKeyCbk(GPIO_NOTIFY_KEY_FUNC cbk) {
	s_key_notify_func = cbk;
}

void drvSetLedBrt(int nBrtVal) {
	
	CHECK(nBrtVal >= JC_KEYBOARD_BRIGHTNESS_MIN && nBrtVal <= JC_KEYBOARD_BRIGHTNESS_MAX, , "Error nBrtVal %d out of range!", nBrtVal);
	char val = nBrtVal;
	CHECK(!s_ioctl_cmd(KEYBOARD_IOC_SET_BRIGHTNESS, &val), , "Error s_ioctl_cmd!");
	nBrtVal = val;
}

void drvLightLED(int nKeyIndex) {
	char val = nKeyIndex;
	CHECK(!s_ioctl_cmd(KEYBOARD_IOC_KEY_LED_ON, &val), , "Error s_ioctl_cmd!");
}

void drvDimLED(int nKeyIndex) {
	char val = nKeyIndex;
	CHECK(!s_ioctl_cmd(KEYBOARD_IOC_KEY_LED_OFF, &val), , "Error s_ioctl_cmd!");
}

/*lsr modify 20220505 non*/
void drvLightAllLED(void) {
	char val = ALL_KEY_INDEX;
	CHECK(!s_ioctl_cmd(KEYBOARD_IOC_KEY_LED_ON, &val), , "Error s_ioctl_cmd!");
}

void drvDimAllLED(void) {
	char val = ALL_KEY_INDEX;
	CHECK(!s_ioctl_cmd(KEYBOARD_IOC_KEY_LED_OFF, &val), , "Error s_ioctl_cmd!");
}

int drvGetLEDStatus(int nKeyIndex) { /*lsr modify 20220510 non*/
	ERR("Error non-supported!");
	return -1;
}

int drvIfBrdReset(void) {
	CHECK(!s_ioctl_cmd(KEYBOARD_IOC_RESET, NULL), -1, "Error s_ioctl_cmd!");
	return 0;
}

//KEYBOARD_IOC_GET_PANEL_VER,2023-01-03
int getKeyboardMcuVersion(void) {

	if(!key_inited)
	{
		return -1;
	}
	CHECK(!s_ioctl_cmd(KEYBOARD_IOC_GET_PANEL_VER, &s_keyboard_info.version), -1, "Error :getKeyboardMcuVersion:s_ioctl_cmd!");

	return s_keyboard_info.version;
}


