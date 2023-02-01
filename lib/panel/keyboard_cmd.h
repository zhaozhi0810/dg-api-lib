/*
 * jc_keyboard_cmd.h
 *
 *  Created on: Jan 5, 2022
 *      Author: zlf
 */

#ifndef JC_KEYBOARD_CMD_H_
#define JC_KEYBOARD_CMD_H_

#include <linux/ioctl.h>

#define KEYBOARD_IOC_MAGIC	'J'

#define KEYBOARD_IOC_GET_BRIGHTNESS		_IOR(KEYBOARD_IOC_MAGIC, 0, char)
#define KEYBOARD_IOC_SET_BRIGHTNESS		_IOW(KEYBOARD_IOC_MAGIC, 1, char)

#define KEYBOARD_IOC_GET_PANEL_MODEL	_IOR(KEYBOARD_IOC_MAGIC, 2, char)
#define KEYBOARD_IOC_GET_PANEL_VER		_IOR(KEYBOARD_IOC_MAGIC, 3, char)

#define KEYBOARD_IOC_RESET				_IOW(KEYBOARD_IOC_MAGIC, 4, char)

#define KEYBOARD_IOC_KEY_LED_ON			_IOW(KEYBOARD_IOC_MAGIC, 5, char)
#define KEYBOARD_IOC_KEY_LED_OFF		_IOW(KEYBOARD_IOC_MAGIC, 6, char)

#endif /* JC_KEYBOARD_CMD_H_ */
