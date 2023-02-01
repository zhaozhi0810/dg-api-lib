/*
 * screen.h
 *
 *  Created on: Nov 23, 2021
 *      Author: zlf
 */

#ifndef LIB_PANEL_SCREEN_H_
#define LIB_PANEL_SCREEN_H_

#define SCREEN_UART_DEVICE		"/dev/ttyS4"

#define UART_FRAME_HEADER_0		(0xa5)
#define UART_FRAME_HEADER_1		(0x5a)

#define SCREEN_BRIGHTNESS_MIN	(0)
#define SCREEN_BRIGHTNESS_MAX	(0xff)

#define UART_FRAME_VERIFY(arg0, arg1, arg2, arg3, arg4)	\
	((arg0 + arg1 + arg2 + arg3 + arg4) & 0xFF)

typedef struct {
	unsigned char cmd_header0;
	unsigned char cmd_header1;
	unsigned char cmd;
	unsigned char data;
	unsigned char reserve0;
	unsigned char reserve1;
	unsigned char reserve2;
	unsigned char verify;
} SCREEN_UART_FRAME_FORMAT_S;

typedef enum {
	SCREEN_COMM_CMD_SET_BRIGHTNESS = 0x80,
	SCREEN_COMM_CMD_WAKE_OFF = 0x84,
	SCREEN_COMM_CMD_WAKE = 0x86,
	SCREEN_COMM_CMD_GET_MODEL = 0x88,
	SCREEN_COMM_CMD_GET_VER = 0x8a,
	SCREEN_COMM_CMD_MAX,
} SCREEN_COMM_CMD_E;

#endif /* LIB_PANEL_SCREEN_H_ */
