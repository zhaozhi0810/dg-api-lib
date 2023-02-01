/*
* @Author: dazhi
* @Date:   2022-10-27 14:01:26
* @Last Modified by:   dazhi
* @Last Modified time: 2022-10-27 14:34:34
*/
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


#include "debug.h"
#include "uart.h"
#include "screen.h"
//#include "mcu.h"


static int s_uart_cmd_send(int uart_fd, unsigned char cmd, unsigned char *data) {
	CHECK(uart_fd > 0, -1, "Error uart_fd is null!");
	SCREEN_UART_FRAME_FORMAT_S cmd_send_format;

	bzero(&cmd_send_format, sizeof(SCREEN_UART_FRAME_FORMAT_S));
	cmd_send_format.cmd_header0 = UART_FRAME_HEADER_0;
	cmd_send_format.cmd_header1 = UART_FRAME_HEADER_1;
	cmd_send_format.cmd = cmd;
	if(data) {
		cmd_send_format.data = *data;
	}
	cmd_send_format.verify = UART_FRAME_VERIFY(
			cmd_send_format.cmd,
			cmd_send_format.data,
			0,
			0,
			0);
	CHECK(!uart_send(uart_fd, &cmd_send_format, sizeof(SCREEN_UART_FRAME_FORMAT_S)), -1, "Error uart_send!");
	return 0;
}

static int s_uart_cmd_recv(int uart_fd, unsigned char cmd, unsigned char *data) {
	CHECK(uart_fd > 0, -1, "Error uart_fd is null!");

	SCREEN_UART_FRAME_FORMAT_S cmd_reply_format;

	bzero(&cmd_reply_format, sizeof(SCREEN_UART_FRAME_FORMAT_S));
	CHECK(!uart_recv(uart_fd, &cmd_reply_format, sizeof(SCREEN_UART_FRAME_FORMAT_S)), -1, "Error uart_recv!");

	CHECK(cmd_reply_format.cmd_header0 == UART_FRAME_HEADER_0 && cmd_reply_format.cmd_header1 == UART_FRAME_HEADER_1, -1,
			"Error UART frame head: %#x %#x", cmd_reply_format.cmd_header0, cmd_reply_format.cmd_header1);
	CHECK(cmd_reply_format.verify == UART_FRAME_VERIFY(cmd_reply_format.cmd, cmd_reply_format.data, 0, 0, 0), -1, "Error UART frame verity!");
	CHECK(cmd_reply_format.cmd == cmd + 1, -1, "Error UART frame reply cmd is %d!", cmd_reply_format.cmd);
	if(data) {
		*data = cmd_reply_format.data;
	}
	return 0;
}



void drvSetLcdBrt(int nBrtVal) {
	CHECK(nBrtVal >= SCREEN_BRIGHTNESS_MIN && nBrtVal <= SCREEN_BRIGHTNESS_MAX, , "Error screen brightness out of range!");

	int uart_fd = uart_device_init(SCREEN_UART_DEVICE);
	CHECK(uart_fd > 0, , "Error s_uart_device_init!");

	if(s_uart_cmd_send(uart_fd, SCREEN_COMM_CMD_SET_BRIGHTNESS, (unsigned char *)&nBrtVal)) {
		ERR("Error s_uart_cmd_send!");
		uart_device_exit(uart_fd);
		return;
	}
	if(s_uart_cmd_recv(uart_fd, SCREEN_COMM_CMD_SET_BRIGHTNESS, NULL)) {
		ERR("Error s_uart_cmd_recv!");
		uart_device_exit(uart_fd);
		return;
	}
	uart_device_exit(uart_fd);
}


int main(int argc ,char *argv[])
{
	int brightness = 0;
	if(argc < 2){
		printf("USAGE: %s <bright value(0-255)>\n",argv[0]);
		return -1;
	}


	brightness = atoi(argv[1]);

	if(brightness < 0)
		brightness = 0;
	else if (brightness > 255)
		brightness = 255;


	drvSetLcdBrt(brightness);

	return 0;
}




