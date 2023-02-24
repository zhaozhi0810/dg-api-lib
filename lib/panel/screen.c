/*
 * panel.c
 *
 *  Created on: Nov 19, 2021
 *      Author: zlf
 */

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#include "debug.h"
#include "screen.h"
#include "uart.h"
#include "mcu.h"

//2022-11-24  不想反复打开了，就打开一次吧，把所有关闭都取消了。
static int uart_fd = -1;

//2022-11-24  不想反复打开了，就打开一次吧，把所有关闭都取消了。
static int lcd_uart_open(void)
{
	uart_fd = uart_device_init(SCREEN_UART_DEVICE);
	CHECK(uart_fd > 0, -1, "Error s_uart_device_init!");

	return 0;
}



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
#if 1
	printf("s_uart_cmd_recv: h0 = %#x, h1 = %#x cmd = %#x, data = %#x, reserve0 = %#x,reserve1 = %#x,reserve2 = %#x,verify = %#x\n",
				cmd_reply_format.cmd_header0,
				cmd_reply_format.cmd_header1,
				cmd_reply_format.cmd,
				cmd_reply_format.data,
				cmd_reply_format.reserve0,
				cmd_reply_format.reserve1,
				cmd_reply_format.reserve2,
				cmd_reply_format.verify);
#endif

	CHECK(cmd_reply_format.cmd_header0 == UART_FRAME_HEADER_0 && cmd_reply_format.cmd_header1 == UART_FRAME_HEADER_1, -1,
			"Error UART frame head: %#x %#x", cmd_reply_format.cmd_header0, cmd_reply_format.cmd_header1);
	CHECK(cmd_reply_format.verify == UART_FRAME_VERIFY(cmd_reply_format.cmd, cmd_reply_format.data, 0, 0, 0), -1, "Error UART frame verity!");
	CHECK(cmd_reply_format.cmd == cmd + 1, -1, "Error UART frame reply cmd is %d!", cmd_reply_format.cmd);
	if(data) {
		*data = cmd_reply_format.data;
	}
	return 0;
}

void drvEnableLcdScreen(void) {
//	int uart_fd = uart_device_init(SCREEN_UART_DEVICE);
//	CHECK(uart_fd > 0, , "Error s_uart_device_init!");
	if(uart_fd <= 0)//2022-11-24  不想反复打开了，就打开一次吧，把所有关闭都取消了。
	{
		if(lcd_uart_open() < 0)
			return ;
	}
	
	if(s_uart_cmd_send(uart_fd, SCREEN_COMM_CMD_WAKE, NULL)) {
		ERR("Error s_uart_cmd_send!");
	//	uart_device_exit(uart_fd);//2022-11-24  不想反复打开了，就打开一次吧，把所有关闭都取消了。
		return;
	}
	if(s_uart_cmd_recv(uart_fd, SCREEN_COMM_CMD_WAKE, NULL)) {
		ERR("Error s_uart_cmd_recv!");
	//	uart_device_exit(uart_fd);//2022-11-24  不想反复打开了，就打开一次吧，把所有关闭都取消了。
		return;
	}
//	uart_device_exit(uart_fd);//2022-11-24  不想反复打开了，就打开一次吧，把所有关闭都取消了。
}

void drvDisableLcdScreen(void) {
	if(uart_fd <= 0)//2022-11-24  不想反复打开了，就打开一次吧，把所有关闭都取消了。
	{
		if(lcd_uart_open() < 0)
			return ;
	}
	if(s_uart_cmd_send(uart_fd, SCREEN_COMM_CMD_WAKE_OFF, NULL)) {
		ERR("Error s_uart_cmd_send!");
	//	uart_device_exit(uart_fd);//2022-11-24  不想反复打开了，就打开一次吧，把所有关闭都取消了。
		return;
	}
	if(s_uart_cmd_recv(uart_fd, SCREEN_COMM_CMD_WAKE_OFF, NULL)) {
		ERR("Error s_uart_cmd_recv!");
	//	uart_device_exit(uart_fd);//2022-11-24  不想反复打开了，就打开一次吧，把所有关闭都取消了。
		return;
	}
//	uart_device_exit(uart_fd);//2022-11-24  不想反复打开了，就打开一次吧，把所有关闭都取消了。
}


/*
	单片机的返回值（2022-12-15 协定）
	0x00:55所7寸屏
	0x01:55所5寸屏
	0x02:717所7寸屏
	0x03:717所5寸屏
	0x04:JC7寸屏
	0x05:JC 5寸屏
	0x06:JC 5寸屏2

	API需要返回值，需要转换一下上面的值：
	5:捷诚5寸触摸屏
	6:捷诚7寸触摸屏
	7:717 5 寸触摸屏
	8:717 7寸触摸年
	9:55所 5寸触摸屏
	10:55所 7寸触摸屏

*/
int drvGetLCDType(void) {
	unsigned char type = 0;
	if(uart_fd <= 0)//2022-11-24  不想反复打开了，就打开一次吧，把所有关闭都取消了。
	{
		if(lcd_uart_open() < 0)
			return -1;
	}

	if(s_uart_cmd_send(uart_fd, SCREEN_COMM_CMD_GET_MODEL, NULL)) {
		ERR("Error s_uart_cmd_send!");
	//	uart_device_exit(uart_fd);
		return -1;
	}
	if(s_uart_cmd_recv(uart_fd, SCREEN_COMM_CMD_GET_MODEL, &type)) {
		ERR("Error s_uart_cmd_recv!");
	//	uart_device_exit(uart_fd);
		return -1;
	}
//	uart_device_exit(uart_fd);
	printf("drvGetLCDType : type = %d\n",type);
	if(type < 7)    //2023-01-03
	{
		switch(type)
		{
			case 4:   //JC7寸屏
				type = 6;
			break;
			case 5:   //JC 5寸屏
			case 6:   //JC 新5寸屏
				type = 5;
			break;
			case 0: //55所7寸屏
				type = 10;
			break;
			case 1: //55所5寸屏
				type = 9;
			break;
			case 2: //717所7寸屏
				type = 8;
			break;
			case 3: //717所5寸屏
				type = 7;
			break;	
		}
		return type;
	}
	
	ERR("Error non-supported LCD model %#x", type);
	return -1;
}

void drvSetLcdBrt(int nBrtVal) {
	CHECK(nBrtVal >= SCREEN_BRIGHTNESS_MIN && nBrtVal <= SCREEN_BRIGHTNESS_MAX, , "Error screen brightness out of range!");

	if(uart_fd <= 0)//2022-11-24  不想反复打开了，就打开一次吧，把所有关闭都取消了。
	{
		if(lcd_uart_open() < 0)
			return ;
	}

	//2022-11-24 调试55所lcd亮度的时候，发现这个值应是3的倍数才行。所以进行这样的调整。
	nBrtVal = nBrtVal/3*3;

	if(s_uart_cmd_send(uart_fd, SCREEN_COMM_CMD_SET_BRIGHTNESS, (unsigned char *)&nBrtVal)) {
		ERR("Error s_uart_cmd_send!");
	//	uart_device_exit(uart_fd);
		return;
	}
	//2022-11-24 应答数据不再读取，加速反应时间
//	if(s_uart_cmd_recv(uart_fd, SCREEN_COMM_CMD_SET_BRIGHTNESS, NULL)) {
//		ERR("Error s_uart_cmd_recv!");
//	//	uart_device_exit(uart_fd);
//		return;
//	}
//	uart_device_exit(uart_fd);
}

int drvLcdReset(void) {
	return mcu_lcd_reset();
}


//获得单片机软件版本，2023-01-03
int drvGetLCDMcuVersion(void) {
	unsigned char type = 0;
	if(uart_fd <= 0)//2022-11-24  不想反复打开了，就打开一次吧，把所有关闭都取消了。
	{
		if(lcd_uart_open() < 0)
			return -1;
	}

	if(s_uart_cmd_send(uart_fd, SCREEN_COMM_CMD_GET_VER, NULL)) {
		ERR("Error：LCD Mcu Version： s_uart_cmd_send!");
	//	uart_device_exit(uart_fd);
		return -1;
	}
	if(s_uart_cmd_recv(uart_fd, SCREEN_COMM_CMD_GET_VER, &type)) {
		ERR("Error：LCD Mcu Version： s_uart_cmd_recv!");
	//	uart_device_exit(uart_fd);
		return -1;
	}

	return type;
}



