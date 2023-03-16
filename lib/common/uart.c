/*
 * uart.c
 *
 *  Created on: Nov 26, 2021
 *      Author: zlf
 */

#include <termios.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>

#include "debug.h"
#include "mcu.h"



static int s_uart_device_param_init(int uart_fd) {
	CHECK(uart_fd > 0, -1, "Error uart_fd is %d", uart_fd);

	struct termios termios;

	CHECK(!tcgetattr(uart_fd, &termios), -1, "Error tcgetattr with %d: %s", errno, strerror(errno));
	cfmakeraw(&termios);
	CHECK(!cfsetspeed(&termios, B115200), -1, "Error cfsetspeed with %d: %s", errno, strerror(errno));
	//termios.c_lflag &= (~(ICANON|ECHO|ECHOE|ISIG)); /*lsr add 20220406*/	
	termios.c_cflag &= (~CSIZE);  //控制模式，屏蔽字符大小位
	termios.c_cflag |= CS8;       //data bits 8位数据
	termios.c_cflag &= (~PARENB);   //no parity check
	termios.c_cflag &= (~CSTOPB);   //1 stop bits
	tcflush(uart_fd, TCIOFLUSH);				//溢出的数据可以接收，但不读
	CHECK(!tcsetattr(uart_fd, TCSANOW, &termios), -1, "Error tcsetattr with %d: %s", errno, strerror(errno));
	return 0;
}

int uart_device_init(char *file) {
	CHECK(file, -1, "Error file is null!");
	int uart_fd = 0;
	CHECK((uart_fd = open(file, O_RDWR)) > 0, -1, "Error open with %d: %s", errno, strerror(errno));
	if(s_uart_device_param_init(uart_fd)) {
		ERR("Error s_uart_device_param_init!");
		CHECK(!close(uart_fd), -1, "Error close with %d: %s", errno, strerror(errno));
		return -1;
	}
	return uart_fd;
}

int uart_device_exit(int uart_fd) {
	CHECK(uart_fd > 0, -1, "Error s_uart_fd is %d", uart_fd);
	CHECK(!close(uart_fd), -1, "Error close with %d: %s", errno, strerror(errno));
	return 0;
}

int uart_send(int uart_fd, void *buf, long unsigned int buf_lenth) {
	CHECK(uart_fd > 0, -1, "Error uart has not been init!");
	CHECK(buf, -1, "Error buf is null!");
	CHECK(buf_lenth, -1, "Error buf_lenth is %lu", buf_lenth);

	CHECK(write(uart_fd, buf, buf_lenth) == buf_lenth, -1, "Error write with %d: %s", errno, strerror(errno));
	return 0;
}

int uart_recv(int uart_fd, void *buf, long unsigned int buf_lenth) {
	CHECK(uart_fd > 0, -1, "Error uart has not been init!");
	CHECK(buf, -1, "Error buf is null!");
	CHECK(buf_lenth, -1, "Error buf_lenth is %lu", buf_lenth);

	int ret = 0,i;
	fd_set readfds;
	struct timeval timeout = {1, 0};

	FD_ZERO(&readfds);
	FD_SET(uart_fd, &readfds);
	CHECK((ret = select(uart_fd + 1, &readfds, NULL, NULL, &timeout)) >= 0, -1, "Error select with %d: %s", errno, strerror(errno));
	CHECK(ret, -1, "Error select timeout!");
	CHECK(FD_ISSET(uart_fd, &readfds), -1, "Error select not uart_fd!");
	//CHECK(read(uart_fd, buf, buf_lenth) == buf_lenth, -1, "Error read with %d: %s", errno, strerror(errno));
	ret = read(uart_fd, buf, buf_lenth);
#if 0  //2023-03-01  for debug
	printf("uart_recv ---------------2023+++++\n");
	for(i=0;i<ret;i++)
		printf("buf[%d] = %#x\n",i,((unsigned char*)buf)[i]);
#endif	
	if(ret != buf_lenth)
	{
		printf("uart_recv len = %d < buf_lenth = %ld\n",ret,buf_lenth);
		for(i=0;i<ret;i++)
			printf("buf[%d] = %#x\n",i,((unsigned char*)buf)[i]);
#if 0  //2023-03-01  for debug
		if(((unsigned char*)buf)[ret-1] == MCU_UART_FRAME_TAIL)
		{
			((unsigned char*)buf)[buf_lenth - 1] = MCU_UART_FRAME_TAIL;
			for(i=ret-1;i<buf_lenth - 1;i++)
				((unsigned char*)buf)[i] = 0;
			printf("uart_recv ---------------\n");
			for(i=0;i<buf_lenth;i++)
				printf("buf[%d] = %#x\n",i,((unsigned char*)buf)[i]);
			return 0;   //假装成功
		}
#endif
		return -1;
	}
	return 0;
}
