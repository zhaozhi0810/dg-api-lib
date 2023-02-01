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

static int s_uart_device_param_init(int uart_fd) {
	CHECK(uart_fd > 0, -1, "Error uart_fd is %d", uart_fd);

	struct termios termios;

	CHECK(!tcgetattr(uart_fd, &termios), -1, "Error tcgetattr with %d: %s", errno, strerror(errno));
	CHECK(!cfsetspeed(&termios, B115200), -1, "Error cfsetspeed with %d: %s", errno, strerror(errno));
	termios.c_lflag &= (~(ICANON|ECHO|ECHOE|ISIG)); /*lsr add 20220406*/
	termios.c_cflag &= (~CSIZE);
	termios.c_cflag |= CS8;
	termios.c_cflag &= (~PARENB);
	termios.c_cflag &= (~CSTOPB);
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

	int ret = 0;
	fd_set readfds;
	struct timeval timeout = {1, 0};

	FD_ZERO(&readfds);
	FD_SET(uart_fd, &readfds);
	CHECK((ret = select(uart_fd + 1, &readfds, NULL, NULL, &timeout)) >= 0, -1, "Error select with %d: %s", errno, strerror(errno));
	CHECK(ret, -1, "Error select timeout!");
	CHECK(FD_ISSET(uart_fd, &readfds), -1, "Error select not uart_fd!");
	CHECK(read(uart_fd, buf, buf_lenth) == buf_lenth, -1, "Error read with %d: %s", errno, strerror(errno));
	return 0;
}
