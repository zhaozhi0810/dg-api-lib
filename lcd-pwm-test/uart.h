/*
 * uart.h
 *
 *  Created on: Nov 26, 2021
 *      Author: zlf
 */

#ifndef LIB_COMMON_UART_H_
#define LIB_COMMON_UART_H_

extern int uart_device_init(char *file);
extern int uart_device_exit(int uart_fd);

extern int uart_send(int uart_fd, void *buf, long unsigned int buf_lenth);
extern int uart_recv(int uart_fd, void *buf, long unsigned int buf_lenth);

#endif /* LIB_COMMON_UART_H_ */
