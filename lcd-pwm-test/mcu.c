/*
 * mcu.c
 *
 *  Created on: Nov 29, 2021
 *      Author: zlf
 */
#include <errno.h>
#include <string.h>

#include "debug.h"
#include "uart.h"
#include "mcu.h"

#define MCU_UART_FRAME_DATA_LENTH	(4)

static int s_mcu_uart_send(int mcu_uart_fd, int mcu_uart_cmd, char *send_buf) {
	CHECK(mcu_uart_fd > 0, -1, "Error mcu_uart_fd is %d", mcu_uart_fd);
	UART_FRAME_FORMAT_S cmd_send_format;

	bzero(&cmd_send_format, sizeof(UART_FRAME_FORMAT_S));
	cmd_send_format.header = MCU_UART_FRAME_HEADER;
	cmd_send_format.cmd = mcu_uart_cmd;
	if(send_buf) {
		cmd_send_format.data0 = send_buf[0];
		cmd_send_format.data1 = send_buf[1];
		cmd_send_format.data2 = send_buf[2];
		cmd_send_format.data3 = send_buf[3];
	}
	cmd_send_format.verify = UART_FRAME_VERIFY(
			cmd_send_format.cmd,
			cmd_send_format.data0,
			cmd_send_format.data1,
			cmd_send_format.data2,
			cmd_send_format.data3);
	cmd_send_format.tail = MCU_UART_FRAME_TAIL;
	CHECK(!uart_send(mcu_uart_fd, &cmd_send_format, sizeof(UART_FRAME_FORMAT_S)), -1, "Error uart_send!");
	return 0;
}

static int s_mcu_uart_recv(int mcu_uart_fd, int mcu_uart_cmd, void *recv_buf) {
	CHECK(mcu_uart_fd > 0, -1, "Error mcu_uart_fd is %d", mcu_uart_fd);
	CHECK(recv_buf, -1, "Error recv_buf is null!");

	UART_FRAME_FORMAT_S cmd_reply_format;
	bzero(&cmd_reply_format, sizeof(UART_FRAME_FORMAT_S));
	CHECK(!uart_recv(mcu_uart_fd, &cmd_reply_format, sizeof(UART_FRAME_FORMAT_S)), -1, "Error uart_recv!");
	CHECK(cmd_reply_format.header == MCU_UART_FRAME_HEADER, -1, "Error UART frame head: %#x", cmd_reply_format.header);
	CHECK(cmd_reply_format.tail == MCU_UART_FRAME_TAIL, -1, "Error UART frame tail: %#x", cmd_reply_format.tail);
	CHECK(cmd_reply_format.cmd == mcu_uart_cmd, -1, "Error UART frame reply cmd is %#x!", cmd_reply_format.cmd);
	memcpy(recv_buf, &cmd_reply_format.data0, MCU_UART_FRAME_DATA_LENTH);
	return 0;
}

float mcu_get_voltage(void) {
	int uart_fd = 0;
	float val = 0;

	CHECK((uart_fd = uart_device_init(MCU_UART_DEVICE)) > 0, 0, "Error uart_device_init!");
	if(s_mcu_uart_send(uart_fd, MCU_COMM_CMD_GET_VOLTAGE, NULL)) {
		ERR("Error s_mcu_uart_send!");
		uart_device_exit(uart_fd);
		return 0;
	}
	if(s_mcu_uart_recv(uart_fd, MCU_COMM_CMD_GET_VOLTAGE_REPLY, &val)) {
		ERR("Error s_mcu_uart_recv!");
		uart_device_exit(uart_fd);
		return 0;
	}
	uart_device_exit(uart_fd);
	return val;
}

float mcu_get_electricity(void) {
	int uart_fd = 0;
	float val = 0;

	CHECK((uart_fd = uart_device_init(MCU_UART_DEVICE)) > 0, 0, "Error uart_device_init!");
	if(s_mcu_uart_send(uart_fd, MCU_COMM_CMD_GET_ELECTRICITY, NULL)) {
		ERR("Error s_mcu_uart_send!");
		uart_device_exit(uart_fd);
		return 0;
	}
	if(s_mcu_uart_recv(uart_fd, MCU_COMM_CMD_GET_ELECTRICITY_REPLY, &val)) {
		ERR("Error s_mcu_uart_recv!");
		uart_device_exit(uart_fd);
		return 0;
	}
	uart_device_exit(uart_fd);
	return val;
}

int mcu_lcd_reset(void) {
	int uart_fd = 0;
	char send_buf[MCU_UART_FRAME_DATA_LENTH] = {0};
	CHECK((uart_fd = uart_device_init(MCU_UART_DEVICE)) > 0, -1, "Error uart_device_init!");
	send_buf[0] = RESET_STATUS_RESET;
	if(s_mcu_uart_send(uart_fd, MCU_COMM_CMD_LCD_RESET, send_buf)) {
		ERR("Error s_mcu_uart_send!");
		uart_device_exit(uart_fd);
		return -1;
	}
	uart_device_exit(uart_fd);
	return 0;
}

int mcu_if_board_reset(void) {
	int uart_fd = 0;
	char send_buf[MCU_UART_FRAME_DATA_LENTH] = {0};
	CHECK((uart_fd = uart_device_init(MCU_UART_DEVICE)) > 0, -1, "Error uart_device_init!");
	send_buf[0] = RESET_STATUS_RESET;
	if(s_mcu_uart_send(uart_fd, MCU_COMM_CMD_IF_BOARD_RESET, send_buf)) {
		ERR("Error s_mcu_uart_send!");
		uart_device_exit(uart_fd);
		return -1;
	}
	uart_device_exit(uart_fd);
	return 0;
}

int mcu_core_board_reset(void) {
	int uart_fd = 0;
	char send_buf[MCU_UART_FRAME_DATA_LENTH] = {0};
	CHECK((uart_fd = uart_device_init(MCU_UART_DEVICE)) > 0, -1, "Error uart_device_init!");
	send_buf[0] = RESET_STATUS_RESET;
	if(s_mcu_uart_send(uart_fd, MCU_COMM_CMD_CORE_BOARD_RESET, send_buf)) {
		ERR("Error s_mcu_uart_send!");
		uart_device_exit(uart_fd);
		return -1;
	}
	uart_device_exit(uart_fd);
	return 0;
}

int mcu_watchdog_enable(void) {
	int uart_fd = 0;
	char send_buf[MCU_UART_FRAME_DATA_LENTH] = {0};
	CHECK((uart_fd = uart_device_init(MCU_UART_DEVICE)) > 0, -1, "Error uart_device_init!");
	send_buf[0] = MCU_WATCHDOG_ENABLE;
	if(s_mcu_uart_send(uart_fd, MCU_COMM_CMD_WATCHDOG_ENABLE, send_buf)) {
		ERR("Error s_mcu_uart_send!");
		uart_device_exit(uart_fd);
		return -1;
	}
	uart_device_exit(uart_fd);
	return 0;
}

int mcu_watchdog_disable(void) {
	int uart_fd = 0;
	char send_buf[MCU_UART_FRAME_DATA_LENTH] = {0};
	CHECK((uart_fd = uart_device_init(MCU_UART_DEVICE)) > 0, -1, "Error uart_device_init!");
	send_buf[0] = MCU_WATCHDOG_DISABLE;
	if(s_mcu_uart_send(uart_fd, MCU_COMM_CMD_WATCHDOG_ENABLE, send_buf)) {
		ERR("Error s_mcu_uart_send!");
		uart_device_exit(uart_fd);
		return -1;
	}
	uart_device_exit(uart_fd);
	return 0;
}

float mcu_get_temperature(void) {
	int uart_fd = 0;
	float val = 0;

	CHECK((uart_fd = uart_device_init(MCU_UART_DEVICE)) > 0, 0, "Error uart_device_init!");
	if(s_mcu_uart_send(uart_fd, MCU_COMM_CMD_IF_BOARD_TEMP, NULL)) {
		ERR("Error s_mcu_uart_send!");
		uart_device_exit(uart_fd);
		return 0;
	}
	if(s_mcu_uart_recv(uart_fd, MCU_COMM_CMD_IF_BOARD_TEMP_REPLY, &val)) {
		ERR("Error s_mcu_uart_recv!");
		uart_device_exit(uart_fd);
		return 0;
	}
	uart_device_exit(uart_fd);
	return val;
}
