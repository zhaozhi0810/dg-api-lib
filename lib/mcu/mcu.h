/*
 * mcu.h
 *
 *  Created on: Nov 29, 2021
 *      Author: zlf
 */

#ifndef LIB_COMMON_MCU_MCU_H_
#define LIB_COMMON_MCU_MCU_H_

#define MCU_UART_DEVICE			"/dev/ttyS0"

#define MCU_UART_FRAME_HEADER	(0xa5)
#define MCU_UART_FRAME_TAIL		(0x5a)

#define MCU_COMM_CMD_GET_VOLTAGE			(0x80)
#define MCU_COMM_CMD_GET_VOLTAGE_REPLY		(MCU_COMM_CMD_GET_VOLTAGE + 1)

#define MCU_COMM_CMD_GET_ELECTRICITY		(0x82)
#define MCU_COMM_CMD_GET_ELECTRICITY_REPLY	(MCU_COMM_CMD_GET_ELECTRICITY + 1)

#define MCU_COMM_CMD_LCD_RESET				(0x84)
#define MCU_COMM_CMD_LCD_RESET_REPLY		(MCU_COMM_CMD_LCD_RESET + 1)	/* Unused */

#define MCU_COMM_CMD_IF_BOARD_RESET			(0x86)
#define MCU_COMM_CMD_IF_BOARD_RESET_REPLY	(MCU_COMM_CMD_IF_BOARD_RESET + 1)	/* Unused */

#define MCU_COMM_CMD_CORE_BOARD_RESET		(0x88)
#define MCU_COMM_CMD_CORE_BOARD_RESET_REPLY	(MCU_COMM_CMD_CORE_BOARD_RESET + 1)	/* Unused */

#define MCU_COMM_CMD_WATCHDOG_ENABLE		(0x8a)
#define MCU_COMM_CMD_WATCHDOG_ENABLE_REPLY	(MCU_COMM_CMD_WATCHDOG_ENABLE + 1)	/* Unused */

#define MCU_COMM_CMD_IF_BOARD_TEMP			(0x8c)
#define MCU_COMM_CMD_IF_BOARD_TEMP_REPLY	(MCU_COMM_CMD_IF_BOARD_TEMP + 1)

#define UART_FRAME_VERIFY(arg0, arg1, arg2, arg3, arg4)	\
	((arg0 + arg1 + arg2 + arg3 + arg4) & 0xFF)

typedef struct {
	unsigned char header;
	unsigned char cmd;
	unsigned char data0;
	unsigned char data1;
	unsigned char data2;
	unsigned char data3;
	unsigned char verify;
	unsigned char tail;
} UART_FRAME_FORMAT_S;

enum {
	MCU_WATCHDOG_DISABLE,
	MCU_WATCHDOG_ENABLE,
};

typedef enum {
	RESET_STATUS_NO_NEED_RESET,
	RESET_STATUS_RESET,
} RESET_STATUS_E;

extern float mcu_get_voltage(void);
extern float mcu_get_electricity(void);

extern int mcu_lcd_reset(void);
extern int mcu_if_board_reset(void);
extern int mcu_core_board_reset(void);

extern int mcu_watchdog_enable(void);
extern int mcu_watchdog_disable(void);

extern float mcu_get_temperature(void);

#endif /* LIB_COMMON_MCU_MCU_H_ */
