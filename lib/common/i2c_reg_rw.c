/*
 * i2c_reg_rw.c
 *
 *  Created on: Nov 30, 2021
 *      Author: zlf
 */

#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "debug.h"

int i2c_adapter_init(char *i2c_adapter_file, int i2c_device_addr) {
	CHECK(i2c_adapter_file, -1, "Error i2c_adapter_file is null!");
	CHECK(i2c_device_addr >= 0 && i2c_device_addr <= (0xff>>1), -1, "Error i2c_device_addr out of range!");

	int i2c_adapter_fd = 0;
	CHECK((i2c_adapter_fd = open(i2c_adapter_file, O_RDWR)) > 0, -1, "Error open with %d: %s", errno, strerror(errno));

	if(ioctl(i2c_adapter_fd, I2C_SLAVE_FORCE, i2c_device_addr)) {
		ERR("Error ioctl with %d: %s", errno, strerror(errno));
		close(i2c_adapter_fd);
		return -1;
	}
	return i2c_adapter_fd;
}

int i2c_adapter_exit(int i2c_adapter_fd) {
	CHECK(i2c_adapter_fd > 0, -1, "Error i2c_adapter_fd is %d", i2c_adapter_fd);
	CHECK(!close(i2c_adapter_fd), -1, "Error close with %d: %s", errno, strerror(errno));
	return 0;
}

int i2c_device_reg_read(int i2c_adapter_fd, unsigned char addr, unsigned char *value) {
	CHECK(i2c_adapter_fd > 0, -1, "Error i2c_adapter_fd is %d", i2c_adapter_fd);
	CHECK(value, -1, "Error value is null!");
	struct i2c_smbus_ioctl_data i2c_smbus_ioctl_data;
	union i2c_smbus_data i2c_smbus_data;
	bzero(&i2c_smbus_data, sizeof(union i2c_smbus_data));
	bzero(&i2c_smbus_ioctl_data, sizeof(struct i2c_smbus_ioctl_data));
	i2c_smbus_ioctl_data.read_write = I2C_SMBUS_READ;
	i2c_smbus_ioctl_data.command = addr;
	i2c_smbus_ioctl_data.size = I2C_SMBUS_BYTE_DATA;
	i2c_smbus_ioctl_data.data = &i2c_smbus_data;
	CHECK(!ioctl(i2c_adapter_fd, I2C_SMBUS, &i2c_smbus_ioctl_data), -1, "Error ioctl with %d: %s", errno, strerror(errno));
	*value = i2c_smbus_data.byte;
	return 0;
}

int i2c_device_reg_write(int i2c_adapter_fd, unsigned char addr, unsigned char value) {
	CHECK(i2c_adapter_fd > 0, -1, "Error i2c_adapter_fd is %d", i2c_adapter_fd);
	struct i2c_smbus_ioctl_data i2c_smbus_ioctl_data;
	union i2c_smbus_data i2c_smbus_data;
	unsigned char value_temp = 0;
	bzero(&i2c_smbus_data, sizeof(union i2c_smbus_data));
	bzero(&i2c_smbus_ioctl_data, sizeof(struct i2c_smbus_ioctl_data));
	i2c_smbus_data.byte = value;
	i2c_smbus_ioctl_data.read_write = I2C_SMBUS_WRITE;
	i2c_smbus_ioctl_data.command = addr;
	i2c_smbus_ioctl_data.size = I2C_SMBUS_BYTE_DATA;
	i2c_smbus_ioctl_data.data = &i2c_smbus_data;
	CHECK(!ioctl(i2c_adapter_fd, I2C_SMBUS, &i2c_smbus_ioctl_data), -1, "Error ioctl with %d: %s", errno, strerror(errno));
	CHECK(!i2c_device_reg_read(i2c_adapter_fd, addr, &value_temp), -1, "Error i2c_device_reg_read!");
	CHECK(value == value_temp, -1, "Error write byte %#x is unequal read byte %#x", value, value_temp);
	return 0;
}
