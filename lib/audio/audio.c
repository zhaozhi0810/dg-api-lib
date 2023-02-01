/*
 * audio.c
 *
 *  Created on: Nov 22, 2021
 *      Author: zlf
 */

#include <stdlib.h>

#include "debug.h"
#include "gpio.h"
#include "codec.h"
#include "misc.h"
#include "i2c_reg_rw.h"

#define SPEAKER_ENABLE_PIN	(32*4 + 8*2 + 7)	// GPIO4_C7, AD7, 151
#define WARNING_ENABLE_PIN	(32*4 + 8*3 + 6)	// GPIO4_D6, AG4, 158

#define I2C_ADAPTER_DEVICE	"/dev/i2c-4"
#define I2C_DEVICE_ADDR		(0x10)

static int s_write_reg(unsigned char addr, unsigned char val) {
	int i2c_adapter_fd = 0;
	CHECK((i2c_adapter_fd = i2c_adapter_init(I2C_ADAPTER_DEVICE, I2C_DEVICE_ADDR)) > 0, -1, "Error i2c_adapter_init!");
	if(i2c_device_reg_write(i2c_adapter_fd, addr, val)) {
		ERR("Error i2c_device_reg_write!");
		i2c_adapter_exit(i2c_adapter_fd);
		return -1;
	}
	i2c_adapter_exit(i2c_adapter_fd);
	return 0;
}

static int s_read_reg(unsigned char addr, unsigned char *val) {
	CHECK(val, 0, "Error val is null!");
	int i2c_adapter_fd = 0;
	CHECK((i2c_adapter_fd = i2c_adapter_init(I2C_ADAPTER_DEVICE, I2C_DEVICE_ADDR)) > 0, -1, "Error i2c_adapter_init!");
	if(i2c_device_reg_read(i2c_adapter_fd, addr, val)) {
		ERR("Error i2c_device_reg_write!");
		i2c_adapter_exit(i2c_adapter_fd);
		return -1;
	}
	i2c_adapter_exit(i2c_adapter_fd);
	return 0;
}

void drvEnableTune(void) {
	ERR("Error non-supported!");
}

void drvDisableTune(void) {
	ERR("Error non-supported!");
}

void drvAdjustTune(void) {
	ERR("Error non-supported!");
}

void drvSetTuneUp(void) {
	ERR("Error non-supported!");
}

void drvSetTuneDown(void) {
	ERR("Error non-supported!");
}

void drvSelectHandFreeMic(void) {
	unsigned char val = 0;
	CHECK(!s_read_reg(ES8388_ADCCONTROL2, &val), , "Error s_read_reg!");
	val &= 0x0f;
	CHECK(!s_write_reg(ES8388_ADCCONTROL2, val), , "Error s_write_reg!");
}

void drvSelectHandMic(void) {
	unsigned char val = 0;
	CHECK(!s_read_reg(ES8388_ADCCONTROL2, &val), , "Error s_read_reg!");
	val &= 0x0f;
	val |= 0x50;
	CHECK(!s_write_reg(ES8388_ADCCONTROL2, val), , "Error s_write_reg!");
}

void drvSelectEarphMic(void) {
	drvSelectHandMic();
}

void drvDisableSpeaker(void) {
#if 0
	CHECK(!gpio_init(SPEAKER_ENABLE_PIN, true, GPIO_INPUT_EDGE_NONE), , "Error gpio_init!");
	if(gpio_set_value(SPEAKER_ENABLE_PIN, 0)) {
		ERR("Error gpio_set_value!");
	}
	gpio_exit(SPEAKER_ENABLE_PIN);
#else
	unsigned char val = 0;
	CHECK(!s_read_reg(ES8388_DACPOWER, &val), , "Error s_read_reg!");
	val &= ~((unsigned char)0x1 << 3);
	CHECK(!s_write_reg(ES8388_DACPOWER, val), , "Error s_write_reg!");
#endif
}

void drvEnableSpeaker(void) {
#if 0
	CHECK(!gpio_init(SPEAKER_ENABLE_PIN, true, GPIO_INPUT_EDGE_NONE), , "Error gpio_init!");
	if(gpio_set_value(SPEAKER_ENABLE_PIN, 1)) {
		ERR("Error gpio_set_value!");
	}
	gpio_exit(SPEAKER_ENABLE_PIN);
#else
	unsigned char val = 0;
	CHECK(!s_read_reg(ES8388_DACPOWER, &val), , "Error s_read_reg!");
	val |= (0x1 << 3);
	CHECK(!s_write_reg(ES8388_DACPOWER, val), , "Error s_write_reg!");
#endif
}

void drvDisableWarning(void) {
	CHECK(!gpio_init(WARNING_ENABLE_PIN, true, GPIO_INPUT_EDGE_NONE), , "Error gpio_init!");
	if(gpio_set_value(WARNING_ENABLE_PIN, 1)) {
		ERR("Error gpio_set_value!");
	}
	gpio_exit(WARNING_ENABLE_PIN);
}

void drvEnableWarning(void) {
	CHECK(!gpio_init(WARNING_ENABLE_PIN, true, GPIO_INPUT_EDGE_NONE), , "Error gpio_init!");
	if(gpio_set_value(WARNING_ENABLE_PIN, 0)) {
		ERR("Error gpio_set_value!");
	}
	gpio_exit(WARNING_ENABLE_PIN);
}

int drvGetMicStatus(void) {
	return get_headset_insert_status()? 1:0;
}

int drvGetHMicStatus(void) {
	return get_handle_insert_status()? 1:0;
}

void drvAddSpeakVolume(int value) {
	CHECK(value > 0 && value <= 100, , "Error value out of range!");
	unsigned char val = 0;
	unsigned char val_max = 0x21;
	unsigned char val_step = val_max*value/100;

	CHECK(!s_read_reg(ES8388_DACCONTROL26, &val), , "Error s_read_reg!");
	val += val_step;
	val = (val > val_max)? val_max:val;
	CHECK(!s_write_reg(ES8388_DACCONTROL26, val), , "Error s_write_reg!");
}

void drvSubSpeakVolume(int value) {
	CHECK(value > 0 && value <= 100, , "Error value out of range!");
	unsigned char val = 0;
	unsigned char val_max = 0x21;
	unsigned char val_step = val_max*value/100;

	CHECK(!s_read_reg(ES8388_DACCONTROL26, &val), , "Error s_read_reg!");
	val -= val_step;
	val = (val < 0)? 0:val;
	CHECK(!s_write_reg(ES8388_DACCONTROL26, val), , "Error s_write_reg!");
}
/*lsr add 20220505*/
void drvSetSpeakVolume(int value){
		CHECK(value > 0 && value <= 100, , "Error value out of range!");
	unsigned char val = 0;
	unsigned char val_max = 0x21;
	unsigned char val_step = val_max*value/100;

	val = val_step;
	val = (val < 0)? 0:val;
	CHECK(!s_write_reg(ES8388_DACCONTROL26, val), , "Error s_write_reg!");

}
void drvAddHandVolume(int value) {
	CHECK(value > 0 && value <= 100, , "Error value out of range!");
	unsigned char val = 0;
	unsigned char val_max = 0x21;
	unsigned char val_step = val_max*value/100;

	CHECK(!s_read_reg(ES8388_DACCONTROL25, &val), , "Error s_read_reg!");
	val += val_step;
	val = (val > val_max)? val_max:val;
	CHECK(!s_write_reg(ES8388_DACCONTROL25, val), , "Error s_write_reg!");
}

void drvSubHandVolume(int value) {
	CHECK(value > 0 && value <= 100, , "Error value out of range!");
	unsigned char val = 0;
	unsigned char val_max = 0x21;
	unsigned char val_step = val_max*value/100;

	CHECK(!s_read_reg(ES8388_DACCONTROL25, &val), , "Error s_read_reg!");
	val -= val_step;
	val = (val < 0)? 0:val;
	CHECK(!s_write_reg(ES8388_DACCONTROL25, val), , "Error s_write_reg!");
}

/*lsr add 20220505*/
void drvSetHandVolume(int value) {
	CHECK(value > 0 && value <= 100, , "Error value out of range!");
	unsigned char val = 0;
	unsigned char val_max = 0x21;
	unsigned char val_step = val_max*value/100;

	val = val_step;
	val = (val < 0)? 0:val;
	CHECK(!s_write_reg(ES8388_DACCONTROL25, val), , "Error s_write_reg!");
}


void drvAddEarphVolume(int value) {
	CHECK(value > 0 && value <= 100, , "Error value out of range!");
	unsigned char val = 0;
	unsigned char val_max = 0x21;
	unsigned char val_step = val_max*value/100;

	CHECK(!s_read_reg(ES8388_DACCONTROL27, &val), , "Error s_read_reg!");
	val += val_step;
	val = (val > val_max)? val_max:val;
	CHECK(!s_write_reg(ES8388_DACCONTROL27, val), , "Error s_write_reg!");
}

void drvSubEarphVolume(int value) {
	CHECK(value > 0 && value <= 100, , "Error value out of range!");
	unsigned char val = 0;
	unsigned char val_max = 0x21;
	unsigned char val_step = val_max*value/100;

	CHECK(!s_read_reg(ES8388_DACCONTROL27, &val), , "Error s_read_reg!");
	val -= val_step;
	val = (val < 0)? 0:val;
	CHECK(!s_write_reg(ES8388_DACCONTROL27, val), , "Error s_write_reg!");
}

/*lsr add 20220505*/
void drvSetEarphVolume(int value) {
	CHECK(value > 0 && value <= 100, , "Error value out of range!");
	unsigned char val = 0;
	unsigned char val_max = 0x21;
	unsigned char val_step = val_max*value/100;

	val = val_step;
	val = (val < 0)? 0:val;
	CHECK(!s_write_reg(ES8388_DACCONTROL27, val), , "Error s_write_reg!");
}

void drvEnableHandout(void) {
	unsigned char val = 0;
	CHECK(!s_read_reg(ES8388_DACPOWER, &val), , "Error s_read_reg!");
	val |= (0x1 << 4);
	CHECK(!s_write_reg(ES8388_DACPOWER, val), , "Error s_write_reg!");
}

void drvDisableHandout(void) {
	unsigned char val = 0;
	CHECK(!s_read_reg(ES8388_DACPOWER, &val), , "Error s_read_reg!");
	val &= ~((unsigned char)0x1 << 4);
	CHECK(!s_write_reg(ES8388_DACPOWER, val), , "Error s_write_reg!");
}

void drvEnableEarphout(void) {
	unsigned char val = 0;
	CHECK(!s_read_reg(ES8388_DACPOWER, &val), , "Error s_read_reg!");
	val |= (0x1 << 2);
	CHECK(!s_write_reg(ES8388_DACPOWER, val), , "Error s_write_reg!");
}

void drvDisableEarphout(void) {
	unsigned char val = 0;
	CHECK(!s_read_reg(ES8388_DACPOWER, &val), , "Error s_read_reg!");
	val &= ~((unsigned char)0x1 << 2);
	CHECK(!s_write_reg(ES8388_DACPOWER, val), , "Error s_write_reg!");
}