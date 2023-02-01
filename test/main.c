/*
 * main.c
 *
 *  Created on: Nov 19, 2021
 *      Author: zlf
 */

#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>

#include "drv722.h"
#include "debug.h"

#define SCREEN_BRIGHTNESS_MIN		(0)
#define SCREEN_BRIGHTNESS_MAX		(0xff)
#define PANEL_KEY_BRIGHTNESS_MIN	(0)
#define PANEL_KEY_BRIGHTNESS_MAX	(100)

#define KEY_VALUE_MIN				(0x1)
#define KEY_VALUE_MAX				(0x2B)    //原来是0x27

#define WATCHDOG_TIMEOUT			(20)	// s

enum {
	TEST_ITEM_WATCHDOG_ENABLE,
	TEST_ITEM_WATCHDOG_DISABLE,
	TEST_ITEM_SCREEN_ENABLE,
	TEST_ITEM_SPEAKER_ENABLE,
	TEST_ITEM_SPEAKER_DISABLE,
	TEST_ITEM_SPEAKER_VOLUME_ADD,
	TEST_ITEM_SPEAKER_VOLUME_SUB,
	TEST_ITEM_WARNING_ENABLE,
	TEST_ITEM_WARNING_DISABLE,
	TEST_ITEM_HAND_ENABLE,
	TEST_ITEM_HAND_DISABLE,
	TEST_ITEM_HAND_VOLUME_ADD,
	TEST_ITEM_HAND_VOLUME_SUB,
	TEST_ITEM_HEADSET_ENABLE,
	TEST_ITEM_HEADSET_DISABLE,
	TEST_ITEM_HEADSET_VOLUME_ADD,
	TEST_ITEM_HEADSET_VOLUME_SUB,
	TEST_ITEM_USB0_ENABLE,
	TEST_ITEM_USB0_DISABLE,
	TEST_ITEM_USB1_ENABLE,
	TEST_ITEM_USB1_DISABLE,
	TEST_ITEM_GET_CPU_TEMPERATURE,
	TEST_ITEM_GET_INTERFACE_BOARD_TEMPERATURE,
	TEST_ITEM_SET_KEYBOARD_LED_BRIGHTNESS,
	TEST_ITEM_SET_KEYBOARD_LED_ON_OFF,
	TEST_ITEM_SET_ALLKEYBOARD_LED_ON_OFF,
	TEST_ITEM_SET_SCREEN_BRIGHTNESS,
	TEST_ITEM_RESET_KEYBOARD,
	TEST_ITEM_RESET_SCREEN,
	TEST_ITEM_RESET_CORE_BOARD,
	TEST_ITEM_GET_VOLGATE,
	TEST_ITEM_GET_ELECTRICITY,
	TEST_ITEM_GET_KEYBOARD_MODEL,
	TEST_ITEM_GET_LCD_MODEL,
	TEST_ITEM_AUDIO_SELECT_PANEL_MIC,
	TEST_ITEM_AUDIO_SELECT_HAND_MIC,
	TEST_ITEM_AUDIO_SELECT_HEADSET_MIC,
	TEST_ITEM_TOUCHSCREEN_ENABLE,
	TEST_ITEM_TOUCHSCREEN_DISABLE,
	TEST_ITEM_GET_RTC,
	TEST_ITEM_SET_RTC,
	TEST_ITEM_GET_HEADSET_INSERT_STATUS,
	TEST_ITEM_GET_HANDLE_INSERT_STATUS,
	TEST_ITEM_GET_LCD_MCUVERSION_STATUS, //获得LCD 屏单片机版本信息
	TEST_ITEM_GET_BORAD_MCUVERSION_STATUS,  //获得导光面板按键版本信息
	TEST_ITEM_GET_YT8521SH_STATUS
};

extern int drvCoreBoardExit(void);

static bool s_main_thread_exit = false;
static bool s_watchdog_feed_thread_exit = false;

static void s_show_usage(void) {
	printf("Usage:\n");
	printf("\t%2d - Watchdog enable\n", TEST_ITEM_WATCHDOG_ENABLE);
	printf("\t%2d - Watchdog disable\n", TEST_ITEM_WATCHDOG_DISABLE);
	printf("\t%2d - Screen wake on/off\n", TEST_ITEM_SCREEN_ENABLE);
	printf("\t%2d - Speaker enable\n", TEST_ITEM_SPEAKER_ENABLE);
	printf("\t%2d - Speaker disable\n", TEST_ITEM_SPEAKER_DISABLE);
	printf("\t%2d - Speaker volume increase 10%%\n", TEST_ITEM_SPEAKER_VOLUME_ADD);
	printf("\t%2d - Speaker volume decrease 10%%\n", TEST_ITEM_SPEAKER_VOLUME_SUB);
	printf("\t%2d - Warning enable\n", TEST_ITEM_WARNING_ENABLE);
	printf("\t%2d - Warning disable\n", TEST_ITEM_WARNING_DISABLE);
	printf("\t%2d - Hand enable\n", TEST_ITEM_HAND_ENABLE);
	printf("\t%2d - Hand disable\n", TEST_ITEM_HAND_DISABLE);
	printf("\t%2d - Hand volume increase 10%%\n", TEST_ITEM_HAND_VOLUME_ADD);
	printf("\t%2d - Hand volume decrease 10%%\n", TEST_ITEM_HAND_VOLUME_SUB);
	printf("\t%2d - Headset enable\n", TEST_ITEM_HEADSET_ENABLE);
	printf("\t%2d - Headset disable\n", TEST_ITEM_HEADSET_DISABLE);
	printf("\t%2d - Headset volume increase 10%%\n", TEST_ITEM_HEADSET_VOLUME_ADD);
	printf("\t%2d - Headset volume decrease 10%%\n", TEST_ITEM_HEADSET_VOLUME_SUB);
	printf("\t%2d - USB0 enable\n", TEST_ITEM_USB0_ENABLE);
	printf("\t%2d - USB0 disable\n", TEST_ITEM_USB0_DISABLE);
	printf("\t%2d - USB1 enable\n", TEST_ITEM_USB1_ENABLE);
	printf("\t%2d - USB1 disable\n", TEST_ITEM_USB1_DISABLE);
	printf("\t%2d - Get CPU temperature\n", TEST_ITEM_GET_CPU_TEMPERATURE);
	printf("\t%2d - Get interface board temperature\n", TEST_ITEM_GET_INTERFACE_BOARD_TEMPERATURE);
	printf("\t%2d - Set keyboard led brightness\n", TEST_ITEM_SET_KEYBOARD_LED_BRIGHTNESS);
	printf("\t%2d - Set keyboard led on/off status\n", TEST_ITEM_SET_KEYBOARD_LED_ON_OFF);
	printf("\t%2d - Set ALL keyboard led on/off status\n", TEST_ITEM_SET_ALLKEYBOARD_LED_ON_OFF);
	printf("\t%2d - Set screen brightness\n", TEST_ITEM_SET_SCREEN_BRIGHTNESS);
	printf("\t%2d - Reset keyboard\n", TEST_ITEM_RESET_KEYBOARD);
	printf("\t%2d - Reset screen\n", TEST_ITEM_RESET_SCREEN);
	printf("\t%2d - Reset core board\n", TEST_ITEM_RESET_CORE_BOARD);
	printf("\t%2d - Get voltage\n", TEST_ITEM_GET_VOLGATE);
	printf("\t%2d - Get electricity\n", TEST_ITEM_GET_ELECTRICITY);
	printf("\t%2d - Get keyboard model\n", TEST_ITEM_GET_KEYBOARD_MODEL);
	printf("\t%2d - Get lcd model\n", TEST_ITEM_GET_LCD_MODEL);
	printf("\t%2d - Select panel mic\n", TEST_ITEM_AUDIO_SELECT_PANEL_MIC);
	printf("\t%2d - Select hand mic\n", TEST_ITEM_AUDIO_SELECT_HAND_MIC);
	printf("\t%2d - Select headset mic\n", TEST_ITEM_AUDIO_SELECT_HEADSET_MIC);
	printf("\t%2d - Touchscreen enable\n", TEST_ITEM_TOUCHSCREEN_ENABLE);
	printf("\t%2d - Touchscreen disable\n", TEST_ITEM_TOUCHSCREEN_DISABLE);
	printf("\t%2d - Get RTC\n", TEST_ITEM_GET_RTC);
	printf("\t%2d - Set RTC\n", TEST_ITEM_SET_RTC);
	printf("\t%2d - Get headset insert status\n", TEST_ITEM_GET_HEADSET_INSERT_STATUS);
	printf("\t%2d - Get handle insert status\n", TEST_ITEM_GET_HANDLE_INSERT_STATUS);
	printf("\t%2d - Get LCD MCU software Version\n", TEST_ITEM_GET_LCD_MCUVERSION_STATUS);
	printf("\t%2d - Get KEYBORAD MCU software Version\n", TEST_ITEM_GET_BORAD_MCUVERSION_STATUS);
	printf("\t%2d - HAS YT8521SH Device??\n", TEST_ITEM_GET_YT8521SH_STATUS);
	printf("\tOther - Exit\n");
}

static void s_sighandler(int signum) {
	INFO("Receive signal %d, program will be exit!", signum);
}

static int s_signal_init(void) {
	struct sigaction act;
	bzero(&act, sizeof(struct sigaction));
	CHECK(!sigemptyset(&act.sa_mask), -1, "Error sigemptyset with %d: %s", errno, strerror(errno));
	CHECK(!sigaddset(&act.sa_mask, SIGINT), -1, "Error sigaddset with %d: %s", errno, strerror(errno));
	CHECK(!sigaddset(&act.sa_mask, SIGQUIT), -1, "Error sigaddset with %d: %s", errno, strerror(errno));
	CHECK(!sigaddset(&act.sa_mask, SIGTERM), -1, "Error sigaddset with %d: %s", errno, strerror(errno));
	act.sa_handler = s_sighandler;
	CHECK(!sigaction(SIGINT, &act, NULL), -1, "Error sigaction with %d: %s", errno, strerror(errno));
	CHECK(!sigaction(SIGQUIT, &act, NULL), -1, "Error sigaction with %d: %s", errno, strerror(errno));
	CHECK(!sigaction(SIGTERM, &act, NULL), -1, "Error sigaction with %d: %s", errno, strerror(errno));
	return 0;
}

static void s_gpio_notify_func(int gpio, int val) {
	INFO("Key %#x, value %#x", gpio, val);
}

static void s_panel_key_notify_func(int gpio, int val) {
	INFO("Key %#x, value %#x", gpio, val);
}

static void *s_watchdog_feed_thread(void *param) {
	INFO("Start feed watchdog!");
	unsigned int index = 0;
	s_watchdog_feed_thread_exit = false;
	while(!s_watchdog_feed_thread_exit) {
		if(!(index % WATCHDOG_TIMEOUT)) {
			drvWatchDogFeeding();
			INFO("Watchdog feed success!");
		}
		index ++;
		sleep(1);
	}
	INFO("Stop feed watchdog!");
	return NULL;
}

int main(int args, char *argv[]) {
	int test_item_index = -1;
	pthread_t watchdog_feed_thread_id = 0;

	INFO("Enter %s", __func__);
	CHECK(!s_signal_init(), -1, "Error s_signal_init!");
	CHECK(!drvCoreBoardInit(), -1, "Error drvCoreBoardInit!");

	drvSetGpioCbk(s_gpio_notify_func);
	drvSetGpioKeyCbk(s_panel_key_notify_func);

	if(drvWatchDogEnable()) {
		ERR("Error drvWatchDogEnable!");
		drvCoreBoardExit();
		return -1;
	}
	if(pthread_create(&watchdog_feed_thread_id, NULL, s_watchdog_feed_thread, NULL)) {
		ERR("Error pthread_create with %d: %s", errno, strerror(errno));
		drvWatchDogDisable();
		drvCoreBoardExit();
		return -1;
	}

	while(!s_main_thread_exit) {
		test_item_index = -1;
		s_show_usage();

		INFO("Please input test item:");
		if(scanf("%d", &test_item_index) != 1) {
			ERR("Error scanf with %d: %s", errno, strerror(errno));
			s_main_thread_exit = true;
			continue;
		}
		switch(test_item_index) {
		case TEST_ITEM_WATCHDOG_ENABLE:
			if(watchdog_feed_thread_id) {
				INFO("Watchdog is running!");
				break;
			}
			if(drvWatchDogEnable() == EXIT_FAILURE) {
				ERR("Error drvWatchDogEnable!");
				break;
			}
			if(pthread_create(&watchdog_feed_thread_id, NULL, s_watchdog_feed_thread, NULL)) {
				ERR("Error pthread_create with %d: %s", errno, strerror(errno));
				drvWatchDogDisable();
				break;
			}
			INFO("Execute enable watchdog success!");
			break;
		case TEST_ITEM_WATCHDOG_DISABLE:
			if(!watchdog_feed_thread_id) {
				INFO("Watchdog is not running!");
				break;
			}
			if(drvWatchDogDisable() == EXIT_FAILURE) {
				ERR("Error drvWatchDogDisable!");
				break;
			}
			s_watchdog_feed_thread_exit = true;
			if(pthread_join(watchdog_feed_thread_id, NULL)) {
				ERR("Error pthread_join with %d: %s", errno, strerror(errno));
				break;
			}
			watchdog_feed_thread_id = 0;
			INFO("Execute disable watchdog success!");
			break;
		case TEST_ITEM_SCREEN_ENABLE: {
			int i = 0;
			for(; i < 3; i ++) {
				drvDisableLcdScreen();
				INFO("Disable LCD screen success!");
				sleep(1);
				drvEnableLcdScreen();
				INFO("Enable LCD screen success!");
				sleep(1);
			}
			break;
		}
		case TEST_ITEM_SPEAKER_ENABLE:
			drvEnableSpeaker();
			break;
		case TEST_ITEM_SPEAKER_DISABLE:
			drvDisableSpeaker();
			break;
		case TEST_ITEM_SPEAKER_VOLUME_ADD:
			drvAddSpeakVolume(10);
			break;
		case TEST_ITEM_SPEAKER_VOLUME_SUB:
			drvSubSpeakVolume(10);
			break;
		case TEST_ITEM_WARNING_ENABLE:
			drvEnableWarning();
			break;
		case TEST_ITEM_WARNING_DISABLE:
			drvDisableWarning();
			break;
		case TEST_ITEM_HAND_ENABLE:
			drvEnableHandout();
			break;
		case TEST_ITEM_HAND_DISABLE:
			drvDisableHandout();
			break;
		case TEST_ITEM_HAND_VOLUME_ADD:
			drvAddHandVolume(10);
			break;
		case TEST_ITEM_HAND_VOLUME_SUB:
			drvSubHandVolume(10);
			break;
		case TEST_ITEM_HEADSET_ENABLE:
			drvEnableEarphout();
			break;
		case TEST_ITEM_HEADSET_DISABLE:
			drvDisableEarphout();
			break;
		case TEST_ITEM_HEADSET_VOLUME_ADD:
			drvAddEarphVolume(10);
			break;
		case TEST_ITEM_HEADSET_VOLUME_SUB:
			drvSubEarphVolume(10);
			break;
		case TEST_ITEM_USB0_ENABLE:
			drvEnableUSB0();
			break;
		case TEST_ITEM_USB0_DISABLE:
			drvDisableUSB0();
			break;
		case TEST_ITEM_USB1_ENABLE:
			drvEnableUSB1();
			break;
		case TEST_ITEM_USB1_DISABLE:
			drvDisableUSB1();
			break;
		case TEST_ITEM_GET_CPU_TEMPERATURE: {
			float temp = drvGetCPUTemp();
			if(temp) {
				INFO("CPU temperature is %0.3f", temp);
			}
			else {
				ERR("Error get CPU temperature!");
			}
			break;
		}
		case TEST_ITEM_GET_INTERFACE_BOARD_TEMPERATURE: {
			float temp = drvGetBoardTemp();
			if(temp) {
				INFO("Interface board temperature is %0.3f", temp);
			}
			else {
				ERR("Error get interface board temperature!");
			}
			break;
		}
			break;
		case TEST_ITEM_SET_KEYBOARD_LED_BRIGHTNESS: {
			int nBrtVal = 0;
			INFO("Please input brightness: (%u-%u)", PANEL_KEY_BRIGHTNESS_MIN, PANEL_KEY_BRIGHTNESS_MAX);
			if(scanf("%d", &nBrtVal) != 1) {
				ERR("Error scanf with %d: %s", errno, strerror(errno));
				continue;
			}
			if(nBrtVal > PANEL_KEY_BRIGHTNESS_MAX || nBrtVal < PANEL_KEY_BRIGHTNESS_MIN) {
				ERR("Error brightness out of range!");
				break;
			}
			drvSetLedBrt(nBrtVal);
			break;
		}
		case TEST_ITEM_SET_KEYBOARD_LED_ON_OFF: {   /*lsr modify 20220613*/
			int KeyIndex = 0;
			INFO("Please input KeyIndex: (%u-%u)", 1, 45);  //2023-01-15 1-45 by dazhi
			if(scanf("%d", &KeyIndex) != 1) {
				ERR("Error scanf with %d: %s", errno, strerror(errno));
				continue;
			}

			if(KeyIndex < 1 || KeyIndex > 45)
			{
				ERR("Error 输入超出范围(1-45)");
				continue;
			}	

			drvLightLED(KeyIndex);
			sleep(1);
			drvDimLED(KeyIndex); 
			sleep(1);
			drvLightLED(KeyIndex);
			sleep(1);
			drvDimLED(KeyIndex); 			
			break;
		}
		case TEST_ITEM_SET_ALLKEYBOARD_LED_ON_OFF: {			
			int i = 0;
			drvSetLedBrt(PANEL_KEY_BRIGHTNESS_MAX);   
			for(; i < 3; i ++) {
			    drvLightAllLED();
				INFO("All key LED is lighting-on!");
				sleep(1);  /*lsr add 20220613*/
				drvDimAllLED();
				INFO("All key LED is lighting-off!");
				sleep(1);  /*lsr add 20220613*/
			}
			break;
		}
		case TEST_ITEM_SET_SCREEN_BRIGHTNESS: {
			int nBrtVal = 0;
			INFO("Please input brightness: (%u-%u)", SCREEN_BRIGHTNESS_MIN, SCREEN_BRIGHTNESS_MAX);
			if(scanf("%d", &nBrtVal) != 1) {
				ERR("Error scanf with %d: %s", errno, strerror(errno));
				continue;
			}
			if(nBrtVal > SCREEN_BRIGHTNESS_MAX || nBrtVal < SCREEN_BRIGHTNESS_MIN) {
				ERR("Error input brightness out of range!");
				break;
			}
			drvSetLcdBrt(nBrtVal);
			break;
		}
		case TEST_ITEM_RESET_KEYBOARD:
			drvIfBrdReset();
			break;
		case TEST_ITEM_RESET_SCREEN:
			if(drvLcdReset()) {
				ERR("Error drvLcdReset!");
			}
			break;
		case TEST_ITEM_RESET_CORE_BOARD:
			if(drvCoreBrdReset()) {
				ERR("Error drvCoreBrdReset!");
			}
			break;
		case TEST_ITEM_GET_VOLGATE: {
			float val = drvGetVoltage();
			if(val <= 0) {
				ERR("Error drvGetVoltage!");
				break;
			}
			INFO("Current voltage is %.3f", val);
			break;
		}
		case TEST_ITEM_GET_ELECTRICITY: {
			float val = drvGetCurrent();
			if(val <= 0) {
				ERR("Error drvGetCurrent!");
				break;
			}
			INFO("Current electricity is %.3f", val);
			break;
		}
		case TEST_ITEM_GET_KEYBOARD_MODEL:
			INFO("Keyboard model is %d", getKeyboardType());
			break;
		case TEST_ITEM_GET_LCD_MODEL: {
			int type = drvGetLCDType();
			if(type > 0) {    //2023-01-05 修改
				INFO("LCD model is %#x", type);
			}
			else {
				ERR("Error drvGetLCDType %d", type);
			}
			break;
		}
		case TEST_ITEM_AUDIO_SELECT_PANEL_MIC:
			drvSelectHandFreeMic();
			break;
		case TEST_ITEM_AUDIO_SELECT_HAND_MIC:
			drvSelectHandMic();
			break;
		case TEST_ITEM_AUDIO_SELECT_HEADSET_MIC:
			drvSelectEarphMic();
			break;
		case TEST_ITEM_TOUCHSCREEN_ENABLE:
			drvEnableTouchModule();
			break;
		case TEST_ITEM_TOUCHSCREEN_DISABLE:
			drvDisableTouchModule();
			break;
		case TEST_ITEM_GET_RTC: {
			long secs = 0;
			if((secs = drvGetRTC()) == EXIT_FAILURE) {
				ERR("Error drvGetRTC!");
				break;
			}
			INFO("RTC seconds: %ld", secs);
			break;
		}
		case TEST_ITEM_SET_RTC: {
			long secs = 0;
			INFO("Please input RTC seconds:");
			if(scanf("%ld", &secs) != 1) {
				ERR("Error scanf with %d: %s", errno, strerror(errno));
				continue;
			}
			if(drvSetRTC(secs) == EXIT_FAILURE) {
				ERR("Error drvSetRTC!");
				break;
			}
			break;
		}
		case TEST_ITEM_GET_HEADSET_INSERT_STATUS: {
			int status = 0;
			if((status = drvGetMicStatus()) < 0) {
				ERR("Error drvGetMicStatus!");
				break;
			}
			INFO("Headset %s!", status? "insert":"no insert");
			break;
		}
		case TEST_ITEM_GET_HANDLE_INSERT_STATUS: {
			int status = 0;
			if((status = drvGetHMicStatus()) < 0) {
				ERR("Error drvGetMicStatus!");
				break;
			}
			INFO("Hand %s!", status? "insert":"no insert");
			break;
		}
		case TEST_ITEM_GET_LCD_MCUVERSION_STATUS:  //获得lcd单片机版本信息，2023-01-03
		{	
			int type = drvGetLCDMcuVersion();
			if(type >= 0) {
				INFO("LCD Mcu Version is %#x", type);
			}
			else {
				ERR("Error : LCD Mcu Version %d", type);
			}
			break;
		}
		break;
		case TEST_ITEM_GET_BORAD_MCUVERSION_STATUS:  //获得导光面板按键版本信息，2023-01-03
		{	
			int type = getKeyboardMcuVersion();
			if(type >= 0) {
				INFO("Keyboard Mcu Version is %#x", type);
			}
			else {
				ERR("Error : Keyboard Mcu Version  %d", type);
			}
			break;
		}
		break;
		case TEST_ITEM_GET_YT8521SH_STATUS:
			if(drvHasYt8521sh() == 1)
			{
				printf("Get YT8521SH \n");
			}
			else
			 	printf("find YT8521SH failed\n");
		break;
		default:
			s_main_thread_exit = true;
			break;
		}
	}

	if(watchdog_feed_thread_id) {
		s_watchdog_feed_thread_exit = true;
		pthread_cancel(watchdog_feed_thread_id);   //线程取消
		pthread_join(watchdog_feed_thread_id, NULL);
		watchdog_feed_thread_id = 0;
		drvWatchDogDisable();
	}
	drvCoreBoardExit();
	INFO("Exit %s", __func__);
	return 0;
}
