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
	TEST_ITEM_WATCHDOG_ENABLE,  //0
	TEST_ITEM_WATCHDOG_DISABLE,  //1
	TEST_ITEM_SCREEN_ENABLE,    //2
	TEST_ITEM_SPEAKER_ENABLE,   //3
	TEST_ITEM_SPEAKER_DISABLE,  //4
	TEST_ITEM_SPEAKER_VOLUME_ADD,  //5
	TEST_ITEM_SPEAKER_VOLUME_SUB,  //6
	TEST_ITEM_WARNING_ENABLE,     //7
	TEST_ITEM_WARNING_DISABLE,   //8
	TEST_ITEM_HAND_ENABLE,      //9
	TEST_ITEM_HAND_DISABLE,     //10
	TEST_ITEM_HAND_VOLUME_ADD,   //11
	TEST_ITEM_HAND_VOLUME_SUB,   //12
	TEST_ITEM_HEADSET_ENABLE,    //13
	TEST_ITEM_HEADSET_DISABLE,   //14
	TEST_ITEM_HEADSET_VOLUME_ADD,   //15
	TEST_ITEM_HEADSET_VOLUME_SUB,   //16
	TEST_ITEM_USB0_ENABLE,        //17
	TEST_ITEM_USB0_DISABLE,      //18
	TEST_ITEM_USB1_ENABLE,      //19
	TEST_ITEM_USB1_DISABLE,     //20
	TEST_ITEM_GET_CPU_TEMPERATURE,    //21
	TEST_ITEM_GET_INTERFACE_BOARD_TEMPERATURE,   //22
	TEST_ITEM_SET_KEYBOARD_LED_BRIGHTNESS,     //23
	TEST_ITEM_SET_KEYBOARD_LED_ON_OFF,         //24
	TEST_ITEM_SET_ALLKEYBOARD_LED_ON_OFF,      //25
	TEST_ITEM_SET_SCREEN_BRIGHTNESS,           //26
	TEST_ITEM_RESET_KEYBOARD,                  //27
	TEST_ITEM_RESET_SCREEN,                    //28
	TEST_ITEM_RESET_CORE_BOARD,                //29
	TEST_ITEM_GET_VOLGATE,                     //30
	TEST_ITEM_GET_ELECTRICITY,                 //31
	TEST_ITEM_GET_KEYBOARD_MODEL,             //32
	TEST_ITEM_GET_LCD_MODEL,                  //33
	TEST_ITEM_AUDIO_SELECT_PANEL_MIC,         //34
	TEST_ITEM_AUDIO_SELECT_HAND_MIC,          //35
	TEST_ITEM_AUDIO_SELECT_HEADSET_MIC,       //36
	TEST_ITEM_TOUCHSCREEN_ENABLE,             //37
	TEST_ITEM_TOUCHSCREEN_DISABLE,             //38
	TEST_ITEM_GET_RTC,             //39
	TEST_ITEM_SET_RTC,             //40
	TEST_ITEM_GET_HEADSET_INSERT_STATUS,             //41
	TEST_ITEM_GET_HANDLE_INSERT_STATUS,             //42
	TEST_ITEM_GET_LCD_MCUVERSION_STATUS, //获得LCD 屏单片机版本信息,43
	TEST_ITEM_GET_BORAD_MCUVERSION_STATUS,  //获得导光面板按键版本信息,44
	TEST_ITEM_GET_YT8521SH_STATUS              //45
}; 

extern int drvCoreBoardExit(void);

static bool s_main_thread_exit = false;
static bool s_watchdog_feed_thread_exit = false;




//左侧（中括号中的数）是722自动自定义的键值 比如KC_L1 --> 1
//右侧是s_user_key_value中对应值得索引号，比如数字0，在数组中排13，就是0xd，
//与驱动中正好是反的，目的就是在drv722test中的点亮灯的序号正好是文档表格中的数字
//2023-02-22，by zhaozhi
static const unsigned char s_user_map_led_value[] = {
	[0x01] = 1	, //图示1左1 	 //KC_L1   1
	[0x02] = 3	, //图示2左2 	 //KC_L2   3
	[0x03] = 5	, //图示3左3 	 //KC_L3   5
	[0x04] = 7	, //图示4左4 	 //KC_L4   7
	[0x05] = 9	, //图示5左5 	 //KC_L5   9
	[0x06] = 44 , //图示6左6 	 //KC_L6   44
	[0x07] = 2	, //图示7右1 	 //KC_R1   2
	[0x08] = 4	, //图示8右2 	 //KC_R2   4
	[0x09] = 6	, //图示9右3 	 //KC_R3   6
	[0x0A] = 8	, //图示10右4	 //KC_R4	8  
	[0x0B] = 10 , //图示11右5	 //KC_R5	10
	[0x0C] = 45 , //图示12右6	//KC_R6    45
	[0x0D] = 27 , //0		//KC_NUM0	27
	[0x0E] = 18 , //1		//KC_NUM1	18
	[0x0F] = 19 , //2		//KC_NUM2	19	
	[0x10] = 20 , //3		//KC_NUM3	20	
	[0x11] = 21 , //4		//KC_NUM4	21	
	[0x12] = 22 , //5		//KC_NUM5	22	
	[0x13] = 23 , //6		//KC_NUM6	23	
	[0x14] = 24 , //7		//KC_NUM7	24	
	[0x15] = 25 , //8		//KC_NUM8	25	
	[0x16] = 26 , //9		//KC_NUM9	26	
	[0x17] = 28 , //*		//KC_DOT   28	
	[0x18] = 29 , //#		//KC_CLEAR	 29 
	[0x19] = 13 , //电话（拨号）			//KC_TEL   13
	[0x1A] = 35 , //音量+ 	//KC_VOLUME_UP	   35
	[0x1B] = 36 , //音量- 	//KC_VOLUME_DOWN	36	
	[0x27] = 14 , //TEST		//KC_TESKKEY   14	//原0x1c改0x27
	[0x1D] = 11 , //内通			//KC_INC	  11
	[0x1E] = 12 , //外通			//KC_EXC	  12 
	[0x1F] = 30 , //上 		//KC_UP 	  30
	[0x20] = 31 , //下 		//KC_DOWN	  31
	[0x21] = 17 , //PTT 	//KC_PTT		17	
	[0x22] = 32 , //左 		//KC_LEFT	32
	[0x23] = 33 , //右 		//KC_FIGHT	33
	[0x24] = 34 , //OK			//KC_OK 	34
	[0x25] = 15 , //保持			//KC_HOLD	15
	[0x26] = 16 , //设置/摘挂机			//KC_RECOV	16
	[0x27] = 42 , //测试/复位		//KC_xxx	
	[0x28] = 37 , //指示灯-红		//LED_RED  37	
	[0x29] = 38 , //指示灯-绿		//LED_GREEN 38	
	[0x2A] = 39 , //指示灯-蓝		//KC_BLUE	39	
	[0x2B] = 40  //全部键灯 		//ALL_KEY_LED  40
};






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
			INFO("Please input KeyIndex: (1-43)");  //2023-01-15 1-45 by dazhi
			if(scanf("%d", &KeyIndex) != 1) {
				ERR("Error scanf with %d: %s", errno, strerror(errno));
				continue;
			}

			if(KeyIndex < 1 || KeyIndex > 43)
			{
				ERR("Error 输入超出范围(1-43)");
				continue;
			}	

			KeyIndex = s_user_map_led_value[KeyIndex];  //转换一下，2023-02-22
			

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
