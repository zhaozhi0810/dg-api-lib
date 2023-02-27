/*
 * misc.c
 *
 *  Created on: Nov 22, 2021
 *      Author: zlf
 */
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>
#include <sys/select.h>
#include <linux/input.h>
#include <linux/rtc.h>

#include "debug.h"
#include "drv722.h"
#include "keyboard.h"
#include "mcu.h"
#include "gpio.h"
#include "fs.h"
#include <dirent.h>


#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <linux/mii.h>
#include <linux/sockios.h>



#define USB0_ENABLE_PIN	(32*2 + 8*3 + 2)	// GPIO2_D2, AL4, 90
#define USB1_ENABLE_PIN	(32*4 + 8*3 + 1)	// GPIO4_D1, AK4, 153

#define CPU0_TEMP_DEV		"/sys/class/thermal/thermal_zone0/temp"
#define PROC_INPUT_DEVICES	"/proc/bus/input/devices"
#define RTC_DEV				"/dev/rtc"

static GPIO_NOTIFY_FUNC s_gpio_notify_func = NULL;
static pthread_t s_recv_gpio_event_thread_id = 0;
static pthread_mutex_t s_recv_gpio_event_thread_mutex;
static bool s_recv_gpio_event_thread_exit = false;

static bool s_egn_earmic_insert = false;
static bool s_egn_hmic_insert = false;

int s_write_reg(unsigned char addr, unsigned char val);




int gpio_notify_func(int gpio, int val) {
	CHECK(s_gpio_notify_func, -1, "Error s_gpio_notify_func is null!");
	pthread_mutex_lock(&s_recv_gpio_event_thread_mutex);
	if(s_gpio_notify_func)
		s_gpio_notify_func(gpio, val);
	pthread_mutex_unlock(&s_recv_gpio_event_thread_mutex);
	return 0;
}

static void *s_recv_gpio_event_thread(void *param) {
	int ret = 0;
	int fd = 0;
	fd_set readfds;
//	struct timeval timeout;
	struct input_event input_event;
	char gpio_event_dev[64] = "\0";
	int input_device_num = 0;

	INFO("Enter %s", __func__);
	CHECK((input_device_num = get_event_dev("gpio-keys")) >= 0, NULL, "Error s_get_event_dev!");
	snprintf(gpio_event_dev, sizeof(gpio_event_dev), "/dev/input/event%d", input_device_num);
	CHECK((fd = open(gpio_event_dev, O_RDONLY)) > 0, NULL, "Error open with %d: %s", errno, strerror(errno));
	while(!s_recv_gpio_event_thread_exit) {
		// timeout.tv_sec = 1;
		// timeout.tv_usec = 0;
		FD_ZERO(&readfds);
		FD_SET(fd, &readfds);
		if((ret = select(fd + 1, &readfds, NULL, NULL, NULL)) < 0) {			
			if(errno == 4)
				continue;
			ERR("Error select with %d: %s", errno, strerror(errno));
			s_recv_gpio_event_thread_exit = true;
		}
		else if(!ret) {
//			ERR("Error select timeout!");
		}
		else if(FD_ISSET(fd, &readfds)) {
			bzero(&input_event, sizeof(struct input_event));
			if(read(fd, &input_event, sizeof(struct input_event)) != sizeof(struct input_event)) {
				ERR("Error read with %d: %s\n", errno, strerror(errno));
				s_recv_gpio_event_thread_exit = true;
				continue;
			}
			if(input_event.type == EV_KEY) {
				if(gpio_notify_func(input_event.code, input_event.value)) {
					INFO("Key %#x, value %#x", input_event.code, input_event.value);
				}
				if(input_event.code == egn_earmic) {
					s_egn_earmic_insert = input_event.value? true:false;
				}
				else if(input_event.code == egn_hmic) {
					s_egn_hmic_insert = input_event.value? true:false;
				}
			}
		}
	}
	close(fd);
	INFO("Exit %s", __func__);
	return NULL;
}

static int s_gpio_status_check_init(void) {
	CHECK(!pthread_mutex_init(&s_recv_gpio_event_thread_mutex, NULL), -1, "Error pthread_mutex_init with %d: %s", errno, strerror(errno));
	if(pthread_create(&s_recv_gpio_event_thread_id, NULL, s_recv_gpio_event_thread, NULL)) {
		ERR("Error pthread_create with %d: %s", errno, strerror(errno));
		pthread_mutex_destroy(&s_recv_gpio_event_thread_mutex);
		return -1;
	}
	return 0;
}

static int s_gpio_status_check_exit(void) {
	s_recv_gpio_event_thread_exit = true;
	pthread_cancel(s_recv_gpio_event_thread_id);   //线程取消
	pthread_join(s_recv_gpio_event_thread_id, NULL);
	pthread_mutex_destroy(&s_recv_gpio_event_thread_mutex);
	s_recv_gpio_event_thread_id = 0;
	return 0;
}

void drvSetGpioCbk(GPIO_NOTIFY_FUNC cbk) {
	s_gpio_notify_func = cbk;
}




int drvCoreBoardInit(void) {
	keyboard_init();  //只调用不判断  2022-11-24

	s_write_reg( 0x1a, 0);   //2023-02-22增加
	s_write_reg( 0x1b, 0);   //2023-02-22增加
	s_write_reg( 0x12, 0x3a);   //2023-02-23增加
//	printf("--drvCoreBoardInit 2023-02-22\n");

	
	if(s_gpio_status_check_init()) {
		ERR("Error s_gpio_status_check_init!");
		keyboard_exit();
		return -1;
	}


//	printf("drvCoreBoardInit 2023-02-22\n");

	
	return 0;
}

int drvCoreBoardExit(void) {
	keyboard_exit();
	s_gpio_status_check_exit();
	return 0;
}

void drvEnableUSB0(void) {
	CHECK(!gpio_init(USB0_ENABLE_PIN, true, GPIO_INPUT_EDGE_NONE), , "Error gpio_init!");
	if(gpio_set_value(USB0_ENABLE_PIN, 0)) {
		ERR("Error gpio_set_value!");
	}
	gpio_exit(USB0_ENABLE_PIN);
}

void drvDisableUSB0(void) {
	CHECK(!gpio_init(USB0_ENABLE_PIN, true, GPIO_INPUT_EDGE_NONE), , "Error gpio_init!");
	if(gpio_set_value(USB0_ENABLE_PIN, 1)) {
		ERR("Error gpio_set_value!");
	}
	gpio_exit(USB0_ENABLE_PIN);
}

void drvEnableUSB1(void) {
	CHECK(!gpio_init(USB1_ENABLE_PIN, true, GPIO_INPUT_EDGE_NONE), , "Error gpio_init!");
	if(gpio_set_value(USB1_ENABLE_PIN, 0)) {
		ERR("Error gpio_set_value!");
	}
	gpio_exit(USB1_ENABLE_PIN);
}

void drvDisableUSB1(void) {
	CHECK(!gpio_init(USB1_ENABLE_PIN, true, GPIO_INPUT_EDGE_NONE), , "Error gpio_init!");
	if(gpio_set_value(USB1_ENABLE_PIN, 1)) {
		ERR("Error gpio_set_value!");
	}
	gpio_exit(USB1_ENABLE_PIN);
}

float drvGetCPUTemp(void) {
	int fd = 0;
	char temp_str[16] = "\0";
	long temp = 0;
	CHECK((fd = open(CPU0_TEMP_DEV, O_RDONLY)) > 0, 0, "Error open with %d: %s", errno, strerror(errno));
	if(read(fd, temp_str, sizeof(temp_str)) <= 0) {
		ERR("Error read with %d: %s", errno, strerror(errno));
		close(fd);
		return 0;
	}
	CHECK(!close(fd), 0, "Error close with %d: %s", errno, strerror(errno));
	temp = strtol(temp_str, NULL, 10);
	return ((float)temp/1000);
}

float drvGetBoardTemp(void) {
	return mcu_get_temperature();
}

int drvCoreBrdReset(void) {
	if(mcu_core_board_reset()) {
		ERR("Error mcu_core_board_reset!");
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

float drvGetVoltage(void) {
	return mcu_get_voltage();
}

float drvGetCurrent(void) {
	return mcu_get_electricity();
}

long drvGetRTC(void) {
	int fd = 0;
	struct rtc_time rtc_time;
	struct tm tp;

	CHECK((fd = open(RTC_DEV, O_RDONLY)) > 0, EXIT_FAILURE, "Error open with %d: %s", errno, strerror(errno));
	bzero(&rtc_time, sizeof(struct rtc_time));
	if(ioctl(fd, RTC_RD_TIME, &rtc_time)) {
		ERR("Error ioctl with %d: %s", errno, strerror(errno));
		close(fd);
		return EXIT_FAILURE;
	}
	CHECK(!close(fd), EXIT_FAILURE, "Error close with %d: %s", errno, strerror(errno));
	fd = 0;

	bzero(&tp, sizeof(struct tm));
	tp.tm_sec = rtc_time.tm_sec;
	tp.tm_min = rtc_time.tm_min;
	tp.tm_hour = rtc_time.tm_hour;
	tp.tm_mday = rtc_time.tm_mday;
	tp.tm_mon = rtc_time.tm_mon;
	tp.tm_year = rtc_time.tm_year;
	tp.tm_wday = rtc_time.tm_wday;
	tp.tm_yday = rtc_time.tm_yday;
	tp.tm_isdst = rtc_time.tm_isdst;
	return mktime(&tp);
}

long drvSetRTC(long secs) {
	int fd = 0;
	struct rtc_time rtc_time;
	struct tm *tp = NULL;

	CHECK(tp = localtime(&secs), EXIT_FAILURE, "Error localtime with %d: %s", errno, strerror(errno));

	CHECK((fd = open(RTC_DEV, O_RDONLY)) > 0, EXIT_FAILURE, "Error open with %d: %s", errno, strerror(errno));
	bzero(&rtc_time, sizeof(struct rtc_time));
	rtc_time.tm_sec = tp->tm_sec;
	rtc_time.tm_min = tp->tm_min;
	rtc_time.tm_hour = tp->tm_hour;
	rtc_time.tm_mday = tp->tm_mday;
	rtc_time.tm_mon = tp->tm_mon;
	rtc_time.tm_year = tp->tm_year;
	rtc_time.tm_wday = tp->tm_wday;
	rtc_time.tm_yday = tp->tm_yday;
	rtc_time.tm_isdst = tp->tm_isdst;
	if(ioctl(fd, RTC_SET_TIME, &rtc_time)) {
		ERR("Error ioctl with %d: %s", errno, strerror(errno));
		close(fd);
		return EXIT_FAILURE;
	}
	CHECK(!close(fd), EXIT_FAILURE, "Error close with %d: %s", errno, strerror(errno));
	return EXIT_SUCCESS;
}

bool get_headset_insert_status(void) {
	int value ;

	//读取引脚的信息
	if(gpio_get_value(65, &value) == 0)
	{
		return !value;   //低电平有效，读到的是0，返回1.
	}

	return s_egn_earmic_insert;
}

bool get_handle_insert_status(void) {
	int value ;

	//读取引脚的信息
	if(gpio_get_value(70, &value) == 0)
	{
		return !value;  //低电平有效，读到的是0，返回1.
	}

	return s_egn_hmic_insert;
}




#if 0  //2023-02-27  del by zhaozhi


static struct ifreq ifr;
static int fd;



static int mdio_read(int skfd, int location)
{
	struct mii_ioctl_data *mii = (struct mii_ioctl_data *)&ifr.ifr_data;
	mii->reg_num = location;
	if (ioctl(skfd, SIOCGMIIREG, &ifr) < 0) {
		fprintf(stderr, "SIOCGMIIREG on %s failed: %s\n", ifr.ifr_name,
				strerror(errno));
		return -1;
	}
	return mii->val_out;
}



/*
	参数 dev_name，分别表示eth0-eth9

	返回值：
	-1 出错
	0  表示没找到
	1  表示找到了
 */
static int gmac_read_phyid(char* dev_name)
{
//	char dev_name[8] = {"eth"};
	int dev_id;
	int ret;
	// dev_name[3] = c_num;
	// dev_name[4] = '\0';

	strncpy(ifr.ifr_name, dev_name, IFNAMSIZ);

	ret = ioctl(fd, SIOCGMIIPHY, &ifr);
	if (ret < 0) {
		fprintf(stderr, "SIOCGMIIPHY on %s failed: %s\n", ifr.ifr_name,
				strerror(errno));
		return -1;
	}

	dev_id = (mdio_read(fd, 2)<<16) |  mdio_read(fd, 3);

	if(dev_id == 0x0000011a)
		return 1;

	return 0;
}




//找到系统中phy的地址，
static int gmac_find_ethdev(void)
{
//	int addr;
	DIR *dir = NULL;
	struct dirent *file;

	if(access("/sys/class/net/",F_OK))
	{
		printf("/sys/class/net/ not exist !!\n");
		return -1;
	}
	
	if((dir = opendir("/sys/class/net/")) == NULL) {  
		printf("opendir /sys/class/net/ failed!");
		return -1;
	}
	while((file = readdir(dir))) {
		
		if(file->d_name[0] == '.')
			continue;
		//总线号也是可以找到的，这里就没有去识别总线号了
		else if(strncmp(file->d_name,"eth",3) == 0)
		{
			printf("name = %s\n",file->d_name);

			if(gmac_read_phyid(file->d_name) == 1)
			{
				closedir(dir);
				return 1;
			}

			//addr = strtol(file->d_name+3, NULL, 10);
			//printf("addr = %#x\n",addr);
			//closedir(dir);
			//return addr;   //返回设备地址			
		}	
	
	}
	closedir(dir);
	return -1;
}






 


//2023-02-01 增加对 Yt8521sh 的检测  by zhaodazhi
//返回1表示存在Yt8521sh，其他表示不存在
int drvHasYt8521sh(void)
{
//	struct mii_ioctl_data *mii = (struct mii_ioctl_data *)&ifr.ifr_data;
	int ret;
	fd = socket(AF_INET, SOCK_DGRAM,0);
	if (fd < 0) {
		perror("open");
		exit(1);
	}
 	
 	ret = gmac_find_ethdev();
	// if(ret == 1)
	// 	printf("Get YT8521SH \n");
	// else
	// 	printf("find YT8521SH failed\n");
 
	close(fd);
 
	return (ret == 1);	// 返回1 表示存在，其他表示不存在
}

#endif   //#if 0  //2023-02-27  del by zhaozhi

