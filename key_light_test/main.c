/*
* @Author: dazhi
* @Date:   2023-02-28 15:49:15
* @Last Modified by:   dazhi
* @Last Modified time: 2023-03-09 15:26:24
*/


#include "drv722.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

float mcu_get_electricity(void);


int main(int argc,char* argv[])
{
	int delay_time_ms = 0;

	printf("Usage : %s [time_ms]-2023-03-09\n",argv[0]);
	drvCoreBoardInit();


	if(argc == 1)  //如果没有参数的话
	{
		delay_time_ms = 100000;   //100ms	
	}
	else if(argc == 2)
	{
		delay_time_ms = atoi(argv[1]);

		if(delay_time_ms < 100)
			delay_time_ms = 100;   //最小为100ms

		delay_time_ms *= 1000;  //算为毫秒

		if(delay_time_ms < 100000)   //最小为100ms
			delay_time_ms = 100000;
	}	

	printf("延时时间 =  %d ms\n",delay_time_ms/1000);

	drvSetLedBrt(100);   

	while(1)
	{	
		drvLightAllLED();
		printf("All key LED is lighting-on!\n");
		usleep(delay_time_ms);  /*lsr add 20220613*/
		drvDimAllLED();
		printf("All key LED is lighting-off!\n");
		usleep(delay_time_ms);  /*lsr add 20220613*/
	}
}




