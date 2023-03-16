/*
* @Author: dazhi
* @Date:   2023-02-28 15:49:15
* @Last Modified by:   dazhi
* @Last Modified time: 2023-03-01 16:39:31
*/


#include "drv722.h"
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

float mcu_get_electricity(void);


int main(int argc,char* argv[])
{
	int delay_time_ms = 0;
	float val = 0.0;

	if(argc != 2)
	{
		printf("Usage %s <delay_time(ms)>\n",argv[0]);
		return -1;
	}	

	delay_time_ms = atoi(argv[1]);

	if(delay_time_ms < 1)
		delay_time_ms = 1;

	delay_time_ms *= 1000;  //算为毫秒

	if(delay_time_ms < 1000)
		delay_time_ms = 1000;


	while(1)
	{	
		// val = drvGetVoltage();
		// printf("电压值 = %.2f\n",val);	
		val = mcu_get_electricity();
		printf("电流值 = %.2f\n",val);
		usleep(delay_time_ms);
	}
}




