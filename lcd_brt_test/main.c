

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/input.h>
#include <dirent.h>
#include <stdlib.h>
#include "../include/drv722.h"
#include <pthread.h>

//aarch64-linux-gnu-gcc ./main.c -o lcd_brt_test -L../lib -ldrv722 -lpthread


int main(int argc,char * argv[])
{
    int i = 0;
    int sleeptime = 1000; //1s
    unsigned int step = 1;

    printf("Usage : %s [sleeptime ms] [step 1-50]\n",argv[0]);

    if(argc >= 2)
    {
        sleeptime = atoi(argv[1]);
        if(argc >= 3)
            step = atoi(argv[2]);
    }

    if(sleeptime < 1)
        sleeptime = 1;
    else if(sleeptime > 10000)
        sleeptime = 10000;  //最大10s

    sleeptime *= 1000;  //us --> ms

    if(step > 50)
        step = 50;
    else if(step == 0)
        step = 1;

    while(1)
    {
	    printf("lcd 由暗到亮  %dms变化一次\n",sleeptime/1000);

	    for(i=0;i<256;i+=step)
	    {
	        drvSetLcdBrt(i);
	        usleep(sleeptime);   //100ms 调整
	        printf("lcd Brt=%d\n",i);
	    }

	    printf("已达到最亮，休眠2s \n");
	    sleep(2);  //休眠5秒

	    printf("lcd 由亮到暗  %dms变化一次\n",sleeptime/1000);

	    for(i=0;(i<256) && (255-i>0);i+=step)
	    {
	        drvSetLcdBrt(255-i);
	        usleep(sleeptime);   //100ms 调整
	        printf("Brt i=%d\n",255-i);
	    }
	    printf("已达到最暗，休眠2s \n");
	    sleep(2);  //休眠5秒
 	}
    printf("程序结束\n");

    return 0;
}


