

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/input.h>
#include <dirent.h>
#include <stdlib.h>
#include "../include/drv722.h"


//aarch64-linux-gnu-gcc ./main.c -o wdDisable -L../lib -ldrv722 -lpthread
int mcu_watchdog_disable(void);


int main(int argc,char * argv[])
{
    if(mcu_watchdog_disable()) {
		printf("Error mcu_watchdog_disable!");
		return -1;
	}

    printf("关闭看门狗\n");	

    return 0;
}


