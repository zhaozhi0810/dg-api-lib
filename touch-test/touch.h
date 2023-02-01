#ifndef _TOUCH_H_
#define _TOUCH_H_

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/input.h>
#include <dirent.h>
#include <stdlib.h>
int x;
int y;
int touch_init();//
int touch_close();//
int get_coord(); //获取手指触摸的坐标
int get_direction();//
void *get_touch(void *arg);//

#endif