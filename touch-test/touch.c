// #include "touch.h"

// #define UP 1
// #define DOWN 2
// #define LEFT 3
// #define RIGHT 4

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/input.h>
#include <dirent.h>
#include <stdlib.h>



static int fd;
static int x,y;


static int touch_init(void)
{
    fd = open("/dev/input/event1", O_RDWR);
    if (fd == -1)
    {
        perror("open event1 error ");
        return -1;
    }
}

// int touch_close()
// {
//     close(fd);
// }
//获取手指触摸的坐标
int main(void)
{
    struct input_event ev;
    x = -1, y = -1;
   
    touch_init();

    while(1)
    {
        while (1)
        {
            //读取
            read(fd, &ev, sizeof(ev));

            //解析
            if (ev.type == EV_ABS) //触摸事件
            {
                if (ev.code == ABS_X) //x轴
                {
                    x = ev.value;
                }
                else if (ev.code == ABS_Y) //y轴
                {
                    y = ev.value;
                }
                else if (ev.code == ABS_PRESSURE && ev.value == 0) //压力事件，当压力值为0就退出（手指已经离开屏幕了）
                {
                    if (x != -1 && y != -1)
                    {
                        
                        break;
                    }
                }
            }
            else if (ev.type == EV_KEY && ev.code == BTN_TOUCH && ev.value == 0) //按键事件  ，按键松开就退出
            {
                if (x != -1 && y != -1)
                {
                   
                    break;
                }
            }
        }

        printf("( %d , %d )\n", x, y);
    }


    return 0;
    //关闭文件
 //   close(fd);
}

// int get_direction()
// {
//     //打开触摸屏文件
//     // int fd = open("/dev/input/event0", O_RDWR);
//     // if (fd == -1)
//     // {
//     //     perror("open event0 error ");
//     //     return -1;
//     // }

//     //读取数据
//     struct input_event ev;
//     int x0 = -1, y0 = -1; //起点坐标
//     int x1 = -1, y1 = -1; //终点坐标
//     while (1)
//     {
//         //读取
//         int re = read(fd, &ev, sizeof(ev));
//         if (re != sizeof(ev))
//         {
//             continue;
//         }

//         //解析
//         if (ev.type == EV_ABS) //触摸事件
//         {
//             if (ev.code == ABS_X) //x轴
//             {
//                 if (x0 == -1)
//                 {
//                     x0 = ev.value;
//                 }
//                 x1 = ev.value;
//             }
//             else if (ev.code == ABS_Y) //y轴
//             {
//                 if (y0 == -1)
//                 {
//                     y0 = ev.value;
//                 }
//                 y1 = ev.value;
//             }
//             else if (ev.code == ABS_PRESSURE && ev.value == 0) //压力事件，当压力值为0就退出（手指已经离开屏幕了）
//             {
//                 if (x0 != -1 && y0 != -1)
//                 {
//                     break;
//                 }
//             }
//         }
//         else if (ev.type == EV_KEY && ev.code == BTN_TOUCH && ev.value == 0) //按键事件  ，按键松开就退出
//         {
//             if (x0 != -1 && y0 != -1)
//             {
//                 break;
//             }
//         }
//     }
//     //关闭文件
//     close(fd);

//     printf("( %d , %d ) ---- ( %d , %d )\n", x0, y0, x1, y1);

//     int xx = abs(x0 - x1);
//     //xx = x0-x1>0 ? x0-x1 : x1-x0;
//     int yy = abs(y0 - y1);

//     if (xx > yy) //水平方向  左右
//     {
//         if (x0 > x1)
//         {
//             return LEFT;
//         }
//         else
//         {
//             return RIGHT;
//         }
//     }
//     else //垂直方向 上下
//     {
//         if (y0 > y1)
//         {
//             return UP;
//         }
//         else
//         {
//             return DOWN;
//         }
//     }
// }

// //外部声明
// extern int xxx;
// extern int yyy;
// //线程函数 ： 获取手指触摸的坐标
// void *get_touch(void *arg)
// {
//     touch_init();

//     while (1)
//     {
//         //读取数据
//         struct input_event ev;

//         while (1)
//         {
//             //读取
//             int re = read(fd, &ev, sizeof(ev));
//             if (re != sizeof(ev))
//             {
//                 continue;
//             }

//             //解析
//             if (ev.type == EV_ABS) //触摸事件
//             {
//                 if (ev.code == ABS_X) //x轴
//                 {
//                     xxx = ev.value;
//                 }
//                 else if (ev.code == ABS_Y) //y轴
//                 {
//                     yyy = ev.value;
//                 }
//                 else if (ev.code == ABS_PRESSURE && ev.value == 0) //压力事件，当压力值为0就退出（手指已经离开屏幕了）
//                 {
//                     if (xxx != -1 && yyy != -1)
//                     {
//                         break;
//                     }
//                 }
//             }
//             else if (ev.type == EV_KEY && ev.code == BTN_TOUCH && ev.value == 0) //按键事件  ，按键松开就退出
//             {
//                 if (xxx != -1 && yyy != -1)
//                 {
//                     break;
//                 }
//             }
//         }

//         printf("( %d , %d )\n", xxx, yyy);
//     }

//     touch_close();
// }
// 
// 

