/*
* @Author: dazhi
* @Date:   2023-02-01 16:56:32
* @Last Modified by:   dazhi
* @Last Modified time: 2023-02-01 17:31:09
*/
/*
	1、 先在 /sys/class/net 目录中找到eth 开头的目录名，这就表示这个gmac的驱动已经加上了
	2、 确定了网卡文件名，就使用ioctl的方法读取寄存器2，3的值（组合为32位），0x0000011a就表示是YT8521SH这个芯片
*/
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <linux/mii.h>
#include <linux/sockios.h>

#include <dirent.h>

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
 
 
//#define DEVICE getenv("ETH_DEVICE")?getenv("ETH_DEVICE"):"eth2"
 
static int fd = -1;
static struct ifreq ifr;
 



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






 
// static void mdio_write(int skfd, int location, int value)
// {
// 	struct mii_ioctl_data *mii = (struct mii_ioctl_data *)&ifr.ifr_data;
// 	mii->reg_num = location;
// 	mii->val_in = value;
// 	if (ioctl(skfd, SIOCSMIIREG, &ifr) < 0) {
// 		fprintf(stderr, "SIOCSMIIREG on %s failed: %s\n", ifr.ifr_name,
// 				strerror(errno));
// 	}
// }
// static void help()
// {
// 	printf("help :\n");
// 	printf("set ETH_DEVICE to change device\n");
// 	printf("\t\tmdio_cmd addr reg [val]\n");
// }
 

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



// int main(int argc, char *argv[])
// {
// 	struct mii_ioctl_data *mii = (struct mii_ioctl_data *)&ifr.ifr_data;
// 	// int addr, reg;
// 	int ret;
// 	fd = socket(AF_INET, SOCK_DGRAM,0);
// 	if (fd < 0) {
// 		perror("open");
// 		exit(1);
// 	}
 
// 	if(gmac_find_ethdev() == 1)
// 		printf("Get YT8521SH \n");
// 	else
// 		printf("find YT8521SH failed\n");
 
// 	close(fd);
 
// 	return 0;
// }