/*
 * drv722.h
 *
 *  Created on: Nov 19, 2021
 *      Author: zlf
 */

#ifndef DRV722_H_
#define DRV722_H_

typedef enum {
	egn_fptt = 17,
	egn_ptt = 25,
	egn_hptt = 30,
	egn_earmic = 48,
	egn_hmic = 49,
} enGpioNotify;

typedef enum {
	GPIO_STATUS_RELEASE,
	GPIO_STATUS_PRESS,
} enGpioNotifyStatus;

typedef void (*GPIO_NOTIFY_FUNC)(int gpio, int val);
typedef void (*GPIO_NOTIFY_KEY_FUNC)(int gpio, int val);

extern int drvCoreBoardInit(void);
extern int drvCoreBoardExit(void);

extern int drvWatchDogEnable(void);
extern int drvWatchDogDisable(void);
extern void drvWatchDogFeeding(void);
extern int drvWatchdogSetTimeout(int timeout);
extern int drvWatchdogGetTimeout(void);

extern void drvDisableSpeaker(void);
extern void drvEnableSpeaker(void);
extern void drvDisableWarning(void);
extern void drvEnableWarning(void);

extern void drvEnableUSB0(void);
extern void drvEnableUSB1(void);
extern void drvDisableUSB0(void);
extern void drvDisableUSB1(void);

extern void drvEnableLcdScreen(void);
extern void drvDisableLcdScreen(void);

extern void drvEnableTouchModule(void);
extern void drvDisableTouchModule(void);

extern int drvGetLCDType(void);
extern int getKeyboardType(void);

extern float drvGetCPUTemp(void);
extern float drvGetBoardTemp(void);

extern long drvGetRTC(void);
extern long drvSetRTC(long secs);

extern void drvSetLedBrt(int nBrtVal);
extern void drvSetLcdBrt(int nBrtVal);
extern void drvLightLED(int nKeyIndex);
extern void drvDimLED(int nKeyIndex);
extern void drvLightAllLED(void);
extern void drvDimAllLED(void);
extern int drvGetLEDStatus(int nKeyIndex);

extern int drvGetMicStatus(void);
extern int drvGetHMicStatus(void);

extern void drvSetGpioCbk(GPIO_NOTIFY_FUNC cbk);
extern void drvSetGpioKeyCbk(GPIO_NOTIFY_KEY_FUNC cbk);

extern float drvGetVoltage(void);
extern float drvGetCurrent(void);
extern int drvLcdReset(void);
extern int drvIfBrdReset(void);
extern int drvCoreBrdReset(void);

extern void drvEnableTune(void);
extern void drvDisableTune(void);
extern void drvAdjustTune(void);
extern void drvSetTuneUp(void);
extern void drvSetTuneDown(void);

extern void drvAddSpeakVolume(int value);
extern void drvSubSpeakVolume(int value);
extern void drvSetSpeakVolume(int value); /*lsr add 20220510*/
extern void drvAddHandVolume(int value);
extern void drvSubHandVolume(int value);
extern void drvSetHandVolume(int value); /*lsr add 20220510*/
extern void drvAddEarphVolume(int value);
extern void drvSubEarphVolume(int value);
extern void drvSetEarphVolume(int value); /*lsr add 20220510*/


extern void drvEnableHandout(void);
extern void drvDisableHandout(void);
extern void drvEnableEarphout(void);
extern void drvDisableEarphout(void);

extern void drvSelectHandFreeMic(void);
extern void drvSelectHandMic(void);
extern void drvSelectEarphMic(void);

extern int getKeyboardMcuVersion(void);  //2023-01-03
extern int drvGetLCDMcuVersion(void);  //2023-01-03

//2023-02-01 增加对 Yt8521sh 的检测  by zhaodazhi
//返回1表示存在Yt8521sh，其他表示不存在
// misc/mdio-cmd-rk3399.c
extern int drvHasYt8521sh(void);


//2023-02-28 禁止所有mic通道        by zhaodazhi
extern void drvMuteAllMic(void);

#endif /* DRV722_H_ */
