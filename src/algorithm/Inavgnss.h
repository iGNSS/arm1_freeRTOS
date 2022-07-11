/*****************************************************************************************
All rights reserved I-NAV 2022 2023
***********************************************************************************/

/***********************************************************************************
Modification history

|-----------+---------------+-------------------|
|Author          |    Date               |    Done                      |
|-----------+---------------+-------------------|
|DengWei       |  2022-6-8          | First Creation             |
|-----------+---------------+-------------------|  
***********************************************************************************/
#ifndef _INAV_GNSS_H
#define _INAV_GNSS_H


enum ENavGnssStatus 
{
    NAV_GNSS_STATUS_LOST =0,       	//失锁
    NAV_GNSS_STATUS_SPP=1,			//单点定位
    NAV_GNSS_STATUS_RTD=2,		//伪距差分
    NAV_GNSS_STATUS_RTK_FIX=4,	      //RTK固定解
    NAV_GNSS_STATUS_RTK_FLOAT=5,	//RTK浮点解
};


#include "constant.h"
// GNSS原始数据
typedef struct I_NAV_GNSS_RESULT_T {
	unsigned int	gpsweek; //gps周自 1980-1-6 至当前的星期数（ 格林尼治时间） 
	float 		gpssecond;//gps周内秒自本周日 00:00:00 至当前的秒数（ 格林尼治时间）
	float 		heading;//偏航角（ 0~359.99） 
	float			pitch;//俯仰角（ -90~90） 
	float			roll;//横滚角（ -180~180） 
	float			latitude;//纬度（ -90~90） 
	float			longitude;//经度（ -180~180）
	float			altitude;//高度， 单位（ 米） 
	float			ve;//东向速度， 单位（ 米/秒） 
	float 		vn;//北向速度， 单位（ 米/秒）
	float			vu;//天向速度， 单位（ 米/秒）
	float			baseline;//基线长度， 单位（ 米） 
	unsigned int	nsv;//卫星数 
	unsigned int	gnsslocatestatus;
	unsigned int	gnssstatus;//系统状态：0： 失锁： 单点定位2： 伪距差分4： 载波相位差分（ 定点）5： 载波相位差分（ 浮点）
	double		utc;//UTC 时间

	//是否提供准确定位精度定速误差
	unsigned char supportposvelstd; //0不支持，1支持
	//精度信息
	//定位精度
	double		latstd;//纬度精度
	double		logstd;//经度精度
	double		hstd;//高程精度
	//定速精度
	double		vestd;//东向速度精度
	double		vnstd;//北向速度精度
	double		vustd;//天向速度精度

	//gnss启动标志
	unsigned int	gnssstartflag;// 1: 已经可以获取gnss结果
		
}I_NAV_GNSS_RESULT;


#ifdef __cplusplus
extern "C" {
#endif
void InitialGnssResultParm(void);
I_NAV_GNSS_RESULT *GetGnssResultPointer(void);

#ifdef __cplusplus
}
#endif

#endif
