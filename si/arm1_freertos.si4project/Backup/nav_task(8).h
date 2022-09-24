#ifndef NAV_TASK_H_INCLUDED
#define NAV_TASK_H_INCLUDED

#undef COMMON_EXT
#ifdef  __GOL_NAV_TASK_C__
    #define COMMON_EXT
#else
    #define COMMON_EXT extern
#endif	

#ifdef __cplusplus
extern "C" {
#endif

#include "insdef.h"
#include "gd32f4xx.h"
#include "insdef.h"
#include "gd32f4xx.h"
#include "frame_analysis.h"
#include "gnss.h"


#include "FreeRTOS.h"
#include "semphr.h"

typedef struct EXPORT_RESULT_T {
	//imu
	double  imusecond;
	double 	gyro[3];
	double 	accm[3];

	//ins 姿态 速度 位置 速度精度 位置精度
	double dt;
	double att[3]; //pitch roll heading
	double v_n[3];
	double pos[3];
	double insvnstd[3];
	double insposstd[3];

	//GNSS
	unsigned int	nsv;//收星数
	unsigned int	gnsslocatestatus;//定位状态
	unsigned int	gnssstatus;//系统状态：0： 失锁1： 单点定位2： 伪距差分4： 载波相位差分（ 定点）5： 载波相位差分（ 浮点）
	unsigned int	gpsweek; //gps周自 1980-1-6 至当前的星期数（ 格林尼治时间） 
	float 		gpssecond;
	float 		heading;//偏航角（ 0~359.99） 
	float		pitch;//俯仰角（ -90~90） 
	float		roll;//横滚角（ -180~180） 
	float		latitude;//纬度（ -90~90） 
	float		longitude;//经度（ -180~180）
	float		altitude;//高度， 单位（ 米） 
	float		ve;//东向速度， 单位（ 米/秒） 
	float 		vn;//北向速度， 单位（ 米/秒）
	float		vu;//天向速度， 单位（ 米/秒）

	//精度信息
	//定位精度
	double		latstd;//纬度精度
	double		logstd;//经度精度
	double		hstd;//高程精度

	double hdgstddev;					/* 航向角标准差, 单位： m*/
	double ptchstddev;
	
	//定速精度
	double		vestd;//东向速度精度
	double		vnstd;//北向速度精度
	double		vustd;//天向速度精度	
}EXPORT_RESULT;


typedef struct
{
    IMU_PARSE_DATA_TypeDef 	imuInfo;
    GPSDataTypeDef			gnssInfo;
} CombineDataTypeDef;


COMMON_EXT SemaphoreHandle_t xnvaTaskSemaphore;
COMMON_EXT QueueHandle_t xNavQueue;
COMMON_EXT uint8_t xQueueStatus;


COMMON_EXT void nav_imu_task(void* arg);
COMMON_EXT void nav_gnss_task(void* arg);
COMMON_EXT void imu_notify(void);
COMMON_EXT void gnss_notify(void);


#ifdef __cplusplus
}
#endif

#endif 




