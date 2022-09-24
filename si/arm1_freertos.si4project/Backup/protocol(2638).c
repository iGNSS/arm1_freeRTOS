#include "computerFrameParse.h"
#include "data_convert.h"
#include "serial.h"

#include "gnss.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"



union InsGnssData
{
	char  InsGnssDataChar[148*2];
	float InsGnssDataFloat[37*2];
}ins_gnss_data;

struct  InsGipotData
{
	uint16_t 	GPSWeek;		//自 1980-1-6 至当前的星期数（格林尼治时间）
	uint32_t 	GPSTime;		//自本周日 00:00:00 至当前的秒数（格林尼治时间）
	double		Heading;		//偏航角（0~359.99） 
	double		Pitch;			//俯仰角（-90~90）
	double		Roll;			//横滚角（-180~180）
	double		Lattitude;  	//纬度（-90~90）
	double		Longitude;  	//经度（-180~180）
	double		Altitude;  		//高度，单位（米）
	double		Ve;  			//东向速度，单位（米/秒）  
	double		Vn;  			//北向速度，单位（米/秒）
	double		Vu;  			//天向速度，单位（米/秒）
	uint16_t	Baseline;  		//基线长度，单位（米） 
	uint8_t		NSV;  			//卫星数 
	uint8_t		NavStatus;		//>0 表示输出导航数据有效
	uint8_t		GnssStatus;   	//系统状态：0：初始化1：单点定位2：伪距差分4：载波相位差分（定点）5：载波相位差分（浮点）
	
}ins_gipot_data;
