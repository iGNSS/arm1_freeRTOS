#include "computerFrameParse.h"
#include "data_convert.h"
#include "serial.h"

#include "gnss.h"
#include "frame_analysis.h"

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
} ins_gnss_data;

__packed struct InsGipotData
{
    uint16_t 	GPSWeek;		//自 1980-1-6 至当前的星期数（格林尼治时间）
    double 		GPSTime;		//自本周日 00:00:00 至当前的秒数（格林尼治时间）
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

} ins_gipot_data,ins_gpfpd_data;

__packed struct InsGrimuData
{
    uint16_t 	GPSWeek;		//自 1980-1-6 至当前的星期数（格林尼治时间）
    double 		GPSTime;		//自本周日 00:00:00 至当前的秒数（格林尼治时间）
    double		GyroX; 			//陀螺仪 X 轴角速度（°/s）
    double		GyroY; 			//陀螺仪 Y 轴角速度（°/s）
    double		GyroZ; 			//陀螺仪 Z 轴角速度（°/s）
    double		AccelX;  		//加速度计 X 轴加速度（m/s 2 ）
    double		AccelY;  		//加速度计 Y 轴加速度（m/s 2 ）
    double		AccelZ;  		//加速度计 Z 轴加速度（m/s 2 ）
    double		Temp;  			//传感器温度检测输出（℃）

} ins_grimu_data;

typedef __packed struct
{
    char		head1;	  		//帧头  0xAA
    char		head2;	  		//帧头  0x44
    char		head3;	  		//帧头  0x13
    uint8_t		frameLen;		//帧长  不包含帧头和 CRC 校验的帧长度
    uint16_t	frameId;  		//帧号
    uint16_t 	GPSWeek;		//自 1980-1-6 至当前的星期数（格林尼治时间）
    uint32_t 	GPSMilliseconds;//自本周日 00:00:00 至当前的毫秒数（格林尼治时间）
} RawimuHeader;
__packed struct InsRawimuData
{
    RawimuHeader	header;
    uint16_t 		GPSWeek;		//自 1980-1-6 至当前的星期数（格林尼治时间）
    uint32_t 		GPSTime;		//自本周日 00:00:00 至当前的秒数（格林尼治时间）
    long  			IMUStatus; 		//状态	//0X00000001 X 陀螺状态 1：正常 0：异常
    //0X00000002 Y 陀螺状态 1：正常 0：异常
    //0X00000004 Z 陀螺状态 1：正常 0：异常
    //0X00000010 X 加表状态 1：正常 0：异常
    //0X00000020 Y 加表状态 1：正常 0：异常
    //0X00000040 Z 加表状态 1：正常 0：异常
    long  			AccelZ; 		//加表Z输出
    long  			AccelY; 		//加表Y输出
    long  			AccelX; 		//加表X输出
    long			GyroZ; 			//陀螺仪Z输出
    long			GyroY; 			//陀螺仪Y输出
    long			GyroX;			//陀螺仪X输出
    uint32_t		CRC32;			//CRC 校验  32-bitCRC 校验
} ins_rawimu_data;

void protocol_fillGipotData(void* pagric, void* pheading)
{
    GPS_AGRIC_TypeDef* agric = (GPS_AGRIC_TypeDef*)pagric;
    GNSS_Heading_DataTypeDef* heading = (GNSS_Heading_DataTypeDef*)pheading;

    ins_gipot_data.GPSWeek = agric->header.gpsWn;
    ins_gipot_data.GPSTime = agric->header.gpsMs*0.001;
    ins_gipot_data.Heading = agric->Heading;
    ins_gipot_data.Pitch = agric->pitch;
    ins_gipot_data.Roll = agric->roll;
    ins_gipot_data.Lattitude = agric->lat;
    ins_gipot_data.Longitude = agric->lon;
    ins_gipot_data.Altitude = agric->alt;
    ins_gipot_data.Ve = agric->vE;
    ins_gipot_data.Vn = agric->vN;
    ins_gipot_data.Vu = agric->vZ;
    ins_gipot_data.Baseline = heading->baseLen;
    ins_gipot_data.NSV = agric->num_bd_star;
    ins_gipot_data.NSV += agric->num_gps_star;
    ins_gipot_data.NSV += agric->num_gal_star;
    ins_gipot_data.NSV += agric->num_glo_star;
    ins_gipot_data.GnssStatus = agric->rtk_status;
    ins_gipot_data.NavStatus = ins_gipot_data.GnssStatus;

    memcpy((void*)&ins_gpfpd_data.GPSWeek, (void*)&ins_gipot_data.GPSWeek, sizeof(struct InsGipotData));
}

void protocol_fillGrimuData(void* pagric)
{
    GPS_AGRIC_TypeDef* agric = (GPS_AGRIC_TypeDef*)pagric;

    ins_grimu_data.GPSWeek = agric->header.gpsWn;
    ins_grimu_data.GPSTime = agric->header.gpsMs*0.001;
    ins_grimu_data.GyroX   = imuParseData.gyroGrp[0];
    ins_grimu_data.GyroY   = imuParseData.gyroGrp[1];
    ins_grimu_data.GyroZ   = imuParseData.gyroGrp[2];
    ins_grimu_data.AccelX  = imuParseData.accelGrp[0];
    ins_grimu_data.AccelY  = imuParseData.accelGrp[1];
    ins_grimu_data.AccelZ  = imuParseData.accelGrp[2];
    ins_grimu_data.Temp    = imuParseData.sensorTemp;
}


