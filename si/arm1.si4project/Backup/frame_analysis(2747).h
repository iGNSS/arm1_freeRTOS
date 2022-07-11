#ifndef __FRAME_ANALYSIS_H__
#define __FRAME_ANALYSIS_H__

#include "gd32f4xx.h"
#include "insdef.h"


#define	RS422_FRAME_LENGTH	200

typedef struct poll_data
{
    uint16_t	data1;
    uint16_t 	data2;
    uint16_t	data3;
    uint32_t	utc;
    uint8_t		type;
} POLL_DATA, pPOLL_DATA;

typedef union rs422_frame_define
{
    struct
    {
        uint8_t 			header[3];	//0xbd,0xdb,0x0b
        uint16_t 			roll;		//横滚角
        uint16_t 			pitch;		//俯仰角
        uint16_t			azimuth;	//方位角
        uint16_t 			gyroX;		//陀螺x轴
        uint16_t 			gyroY;		//陀螺y轴
        uint16_t			gyroZ;		//陀螺z轴
        uint16_t 			accelX;		//加表x轴
        uint16_t 			accelY;		//加表y轴
        uint16_t			accelZ;		//加表z轴
        uint16_t			latitude;	//纬度
        uint16_t			longitude;	//经度
        uint16_t			altitude;	//高度
        uint16_t			speed_N;	//北向速度
        uint16_t			speed_E;	//东向速度
        uint16_t			speed_G;	//地向速度
        uint8_t				status;
        struct poll_data	poll_frame[6];
        uint8_t				xor_verify1;
        uint32_t			gps_week;
        uint8_t				xor_verify2;

    } data_stream;
    uint16_t fpga_cache[RS422_FRAME_LENGTH];
} RS422_FRAME_DEF, *pRS422_FRAME_DEF;




#endif

