#ifndef __FRAME_ANALYSIS_H__
#define __FRAME_ANALYSIS_H__

#include "gd32f4xx.h"
#include "insdef.h"

#undef COMMON_EXT
#ifdef  __GOL_FRAME_ANALYSIS_C__
#define COMMON_EXT
#else
#define COMMON_EXT extern
#endif


#define RS422_FRAME_HEADER_L                  0xBD
#define RS422_FRAME_HEADER_M                  0xDB
#define RS422_FRAME_HEADER_H                  0x0B

#define	RS422_FRAME_LENGTH	100

typedef struct imu_data
{
    uint8_t syn_low;
    uint8_t syn_high;
    uint8_t accelX_l;
    uint8_t accelX_h;
    uint8_t accelY_l;
    uint8_t accelY_h;
    uint8_t accelZ_l;
    uint8_t accelZ_h;
    uint8_t gyroX_l;
    uint8_t gyroX_h;
    uint8_t gyroY_l;
    uint8_t gyroY_h;
    uint8_t gyroZ_l;
    uint8_t gyroZ_h;
    uint8_t roll_l;
    uint8_t roll_h;
    uint8_t pitch_l;
    uint8_t pitch_h;
    uint8_t	azimuth_l;
    uint8_t	azimuth_h;
    uint8_t	sensor_temp_l;
    uint8_t	sensor_temp_h;
    uint8_t crc;
} IMU_DATA_TypeDef;

typedef enum poll_data_type
{
    locating_info_prec = 0,
    speed_info_prec = 1,
    pos_info_prec = 2,
    dev_inter_temp = 22,
    gps_status     = 32,
    rotate_status = 33
} POLL_DATA_TypeDef;

typedef __packed struct poll_data
{
    uint16_t	data1;
    uint16_t 	data2;
    uint16_t	data3;
    uint32_t	gps_time;
    uint8_t		type;
} POLL_DATA, *pPOLL_DATA;

typedef union
{
    struct
    {
        uint8_t pos:1;
        uint8_t spd:1;
        uint8_t posture:1;
        uint8_t courseAngle:1;
        uint8_t hold:4;
    } statusBits;
    uint8_t dev_status;
} DEV_StatusTypedef;

typedef __packed struct
{
    uint8_t 			header[3];	//0xbd,0xdb,0x0b
    short 				roll;		//横滚角
    short 				pitch;		//俯仰角
    short				azimuth;	//方位角
    short 				gyroX;		//陀螺x轴
    short 				gyroY;		//陀螺y轴
    short				gyroZ;		//陀螺z轴
    short 				accelX;		//加表x轴
    short 				accelY;		//加表y轴
    short				accelZ;		//加表z轴
    long				latitude;	//纬度
    long				longitude;	//经度
    long				altitude;	//高度
    short				vn;			//北向速度
    short				ve;			//东向速度
    short				vu;			//地向速度
    uint8_t				status;		//bit0:位置 bit1:速度 bit2:姿态 bit3:航向角
    uint8_t				reserved[6];
    POLL_DATA			poll_frame;
    uint8_t				xor_verify1;
    uint32_t			gps_week;
    uint8_t				xor_verify2;

} DATA_STREAM;

typedef union rs422_frame_define
{
    DATA_STREAM data_stream;
    uint8_t fpga_cache[RS422_FRAME_LENGTH];
} RS422_FRAME_DEF, *pRS422_FRAME_DEF;

typedef struct
{
    float accelGrp[3];
    float gyroGrp[3];
    float roll;
    float pitch;
    float azimuth;
    float latitude;
    float longitude;
    float altitude;
    float gps_sec;//周内秒
} MIX_DATA_TypeDef;

typedef struct
{
    //float gps_second;	//周内秒
    uint8_t counter;
    double accelGrp[3];	//加速度X,Y,Z轴
    double gyroGrp[3];	//角速度X,Y,Z轴
    double roll;			//横滚角
    double pitch;		//俯仰角
    double azimuth;		//航向角
    double sensorTemp;	//传感器温度
} IMU_PARSE_DATA_TypeDef;

COMMON_EXT IMU_PARSE_DATA_TypeDef imuParseData;

//写数据到DRAM
void frame_writeDram(void);
void frame_pack_and_send(void* imu, void *gps);
void frame_init(void);
uint8_t frame_fill_imu(uint8_t* pData, uint16_t dataLen);

MIX_DATA_TypeDef* frame_get_ptr(void);


#endif

