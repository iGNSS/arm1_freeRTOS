#ifndef __GOL_FRAME_ANALYSIS_C__
#define __GOL_FRAME_ANALYSIS_C__

#include "string.h"
#include "math.h"
#include "frame_analysis.h"
#include "pin_numbers_def.h"

#include "drv_gpio.h"
#include "drv_rtc.h"

#include "serial.h"
#include "exmc_sram.h"
#include "uartadapter.h"
#include "gnss.h"
#include "DRamAdapter.h"
#include "computerFrameParse.h"
#include "inavins.h"
#include "inavgnss.h"
#include "nav_task.h"


RS422_FRAME_DEF	rs422_frame;



uint8_t xor_check(uint8_t *buf, uint16_t len)
{
    uint16_t i = 0;
    uint16_t x = 0;

    for(; i < len; i++)
    {
        x = x ^ (*(buf + i));
    }

    return x;
}

uint8_t sum_check(uint8_t *buf, uint16_t len )
{
    uint8_t checksum = 0, i;

    for (i = 0; i < len; i++)
    {
        checksum += buf[i];
    }

    checksum  = ~checksum;

    return checksum;

}
//参考PA-IMU-460.PDF
#define IMU_FRAME_HEADER_LSB                  0x7F
#define IMU_FRAME_HEADER_MSB                  0x80
//IMU_DATA_TypeDef imu_info;
#define	AXIS_INITIAL_X	1
#define	AXIS_INITIAL_Y	2
#define	AXIS_INITIAL_Z	3
typedef struct
{
    uint8_t index;
    int8_t	axis[3];
} AXIS_TypeDef;
AXIS_TypeDef axisInfo;
const AXIS_TypeDef axisTab[] =
{
    {0,AXIS_INITIAL_X,AXIS_INITIAL_Y,AXIS_INITIAL_Z},
    {1,AXIS_INITIAL_X,AXIS_INITIAL_Y,AXIS_INITIAL_Z},
    {2,AXIS_INITIAL_X,-AXIS_INITIAL_Y,-AXIS_INITIAL_Z},
    {3,AXIS_INITIAL_X,AXIS_INITIAL_Z,-AXIS_INITIAL_Y},
    {4,AXIS_INITIAL_X,-AXIS_INITIAL_Z,AXIS_INITIAL_Y},
    {5,-AXIS_INITIAL_X,AXIS_INITIAL_Y,-AXIS_INITIAL_Z},
    {6,-AXIS_INITIAL_X,-AXIS_INITIAL_Y,AXIS_INITIAL_Z},
    {7,-AXIS_INITIAL_X,AXIS_INITIAL_Z,AXIS_INITIAL_Y},
    {8,-AXIS_INITIAL_X,-AXIS_INITIAL_Z,-AXIS_INITIAL_Y},
    {9,AXIS_INITIAL_Y,AXIS_INITIAL_X,-AXIS_INITIAL_Z},
    {10,AXIS_INITIAL_Y,-AXIS_INITIAL_X,AXIS_INITIAL_Z},
    {11,AXIS_INITIAL_Y,AXIS_INITIAL_Z,AXIS_INITIAL_X},
    {12,AXIS_INITIAL_Y,-AXIS_INITIAL_Z,-AXIS_INITIAL_X},
    {13,-AXIS_INITIAL_Y,AXIS_INITIAL_X,AXIS_INITIAL_Z},
    {14,-AXIS_INITIAL_Y,-AXIS_INITIAL_X,-AXIS_INITIAL_Z},
    {15,-AXIS_INITIAL_Y,AXIS_INITIAL_Z,-AXIS_INITIAL_X},
    {16,-AXIS_INITIAL_Y,-AXIS_INITIAL_Z,AXIS_INITIAL_X},
    {17,AXIS_INITIAL_Z,AXIS_INITIAL_X,AXIS_INITIAL_Y},
    {18,AXIS_INITIAL_Z,-AXIS_INITIAL_X,-AXIS_INITIAL_Y},
    {19,AXIS_INITIAL_Z,AXIS_INITIAL_Y,-AXIS_INITIAL_X},
    {20,AXIS_INITIAL_Z,-AXIS_INITIAL_Y,AXIS_INITIAL_X},
    {21,-AXIS_INITIAL_Z,AXIS_INITIAL_X,-AXIS_INITIAL_Y},
    {22,-AXIS_INITIAL_Z,-AXIS_INITIAL_X,AXIS_INITIAL_Y},
    {23,-AXIS_INITIAL_Z,AXIS_INITIAL_Y,AXIS_INITIAL_X},
    {24,-AXIS_INITIAL_Z,-AXIS_INITIAL_Y,-AXIS_INITIAL_X}
};

#ifdef configUse_naviTest
extern uint8_t navi_test_statusRd(void);
navi_test_t	navi_test_info;
char navi_test_str[300] = {0,};
uint8_t navi_test_sendBuf[300];
void navi_test_pack_send(void)
{
    uint16_t len;
    uint8_t id;
    rtc_update_struct* rtc;

    axisInfo.index = comm_axis_read();
    axisInfo.axis[0] = axisTab[axisInfo.index].axis[0];
    axisInfo.axis[1] = axisTab[axisInfo.index].axis[1];
    axisInfo.axis[2] = axisTab[axisInfo.index].axis[2];

    id = abs(axisInfo.axis[0]);
    id -= 1;
    if(axisInfo.axis[0] > 0)
    {
        navi_test_info.accelX = imuParseData.accelGrp[id];
        navi_test_info.gyroX  = imuParseData.gyroGrp[id];
    }
    else
    {
        navi_test_info.accelX = -imuParseData.accelGrp[id];
        navi_test_info.gyroX  = -imuParseData.gyroGrp[id];
    }
    id = abs(axisInfo.axis[1]);
    id -= 1;
    if(axisInfo.axis[1] > 0)
    {
        navi_test_info.accelY = imuParseData.accelGrp[id];
        navi_test_info.gyroY  = imuParseData.gyroGrp[id];
    }
    else
    {
        navi_test_info.accelY = -imuParseData.accelGrp[id];
        navi_test_info.gyroY  = -imuParseData.gyroGrp[id];
    }
    id = abs(axisInfo.axis[2]);
    id -= 1;
    if(axisInfo.axis[2] > 0)
    {
        navi_test_info.accelZ = imuParseData.accelGrp[id];
        navi_test_info.gyroZ  = imuParseData.gyroGrp[id];
    }
    else
    {
        navi_test_info.accelZ = -imuParseData.accelGrp[id];
        navi_test_info.gyroZ  = -imuParseData.gyroGrp[id];
    }
    navi_test_info.sensor_temp = imuParseData.sensorTemp;
    if(navi_test_info.gps_valid == 0)navi_test_info.timestamp = 0.0;
    rtc = rtc_update();
    navi_test_info.imuTimestamp = rtc->gpsTime;
//    sprintf(navi_test_str, "#rawdata,%d,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%d,%d,%d,%d,%d,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f\r\n", \
//            navi_test_info.counter, navi_test_info.imuTimestamp, navi_test_info.timestamp, navi_test_info.accelX, navi_test_info.accelY, navi_test_info.accelZ, \
//            navi_test_info.gyroX, navi_test_info.gyroY, navi_test_info.gyroZ, navi_test_info.sensor_temp, navi_test_info.gps_valid, \
//            navi_test_info.gpsWeek, navi_test_info.gpsSec, navi_test_info.starNum,navi_test_info.rtkStatus,navi_test_info.lon, \
//            navi_test_info.lat,navi_test_info.alt,navi_test_info.vu,navi_test_info.heading,navi_test_info.pitch,navi_test_info.lon_std, \
//            navi_test_info.lat_std,navi_test_info.alt_std,navi_test_info.hdgstddev,navi_test_info.ptchstddev,navi_test_info.hdop);
    sprintf(navi_test_str, "#rawdata,%d,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f,%0.5f\r\n", \
            navi_test_info.counter, navi_test_info.accelX, navi_test_info.accelY, navi_test_info.accelZ, \
            navi_test_info.gyroX, navi_test_info.gyroY, navi_test_info.gyroZ);
    len = strlen(navi_test_str);
    memcpy((void*)&navi_test_sendBuf[0], (void*)navi_test_str, len);
    Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, len, navi_test_sendBuf);
    navi_test_info.gps_valid = 0;
    navi_test_info.counter++;
}
#endif
extern void Oscilloscope(MIX_DATA_TypeDef* rs422);
uint8_t frame_fill_imu(uint8_t* pData, uint16_t dataLen)
{
#define	Accel_Scale 	20.0
#define	Rate_Scale 		1260.0
#define	Angle_Scale 	360.0
#define	Temp_Scale		200.0
#define	Sensor_Scale 	65536.0

    uint8_t  sum, calSum;
    uint8_t  data_l, data_h;
    short	 temp;
    uint8_t  length = 0;
    uint8_t  ret = INS_ERROR;
    uint8_t  *p = pData;

    while( 1 )
    {
        data_l = *p++;
        data_h = *p++;
        length ++;

        if((IMU_FRAME_HEADER_LSB == data_l) && (IMU_FRAME_HEADER_MSB == data_h))
        {
            sum = sum_check(p, 20 );
            calSum = *(p + 20);

            if(sum == calSum)
            {

                memcpy((void*)&imu_info.syn_low, p-2, sizeof(IMU_DATA_TypeDef));

                temp = *p++;
                temp |= (*p++) << 8;
                imuParseData.accelGrp[0] = temp * Accel_Scale / Sensor_Scale;
                temp = *p++;
                temp |= (*p++) << 8;
                imuParseData.accelGrp[1] = temp * Accel_Scale / Sensor_Scale;
                temp = *p++;
                temp |= (*p++) << 8;
                imuParseData.accelGrp[2] = temp * Accel_Scale / Sensor_Scale;

                temp = *p++;
                temp |= (*p++) << 8;
                imuParseData.gyroGrp[0] = temp * Rate_Scale / Sensor_Scale;
                temp = *p++;
                temp |= (*p++) << 8;
                imuParseData.gyroGrp[1] = temp * Rate_Scale / Sensor_Scale;
                temp = *p++;
                temp |= (*p++) << 8;
                imuParseData.gyroGrp[2] = temp * Rate_Scale / Sensor_Scale;

                temp = *p++;
                temp |= (*p++) << 8;
                imuParseData.roll = temp * Angle_Scale / Sensor_Scale;
                temp = *p++;
                temp |= (*p++) << 8;
                imuParseData.pitch = temp * Angle_Scale / Sensor_Scale;
                temp = *p++;
                temp |= (*p++) << 8;
                imuParseData.azimuth = temp * Angle_Scale / Sensor_Scale;

                temp = *p++;
                temp |= (*p++) << 8;
                imuParseData.sensorTemp = temp * Temp_Scale / Sensor_Scale;

                imuParseData.counter++;
#ifdef configUse_naviTest
                if(navi_test_statusRd())
                    navi_test_pack_send();
#endif
                break;
            }

        }

        if(length > dataLen)break;
    }

#undef	Accel_Scale
#undef	Rate_Scale
#undef	Angle_Scale
#undef	Temp_Scale
#undef	Sensor_Scale
    return ret;
}


//数据封包
void frame_pack_and_send(void* imu, void *gps)
{
#define	Accel_Scale 	12
#define	Rate_Scale 		300
#define	Angle_Scale 	360
#define	Temp_Scale		200
#define	Sensor_Scale 	32768
#define EXP_E    		2.718282

    uint8_t xor,id;
    static uint8_t pull_couter = 0;
    double temp;
    short gyroTemp[3],accelTemp[3];
    //DEV_StatusTypedef status;
    EXPORT_RESULT* result = (EXPORT_RESULT*)imu;
    GPSDataTypeDef* gnss = (GPSDataTypeDef*)gps;

    rs422_frame.data_stream.header[0] = RS422_FRAME_HEADER_L;
    rs422_frame.data_stream.header[1] = RS422_FRAME_HEADER_M;
    rs422_frame.data_stream.header[2] = RS422_FRAME_HEADER_H;

    rs422_frame.data_stream.pitch = (short)(result->att[0]/Angle_Scale*Sensor_Scale);
    rs422_frame.data_stream.roll = (short)(result->att[1]/Angle_Scale*Sensor_Scale);
    rs422_frame.data_stream.azimuth = (short)(result->att[2]/Angle_Scale*Sensor_Scale);

    gyroTemp[0] = (short)(result->gyro[0]/Rate_Scale*Sensor_Scale);
    gyroTemp[1] = (short)(result->gyro[1]/Rate_Scale*Sensor_Scale);
    gyroTemp[2] = (short)(result->gyro[2]/Rate_Scale*Sensor_Scale);

    accelTemp[0] = (short)(result->accm[0]/Accel_Scale*Sensor_Scale);
    accelTemp[1] = (short)(result->accm[1]/Accel_Scale*Sensor_Scale);
    accelTemp[2] = (short)(result->accm[2]/Accel_Scale*Sensor_Scale);
    //////////////////////////////////////////////////////////
    axisInfo.index = comm_axis_read();
    axisInfo.axis[0] = axisTab[axisInfo.index].axis[0];
    axisInfo.axis[1] = axisTab[axisInfo.index].axis[1];
    axisInfo.axis[2] = axisTab[axisInfo.index].axis[2];

    id = abs(axisInfo.axis[0]);
    id -= 1;
    if(axisInfo.axis[0] > 0)
    {
        rs422_frame.data_stream.accelX = accelTemp[id];
        rs422_frame.data_stream.gyroX  = gyroTemp[id];
    }
    else
    {
        rs422_frame.data_stream.accelX = -accelTemp[id];
        rs422_frame.data_stream.gyroX  = -gyroTemp[id];
    }
    id = abs(axisInfo.axis[1]);
    id -= 1;
    if(axisInfo.axis[1] > 0)
    {
        rs422_frame.data_stream.accelY = accelTemp[id];
        rs422_frame.data_stream.gyroY  = gyroTemp[id];
    }
    else
    {
        rs422_frame.data_stream.accelY = -accelTemp[id];
        rs422_frame.data_stream.gyroY  = -gyroTemp[id];
    }
    id = abs(axisInfo.axis[2]);
    id -= 1;
    if(axisInfo.axis[2] > 0)
    {
        rs422_frame.data_stream.accelZ = accelTemp[id];
        rs422_frame.data_stream.gyroZ  = gyroTemp[id];
    }
    else
    {
        rs422_frame.data_stream.accelZ = -accelTemp[id];
        rs422_frame.data_stream.gyroZ  = -gyroTemp[id];
    }
    ////////////////////////////////////////////////////////////////
    rs422_frame.data_stream.latitude = (long)(result->latitude * 10000000);
    rs422_frame.data_stream.longitude = (long)(result->longitude * 10000000);
    rs422_frame.data_stream.altitude = (long)(result->altitude * 1000);

    temp = log(2) / log(EXP_E);
    rs422_frame.data_stream.vn = (short)(result->vn / temp * Sensor_Scale);
    rs422_frame.data_stream.ve = (short)(result->ve / temp * Sensor_Scale);
    rs422_frame.data_stream.vu = (short)(result->vu / temp * Sensor_Scale);

    rs422_frame.data_stream.status = 0;
    if(gnss->ResolveState[0] == 0)
        rs422_frame.data_stream.status |= 0x1;
    else
        rs422_frame.data_stream.status &= ~0x1;
    if(gnss->ResolveState[1] == 0)
        rs422_frame.data_stream.status |= 0x2;
    else
        rs422_frame.data_stream.status &= ~0x2;
    if(gnss->ResolveState[2] == 0)
        rs422_frame.data_stream.status |= 0x4;
    else
        rs422_frame.data_stream.status &= ~0x4;
    //pull_couter = 3;
    switch(pull_couter)
    {
    case 0:
    {
        rs422_frame.data_stream.poll_frame.type = locating_info_prec;
        temp = result->latstd / 100;
        rs422_frame.data_stream.poll_frame.data1 = exp(temp);
        temp = result->logstd / 100;
        rs422_frame.data_stream.poll_frame.data2 = exp(temp);
        temp = result->hstd / 100;
        rs422_frame.data_stream.poll_frame.data3 = exp(temp);
    }
    break;
    case 1:
    {
        rs422_frame.data_stream.poll_frame.type = speed_info_prec;
        temp = result->vestd / 100;
        rs422_frame.data_stream.poll_frame.data1 = exp(temp);
        temp = result->vnstd / 100;
        rs422_frame.data_stream.poll_frame.data2 = exp(temp);
        temp = result->vustd / 100;
        rs422_frame.data_stream.poll_frame.data3 = exp(temp);
    }
    break;
    case 2:
    {
        rs422_frame.data_stream.poll_frame.type = pos_info_prec;
        //temp = result->vestd / 100;
        //rs422_frame.data_stream.poll_frame.data1 = exp(temp);
        temp = result->ptchstddev / 100;
        rs422_frame.data_stream.poll_frame.data2 = exp(temp);
        temp = result->hdgstddev / 100;
        rs422_frame.data_stream.poll_frame.data3 = exp(temp);
    }
    break;
    case 3:
    {
        rs422_frame.data_stream.poll_frame.type = dev_inter_temp;
        rs422_frame.data_stream.poll_frame.data1 = imuParseData.sensorTemp * Sensor_Scale / Temp_Scale;
        rs422_frame.data_stream.poll_frame.data2 = 0;
        rs422_frame.data_stream.poll_frame.data3 = 0;
    }
    break;
    case 4:
    {
        rs422_frame.data_stream.poll_frame.type = gps_status;
        rs422_frame.data_stream.poll_frame.data1 = result->gnsslocatestatus;
        rs422_frame.data_stream.poll_frame.data2 = result->nsv;
        rs422_frame.data_stream.poll_frame.data3 = result->gnssstatus;
    }
    break;
    case 5:
    {
        rs422_frame.data_stream.poll_frame.type = rotate_status;
        rs422_frame.data_stream.poll_frame.data1 = 0;
        rs422_frame.data_stream.poll_frame.data2 = 0;
        rs422_frame.data_stream.poll_frame.data3 = 0;
    }
    break;
    default:
        break;
    }
    pull_couter++;
    if(pull_couter > 5)pull_couter = 0;
    rs422_frame.data_stream.poll_frame.gps_time = result->gpssecond * 4;
    rs422_frame.data_stream.gps_week = result->gpsweek;

    xor = xor_check(rs422_frame.data_stream.header, sizeof(rs422_frame.data_stream) - 6 );
    rs422_frame.data_stream.xor_verify1 = xor;

    xor = xor_check(rs422_frame.data_stream.header, sizeof(rs422_frame.data_stream) - 1 );
    rs422_frame.data_stream.xor_verify2 = xor;

    //if(INS_EOK == gnss_isLocation())
    {
        //gd32_usart_write((uint8_t*)&rs422_frame.data_stream.header[0],sizeof(rs422_frame.data_stream));
        Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, sizeof(rs422_frame.data_stream), (uint8_t*)&rs422_frame.data_stream.header[0]);
    }
#undef	Accel_Scale
#undef	Rate_Scale
#undef	Angle_Scale
#undef	Temp_Scale
#undef	Sensor_Scale
#undef	EXP_E

}

//写数据到DRAM
void frame_writeDram(void)
{
    DRam_Write(0, (uint16_t*)rs422_frame.fpga_cache, RS422_FRAME_LENGTH/2);
}

void frame_init(void)
{
    memset(rs422_frame.fpga_cache, 0x0, sizeof(rs422_frame.fpga_cache));
}


#endif

