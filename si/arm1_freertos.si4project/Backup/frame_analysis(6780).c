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
IMU_DATA_TypeDef imu_info;
#ifdef configUse_naviTest
navi_test_t	navi_test_info;
char navi_test_str[200] = {0,};
void navi_test_pack(void)
{
    sprintf(navi_test_str, "%d,%0.3f,%x,%x,%x,%x,%x,%x,%0.3f,%d,%x,%x,,%d,,%d,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,", \
            navi_test_info.counter, navi_test_info.timestamp, navi_test_info.accelX, navi_test_info.accelY, navi_test_info.accelZ, \
            navi_test_info.gyroX, navi_test_info.gyroY, navi_test_info.gyroZ, navi_test_info.sensor_temp, navi_test_info.gps_valid, \
            navi_test_info.gpsWeek, navi_test_info.gpsSec, navi_test_info.starNum,navi_test_info.rtkStatus,navi_test_info.lon, \
            navi_test_info.lat,navi_test_info.alt,navi_test_info.vu,navi_test_info.heading,navi_test_info.pitch,navi_test_info.lon_std, \
            navi_test_info.lat_std,navi_test_info.alt_std,navi_test_info.hdgstddev,navi_test_info.ptchstddev,navi_test_info.hdop);
    Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, strlen(navi_test_str), (uint8_t*)&navi_test_info.counter);
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
                navi_test_pack();
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

    uint8_t xor;
    static uint8_t pull_couter = 0;
    double temp;
    //DEV_StatusTypedef status;
    EXPORT_RESULT* result = (EXPORT_RESULT*)imu;
    GPSDataTypeDef* gnss = (GPSDataTypeDef*)gps;

    rs422_frame.data_stream.header[0] = RS422_FRAME_HEADER_L;
    rs422_frame.data_stream.header[1] = RS422_FRAME_HEADER_M;
    rs422_frame.data_stream.header[2] = RS422_FRAME_HEADER_H;

    rs422_frame.data_stream.pitch = (short)(result->att[0]/Angle_Scale*Sensor_Scale);
    rs422_frame.data_stream.roll = (short)(result->att[1]/Angle_Scale*Sensor_Scale);
    rs422_frame.data_stream.azimuth = (short)(result->att[2]/Angle_Scale*Sensor_Scale);

    rs422_frame.data_stream.gyroX = (short)(result->gyro[0]/Rate_Scale*Sensor_Scale);
    rs422_frame.data_stream.gyroY = (short)(result->gyro[1]/Rate_Scale*Sensor_Scale);
    rs422_frame.data_stream.gyroZ = (short)(result->gyro[2]/Rate_Scale*Sensor_Scale);

    rs422_frame.data_stream.accelX = (short)(result->accm[0]/Accel_Scale*Sensor_Scale);
    rs422_frame.data_stream.accelY = (short)(result->accm[1]/Accel_Scale*Sensor_Scale);
    rs422_frame.data_stream.accelZ = (short)(result->accm[2]/Accel_Scale*Sensor_Scale);

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

