#ifndef  __GOL_NAV_TASK_C__
#define  __GOL_NAV_TASK_C__
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"

#include "string.h"
#include "UartAdapter.h"
#include "drv_usart.h"
#include "nav_task.h"
#include "frame_analysis.h"
#include "gnss.h"
#include "drv_rtc.h"

/*****************kalman algorithm headers**************************/
#include "constant.h"
#include "matrix.h"
#include "inavgnss.h"
#include "inavins.h"
#include "inavlog.h"

CombineDataTypeDef combineData;
QueueHandle_t xNavQueue;
TaskHandle_t task_imu_handler, task_gnss_handler;

uint8_t xQueueStatus;


extern I_NAV_INS g_NAV_INS;
extern I_NAV_GNSS_RESULT g_NAV_GNSS_RESULT;
extern COMPENSATE_PARAMS g_Compensate_Params;

EXPORT_RESULT  g_Export_Result;

EXPORT_RESULT* SaveResult(EXPORT_RESULT* result, I_NAV_INS* nav_ins, I_NAV_GNSS_RESULT* gnssResult)
{
    result->imusecond = nav_ins->imu.second;
    //-----陀螺-----
    result->gyro[0] = nav_ins->imu.gyro[0];
    result->gyro[1] = nav_ins->imu.gyro[1];
    result->gyro[2] = nav_ins->imu.gyro[2];
    //---加速度计---
    result->accm[0] = nav_ins->imu.accm[0];
    result->accm[1] = nav_ins->imu.accm[1];
    result->accm[2] = nav_ins->imu.accm[2];

    result->dt = nav_ins->dt;
    result->att[0] = nav_ins->ins.att[0] * RAD2DEG; //pitch
    result->att[1] = nav_ins->ins.att[1] * RAD2DEG; //roll
    result->att[2] = nav_ins->ins.att[2] * RAD2DEG; //heading

    result->v_n[0] = nav_ins->ins.vn[0];
    result->v_n[1] = nav_ins->ins.vn[1];
    result->v_n[2] = nav_ins->ins.vn[2];

    result->pos[0] = nav_ins->ins.pos[0] * RAD2DEG;
    result->pos[1] = nav_ins->ins.pos[1] * RAD2DEG;
    result->pos[2] = nav_ins->ins.pos[2];

    result->insvnstd[0] = nav_ins->ins.vnstd[0];
    result->insvnstd[1] = nav_ins->ins.vnstd[1];
    result->insvnstd[2] = nav_ins->ins.vnstd[2];

    result->insposstd[0] = nav_ins->ins.posstd[0] * RAD2DEG;
    result->insposstd[1] = nav_ins->ins.posstd[1] * RAD2DEG;
    result->insposstd[2] = nav_ins->ins.posstd[2];

    result->nsv = gnssResult->nsv;
    result->gnsslocatestatus = gnssResult->gnsslocatestatus;
    result->gnssstatus = gnssResult->gnssstatus;
    result->gpsweek = gnssResult->gpsweek;
    result->gpssecond = gnssResult->gpssecond;

    result->pitch = gnssResult->pitch;
    result->roll = gnssResult->roll;
    result->heading = gnssResult->heading;

    result->latitude = gnssResult->latitude;
    result->longitude = gnssResult->longitude;
    result->altitude = gnssResult->altitude;

    result->ve = gnssResult->ve;
    result->vn = gnssResult->vn;
    result->vu = gnssResult->vu;

    result->latstd = gnssResult->latstd;
    result->logstd = gnssResult->logstd;
    result->hstd = gnssResult->hstd;

    result->hdgstddev = gnssResult->hdgstddev;
    result->ptchstddev = gnssResult->ptchstddev;

    result->vestd = gnssResult->vestd;
    result->vnstd = gnssResult->vnstd;
    result->vustd = gnssResult->vustd;

    return result;
}

unsigned int GetCompensateParm(COMPENSATE_PARAMS *p, void *comm)
{
    p->gnssArmLength[0] = 0.0;
    p->gnssArmLength[1] = 0.0;
    p->gnssArmLength[2] = 0.0;
    p->heading = 0.0;
    return 0;
}

void Kalman_smooth(void)
{
    GetNavIncData(&g_NAV_INS.imu, &combineData);
    GetNavGnssData(&g_NAV_GNSS_RESULT, &combineData);
    GetCompensateParm(&g_Compensate_Params, NULL);
    InterfaceKalman();
    SaveResult(&g_Export_Result, &g_NAV_INS, &g_NAV_GNSS_RESULT);
}

void InitialCompensateParm()
{
    memset(&g_Compensate_Params, 0, sizeof(COMPENSATE_PARAMS));
    g_Compensate_Params.gnssArmLength[0] = -0.4188;
    g_Compensate_Params.gnssArmLength[1] = -0.1746;
    g_Compensate_Params.gnssArmLength[2] = -0.099;
}

void imu_notify(void)
{
	xTaskNotify( task_imu_handler, ( 1UL << 0UL ), eSetBits );
}

void gnss_notify(void)
{
	xTaskNotify( task_gnss_handler, ( 1UL << 0UL ), eSetBits );
}

void nav_imu_task(void* arg)
{
    uint32_t ulImuNotifiedValue;
    InitialNavIncParm();
    //初始化静态检查参数
    InitialStaticDetectParm();
    //初始化补偿量
    InitialCompensateParm();
    while( 1 )
    {
        xTaskNotifyWait( 0x00, /* Don't clear any notification bits on entry. */
						0xffffffff, /* Reset the notification value to 0 on exit. */
						&ulImuNotifiedValue, /* Notified value pass out in ulNotifiedValue. */
						portMAX_DELAY ); /* Block indefinitely. */
        {
            if( ( ulImuNotifiedValue & 0x01 ) != 0 )
            {
                memcpy((void*)&combineData.imuInfo.counter, (void*)&imuParseData.counter, sizeof(IMU_PARSE_DATA_TypeDef));
                //Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, sizeof(IMU_PARSE_DATA_TypeDef), (uint8_t*)&combineData.imuInfo.counter);
                //gd32_usart_write((uint8_t*)&combineData.imuInfo.gps_second, sizeof(IMU_PARSE_DATA_TypeDef));
                //卡尔曼滤波
                Kalman_smooth();
            }
        }
    }
}

void nav_gnss_task(void* arg)
{
    
    uint32_t ulGnssNotifiedValue;
    while( 1 )
    {
        xTaskNotifyWait( 0x00, /* Don't clear any notification bits on entry. */
						0xffffffff, /* Reset the notification value to 0 on exit. */
						&ulGnssNotifiedValue, /* Notified value pass out in ulNotifiedValue. */
						portMAX_DELAY ); /* Block indefinitely. */
        {
            if( ( ulGnssNotifiedValue & 0x01 ) != 0 )
            {
                memcpy((void*)&combineData.gnssInfo.timestamp, (void*)&hGPSData.timestamp, sizeof(GPSDataTypeDef));
            }
        }
    }
}

#endif

