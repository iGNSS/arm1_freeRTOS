#ifndef __GOL_GNSS_C__
#define __GOL_GNSS_C__

#include "gnss.h"
#include "insdef.h"
#include "Time_Unify.h"
#include "arm_math.h"
#include "nav_task.h"
#include "DRamAdapter.h"

#include "drv_usart.h"
#include "main.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"





GPSDataTypeDef hGPSData;							//GPS数据

GNSS_BestPos_DataTypeDef 	hGPSBestPosData;		//最佳位置
GNSS_BestVel_DataTypeDef 	hGPSBestVelData;		//最佳速度
GNSS_Heading_DataTypeDef 	hGPSHeadingData;		//方位角信息
GNSS_ZDA_DataTypeDef 		hGPSZdaData;			//时间日期信息
GNSS_TRA_DataTypeDef 		hGPSTraData;			//方向角信息
GNSS_VTG_DataTypeDef 		hGPSVtgData;			//地面速度
GNSS_RMC_DataTypeDef 		hGPSRmcData;			//推荐定位信息
GNSS_GGA_DataTypeDef 		hGPSGgaData;			//定位信息
GPS_AGRIC_TypeDef    		hGPSAgricData;
GNSS_TIME_SYN_DataTypeDef	hGPSTimeSyn;			//与arm2时间同步


uint8_t gCmdIndex = 0;
uint8_t gCmdTypeIndex = 0;
int cmdKind = 0;
uint8_t gnssSynFlg;

uint8_t gnss_dataIsOk(uint8_t *pBuffer, uint16_t BufferLen)
{
    uint16_t  i;
    uint16_t  j = 0;
    uint8_t  temp;
    uint8_t  sum = 0;
    uint16_t  count = 0;
    uint8_t  High;
    uint8_t  Low;
    uint8_t  verify;

    for(i = 1; i < BufferLen; i++)
    {
        temp = *(pBuffer + i);

        if((0x0a == temp) || (0x0d == temp))
        {
            break;
        }
        else if('*' == temp)
        {
            j = i;
            break;
        }
        else
        {
            sum ^= temp;

            if(',' == temp)
            {
                count++;
            }
        }
    }

    High = *(pBuffer + j + 1);
    Low = *(pBuffer + j + 2);

    if((High >= '0') && (High <= '9'))
    {
        High = High - 0x30;
    }
    else if((High >= 'A') && (High <= 'F'))
    {
        High = High - 0x37;
    }
    else
    {
        return 0;
    }

    if((Low >= '0') && (Low <= '9'))
    {
        Low = Low - 0x30;
    }
    else if((Low >= 'A') && (Low <= 'F'))
    {
        Low = Low - 0x37;
    }
    else
    {
        return INS_ERROR;
    }

    verify = (High << 4) | Low;

    if(verify == sum)
    {
        return INS_EOK;
    }
    else
    {
        return INS_ERROR;
    }
}

uint8_t gnss_gprmcIsLocation(uint8_t *pBuffer, uint8_t BufferLen)
{
    uint8_t  i;
    uint8_t  temp;
    uint8_t  count = 0;

    for(i = 0; i < BufferLen; i++)
    {
        temp = *(pBuffer + i);

        if(',' == temp)
        {
            count++;
        }
        else if('A' == temp)
        {
            if(2 == count)
            {
                return INS_EOK;
            }
        }
    }

    return INS_ERROR;
}

uint8_t gnss_isLocation(void)
{
    return (hGPSRmcData.valid == 'A')?INS_EOK:INS_ERROR;
}


int GNSS_Cmd_Kind_Parser(char* pData, uint16_t dataLen)
{
    int cmdKind = 0;

    if(*pData == '#')
    {
        cmdKind = 1;
    }
    else if(*pData == '$')
    {
        cmdKind = 2;
    }

    return cmdKind;
}

int GNSS_Cmd_Parser(char* pData)
{

    if((0 == strncmp("$GNRMC", (char const *)pData, 6))
            ||(0 == strncmp("$GPRMC", (char const *)pData, 6))
            ||(0 == strncmp("$GLRMC", (char const *)pData, 6)))
    {
        //gnssSynFlg |= (0x1 << 0);
        gCmdTypeIndex = 1;
    }
    else if((0 == strncmp("$GNGGA", (char const *)pData, 6))
            ||(0 == strncmp("$GPGGA", (char const *)pData, 6))
            ||(0 == strncmp("$GLGGA", (char const *)pData, 6))
            ||(0 == strncmp("$BDGGA", (char const *)pData, 6)))
    {
#ifdef configUse_naviTest
        gnssSynFlg |= (0x1 << 1);
#endif
        gCmdTypeIndex = 2;
    }
    else if((0 == strncmp("$GNVTG", (char const *)pData, 6))
            ||(0 == strncmp("$GPVTG", (char const *)pData, 6))
            ||(0 == strncmp("$GLVTG", (char const *)pData, 6)))
    {
        gCmdTypeIndex = 3;
    } else if((0 == strncmp("$GNZDA", (char const *)pData, 6))
              ||(0 == strncmp("$GPZDA", (char const *)pData, 6))
              ||(0 == strncmp("$GLZDA", (char const *)pData, 6)))
    {
        //gnssSynFlg |= (0x1 << 1);
        gCmdTypeIndex = 4;
    }
    else if(0 == strncmp("#HEADINGA", (char const *)pData, 9))
    {
#ifdef configUse_naviTest
        gnssSynFlg |= (0x1 << 2);
#endif
        gCmdTypeIndex = 5;
    }
    else if(0 == strncmp("#BESTPOSA", (char const *)pData, 9))
    {
        //gnssSynFlg |= (0x1 << 3);
        gCmdTypeIndex = 6;
    }
    else if(0 == strncmp("#BESTVELA", (char const *)pData, 9))
    {
        //gnssSynFlg |= (0x1 << 4);
        gCmdTypeIndex = 7;
    }
    else if((0 == strncmp("$GNTRA", (char const *)pData, 6))
            ||(0 == strncmp("$GPTRA", (char const *)pData, 6))
            ||(0 == strncmp("$GLTRA", (char const *)pData, 6)))
    {
        gCmdTypeIndex = 8;
    }
    else if(0 == strncmp("#AGRICA", (char const *)pData, 7))
    {
        gnssSynFlg |= (0x1 << 0);
        gCmdTypeIndex = 9;
    }


    return gCmdTypeIndex;
}

void GNSS_Buff_Parser(char* pData, uint16_t nCmdIndex)
{
    switch(gCmdTypeIndex)
    {
    case 1:
        gnss_RMC_Buff_Parser(pData);
        break;

    case 2:
        gnss_GGA_Buff_Parser(pData);
        break;

    case 3:
        gnss_VTG_Buff_Parser(pData);
        break;

    case 4:
        gnss_ZDA_Buff_Parser(pData);
        break;

    case 5:
        gnss_Heading_Buff_Parser(pData);
        break;

    case 6:
        gnss_BESTPOS_Buff_Parser(pData);
        break;

    case 7:
        gnss_BESTVEL_Buff_Parser(pData);
        break;

    case 8:
        gnss_TRA_Buff_Parser(pData);
        break;

    case 9:
        gnss_AGRIC_Buff_Parser(pData);
        break;
    default:
        break;
    }
}

void gnss_timeWrToArm2(void)
{
    hGPSTimeSyn.year = hGPSRmcData.year;
    hGPSTimeSyn.month = hGPSRmcData.month;
    hGPSTimeSyn.date = hGPSRmcData.day;
    hGPSTimeSyn.hour = hGPSRmcData.hour;
    hGPSTimeSyn.minute = hGPSRmcData.minute;
    hGPSTimeSyn.second = hGPSRmcData.second;

    DRam_Write(100, (uint16_t*)&hGPSTimeSyn.year, 3);

    syn_arm2();	//syn call
}


//推荐定位
int gnss_RMC_Buff_Parser(char* pData)
{
    switch(gCmdIndex)
    {
    case 0:			//字段0
        if((NULL != strstr(pData, "$GNRMC")) \
                || (NULL != strstr(pData, "$GLRMC")) \
                || (NULL != strstr(pData, "$GPRMC")))
        {

        }

        break;

    case 1:			//字段1		UTC时间
        //dtoc((double*)&hGPSRmcData.timestamp, (uint8_t*)pData, 3);
        hGPSRmcData.timestamp = atof(pData);
        hGPSRmcData.hour = ((uint32_t)hGPSRmcData.timestamp) / 10000;
        hGPSRmcData.minute = ((uint32_t)hGPSRmcData.timestamp) % 10000 / 100;
        hGPSRmcData.second = ((uint32_t)hGPSRmcData.timestamp) % 100;
        //gCmdIndex = 2;
        break;

    case 2:			//字段2		定位状态
        hGPSRmcData.valid = *pData;
        //gCmdIndex = 3;
        break;

    case 3:			//字段3		纬度
        //dtoc((double*)&hGPSRmcData.latitude, (uint8_t*)pData, 8);
        hGPSRmcData.latitude = atof(pData);
        //gCmdIndex = 4;
        break;

    case 4:			//字段4		纬度半球
        hGPSRmcData.LatHemisphere = *pData;
        //gCmdIndex = 5;
        break;

    case 5:			//字段5		经度
        //dtoc((double*)&hGPSRmcData.longitude, (uint8_t*)pData, 8);
        hGPSRmcData.longitude = atof(pData);
        //gCmdIndex = 6;
        break;

    case 6:			//字段6		经度半球
        hGPSRmcData.LonHemisphere = *pData;
        //gCmdIndex = 7;
        break;

    case 7:			//字段7		地面速率
        //dtoc((double*)&hGPSRmcData.rate, (uint8_t*)pData, 2);
        hGPSRmcData.rate = atof(pData);
        //gCmdIndex = 8;
        break;

    case 8:			//字段8		地面航向
        //dtoc((double*)&hGPSRmcData.courseAngle, (uint8_t*)pData, 2);
        hGPSRmcData.courseAngle = atof(pData);
        //gCmdIndex = 9;
        break;

    case 9:			//字段9		UTC日期
        hGPSRmcData.date = atol(pData);
        hGPSRmcData.day = hGPSRmcData.date / 10000;
        hGPSRmcData.month = hGPSRmcData.date % 10000 / 100;
        hGPSRmcData.year = hGPSRmcData.date % 100;
        //gCmdIndex = 10;
        break;

    case 10:		//字段10	磁偏角
        //dtoc((double*)&hGPSRmcData.magDeclination, (uint8_t*)pData, 2);
        hGPSRmcData.magDeclination = atof(pData);
        //gCmdIndex = 11;
        break;

    case 11:		//字段11	磁偏角方向
        //dtoc((double*)&hGPSRmcData.MagDeclinationDir, (uint8_t*)pData, 2);
        hGPSRmcData.MagDeclinationDir = atof(pData);
        //gCmdIndex = 12;
        break;

    case 12:		//字段12	模式指示
        hGPSRmcData.mode = *pData;
        //gCmdIndex = 13;
        break;

    case 13:		//字段13	校验值
        break;

    default:
        break;
    }

    if(gCmdIndex == 13)
    {

        return 1;
    }
    else
        return 0;
}

//定位信息
int gnss_GGA_Buff_Parser(char* pData)
{
//	gtime_t t;
    switch(gCmdIndex)
    {
    case 0:				//字段0
        if(NULL != strstr(pData, "$GNGGA"))
            //gCmdIndex = 1;

            break;

    case 1:				//字段1		UTC时间
        //dtoc((double*)&hGPSGgaData.timestamp, (uint8_t*)pData, 3);
        hGPSGgaData.timestamp = atof(pData);
        //str2time(pData, 2, 8, &t);
        //hGPSGgaData.gps_time = (uint32_t)time2gpst(t, (int*)&hGPSGgaData.gps_week);
        //gCmdIndex = 2;
        break;

    case 2:				//字段2		纬度
        //dtoc((double*)&hGPSGgaData.latitude, (uint8_t*)pData, 8);
        hGPSGgaData.latitude = atof(pData);
        //gCmdIndex = 3;
        break;

    case 3:				//字段3		纬度半球
        hGPSGgaData.LatHemisphere = *pData;
        //gCmdIndex = 4;
        break;

    case 4:				//字段4		经度
        //dtoc((double*)&hGPSGgaData.longitude, (uint8_t*)pData, 8);
        hGPSGgaData.longitude = atof(pData);
        //gCmdIndex = 5;
        break;

    case 5:				//字段5		经度半球
        hGPSGgaData.LonHemisphere = *pData;
        //gCmdIndex = 6;
        break;

    case 6:				//字段6		GPS状态
        hGPSGgaData.GPSState = (GNSS_State_TypeDef)( *pData - '0');
        //gCmdIndex = 7;
        break;

    case 7:				//字段7		正使用的卫星数量
        hGPSGgaData.numSatellitesUsed = atoi(pData);
        //gCmdIndex = 8;
        break;

    case 8:				//字段8		HDOP水平精度因子
        //dtoc((double*)&hGPSGgaData.HDOP, (uint8_t*)pData, 2);
        hGPSGgaData.HDOP = atof(pData);
        //gCmdIndex = 9;
        break;

    case 9:				//字段9		海拔高度
        //dtoc((double*)&hGPSGgaData.height, (uint8_t*)pData, 2);
        hGPSGgaData.height = atof(pData);
        //gCmdIndex = 10;
        break;

    case 10:			//字段10	地球椭球面相对大地水准面的高度
        //dtoc((double*)&hGPSGgaData.height_herizon, (uint8_t*)pData, 2);
        hGPSGgaData.height_herizon = atof(pData);
        //gCmdIndex = 11;
        break;

    case 11:			//字段11	差分时间
        //dtoc((double*)&hGPSGgaData.differTime, (uint8_t*)pData, 2);
        hGPSGgaData.differTime = atof(pData);
        //gCmdIndex = 12;
        break;

    case 12:			//字段12	差分站ID号
        hGPSGgaData.differStationID = atoi(pData);
        //gCmdIndex = 13;
        break;

    case 13:			//字段13	校验值
        break;

    default:
        break;
    }

    if(gCmdIndex == 13)
    {
        return 1;
    }
    else
        return 0;
}

//地面速度信息
int gnss_VTG_Buff_Parser(char* pData)
{
    switch(gCmdIndex)
    {
    case 0:				//字段0
        if((NULL != strstr(pData, "$GNVTG")) \
                || (NULL != strstr(pData, "$GLVTG")) \
                || (NULL != strstr(pData, "$GPVTG")))
        {

        }

        break;

    case 1:				//字段1			以真北为参考基准的地面航向
        //dtoc((double*)&hGPSVtgData.courseNorthAngle, (uint8_t*)pData, 4);
        hGPSVtgData.courseNorthAngle = atof(pData);
        //gCmdIndex = 2;
        break;

    case 2:				//字段2			以磁北为参考基准的地面航向
        //dtoc((double*)&hGPSVtgData.courseMagAngle, (uint8_t*)pData, 4);
        hGPSVtgData.trueNorth = *pData;
        //gCmdIndex = 3;
        break;

    case 3:				//字段2			以磁北为参考基准的地面航向
        //dtoc((double*)&hGPSVtgData.courseMagAngle, (uint8_t*)pData, 4);
        hGPSVtgData.courseMagAngle = atof(pData);;
        //gCmdIndex = 4;
        break;

    case 4:				//字段2			以磁北为参考基准的地面航向
        //dtoc((double*)&hGPSVtgData.courseMagAngle, (uint8_t*)pData, 4);
        hGPSVtgData.magneticNorth = *pData;
        //gCmdIndex = 5;
        break;

    case 5:				//字段3			地面速率 节
        //dtoc((double*)&hGPSVtgData.rateKnots, (uint8_t*)pData, 4);
        hGPSVtgData.rateKnots = atof(pData);
        //gCmdIndex = 6;
        break;

    case 6:				//字段2			以磁北为参考基准的地面航向
        //dtoc((double*)&hGPSVtgData.courseMagAngle, (uint8_t*)pData, 4);
        hGPSVtgData.N = *pData;
        //gCmdIndex = 7;
        break;

    case 7:				//字段4			地面速率 公里/小时
        //dtoc((double*)&hGPSVtgData.rateKm, (uint8_t*)pData, 4);
        hGPSVtgData.rateKm = atof(pData);
        //gCmdIndex = 8;
        break;

    case 8:				//字段2			以磁北为参考基准的地面航向
        //dtoc((double*)&hGPSVtgData.courseMagAngle, (uint8_t*)pData, 4);
        hGPSVtgData.K = *pData;
        //gCmdIndex = 9;
        break;

    case 9:				//字段5			模式指示
        hGPSVtgData.mode = *pData;
        //gCmdIndex = 10;
        break;

    case 10:				//字段6			校验值
        break;

    default:
        break;
    }

    if(gCmdIndex == 10)
        return 1;
    else
        return 0;
}

//日期
int gnss_ZDA_Buff_Parser(char* pData)
{
    gtime_t t;
    char str_time[6], year[4], month[2], day[2];
    char str[30] = {'\0'};

    switch(gCmdIndex)
    {
    case 0:					//字段0
        if((NULL != strstr(pData, "$GNZDA"))
                || (NULL != strstr(pData, "$GLZDA"))
                || (NULL != strstr(pData, "$GPZDA")))
        {

        }

        break;

    case 1:					//字段1			UTC时间
        //dtoc((double*)&hGPSZdaData.timestamp, (uint8_t*)pData, 3);
        hGPSZdaData.timestamp = atof(pData);
        hGPSZdaData.hour = ((uint32_t)hGPSZdaData.timestamp) / 10000;
        hGPSZdaData.minute = ((uint32_t)hGPSZdaData.timestamp) % 10000 / 100;
        hGPSZdaData.second = ((uint32_t)hGPSZdaData.timestamp) % 100;
        //gCmdIndex = 2;
        strncpy(str_time, pData, 6);
        break;

    case 2:					//字段2			日期
        hGPSZdaData.day = atoi(pData);
        strncpy(day, pData, 2);
        //gCmdIndex = 3;
        break;

    case 3:					//字段3			月份
        hGPSZdaData.month = atoi(pData);
        strncpy(month, pData, 2);
        //gCmdIndex = 4;
        break;

    case 4:					//字段4			年份
        hGPSZdaData.year = atoi(pData);
        strncpy(year, pData, 4);

        sprintf(str, "%0.4d %0.2d %0.2d %0.2d %0.2d %0.2d", \
                hGPSZdaData.year, hGPSZdaData.month, hGPSZdaData.day, \
                hGPSZdaData.hour, hGPSZdaData.minute, hGPSZdaData.second);

        str2time(str, 0, strlen(str), &t);
        hGPSGgaData.gps_time = time2gpst(t, (int*)&hGPSGgaData.gps_week);

        //gCmdIndex = 5;
        break;

    case 5:					//字段5			当地时域描述  小时
        hGPSZdaData.hour_timeDomain = atoi(pData);
        //gCmdIndex = 6;
        break;

    case 6:					//字段6			当地时域描述  分钟
        hGPSZdaData.minute_timeDomain = atoi(pData);
        //gCmdIndex = 7;
        break;

    case 7:					//字段7			校验和
        break;

    default:
        break;
    }

    if(gCmdIndex == 7)
    {
        return 1;
    }
    else
        return 0;
}

const static RESOLV_Status sol_status[11] =
{
    {0, "SOL_COMPUTED"},
    {1, "INSUFFICIENT_OBS"},
    {2, "NO_CONVERGENCE"},
    {3, "SINGULARITY"},
    {4, "COV_TRACE"},
    {5, "TEST_DIST"},
    {6, "COLD_START"},
    {7, "V_H_LIMIT"},
    {8, "VARIANCE"},
    {9, "RESIDUALS"},
    {13, "INTEGRITY_WARNING"}
};

const static RESOLV_Status pos_status[18] =
{
    {0, "NONE"},
    {1, "FIXEDPOS"},
    {2, "FIXEDHEIGHT"},
    {8, "DOPPLER_VELOCITY"},
    {16, "SINGLE"},
    {17, "PSRDIFF"},
    {18, "SBAS"},
    {32, "L1_FLOAT"},
    {33, "IONOFREE_FLOAT"},
    {34, "NARROW_FLOAT"},
    {48, "L1_INT"},
    {49, "WIDE_INT"},
    {50, "NARROW_INT"},
    {52, "INS"},
    {53, "INS_PSRSP"},
    {54, "INS_PSRDIFF"},
    {55, "INS_RTKFLOA"},
    {56, "INS_RTKFIXED"}
};

//方位角信息
int gnss_Heading_Buff_Parser(char* pData)
{
    uint8_t i;
    char *ch;
    //uint8_t len;
    //char str[4];

    switch(gCmdIndex)
    {
    case 0: 			//字段0
        if(0 == strncmp("#HEADINGA", (char const *)pData, 9))
        {
            memcpy(hGPSHeadingData.header.sync, pData, sizeof("#HEADINGA"));
        }

        break;

    case 1:
    case 2:
        break;

    case 3: 			//字段1			解算状态
        hGPSHeadingData.header.CPUIDle = atof(pData);
        break;

    case 4:
        break;

    case 5: 				//字段4 GPS周数
        hGPSHeadingData.header.gpsWn = atol(pData);
        break;

    case 6: 				//字段5 GPS周内秒
        hGPSHeadingData.header.gpsMs = atol(pData);
        break;

    case 7:
    case 8:
        break;

    case 9:				//字段1			解算状态
        ch = strchr(pData, ';');

        for(i = 0; i < 11; i++)
        {
            if((NULL != strstr(ch, sol_status[i].string)))
            {
                break;
            }
        }

        if(i != 11)
            hGPSHeadingData.calcState = (Resolve_TypeDef)sol_status[i].index;
        else
            hGPSHeadingData.calcState = resolve_Integrity_Invalid;//无效状态

        break;

    case 10:				//字段1			解算状态
        for(i = 0; i < 18; i++)
        {
            if((NULL != strstr(pData, pos_status[i].string)))
            {
                break;
            }
        }

        if(i != 18)
            hGPSHeadingData.posType = (GNSS_POS_TypeDef)pos_status[i].index;
        else
            hGPSHeadingData.posType = pos_type_INS_Invalid;//无效状态

        break;

    case 11:					//字段3				基线长
        hGPSHeadingData.baseLen = atof(pData);
        //gCmdIndex = 3;
        break;

    case 12:					//字段4				航向角
        hGPSHeadingData.courseAngle = atof(pData);
        //gCmdIndex = 4;
        break;

    case 13:					//字段5				俯仰角
        hGPSHeadingData.pitchAngle = atof(pData);
        //gCmdIndex = 4;
        break;

    case 14:					//字段6				保留
        break;

    case 15:					//字段7				航向标准偏差
        hGPSHeadingData.courseStandardDeviation = atof(pData);
        break;

    case 16:					//字段8				俯仰标准偏差
        hGPSHeadingData.pitchStandardDeviation = atof(pData);
        break;

    case 17:					//字段9				基站ID
        //ch = strchr(pData + 1, '"');
        //len = ch - pData - 1;
        //strncpy(str, pData + 1, len);
        hGPSHeadingData.baseStationID = atol(pData + 1);
        break;

    case 18:				//字段10			跟踪的卫星数量
        hGPSHeadingData.numSatellitesTracked = atoi(pData);
        break;

    case 19:				//字段11			使用的卫星数量
        hGPSHeadingData.numSatellitesUsed = atoi(pData);
        break;

    case 20:				//字段12			截止高度角以上的卫星数
        hGPSHeadingData.numSatellitesAboveCutoffAngle = atoi(pData);
        break;

    case 21:				//字段13			截止高度角以上有L2观测的卫星数
        hGPSHeadingData.numSatellitesAboveCutoffAngleL2 = atoi(pData);
        break;

    case 22:				//字段14			保留
        break;

    case 23:				//字段15			解状态
        hGPSHeadingData.solutionState = atoi(pData);
        break;

    case 24:				//字段16			保留
        break;

    case 25:				//字段17			信号掩码
        hGPSHeadingData.sigMask = (Single_Mask_TypeDef)atoi(pData);
        break;

    case 26:				//字段14			校验码
        break;

    default:
        break;
    }

    if(gCmdIndex == 26)
    {
        return 1;
    }
    else
        return 0;
}

//最佳位置
int gnss_BESTPOS_Buff_Parser(char * pData)
{
    uint8_t i;
    char *ch;
    //uint8_t len;
    //char str[4];

    switch(gCmdIndex)
    {
    case 0:				//字段0
        if(0 == strncmp("#BESTPOSA", (char const *)pData, 9))
        {
            memcpy(hGPSBestPosData.header.sync, pData, sizeof("#BESTPOSA"));
        }

        break;

    case 1:
    case 2:
        break;

    case 3:				//字段1			解算状态
        hGPSBestPosData.header.CPUIDle = atof(pData);
        break;

    case 4:
        break;

    case 5:					//字段4 GPS周数
        hGPSBestPosData.header.gpsWn = atol(pData);
        break;

    case 6:					//字段5 GPS周内秒
        hGPSBestPosData.header.gpsMs = atol(pData);
        break;

    case 7:
    case 8:
        break;

    case 9:				//字段1			解算状态
        ch = strchr(pData, ';');

        for(i = 0; i < 11; i++)
        {
            if((NULL != strstr(ch, sol_status[i].string)))
            {
                break;
            }
        }

        if(i != 12)
            hGPSBestPosData.calcState = (Resolve_TypeDef)sol_status[i].index;
        else
            hGPSBestPosData.calcState = resolve_Integrity_Invalid;//无效状态

        break;

    case 10:				//字段1			解算状态
        for(i = 0; i < 18; i++)
        {
            if((NULL != strstr(pData, pos_status[i].string)))
            {
                break;
            }
        }

        if(i != 18)
            hGPSBestPosData.posType = (GNSS_POS_TypeDef)pos_status[i].index;
        else
            hGPSBestPosData.posType = pos_type_INS_Invalid;//无效状态

        break;

    case 11:				//字段3			纬度
        hGPSBestPosData.latitude = atof(pData);
        break;

    case 12:				//字段4			经度
        hGPSBestPosData.longitude = atof(pData);
        break;

    case 13:				//字段5			海拔高
        hGPSBestPosData.height = atof(pData);
        break;

    case 14:				//字段6			高程异常
        hGPSBestPosData.heightVariance = atof(pData);
        break;

    case 15:				//字段7			坐标系id号
        hGPSBestPosData.coordinateID = (coordinateIDTypeDef)atoi(pData);
        break;

    case 16:				//字段8			纬度标准差
        hGPSBestPosData.latitudeStandardDeviation = atof(pData);
        break;

    case 17:				//字段9			经度标准差
        hGPSBestPosData.longitudeStandardDeviation = atof(pData);
        break;

    case 18:				//字段10			高度标准差
        hGPSBestPosData.heightStandardDeviation = atof(pData);
        break;

    case 19:				//字段11			基站ID
        //ch = strchr(pData + 1, '"');
        //len = ch - pData - 1;
        //strncpy(str, pData + 1, len);
        hGPSBestPosData.stationID = atol(pData + 1);
        break;

    case 20:				//字段12			差分龄期
        hGPSBestPosData.differPeriod = atof(pData);
        break;

    case 21:				//字段13			解得龄期
        hGPSBestPosData.solutionPeriod = atof(pData);
        break;

    case 22:				//字段14			跟踪的卫星数
        hGPSBestPosData.numSatellitesTracked = atoi(pData);
        break;

    case 23:				//字段15			解算使用的卫星数
        hGPSBestPosData.numSatellitesUsed = atoi(pData);
        break;

    case 24:				//字段16			保留
        break;

    case 25:				//字段17			保留
        break;

    case 26:				//字段18			保留
        break;

    case 27:				//字段19			保留
        break;

    case 28:				//字段20			保留
        break;

    case 29:				//字段21			保留
        break;

    case 30:				//字段22			信号掩码
        break;

    case 31:				//字段23			校验值
        break;

    default:
        break;
    }

    if(gCmdIndex == 31)
    {
        return 1;
    }
    else
        return 0;
}

//最佳速度
int gnss_BESTVEL_Buff_Parser(char * pData)
{
    uint8_t i;
    char *ch;

    switch(gCmdIndex)
    {
    case 0:				//字段0
        if(0 == strncmp("#BESTVELA", (char const *)pData, 9))
        {
            memcpy(hGPSBestVelData.header.sync, pData, sizeof("#BESTVELA"));
        }

        break;

    case 1:
    case 2:
        break;

    case 3:				//字段1			解算状态
        hGPSBestVelData.header.CPUIDle = atof(pData);
        break;

    case 4:
        break;

    case 5:					//字段4 GPS周数
        hGPSBestVelData.header.gpsWn = atol(pData);
        break;

    case 6:					//字段5 GPS周内秒
        hGPSBestVelData.header.gpsMs = atol(pData);
        break;

    case 7:
    case 8:
        break;

    case 9:				//字段1			解算状态
        ch = strchr(pData, ';');

        for(i = 0; i < 11; i++)
        {
            if((NULL != strstr(ch, sol_status[i].string)))
            {
                break;
            }
        }

        if(i != 12)
            hGPSBestVelData.calcState = (Resolve_TypeDef)sol_status[i].index;
        else
            hGPSBestVelData.calcState = resolve_Integrity_Invalid;//无效状态

        break;

    case 10:				//字段1			解算状态
        for(i = 0; i < 18; i++)
        {
            if((NULL != strstr(pData, pos_status[i].string)))
            {
                break;
            }
        }

        if(i != 18)
            hGPSBestVelData.velType = (GNSS_POS_TypeDef)pos_status[i].index;
        else
            hGPSBestVelData.velType = pos_type_INS_Invalid;//无效状态

        break;

    case 11:
        hGPSBestVelData.latency = atof(pData);
        break;

    case 12:
        hGPSBestVelData.age = atof(pData);
        break;

    case 13:
        hGPSBestVelData.horSpd = strtod(pData,NULL);
        break;

    case 14:
        hGPSBestVelData.trkGnd = strtod(pData,NULL);
        break;

    case 15:
        hGPSBestVelData.vertSpd = strtod(pData,NULL);
        break;

    default:
        break;
    }

    if(gCmdIndex == 15)
    {
        return 1;
    }
    else
        return 0;
}

//方向角输出
int gnss_TRA_Buff_Parser(char * pData)
{
    switch(gCmdIndex)
    {
    case 0:					//字段0
        if(0 == strncmp("#$GNTRA", (char const *)pData, 7))
        {

        }

        break;

    case 1:					//字段1				UTC时间
        hGPSTraData.timestamp = atof(pData);
        break;

    case 2:					//字段2				方向角
        hGPSTraData.headingAngle = atof(pData);
        break;

    case 3:					//字段3				俯仰角
        hGPSTraData.pitchAngle = atof(pData);
        break;

    case 4:					//字段4				横滚角
        hGPSTraData.rollAngle = atof(pData);
        break;

    case 5:					//字段5				GPS质量指示符
        hGPSTraData.SolStatus = (GNSS_State_TypeDef)atoi(pData);
        break;

    case 6:					//字段6				卫星数
        hGPSTraData.starNum = atoi(pData);

        break;

    case 7:					//字段7				差分延迟
        hGPSTraData.Age = atof(pData);
        break;

    case 8:					//字段8				基站号
        hGPSTraData.stationID = atoi(pData);
        //gCmdIndex = 9;
        break;

    case 9:					//字段9				校验和
        break;

    default:
        break;
    }

    if(gCmdIndex == 9)
        return 1;
    else
        return 0;
}

uint8_t gnss_time_is_valid(void)
{
    if(NULL != strstr(hGPSAgricData.header.TimeQuality, "FINE"))
    {
        return INS_EOK;
    }
    return INS_ERROR;
}

//AGRIC 信息
int gnss_AGRIC_Buff_Parser(char * pData)
{

    switch(gCmdIndex)
    {
    case 0:					//字段0
        if(NULL != strstr(pData, "#AGRICA"))
        {
            memcpy(hGPSAgricData.header.sync, pData, sizeof("#AGRICA"));
        }

        break;

    case 1:					//字段1

        hGPSAgricData.header.CPUIDle = atoi(pData);
        break;

    case 2:					//字段2
        strncpy(hGPSAgricData.header.TimeRef, pData, 4);
        break;
    case 3:
        if(NULL != strstr(pData, "FINE"))
        {
            memset(hGPSAgricData.header.TimeQuality, '\0', sizeof(hGPSAgricData.header.TimeQuality));
            memcpy(hGPSAgricData.header.TimeQuality, pData, sizeof("FINE"));
        }
        else if(NULL != strstr(pData, "UNKNOWN"))
        {
            memcpy(hGPSAgricData.header.TimeQuality, pData, sizeof("UNKNOWN"));
        }
        break;

    case 4:					//字段4 GPS周数
        hGPSAgricData.header.gpsWn = atol(pData);
        break;

    case 5:					//字段5 GPS周内秒
        hGPSAgricData.header.gpsMs = atol(pData);
        break;

    case 6:					//字段6,7不解析
    case 7:
    case 8:					//字段8 GPS周闰秒
        hGPSAgricData.header.leapSec = atoi(pData);
        break;

    case 9:					//字段9不解析
        break;

    case 10:					//字段2				指令长度 不解析

        break;

    case 11:					//字段3				utc时间 年
        hGPSAgricData.utc_year = atoi(pData);

        break;

    case 12:					//字段4				utc时间 月
        hGPSAgricData.utc_month = atoi(pData);

        break;

    case 13:					//字段5				utc时间 日
        hGPSAgricData.utc_day = atoi(pData);

        break;

    case 14:					//字段6				utc时间 时
        hGPSAgricData.utc_hour = atoi(pData);

        break;

    case 15:					//字段7				utc时间 分
        hGPSAgricData.utc_min = atoi(pData);

        break;

    case 16:					//字段8				utc时间 秒
        hGPSAgricData.utc_sec = atoi(pData);

        break;

    case 17:					//字段9				流动站定位状态
        hGPSAgricData.rtk_status = atoi(pData);

        break;

    case 18:					//字段10				主从天线 Heading
        hGPSAgricData.heading_status = atoi(pData);
        break;

    case 19:					//字段11				参与解算 GPS 卫星数
        hGPSAgricData.num_gps_star = atoi(pData);

        break;

    case 20:					//字段12				参与解算 BDS 卫星数
        hGPSAgricData.num_bd_star = atoi(pData);

        break;

    case 21:					//字段13				参与解算 GLO 卫星数
        hGPSAgricData.num_glo_star = atoi(pData);

        break;

    case 22:					//字段14~19  未解析
    case 23:
    case 24:
    case 25:
    case 26:
    case 27:

        break;

    case 28:					//字段20				航向角
        hGPSAgricData.Heading = strtod(pData, NULL);

        break;

    case 29:					//字段21			俯仰角
        hGPSAgricData.pitch = strtod(pData, NULL);

        break;

    case 30:					//字段22			横滚角
        hGPSAgricData.roll = strtod(pData, NULL);

        break;

    case 31:					//字段23			未解析

        break;

    case 32:					//字段24			北向速度
        hGPSAgricData.vN = strtod(pData, NULL);

        break;

    case 33:					//字段25			东向速度
        hGPSAgricData.vE = strtod(pData, NULL);

        break;

    case 34:					//字段26		天顶速度
        hGPSAgricData.vZ = strtod(pData, NULL);

        break;

    case 35:					//字段27		北向速度标准差
        hGPSAgricData.sigma_vn = strtod(pData, NULL);

        break;

    case 36:					//字段28			东向速度标准差
        hGPSAgricData.sigma_ve = strtod(pData, NULL);

        break;

    case 37:					//字段29		天顶速度标准差
        hGPSAgricData.sigma_vz = strtod(pData, NULL);

        break;

    case 38:					//字段30	  纬度
        hGPSAgricData.lat = strtod(pData, NULL);

        break;

    case 39:					//字段31	  经度
        hGPSAgricData.lon = strtod(pData, NULL);

        break;

    case 40:					//字段32   高程
        hGPSAgricData.alt = strtod(pData, NULL);

        break;

    case 41:					//字段33~35  未解析
    case 42:
    case 43:

        break;

    case 44:					//字段36 纬度标准差
        hGPSAgricData.sigma_lat = strtod(pData, NULL);

        break;

    case 45:					//字段37 纬度标准差
        hGPSAgricData.sigma_lon = strtod(pData, NULL);

        break;

    case 46:					//字段38 纬度标准差
        hGPSAgricData.sigma_alt = strtod(pData, NULL);

        break;

    case 47:					//字段39~47  未解析
    case 48:
    case 49:
    case 50:
    case 51:
    case 52:
    case 53:
    case 54:
    case 55:
        break;

    case 56:					//字段48 GPS 周内毫秒
        hGPSAgricData.GPS_Sec = atol(pData);
        //hGPSAgricData.GPS_Sec /= 1000;
        //gCmdIndex++;
        break;
	case 57:
    case 58:
    case 59:
    case 60:
    case 61:
        break;
    case 62:					//Galileo 卫星数
        hGPSAgricData.num_gal_star = atoi(pData);
    default:
        break;
    }

    if(gCmdIndex == 62)
        return 1;
    else
        return 0;
}

void gnss_config_movingbase(void)
{
    // 发送 "mode movingbase\r"

}

void gnss_request_GGA(void)
{
    // 发送 "GNGGA COM2 x\r"

}

void gnss_request_GSA(void)
{
    // 发送 "GNGSA COM2 x\r"

}

void gnss_request_RMC(void)
{
    // 发送 "GNRMC COM2 x\r"

}

void gnss_request_HDT(void)
{
    // 发送 "GPHDT COM2 x\r"

}

void gnss_request_HEADING(void)
{
    // 发送 "HEADING COM2 x\r"

}

void gnss_request_OTG(void)
{
    // 发送 "GPOTG COM2 x\r"

}

void gnss_request_ZDA(void)
{
    // 发送 "GPZDA COM2 x\r"

}

void gnss_request_saveconfig(void)
{
    // 发送 "SAVECONFIG"

}

//INS 设置姿态和标准差
void gnss_set_posture(double pitch, double roll, double azimuth,
                      double pitchOffset, double rollOffset, double azimuthOffset)
{
    char str[30] = {'\0',};
    // 发送 SETINITATTITUDE pitch roll azimuth pitchSTD rollSTD azSTD
    // CONFIG INS ATTITUDE pitch roll azimuth pitchSTD rollSTD azSTD
    sprintf(str, "%.2f %.2f %.2f %.2f %.2f %.2f",pitch, roll, azimuth, pitchOffset, rollOffset, azimuthOffset);
    //Uart_SendMsg(UART_TXPORT_COMPLEX_10, 0, strlen(str), (uint8_t *)str);
}

//IMU 至从天线杆臂参数配置
void gnss_set_leverArm(double x, double y, double z,
                       double a, double b, double c)
{
    char str[30] = {'\0',};
    // 发送 SETIMUTOANTOFFSET2 x y z [a] [b] [c]
    // CONFIG IMUTOANT OFFSET x y z [a] [b] [c]
    sprintf(str, "%.2f %.2f %.2f %.2f %.2f %.2f",x, y, z, a, b, c);
    //Uart_SendMsg(UART_TXPORT_COMPLEX_10, 0, strlen(str), (uint8_t *)str);
}

//INS 输出位置偏移配置
void gnss_set_ins_offset(double xoffset, double yoffset, double zoffset)
{
    char str[20] = {'\0',};
    // 发送 SETIMUTOANTOFFSET2 x y z [a] [b] [c]
    // CONFIG INSSOL OFFSET xoffset yoffset zoffset
    sprintf(str, "%.2f %.2f %.2f",xoffset, yoffset, zoffset);
    //Uart_SendMsg(UART_TXPORT_COMPLEX_10, 0, strlen(str), (uint8_t *)str);
}

GPS_AGRIC_TypeDef*  gnss_get_algorithm_dataPtr(void)
{
    return &hGPSAgricData;
}

void gnss_Fetch_Data(void)
{
    hGPSData.timestamp = hGPSRmcData.timestamp;
    //hGPSData.StarNum = hGPSHeadingData.numSatellitesUsed;
    hGPSData.StarNum = hGPSGgaData.numSatellitesUsed;

    hGPSData.PositioningState = hGPSRmcData.valid;
    //位置状态
    hGPSData.ResolveState[0] = hGPSBestPosData.calcState;
    hGPSData.PositionType[0] = hGPSBestPosData.posType;
    //速度状态
    hGPSData.ResolveState[1] = hGPSBestVelData.calcState;
    hGPSData.PositionType[1] = hGPSBestVelData.velType;
    //姿态状态
    hGPSData.ResolveState[2] = hGPSHeadingData.calcState;
    hGPSData.PositionType[2] = hGPSHeadingData.posType;

    hGPSData.hdgstddev = hGPSHeadingData.courseStandardDeviation;
    hGPSData.ptchstddev = hGPSHeadingData.pitchStandardDeviation;


    hGPSData.LonHemisphere = hGPSRmcData.LonHemisphere;
    hGPSData.Lon = hGPSAgricData.lon;
    hGPSData.LatHemisphere = hGPSRmcData.LatHemisphere;
    hGPSData.Lat = hGPSAgricData.lat;
    hGPSData.Altitude = hGPSAgricData.alt;
    hGPSData.Heading = hGPSAgricData.Heading;
    hGPSData.Pitch = hGPSAgricData.pitch;
    hGPSData.Roll = hGPSAgricData.roll;
    hGPSData.baseline = hGPSHeadingData.baseLen;
    //hGPSData.LonStd = hGPSBestPosData.longitudeStandardDeviation;
    //hGPSData.LatStd = hGPSBestPosData.latitudeStandardDeviation;
    //hGPSData.AltitudeStd = hGPSBestPosData.heightStandardDeviation;
    //hGPSData.HeadingStd = hGPSHeadingData.courseStandardDeviation;
    //hGPSData.PitchStd = hGPSHeadingData.pitchStandardDeviation;
    //hGPSData.HDOP = hGPSGgaData.HDOP;
    //hGPSData.GroundSpeed = hGPSRmcData.rate;

    hGPSData.gpsweek = hGPSAgricData.header.gpsWn;
    hGPSData.gpssecond = hGPSAgricData.header.gpsMs;
    hGPSData.ve = hGPSAgricData.vE;
    hGPSData.vn = hGPSAgricData.vN;
    hGPSData.vu = hGPSAgricData.vZ;
    hGPSData.rtkStatus = hGPSAgricData.rtk_status;
    hGPSData.LatStd = hGPSAgricData.sigma_lat;
    hGPSData.LonStd = hGPSAgricData.sigma_lon;
    hGPSData.AltitudeStd = hGPSAgricData.sigma_alt;
    hGPSData.vestd = hGPSAgricData.sigma_ve;
    hGPSData.vnstd = hGPSAgricData.sigma_vn;
    hGPSData.vustd = hGPSAgricData.sigma_vz;
    hGPSData.supportposvelstd = 1;
}

//void gnss_parse_unicore(uint8_t* pData, uint16_t len)
//{
//    uint8_t valid = 0;
//    uint8_t* pDat = pData;
//    uint16_t tCmd = 0;
//    uint16_t dataLen = len;
//    uint8_t  header[3] = {0xAA, 0x44, 0xB5};

//    //找到帧头
//    do
//    {
//        if(0 == memcmp(pDat, header, 3))
//        {
//            valid = 1;
//            break;
//        }

//        pDat++;
//        dataLen--;
//    }
//    while(dataLen > 0);
//}

uint8_t gnss_parse(uint8_t* pData, uint16_t dataLen)
{
    char * pBuf = NULL;
    char * pRxBuf = (char*)pData;
    char gnssCommaSeperator[] = ",";
//    char gnssSemicolonSeperator[] = ";";
    int cmdKind = 0;
    uint8_t ret = INS_EOK;

//    ret = gnss_dataIsOk(pData, dataLen);

//    if(INS_ERROR == ret)
//    {
//    	//数据校验是否通过
//        return INS_ERROR;
//    }

    if(gCmdIndex == 0)
    {
        cmdKind = GNSS_Cmd_Kind_Parser(pRxBuf, dataLen); //命令起始符
        gCmdTypeIndex = GNSS_Cmd_Parser(pRxBuf);//命令类型
    }

    if(gCmdTypeIndex == 1)
    {
        ret = gnss_gprmcIsLocation(pData, dataLen);//判断是否为有效数据

        if(INS_ERROR == ret)
        {
            //到这里说明数据是无效的，这里对数据做相应处理

            return INS_ERROR;
        }
        //gnss_timeWrToArm2();
    }

    if(cmdKind != 0)
    {
        //if(cmdKind == 1)
        //    pBuf = strtok(pRxBuf, gnssSemicolonSeperator);
        //else if(cmdKind == 2)
        pBuf = strtok(pRxBuf, gnssCommaSeperator);

        gCmdIndex = 0;

        while(NULL != pBuf)
        {
            GNSS_Buff_Parser(pBuf, gCmdTypeIndex);
            pBuf = strtok(NULL, gnssCommaSeperator);
            gCmdIndex ++;
        }

        gCmdIndex = 0;

    }

    return ret;
}
#include "drv_rtc.h"
#include "FmcAdapter.h"

void gnss_fill_data(uint8_t* pData, uint16_t dataLen)
{

    if(INS_EOK == gnss_parse(pData, dataLen))
    {
#ifdef configUse_naviTest
        rtc_update_struct* rtc;
        extern navi_test_t	navi_test_info;
        if(0x07 == gnssSynFlg)
        {
            gnssSynFlg = 0;
            rtc = rtc_update();
            navi_test_info.timestamp = rtc->gpsTime;
            //navi_test_info.timestamp = hGPSGgaData.timestamp;
            navi_test_info.gps_valid = 1;
            navi_test_info.gpsWeek = hGPSAgricData.header.gpsWn;
            navi_test_info.gpsSec  = hGPSAgricData.header.gpsMs;
            navi_test_info.starNum = hGPSGgaData.numSatellitesUsed;
            navi_test_info.rtkStatus = hGPSAgricData.rtk_status;
            navi_test_info.lat = hGPSGgaData.latitude;
            navi_test_info.lon = hGPSGgaData.longitude;
            navi_test_info.alt = hGPSGgaData.height;
            navi_test_info.vu = hGPSAgricData.vZ;
            navi_test_info.heading = hGPSHeadingData.courseAngle;
            navi_test_info.pitch = hGPSHeadingData.pitchAngle;
            navi_test_info.lat_std = hGPSAgricData.sigma_lat;
            navi_test_info.lon_std = hGPSAgricData.sigma_lon;
            navi_test_info.alt_std = hGPSAgricData.sigma_alt;
            navi_test_info.hdgstddev = hGPSHeadingData.courseStandardDeviation;
            navi_test_info.ptchstddev = hGPSHeadingData.pitchStandardDeviation;
            navi_test_info.hdop = hGPSGgaData.HDOP;
            gnss_Fetch_Data();gnss_notify();
        }
#else
        if(0x01 == gnssSynFlg)
        {
            gnssSynFlg = 0;
            //DRam_Read(0, &hGPSData.gpscompms, 2);
            //hGPSData.gpscompms = FMC_ReadWord(SPACE_COM, 0x160 >> 1);
            //hGPSData.gpscompms *= 5;
             gnss_Fetch_Data();
            gnss_notify();
        }
#endif
    }

}

void strtok_test(void)
{
    char *ch = "#AGRICA,98,GPS,UNKNOWN,1,8338000,0,0,18,2;GNSS,";
    char *buf;
    char *delim = ",";

    buf = strtok(ch, delim);

    while(NULL != buf)
    {
        buf = strtok(NULL, delim);
    }

}
#endif

