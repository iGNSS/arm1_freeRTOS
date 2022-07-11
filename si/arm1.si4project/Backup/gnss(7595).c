#include "gnss.h"
#include "insdef.h"
#include "Time_Unify.h"
#include "arm_math.h"


GPSDataTypeDef hGPSData;						//GPS数据

GNSS_BestPos_DataTypeDef hGPSBestPosData;		//最佳位置
GNSS_Heading_DataTypeDef hGPSHeadingData;		//方位角信息
GNSS_ZDA_DataTypeDef hGPSZdaData;				//时间日期信息
GNSS_TRA_DataTypeDef hGPSTraData;				//方向角信息
GNSS_VTG_DataTypeDef hGPSVtgData;				//地面速度
GNSS_RMC_DataTypeDef hGPSRmcData;				//推荐定位信息
GNSS_GGA_DataTypeDef hGPSGgaData;				//定位信息

uint8_t gCmdIndex = 0;
uint8_t gCmdTypeIndex = 0;

int GNSS_Cmd_Kind_Parser(char* pData)
{
    if(*pData == '#')
        return 1;
    else if(*pData == '$')
        return 2;

    return 0;
}

int GNSS_Cmd_Parser(char* pData)
{
    if(NULL != strstr(pData, "$GPRMC"))
        gCmdTypeIndex = 1;
    else if(NULL != strstr(pData, "$GLRMC"))
        gCmdTypeIndex = 1;
    else if(NULL != strstr(pData, "$GNRMC"))
        gCmdTypeIndex = 1;
    else if(NULL != strstr(pData, "$GPGGA"))
        gCmdTypeIndex = 2;
    else if(NULL != strstr(pData, "$GLGGA"))
        gCmdTypeIndex = 2;
    else if(NULL != strstr(pData, "$GNGGA"))
        gCmdTypeIndex = 2;
    else if(NULL != strstr(pData, "$BDSGGA"))
        gCmdTypeIndex = 2;
    else if(NULL != strstr(pData, "$GPVTG"))
        gCmdTypeIndex = 3;
    else if(NULL != strstr(pData, "$GLVTG"))
        gCmdTypeIndex = 3;
    else if(NULL != strstr(pData, "$GNVTG"))
        gCmdTypeIndex = 3;
    else if(NULL != strstr(pData, "$GPZDA"))
        gCmdTypeIndex = 4;
    else if(NULL != strstr(pData, "$GLZDA"))
        gCmdTypeIndex = 4;
    else if(NULL != strstr(pData, "$GNZDA"))
        gCmdTypeIndex = 4;
    else if(NULL != strstr(pData, "#HEADING"))
        gCmdTypeIndex = 5;
    else if(NULL != strstr(pData, "#BESTPOS"))
        gCmdTypeIndex = 6;
    else if(NULL != strstr(pData, "$GPTRA"))
        gCmdTypeIndex = 7;

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
            gnss_TRA_Buff_Parser(pData);
            break;

        default:
            break;
    }
}

//推荐定位
int gnss_RMC_Buff_Parser(char* pData)
{
    switch(gCmdIndex)
    {
        case 0:			//字段0
            if(NULL != strstr(pData, "$GPRMC"))
                gCmdIndex = 1;

            if(NULL != strstr(pData, "$GLRMC"))
                gCmdIndex = 1;

            if(NULL != strstr(pData, "$GNRMC"))
                gCmdIndex = 1;

            break;

        case 1:			//字段1		UTC时间
            dtoc((double*)&hGPSRmcData.timestamp, (uint8_t*)pData, 3);
            hGPSRmcData.hour = ((uint32_t)hGPSRmcData.timestamp) / 10000;
            hGPSRmcData.minute = ((uint32_t)hGPSRmcData.timestamp) % 10000 / 100;
            hGPSRmcData.second = ((uint32_t)hGPSRmcData.timestamp) % 100;
            gCmdIndex = 2;
            break;

        case 2:			//字段2		定位状态
            hGPSRmcData.valid = *pData;
            gCmdIndex = 3;
            break;

        case 3:			//字段3		纬度
            dtoc((double*)&hGPSRmcData.latitude, (uint8_t*)pData, 8);
            gCmdIndex = 4;
            break;

        case 4:			//字段4		纬度半球
            hGPSRmcData.LatHemisphere = *pData;
            gCmdIndex = 5;
            break;

        case 5:			//字段5		经度
            dtoc((double*)&hGPSRmcData.longitude, (uint8_t*)pData, 8);
            gCmdIndex = 6;
            break;

        case 6:			//字段6		经度半球
            hGPSRmcData.LonHemisphere = *pData;
            gCmdIndex = 7;
            break;

        case 7:			//字段7		地面速率
            dtoc((double*)&hGPSRmcData.rate, (uint8_t*)pData, 2);
            gCmdIndex = 8;
            break;

        case 8:			//字段8		地面航向
            dtoc((double*)&hGPSRmcData.courseAngle, (uint8_t*)pData, 2);
            gCmdIndex = 9;
            break;

        case 9:			//字段9		UTC日期
            hGPSRmcData.date = atoi(pData);
            hGPSRmcData.year = hGPSRmcData.date / 10000;
            hGPSRmcData.month = hGPSRmcData.date % 10000 / 100;
            hGPSRmcData.day = hGPSRmcData.date / 100;
            gCmdIndex = 10;
            break;

        case 10:		//字段10	磁偏角
            dtoc((double*)&hGPSRmcData.magDeclination, (uint8_t*)pData, 2);
            gCmdIndex = 11;
            break;

        case 11:		//字段11	磁偏角方向
            dtoc((double*)&hGPSRmcData.MagDeclinationDir, (uint8_t*)pData, 2);
            gCmdIndex = 12;
            break;

        case 12:		//字段12	模式指示
            hGPSRmcData.mode = *pData;
            gCmdIndex = 13;
            break;

        case 13:		//字段13	校验值
            break;

        default:
            break;
    }

    if(gCmdIndex == 13)
        return 1;
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
            if(NULL != strstr(pData, "$GPSGGA"))
                gCmdIndex = 1;

            break;

        case 1:				//字段1		UTC时间
            //dtoc((double*)&hGPSGgaData.timestamp, (uint8_t*)pData, 3);
            hGPSGgaData.timestamp = atof(pData);
            //str2time(pData, 2, 8, &t);
            //hGPSGgaData.gps_time = (uint32_t)time2gpst(t, (int*)&hGPSGgaData.gps_week);
            gCmdIndex = 2;
            break;

        case 2:				//字段2		纬度
            dtoc((double*)&hGPSGgaData.latitude, (uint8_t*)pData, 8);
            gCmdIndex = 3;
            break;

        case 3:				//字段3		纬度半球
            hGPSGgaData.LatHemisphere = *pData;
            gCmdIndex = 4;
            break;

        case 4:				//字段4		经度
            dtoc((double*)&hGPSGgaData.longitude, (uint8_t*)pData, 8);
            gCmdIndex = 5;
            break;

        case 5:				//字段5		经度半球
            hGPSGgaData.LonHemisphere = *pData;
            gCmdIndex = 6;
            break;

        case 6:				//字段6		GPS状态
            hGPSGgaData.GPSState = (GNSS_State_TypeDef)( *pData - '0');
            gCmdIndex = 7;
            break;

        case 7:				//字段7		正使用的卫星数量
            hGPSGgaData.numSatellitesUsed = atoi(pData);
            gCmdIndex = 8;
            break;

        case 8:				//字段8		HDOP水平精度因子
            dtoc((double*)&hGPSGgaData.HDOP, (uint8_t*)pData, 2);
            gCmdIndex = 9;
            break;

        case 9:				//字段9		海拔高度
            dtoc((double*)&hGPSGgaData.height, (uint8_t*)pData, 2);
            gCmdIndex = 10;
            break;

        case 10:			//字段10	地球椭球面相对大地水准面的高度
            dtoc((double*)&hGPSGgaData.height_herizon, (uint8_t*)pData, 2);
            gCmdIndex = 11;
            break;

        case 11:			//字段11	差分时间
            dtoc((double*)&hGPSGgaData.differTime, (uint8_t*)pData, 2);
            gCmdIndex = 12;
            break;

        case 12:			//字段12	差分站ID号
            hGPSGgaData.differStationID = atoi(pData);
            gCmdIndex = 13;
            break;

        case 13:			//字段13	校验值
            break;

        default:
            break;
    }

    if(gCmdIndex == 13)
        return 1;
    else
        return 0;
}

//地面速度信息
int gnss_VTG_Buff_Parser(char* pData)
{
    switch(gCmdIndex)
    {
        case 0:				//字段0
            if(NULL != strstr(pData, "$GPVTG"))
                gCmdIndex = 1;

            if(NULL != strstr(pData, "$GLVTG"))
                gCmdIndex = 1;

            if(NULL != strstr(pData, "$GNVTG"))
                gCmdIndex = 1;

            break;

        case 1:				//字段1			以真北为参考基准的地面航向
            dtoc((double*)&hGPSVtgData.courseNorthAngle, (uint8_t*)pData, 4);
            gCmdIndex = 2;
            break;

        case 2:				//字段2			以磁北为参考基准的地面航向
            dtoc((double*)&hGPSVtgData.courseMagAngle, (uint8_t*)pData, 4);
            gCmdIndex = 3;
            break;

        case 3:				//字段3			地面速率 节
            dtoc((double*)&hGPSVtgData.rateKnots, (uint8_t*)pData, 4);
            gCmdIndex = 4;
            break;

        case 4:				//字段4			地面速率 公里/小时
            dtoc((double*)&hGPSVtgData.rateKm, (uint8_t*)pData, 4);
            gCmdIndex = 5;
            break;

        case 5:				//字段5			模式指示
            hGPSVtgData.mode = *pData;
            gCmdIndex = 6;
            break;

        case 6:				//字段6			校验值
            break;

        default:
            break;
    }

    if(gCmdIndex == 6)
        return 1;
    else
        return 0;
}

//日期
int gnss_ZDA_Buff_Parser(char* pData)
{
    gtime_t t;
    char str_time[6], year[4], month[2], day[2];
    char str[15] = {'\0'};

    switch(gCmdIndex)
    {
        case 0:					//字段0
            if(NULL != strstr(pData, "$GPZDA"))
                gCmdIndex = 1;

            if(NULL != strstr(pData, "$GLZDA"))
                gCmdIndex = 1;

            if(NULL != strstr(pData, "$GNZDA"))
                gCmdIndex = 1;

            break;

        case 1:					//字段1			UTC时间
            dtoc((double*)&hGPSZdaData.timestamp, (uint8_t*)pData, 3);
            hGPSZdaData.hour = ((uint32_t)hGPSZdaData.timestamp) / 10000;
            hGPSZdaData.minute = ((uint32_t)hGPSZdaData.timestamp) % 10000 / 100;
            hGPSZdaData.second = ((uint32_t)hGPSZdaData.timestamp) % 100;
            gCmdIndex = 2;
            strncpy(str_time, pData, 6);
            break;

        case 2:					//字段2			日期
            hGPSZdaData.day = atoi(pData);
            strncpy(day, pData, 2);
            gCmdIndex = 3;
            break;

        case 3:					//字段3			月份
            hGPSZdaData.month = atoi(pData);
            strncpy(month, pData, 2);
            gCmdIndex = 4;
            break;

        case 4:					//字段4			年份
            hGPSZdaData.year = atoi(pData);
            strncpy(year, pData, 4);
            sprintf(str, "%s%s%s%s", year, month, day, str_time);
            str2time(str, 1, 14, &t);
            hGPSGgaData.gps_time = (uint32_t)time2gpst(t, (int*)&hGPSGgaData.gps_week);
            gCmdIndex = 5;
            break;

        case 5:					//字段5			当地时域描述  小时
            hGPSZdaData.hour_timeDomain = atoi(pData);
            gCmdIndex = 6;
            break;

        case 6:					//字段6			当地时域描述  分钟
            hGPSZdaData.minute_timeDomain = atoi(pData);
            gCmdIndex = 7;
            break;

        case 7:					//字段7			校验和
            break;

        default:
            break;
    }

    if(gCmdIndex == 7)
        return 1;
    else
        return 0;
}

//方位角信息
int gnss_Heading_Buff_Parser(char* pData)
{
    switch(gCmdIndex)
    {
        case 0:					//字段0
            if(NULL != strstr(pData, "#HEADING"))
                gCmdIndex = 1;

            break;

        case 1:					//字段1				解算状态
            dtoc((double*)&hGPSGgaData.timestamp, (uint8_t*)pData, 3);
            gCmdIndex = 2;
            break;

        case 2:					//字段2				位置类型
            break;

        case 3:					//字段3				基线长
            break;

        case 4:					//字段4				航向角
            break;

        case 5:					//字段5				俯仰角
            break;

        case 6:					//字段6				保留
            break;

        case 7:					//字段7				航向标准偏差
            break;

        case 8:					//字段8				俯仰标准偏差
            break;

        case 9:					//字段9				基站ID
            break;

        case 10:				//字段10			跟踪的卫星数量
            break;

        case 11:				//字段11			使用的卫星数量
            break;

        case 12:				//字段12			截止高度角以上的卫星数
            break;

        case 13:				//字段13			截止高度角以上有L2观测的卫星数
            break;

        case 14:				//字段14			保留
            break;

        case 15:				//字段15			解状态
            break;

        case 16:				//字段16			保留
            break;

        case 17:				//字段17			信号掩码
            break;

        case 18:				//字段14			校验码
            break;

        default:
            break;
    }

    if(gCmdIndex == 18)
        return 1;
    else
        return 0;
}

//最佳位置
int gnss_BESTPOS_Buff_Parser(char * pData)
{
    switch(gCmdIndex)
    {
        case 0:				//字段0
            if(NULL != strstr(pData, "#BESTPOS"))
                gCmdIndex = 1;

            break;

        case 1:				//字段1			解算状态
            break;

        case 2:				//字段2			位置类型
            break;

        case 3:				//字段3			纬度
            break;

        case 4:				//字段4			经度
            break;

        case 5:				//字段5			海拔高
            break;

        case 6:				//字段6			高程异常
            break;

        case 7:				//字段7			坐标系id号
            break;

        case 8:				//字段8			纬度标准差
            break;

        case 9:				//字段9			经度标准差
            break;

        case 10:				//字段10			高度标准差
            break;

        case 11:				//字段11			基站ID
            break;

        case 12:				//字段12			差分龄期
            break;

        case 13:				//字段13			解得龄期
            break;

        case 14:				//字段14			跟踪的卫星数
            break;

        case 15:				//字段15			解算使用的卫星数
            break;

        case 16:				//字段16			保留
            break;

        case 17:				//字段17			保留
            break;

        case 18:				//字段18			保留
            break;

        case 19:				//字段19			保留
            break;

        case 20:				//字段20			保留
            break;

        case 21:				//字段21			保留
            break;

        case 22:				//字段22			信号掩码
            break;

        case 23:				//字段23			校验值
            break;

        default:
            break;
    }

    if(gCmdIndex == 23)
        return 1;
    else
        return 0;
}

//方向角输出
int gnss_TRA_Buff_Parser(char * pData)
{
    switch(gCmdIndex)
    {
        case 0:					//字段0
            if(NULL != strstr(pData, "$GPTRA"))
                gCmdIndex = 1;

            break;

        case 1:					//字段1				UTC时间
            dtoc((double*)&hGPSTraData.timestamp, (uint8_t*)pData, 3);
            hGPSZdaData.hour = ((uint32_t)hGPSTraData.timestamp) / 10000;
            hGPSZdaData.minute = ((uint32_t)hGPSTraData.timestamp) % 10000 / 100;
            hGPSZdaData.second = ((uint32_t)hGPSTraData.timestamp) % 100;
            gCmdIndex = 2;
            break;

        case 2:					//字段2				方向角
            dtoc((double*)&hGPSTraData.headingAngle, (uint8_t*)pData, 8);
            gCmdIndex = 3;
            break;

        case 3:					//字段3				俯仰角
            dtoc((double*)&hGPSTraData.pitchAngle, (uint8_t*)pData, 8);
            gCmdIndex = 4;
            break;

        case 4:					//字段4				横滚角
            dtoc((double*)&hGPSTraData.rollAngle, (uint8_t*)pData, 8);
            gCmdIndex = 5;
            break;

        case 5:					//字段5				GPS质量指示符
            gCmdIndex = 6;
            break;

        case 6:					//字段6				卫星数
            hGPSTraData.starNum = atoi(pData);
            gCmdIndex = 7;
            break;

        case 7:					//字段7				差分延迟
            dtoc((double*)&hGPSTraData.Age, (uint8_t*)pData, 8);
            gCmdIndex = 8;
            break;

        case 8:					//字段8				基站号
            hGPSTraData.stationID = atoi(pData);
            gCmdIndex = 9;
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

void gnss_Fetch_Data(void)
{
    hGPSData.timestamp = hGPSZdaData.timestamp;
    hGPSData.StarNum = hGPSHeadingData.numSatellitesTracked;
    hGPSData.ResolveState = hGPSHeadingData.calcState;
    hGPSData.PositioningState = hGPSRmcData.valid;
    hGPSData.PositionType = hGPSHeadingData.posType;
    hGPSData.LonHemisphere = hGPSRmcData.LonHemisphere;
    hGPSData.Lon = hGPSRmcData.longitude;
    hGPSData.LatHemisphere = hGPSRmcData.LatHemisphere;
    hGPSData.Lat = hGPSRmcData.latitude;
    hGPSData.Altitude = hGPSBestPosData.height;
    hGPSData.Heading = hGPSHeadingData.courseAngle;
    hGPSData.Pitch = hGPSHeadingData.pitchAngle;
    hGPSData.LonStd = hGPSBestPosData.longitudeStandardDeviation;
    hGPSData.LatStd = hGPSBestPosData.latitudeStandardDeviation;
    hGPSData.AltitudeStd = hGPSBestPosData.heightStandardDeviation;
    hGPSData.HeadingStd = hGPSHeadingData.courseStandardDeviation;
    hGPSData.PitchStd = hGPSHeadingData.pitchStandardDeviation;
    hGPSData.HDOP = hGPSGgaData.HDOP;
    hGPSData.GroundSpeed = hGPSRmcData.rate;
}

uint8_t gnss_parse(uint8_t* pData, uint16_t dataLen)
{
//  GPSDataTypeDef tGPSData;
    char * pBuf;
    char * pRxBuf = (char*)pData;
    char gnssCommaSeperator[2] = {',', 0};
    char gnssSemicolonSeperator[2] = {';', 0};
    int cmdKind = 0;
    uint16_t len = 0;
    uint8_t ret = INS_EOK;

    for(;;)
    {
        if(gCmdIndex == 0)
        {
            cmdKind = GNSS_Cmd_Kind_Parser(pRxBuf);//命令起始符
            gCmdTypeIndex = GNSS_Cmd_Parser(pRxBuf);//命令类型
        }

        if(cmdKind != 0)
        {
            if(cmdKind == 1)
                pBuf = strtok(pRxBuf, gnssSemicolonSeperator);
            else if(cmdKind == 2)
                pBuf = strtok(pRxBuf, gnssCommaSeperator);

            gCmdIndex = 1;

            while(NULL != pBuf)
            {
                GNSS_Buff_Parser(pBuf, gCmdTypeIndex);
                pBuf = strtok(NULL, gnssCommaSeperator);
                gCmdIndex ++;
            }

            break;
        }

        pRxBuf++;
        len++;

        if(len > dataLen)
        {
            ret = INS_ERROR;
            break;
        }
    }

    return ret;
}

void gnss_fill_rs422(RS422_FRAME_DEF* rs422)
{
    if(gCmdTypeIndex == 2)
    {
        rs422->data_stream.latitude = hGPSGgaData.latitude;
        rs422->data_stream.longitude = hGPSGgaData.longitude;
        rs422->data_stream.altitude = hGPSGgaData.height;
        rs422->data_stream.status |= BIT(0);
    }
    else if(gCmdTypeIndex == 4)
    {
        rs422->data_stream.gps_week = hGPSGgaData.gps_week;
        rs422->data_stream.poll_frame.gps_time = hGPSGgaData.gps_time;
    }
}

