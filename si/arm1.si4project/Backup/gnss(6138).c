#include "gnss.h"

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


gnss_parse_pattern_t gnss_pattern;

int GNSS_Cmd_Kind_Parser(char* pData)
{
	if(*pData == '#')
		return 1;
	else if(*pData == '$')
		return 2;
	return 0;
}

gnss_parse_pattern_t GNSS_Cmd_Parser(char* pData)
{
	if(NULL != strstr(pData,"$GPRMC"))
	{
		gCmdTypeIndex = 1;
		gnss_pattern.index = 1;
		gnss_pattern.ptr = strstr(pData,"$GPRMC");
	}
	else if(NULL != strstr(pData,"$GLRMC"))
	{
		gCmdTypeIndex = 1;
		gnss_pattern.index = 1;
		gnss_pattern.ptr = strstr(pData,"$GLRMC");
	}
	else if(NULL != strstr(pData,"$GNRMC"))
	{
		gCmdTypeIndex = 1;
		gnss_pattern.index = 1;
		gnss_pattern.ptr = strstr(pData,"$GNRMC");
	}
	else if(NULL != strstr(pData,"$GPGGA"))
	{
		gCmdTypeIndex = 2;
		gnss_pattern.index = 2;
		gnss_pattern.ptr = strstr(pData,"$GPGGA");
	}
	else if(NULL != strstr(pData,"$GLGGA"))
	{
		gCmdTypeIndex = 2;
		gnss_pattern.index = 2;
		gnss_pattern.ptr = strstr(pData,"$GLGGA");
	}
	else if(NULL != strstr(pData,"$GNGGA"))
	{
		gCmdTypeIndex = 2;
		gnss_pattern.index = 2;
		gnss_pattern.ptr = strstr(pData,"$GNGGA");
	}
	else if(NULL != strstr(pData,"$GPVTG"))
	{
		gCmdTypeIndex = 3;
		gnss_pattern.index = 3;
		gnss_pattern.ptr = strstr(pData,"$GPVTG");
	}
	else if(NULL != strstr(pData,"$GLVTG"))
	{
		gCmdTypeIndex = 3;
		gnss_pattern.index = 3;
		gnss_pattern.ptr = strstr(pData,"$GLVTG");
	}
	else if(NULL != strstr(pData,"$GNVTG"))
	{
		gCmdTypeIndex = 3;
		gnss_pattern.index = 3;
		gnss_pattern.ptr = strstr(pData,"$GNVTG");
	}
	else if(NULL != strstr(pData,"$GPZDA"))
	{
		gCmdTypeIndex = 4;
		gnss_pattern.index = 4;
		gnss_pattern.ptr = strstr(pData,"$GPZDA");
	}
	else if(NULL != strstr(pData,"$GLZDA"))
	{
		gCmdTypeIndex = 4;
		gnss_pattern.index = 4;
		gnss_pattern.ptr = strstr(pData,"$GLZDA");
	}
	else if(NULL != strstr(pData,"$GNZDA"))
	{
		gCmdTypeIndex = 4;
		gnss_pattern.index = 4;
		gnss_pattern.ptr = strstr(pData,"$GNZDA");
	}
	else if(NULL != strstr(pData,"#HEADING"))
	{
		gCmdTypeIndex = 5;
		gnss_pattern.index = 5;
		gnss_pattern.ptr = strstr(pData,"#HEADING");
	}
	else if(NULL != strstr(pData,"#BESTPOS"))
	{
		gCmdTypeIndex = 6;
		gnss_pattern.index = 6;
		gnss_pattern.ptr = strstr(pData,"#BESTPOS");
	}
	else if(NULL != strstr(pData,"$GPTRA"))
	{
		gCmdTypeIndex = 7;
		gnss_pattern.index = 7;
		gnss_pattern.ptr = strstr(pData,"$GPTRA");
	}
	return gnss_pattern;
}

void GNSS_Buff_Parser(void)
{
	switch(gnss_pattern.index)
	{
		case 1:gnss_RMC_Buff_Parser(gnss_pattern.ptr);break;
		case 2:gnss_GGA_Buff_Parser(gnss_pattern.ptr);break;
		case 3:gnss_VTG_Buff_Parser(gnss_pattern.ptr);break;
		case 4:gnss_ZDA_Buff_Parser(gnss_pattern.ptr);break;
		case 5:gnss_Heading_Buff_Parser(gnss_pattern.ptr);break;
		case 6:gnss_BESTPOS_Buff_Parser(gnss_pattern.ptr);break;
		case 7:gnss_TRA_Buff_Parser(gnss_pattern.ptr);break;
		default: break;
	}
}

//推荐定位
void gnss_RMC_Buff_Parser(char* pData)
{
	uint8_t gap;
	char*p,*p1;
	p = strstr(pData,",");//第一个","位置
	p++;
	p1 = strstr(p,",");//第二个","位置
	gap = p1 - p;
	//字段1		UTC时间
	if(gap > 7)//有数据
	{
		hGPSRmcData.timestamp = (float)strtod(p,NULL);
	}
	p = p1;
	p++;
	p1 = strstr(p,",");
	gap = p1 - p;
	//字段2		定位状态
	if(gap == 1)//有数据
	{
		hGPSRmcData.valid = *p;
	}
	p = p1;
	p++;
	p1 = strstr(p,",");
	gap = p1 - p;
	//字段3		纬度
	if(gap > 6)//有数据
	{
		hGPSRmcData.latitude = strtod(p,NULL);
	}
	p = p1;
	p++;
	p1 = strstr(p,",");
	gap = p1 - p;
	//字段4		纬度方向
	if(gap == 1)//有数据
	{
		hGPSRmcData.LatHemisphere = *p;
	}
	p = p1;
	p++;
	p1 = strstr(p,",");
	gap = p1 - p;
	//字段5		经度
	if(gap > 6)//有数据
	{
		hGPSRmcData.longitude = strtod(p,NULL);
	}
	p = p1;
	p++;
	p1 = strstr(p,",");
	gap = p1 - p;
	//字段6		经度方向
	if(gap == 1)//有数据
	{
		hGPSRmcData.LonHemisphere = *p;
	}
	p = p1;
	p++;
	p1 = strstr(p,",");
	gap = p1 - p;
	//字段7		地面速率
	if(gap > 2)//有数据
	{
		hGPSRmcData.rate = strtod(p,NULL);
	}
	p = p1;
	p++;
	p1 = strstr(p,",");
	gap = p1 - p;
	//字段8		地面航向
	if(gap > 2)//有数据
	{
		hGPSRmcData.courseAngle = strtod(p,NULL);
	}
	p = p1;
	p++;
	p1 = strstr(p,",");
	gap = p1 - p;
	//字段9		UTC日期
	if(gap == 7)//有数据
	{
		hGPSRmcData.date = atoi(p);
		hGPSRmcData.year = hGPSRmcData.date /10000;
		hGPSRmcData.month = hGPSRmcData.date % 10000/100;
		hGPSRmcData.day = hGPSRmcData.date /100;
	}
	p = p1;
	p++;
	p1 = strstr(p,",");
	gap = p1 - p;
	//字段10	磁偏角
	if(gap == 2)//有数据
	{
		hGPSRmcData.magDeclination = strtod(p,NULL);
	}
	p = p1;
	p++;
	p1 = strstr(p,",");
	gap = p1 - p;
	//字段11	磁偏角方向
	if(gap == 1)//有数据
	{
		hGPSRmcData.MagDeclinationDir = *p;
	}
	p = p1;
	p++;
	//字段12	模式指示
	hGPSRmcData.mode = *p;
	
}

//定位信息
int gnss_GGA_Buff_Parser(char* pData)
{
	uint8_t gap;
	char*p,*p1;
	p = strstr(pData,",");//第一个","位置
	p++;
	p1 = strstr(p,",");//第二个","位置
	gap = p1 - p;
	//字段1		UTC时间
	if(gap > 7)//有数据
	{
		hGPSGgaData.timestamp = (float)strtod(p,NULL);
	}
	p = p1;
	p++;
	p1 = strstr(p,",");
	gap = p1 - p;
	//字段2		纬度
	if(gap > 6)//有数据
	{
		hGPSGgaData.latitude = strtod(p,NULL);
	}
	p = p1;
	p++;
	p1 = strstr(p,",");
	gap = p1 - p;
	//字段3		纬度方向
	if(gap == 1)//有数据
	{
		hGPSGgaData.LatHemisphere = *p;
	}
	p = p1;
	p++;
	p1 = strstr(p,",");
	gap = p1 - p;
	//字段4		经度
	if(gap > 6)//有数据
	{
		hGPSGgaData.longitude = strtod(p,NULL);
	}
	p = p1;
	p++;
	p1 = strstr(p,",");
	gap = p1 - p;
	//字段5		经度方向
	if(gap == 1)//有数据
	{
		hGPSGgaData.LonHemisphere = *p;
	}
	p = p1;
	p++;
	p1 = strstr(p,",");
	gap = p1 - p;
	//字段7		地面速率
	if(gap > 2)//有数据
	{
		hGPSRmcData.rate = strtod(p,NULL);
	}
	p = p1;
	p++;
	p1 = strstr(p,",");
	gap = p1 - p;
	//字段8		地面航向
	if(gap > 2)//有数据
	{
		hGPSRmcData.courseAngle = strtod(p,NULL);
	}
	p = p1;
	p++;
	p1 = strstr(p,",");
	gap = p1 - p;
	//字段9		UTC日期
	if(gap == 7)//有数据
	{
		hGPSRmcData.date = atoi(p);
		hGPSRmcData.year = hGPSRmcData.date /10000;
		hGPSRmcData.month = hGPSRmcData.date % 10000/100;
		hGPSRmcData.day = hGPSRmcData.date /100;
	}
	p = p1;
	p++;
	p1 = strstr(p,",");
	gap = p1 - p;
	//字段10	磁偏角
	if(gap == 2)//有数据
	{
		hGPSRmcData.magDeclination = strtod(p,NULL);
	}
	p = p1;
	p++;
	p1 = strstr(p,",");
	gap = p1 - p;
	//字段11	磁偏角方向
	if(gap == 1)//有数据
	{
		hGPSRmcData.MagDeclinationDir = *p;
	}
	p = p1;
	p++;
	//字段12	模式指示
	hGPSRmcData.mode = *p;
	switch(gCmdIndex)
	{
		case 0:				//字段0		
			if(NULL != strstr(pData,"$GPSGGA"))
				gCmdIndex = 1;
			break;
		case 1:				//字段1		UTC时间
			dtoc((double*)&hGPSGgaData.timestamp,(uint8_t*)pData,3);
			gCmdIndex = 2;
			break;
		case 2:				//字段2		纬度
			dtoc((double*)&hGPSGgaData.latitude,(uint8_t*)pData,8);
			gCmdIndex = 3;
			break;
		case 3:				//字段3		纬度半球
			hGPSGgaData.LatHemisphere = *pData;
			gCmdIndex = 4;
			break;
		case 4:				//字段4		经度
			dtoc((double*)&hGPSGgaData.longitude,(uint8_t*)pData,8);
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
			dtoc((double*)&hGPSGgaData.HDOP,(uint8_t*)pData,2);
			gCmdIndex = 9;
			break;
		case 9:				//字段9		海拔高度
			dtoc((double*)&hGPSGgaData.height,(uint8_t*)pData,2);
			gCmdIndex = 10;
			break;
		case 10:			//字段10	地球椭球面相对大地水准面的高度
			dtoc((double*)&hGPSGgaData.height_herizon,(uint8_t*)pData,2);
			gCmdIndex = 11;
			break;
		case 11:			//字段11	差分时间
			dtoc((double*)&hGPSGgaData.differTime,(uint8_t*)pData,2);
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
			if(NULL != strstr(pData,"$GPVTG"))
				gCmdIndex = 1;
			if(NULL != strstr(pData,"$GLVTG"))
				gCmdIndex = 1;
			if(NULL != strstr(pData,"$GNVTG"))
				gCmdIndex = 1;
			break;
		case 1:				//字段1			以真北为参考基准的地面航向
			dtoc((double*)&hGPSVtgData.courseNorthAngle,(uint8_t*)pData,4);
			gCmdIndex = 2;
			break;
		case 2:				//字段2			以磁北为参考基准的地面航向
			dtoc((double*)&hGPSVtgData.courseMagAngle,(uint8_t*)pData,4);
			gCmdIndex = 3;
			break;
		case 3:				//字段3			地面速率 节
			dtoc((double*)&hGPSVtgData.rateKnots,(uint8_t*)pData,4);
			gCmdIndex = 4;
			break;
		case 4:				//字段4			地面速率 公里/小时
			dtoc((double*)&hGPSVtgData.rateKm,(uint8_t*)pData,4);
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
	switch(gCmdIndex)
	{
		case 0:					//字段0
			if(NULL != strstr(pData,"$GPZDA"))
				gCmdIndex = 1;
			if(NULL != strstr(pData,"$GLZDA"))
				gCmdIndex = 1;
			if(NULL != strstr(pData,"$GNZDA"))
				gCmdIndex = 1;
			break;
		case 1:					//字段1			UTC时间
			dtoc((double*)&hGPSZdaData.timestamp,(uint8_t*)pData,3);
			hGPSZdaData.hour = ((uint32_t)hGPSZdaData.timestamp)/10000;
			hGPSZdaData.minute = ((uint32_t)hGPSZdaData.timestamp)%10000/100;
			hGPSZdaData.second = ((uint32_t)hGPSZdaData.timestamp)%100;
			gCmdIndex = 2;
			break;
		case 2:					//字段2			日期
			hGPSZdaData.day = atoi(pData);
			gCmdIndex = 3;
			break;
		case 3:					//字段3			月份
			hGPSZdaData.month = atoi(pData);
			gCmdIndex = 4;
			break;
		case 4:					//字段4			年份
			hGPSZdaData.year = atoi(pData);
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
			if(NULL != strstr(pData,"#HEADING"))
				gCmdIndex = 1;
			break;
		case 1:					//字段1				解算状态
			dtoc((double*)&hGPSGgaData.timestamp,(uint8_t*)pData,3);
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
			if(NULL != strstr(pData,"#BESTPOS"))
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
			if(NULL != strstr(pData,"$GPTRA"))
				gCmdIndex = 1;
			break;
		case 1:					//字段1				UTC时间
			dtoc((double*)&hGPSTraData.timestamp,(uint8_t*)pData,3);
			hGPSZdaData.hour = ((uint32_t)hGPSTraData.timestamp)/10000;
			hGPSZdaData.minute = ((uint32_t)hGPSTraData.timestamp)%10000/100;
			hGPSZdaData.second = ((uint32_t)hGPSTraData.timestamp)%100;
			gCmdIndex = 2;
			break;
		case 2:					//字段2				方向角
			dtoc((double*)&hGPSTraData.headingAngle,(uint8_t*)pData,8);
			gCmdIndex = 3;
			break;
		case 3:					//字段3				俯仰角
			dtoc((double*)&hGPSTraData.pitchAngle,(uint8_t*)pData,8);
			gCmdIndex = 4;
			break;
		case 4:					//字段4				横滚角
			dtoc((double*)&hGPSTraData.rollAngle,(uint8_t*)pData,8);
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
			dtoc((double*)&hGPSTraData.Age,(uint8_t*)pData,8);
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
