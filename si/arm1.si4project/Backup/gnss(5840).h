#ifndef ____GNSS_H____
#define ____GNSS_H____

#include "gd32f4xx.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "data_convert.h"

typedef enum coordinate_id_t{
	coordinate_id_Airy_1830							= 1,
	coordinate_id_Modified_Airy						= 2,
	coordinate_id_Australian_National				= 3,
	coordinate_id_Bessel_1841						= 4,
	coordinate_id_Clarke_1866						= 5,
	coordinate_id_Clarke_1880						= 6,
	coordinate_id_Everest_India_1830				= 7,
	coordinate_id_Everest_Brunei_E_Malaysia			= 8,
	coordinate_id_Everest_W_Malaysia_Singapore		= 9,
	coordinate_id_Geodetic_Reference_System_1980	= 10,
	coordinate_id_Helmert_1906						= 11,
	coordinate_id_Hough_1960						= 12,
	coordinate_id_Parameters_of_the_Earth			= 13,
	coordinate_id_South_American_1969				= 14,
	coordinate_id_World_Geogetic_System_1972		= 15,
	coordinate_id_World_Geogetic_System_1984		= 16,
}coordinateIDTypeDef;

typedef enum pos_speed_type_t{
	pos_type_NONE				=	0,		//	无解
	pos_type_FixedPOS			=	1,		//	位置由Fix Position命令指定
	pos_type_FixedHeight		=	2,		//	暂不支持
	pos_type_DopplerVelocity	=	8,		//	速度由即时多普勒信息导出
	pos_type_Signle				=	16,		//	单点定位
	pos_type_PSRDiff			=	17,		//	伪距差分解
	pos_type_SBAS				=	18,		//	SBAS定位
	pos_type_L1_Float			=	32,		//	L1浮点解
	pos_type_IoNoFree_Float		=	33,		//	消电离层浮点解
	pos_type_Narrow_Float		=	34,		//	窄巷浮点解
	pos_type_L1_INT				=	48,		//	L1固定解
	pos_type_Wide_INT			=	49,		//	宽巷固定解
	pos_type_Narrow_INT			=	50,		//	窄巷固定解
	pos_type_INS				=	52,		//	纯惯导定位解
	pos_type_INS_PSRSP			=	53,		//	惯导与单点定位组合解
	pos_type_INS_PSDiff			=	54,		//	惯导与伪距差分定位组合解
	pos_type_INS_RTKFloat		=	55,		//	惯导与载波相位差分浮点解组合解
	pos_type_INS_RTKFixed		=	56,		//	惯导与载波相位差固定解组合解
}GNSS_POS_TypeDef;

typedef enum resolve_type_t{
	resolve_SOL_Computed				=	0,		//	已解出
	resolve_INSSufficient_OBS			=	1,		//	观察数据不足
	resolve_No_Convergence				=	2,		//	无法收敛
	resolve_Singularity					=	3,		//	参数矩阵的奇异性
	resolve_COV_Trace					=	4,		//	协方差矩阵的迹超过最大值 ( 迹 > 1000m )
	resolve_Test_Dist					=	5,		//	超出测试距离 ( 距离超过10km, 最多拒绝3次  )
	resolve_Cold_Start					=	6,		//	尚未从冷启动收敛
	resolve_V_H_Limit					=	7,		//	超出高度或速度限制
	resolve_Variance					=	8,		//	方差超出限制
	resolve_Residuals					=	9,		//	残差过大
	resolve_Integrity_Warning			=	13,		//	残差过大导致位置不可靠
}Resolve_TypeDef;

typedef enum single_mask_t
{
	single_mask_GPS_L1			=	1,			//	使用GPS L1计算
	single_mask_GPS_L2			=	2,			//	使用GPS L2计算
	single_mask_GPS_L5			=	4,			//	使用GPS L5计算
	single_mask_BDS_B3			=	8,			//	使用BDS B3计算
	single_mask_GOLNASS_L1		=	16,			//	使用GOLNASS L1计算
	single_mask_GOLNASS_L2		=	32,			//	使用GOLNASS L1计算
	single_mask_BDS_B1			=	64,			//	使用BDS B1计算
	single_mask_BDS_B2			=	128,		//	使用BDS B2计算
}Single_Mask_TypeDef;

typedef enum gnss_state_t
{
	gnss_state_NONE				= 0,
	gnss_state_SIGNLE_POINT		= 1,
	gnss_state_SBAS				= 2,
	gnss_state_INVALID_PPS		= 3,
	gnss_state_RTK_STABLE		= 4,
	gnss_state_RTK_FLOAT		= 5,
	gnss_state_VALUEING			= 6,
	gnss_state_MANUAL_BOOT		= 7,
	gnss_state_RTK_WIDE_LANE	= 8,
	gnss_state_PSEUDORANGE		= 9,
}GNSS_State_TypeDef;


typedef struct gnss_tra_t
{
	float timestamp;					//时间戳
	double headingAngle;				//方向角
	double pitchAngle;					//俯仰角
	double rollAngle;					//横滚角
	char SolStatus;						//GPS质量指示符
	int starNum;						//卫星数
	double Age;							//差分延迟
	uint32_t stationID;					//基站号
	uint32_t checkCode;					//校验和

}GNSS_TRA_DataTypeDef;

typedef struct gnss_rmc_t
{
	uint8_t hour;						//小时
	uint8_t minute;						//分钟
	uint8_t second;						//秒
	uint16_t microSecond;				//毫秒
	float timestamp;					//时间戳
	char valid;							//定位有效位标志
	double latitude;					//纬度
	char LatHemisphere;					//纬度半球
	double longitude;					//经度
	char LonHemisphere;					//经度半球
	double rate;						//地面速率
	double courseAngle;					//地面航向
	uint32_t date;						//日期
	uint8_t day;						//日
	uint8_t month;						//月
	uint8_t year;						//年
	double magDeclination;				//磁偏角
	double MagDeclinationDir;			//磁偏角方向
	char mode;							//模式
}GNSS_RMC_DataTypeDef;

typedef struct gnss_vtg_t
{
	double courseNorthAngle;			//以真北为参考基准的地面航向
	double courseMagAngle;				//以磁北为参考基准的地面航向
	double rateKnots;					//地面速率(节)
	double rateKm;						//地面速率(公里/小时)
	char mode;							//模式
}GNSS_VTG_DataTypeDef;

typedef struct gnss_zda_t
{
	uint8_t hour;						//小时
	uint8_t minute;						//分钟
	uint8_t second;						//秒
	uint16_t microSecond;				//毫秒
	float timestamp;
	uint8_t day;						//日
	uint8_t month;						//月
	uint8_t year;						//年
	double hour_timeDomain;				//以真北为参考基准的地面航向
	double minute_timeDomain;			//以磁北为参考基准的地面航向
}GNSS_ZDA_DataTypeDef;



typedef struct gnss_heading_t
{
	Resolve_TypeDef calcState;					//解算状态
	GNSS_POS_TypeDef posType;					//位置类型
	uint16_t baseLen;							//基线长
	double courseAngle;							//航向角
	double pitchAngle;							//俯仰角
	double courseStandardDeviation;				//航向标准偏差
	double pitchStandardDeviation;				//俯仰标准偏差
	uint32_t baseStationID;						//基站id
	uint8_t numSatellitesTracked;				//跟踪的卫星数
	uint8_t numSatellitesUsed;					//使用的卫星数
	uint8_t numSatellitesAboveCutoffAngle;		//截止高度角以上的卫星数
	uint8_t numSatellitesAboveCutoffAngleL2;	//截止高度角以上有L2观测的卫星数
	uint8_t solutionState;						//解状态
	Single_Mask_TypeDef sigMask;				//信号掩码
}GNSS_Heading_DataTypeDef;


typedef struct gnss_bestpos_t
{
	char calcState[50];							//解算状态
	char posType[50];							//位置类型
	double latitude;							//纬度
	double longitude;							//经度
	double height;								//高度
	double heightVariance;						//高程异常
	coordinateIDTypeDef coordinateID;			//坐标系id号
	double latitudeStandardDeviation;			//纬度标准差
	double longitudeStandardDeviation;			//经度标准差
	double heightStandardDeviation;				//高度标准差
	uint8_t stationID;							//基站ID
	double differPeriod;						//差分龄期
	double solutionPeriod;						//解得龄期
	uint8_t numSatellitesTracked;				//跟踪的卫星数
	uint8_t numSatellitesUsed;					//使用的卫星数
}GNSS_BestPos_DataTypeDef;



typedef struct gnss_gga_t{
	float timestamp;							//时间戳
	double latitude;							//纬度
	char LatHemisphere;							//纬度半球
	double longitude;							//经度
	char LonHemisphere;							//经度半球
	GNSS_State_TypeDef GPSState;				//GPS状态
	uint8_t numSatellitesUsed;					//使用的卫星数
	float HDOP;									//HDOP水平精度因子
	float height;								//海拔高度
	float height_herizon;						//地球椭球面相对大地水准面的高度
	float differTime;							//差分时间
	uint16_t differStationID;					//差分站ID号
}GNSS_GGA_DataTypeDef;



typedef struct GPS_Data_t
{
	float timestamp;					/* 时间戳, 单位: s , 精度: 0.0001*/
	uint8_t StarNum;					/* 星数 */
	Resolve_TypeDef ResolveState;		/* 解算状态 */
	uint8_t PositioningState;			/* 定位状态 */
	GNSS_POS_TypeDef PositionType;		/* 位置类型 */
	char LonHemisphere;					/* 经度半球 E东经 或 W西经  */
	float Lon;							/* 经度, 单位: °, 精度: 1e-7*/
	char LatHemisphere;					/* 纬度半球 N北纬 或 S南纬 */
	float Lat;							/* 纬度, 单位: °, 精度: 1e-7*/
	float Altitude;						/* 高度, 单位: m, 精度:0.01*/
	float Heading;						/* 航向角, 单位: °, 精度: 0.01*/
	float Pitch;						/* 俯仰角, 单位: °, 精度: 0.01*/
	float Roll;							/* 横滚角, 单位: °, 精度: 0.01*/
	float LonStd;						/* 经度标准差, 单位: °*/
	float LatStd;						/* 纬度标准差, 单位: °*/
	float AltitudeStd;					/* 高度标准差, 单位: m*/
	float HeadingStd;					/* 航向角标准差, 单位： m*/
	float PitchStd;						/* 俯仰角标准差, 单位: m */
	float HDOP;							/* HDOP水平精度因子 0.5 - 99.9 */
	float GroundSpeed;					/* 速度, 单位: m/s, 精度: 0.1*/
}GPSDataTypeDef;


extern GPSDataTypeDef hGPSData;						//GPS数据
extern uint8_t gCmdIndex;
extern uint8_t gCmdTypeIndex;

int GNSS_Cmd_Kind_Parser(char* pData);
int GNSS_Cmd_Parser(char* pData);
void GNSS_Buff_Parser(char* pData, uint16_t nCmdIndex);
int gnss_RMC_Buff_Parser(char* pData);
int gnss_GGA_Buff_Parser(char* pData);
int gnss_VTG_Buff_Parser(char* pData);
int gnss_ZDA_Buff_Parser(char* pData);
int gnss_Heading_Buff_Parser(char* pData);
int gnss_BESTPOS_Buff_Parser(char * pData);
int gnss_TRA_Buff_Parser(char * pData);
void gnss_config_movingbase(void);
void gnss_request_GGA(void);
void gnss_request_GSA(void);
void gnss_request_RMC(void);
void gnss_request_HDT(void);
void gnss_request_HEADING(void);
void gnss_request_OTG(void);
void gnss_request_ZDA(void);
void gnss_request_saveconfig(void);

#endif //____GNSS_H____
