#ifndef ____GNSS_H____
#define ____GNSS_H____

#undef COMMON_EXT
#ifdef  __GOL_GNSS_C__
#define COMMON_EXT
#else
#define COMMON_EXT extern
#endif

#include "gd32f4xx.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "data_convert.h"
#include "frame_analysis.h"

typedef enum coordinate_id_t {
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
} coordinateIDTypeDef;

typedef enum pos_speed_type_t {
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
    pos_type_INS_Invalid		=	0xff	//	无效的数据
} GNSS_POS_TypeDef,GNSS_VEL_TypeDef;

typedef enum resolve_type_t {
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
    resolve_Integrity_Invalid			=	0xff	//	无效的数据
} Resolve_TypeDef;

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
} Single_Mask_TypeDef;

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
} GNSS_State_TypeDef;


typedef struct gnss_tra_t
{
    float timestamp;					//时间戳
    double headingAngle;				//方向角
    double pitchAngle;					//俯仰角
    double rollAngle;					//横滚角
    GNSS_State_TypeDef SolStatus;		//GPS质量指示符
    int starNum;						//卫星数
    double Age;							//差分延迟
    uint32_t stationID;					//基站号
    uint32_t checkCode;					//校验和

} GNSS_TRA_DataTypeDef;

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
    double knot;						//地面速率
    float courseAngle;					//地面航向
    uint32_t date;						//日期
    uint8_t day;						//日
    uint8_t month;						//月
    uint8_t year;						//年
    double magDeclination;				//磁偏角
    double MagDeclinationDir;			//磁偏角方向
    char mode;							//模式
} GNSS_RMC_DataTypeDef;

typedef struct gnss_time_syn_t
{
    uint8_t year;
    uint8_t month;
    uint8_t date;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
} GNSS_TIME_SYN_DataTypeDef;

typedef struct gnss_vtg_t
{
    double courseNorthAngle;			//以真北为参考基准的地面航向
    char   trueNorth;					//真北参考系
    double courseMagAngle;				//以磁北为参考基准的地面航向
    char   magneticNorth;				//磁北参考系
    double rateKnots;					//地面速率(节)
    char   N;
    double rateKm;						//地面速率(公里/小时)
    char   K;
    char mode;							//模式
} GNSS_VTG_DataTypeDef;

typedef struct gnss_zda_t
{
    uint8_t hour;						//小时
    uint8_t minute;						//分钟
    uint8_t second;						//秒
    uint16_t microSecond;				//毫秒
    float timestamp;
    uint8_t day;						//日
    uint8_t month;						//月
    uint16_t year;						//年
    double hour_timeDomain;				//以真北为参考基准的地面航向
    double minute_timeDomain;			//以磁北为参考基准的地面航向
} GNSS_ZDA_DataTypeDef;


typedef struct
{
    char sync[10];			//同步字符
    float CPUIDle;  		//处理器空闲时间的最小百分比，每秒计算 1 次
    uint16_t gpsWn;			//GPS 周数
    uint32_t gpsMs;			//GPS 周内秒，精确到 ms

} BESTPOS_Header_TypeDef,BESTVEL_Header_TypeDef,HEADING_Header_TypeDef;


typedef struct gnss_heading_t
{
    HEADING_Header_TypeDef header;
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
} GNSS_Heading_DataTypeDef;

typedef struct gnss_bestpos_t
{
    BESTPOS_Header_TypeDef header;
    Resolve_TypeDef calcState;					//解算状态
    GNSS_POS_TypeDef posType;					//位置类型
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
} GNSS_BestPos_DataTypeDef;

typedef struct gnss_bestvel_t
{
    BESTVEL_Header_TypeDef header;
    Resolve_TypeDef calcState;					//解算状态
    GNSS_VEL_TypeDef velType;					//速度类型
    float latency;								//根据速度时标计算的延迟值，以秒为单位
    float age;									//差分龄期
    double horSpd;								//对地水平速度，m/s
    double trkGnd;								//相对于真北的实际对地运动方向（相对地面轨迹），deg
    double vertSpd;								//垂直速度，m/s，正值表示高度增加（向上），负值表示高度下降（向下）

} GNSS_BestVel_DataTypeDef;


typedef struct gnss_gga_t {
    float timestamp;							//时间戳
    double latitude;							//纬度
    char LatHemisphere;							//纬度半球
    double longitude;							//经度
    char LonHemisphere;							//经度半球
    GNSS_State_TypeDef GPSState;				//GPS状态
    uint8_t numSatellitesUsed;					//使用的卫星数
    float HDOP;									//HDOP水平精度因子
    float height;								//海拔高度
    char a_units ;								//天线高度单位（M = m）
    float undulation;							//大地水准面差距–大地水准面和 WGS84 椭球面之间的距离。大地水准面高于椭球面为正值，否则，为负值
    char u_units ;								//大地水准面差距单位
    uint16_t age;  								//差分数据龄期，秒为单位
    uint16_t stn_ID;  							//差分基站 ID
    uint32_t gps_week;							//GPS周
    uint32_t gps_time;							//GPS周内秒
} GNSS_GGA_DataTypeDef;


typedef struct GPS_Data_t
{
    float timestamp;					/* 时间戳, 单位: s , 精度: 0.0001*/
    uint8_t StarNum;					/* 星数 */
    uint8_t PositioningState;			/* 定位状态 */
    Resolve_TypeDef ResolveState[3];	/* 解算状态 */
    GNSS_POS_TypeDef PositionType[3];	/* 位置/速度/姿态类型 */
    char LonHemisphere;					/* 经度半球 E东经 或 W西经  */
    double Lon;							/* 经度, 单位: °, 精度: 1e-7*/
    char LatHemisphere;					/* 纬度半球 N北纬 或 S南纬 */
    double Lat;							/* 纬度, 单位: °, 精度: 1e-7*/
    float Altitude;						/* 高度, 单位: m, 精度:0.01*/
    float Heading;						/* 航向角, 单位: °, 精度: 0.01*/
    float Pitch;						/* 俯仰角, 单位: °, 精度: 0.01*/
    float Roll;							/* 横滚角, 单位: °, 精度: 0.01*/
    float baseline;						/* 基线长, 单位: m*/

    float HDOP;							/* HDOP水平精度因子 0.5 - 99.9 */
    float GroundSpeed;					/* 速度, 单位: m/s, 精度: 0.1*/
    unsigned int	gpsweek;
    uint32_t 		gpssecond;
    float			ve;
    float 		vn;
    float			vu;
    unsigned int	rtkStatus;
    unsigned char supportposvelstd;

    float LonStd;						/* 经度标准差, 单位: °*/
    float LatStd;						/* 纬度标准差, 单位: °*/
    float AltitudeStd;					/* 高度标准差, 单位: m*/

    double hdgstddev;					/* 航向角标准差, 单位： m*/
    double ptchstddev;					/* 俯仰角标准差, 单位: m */

    double		vestd;
    double		vnstd;
    double		vustd;

    uint16_t	gpscompms;
} GPSDataTypeDef;


typedef struct
{
    uint8_t index;
    char *string;
} RESOLV_Status;

typedef struct
{
    char sync[10];			//同步字符
    uint8_t CPUIDle;  		//处理器空闲时间的最小百分比，每秒计算 1 次
    char TimeRef[10];		//接收机工作的时间系统
    char TimeQuality[10];	//GPS 时间质量
    uint16_t gpsWn;			//GPS 周数
    uint32_t gpsMs;			//GPS 周内秒，精确到 ms
    uint8_t  leapSec; 		//闰秒
} Log_Header_TypeDef;

typedef struct
{
    Log_Header_TypeDef header;
    uint8_t  utc_year;	//utc年
    uint8_t  utc_month;	//utc月
    uint8_t  utc_day;	//utc日
    uint8_t  utc_hour;	//utc时
    uint8_t  utc_min;	//utc分
    uint8_t  utc_sec;	//utc秒
    uint8_t  rtk_status;//流动站定位状态 0:无效解 1：单点定位解 2：伪距差分 4：固定解 5：浮动解
    uint8_t  heading_status;//主从天线 Heading 解状态 0:无效解           4：固定解 5：浮动解
    uint8_t  num_gps_star;//参与解算 GPS 卫星数
    uint8_t  num_bd_star;//参与解算 BD 卫星数
    uint8_t  num_glo_star;//参与解算 GLO卫星数
    uint8_t  num_gal_star;//Galileo 卫星数
    float    Heading;  //航向角
    float    pitch;		//俯仰角
    float    roll;  //横滚角
    float    vN;	//北向速度
    float    vE;	//东向速度
    float    vZ;	//天顶速度
    float    sigma_vn;
    float    sigma_ve;
    float    sigma_vz;
    double   lat;//纬度
    double   lon;//经度
    double   alt;//高度
    float    sigma_lat;
    float    sigma_lon;
    float    sigma_alt;
    uint32_t GPS_Sec;	//周内秒(毫秒)
} GPS_AGRIC_TypeDef;

typedef struct arm1_to_kalam_mix_data
{
    uint8_t  utc_year;	//utc年
    uint8_t  utc_month;	//utc月
    uint8_t  utc_day;	//utc日
    uint8_t  utc_hour;	//utc时
    uint8_t  utc_min;	//utc分
    uint8_t  utc_sec;	//utc秒
    uint8_t  rtk_status;//流动站定位状态 0:无效解 1：单点定位解 2：伪距差分 4：固定解 5：浮动解
    uint8_t  num_gps_star;//参与解算 GPS 卫星数
    uint8_t  num_bd_star;//参与解算 BD 卫星数
    uint8_t  num_glo_ar;//参与解算 GLO卫星数
    float    Heading;  //航向角
    float    pitch;		//俯仰角
    float    roll;  //横滚角
    float    vN;	//北向速度
    float    vE;	//东向速度
    float    vZ;	//天顶速度
    float    sigma_vn;
    float    sigma_ve;
    float    sigma_vz;
    double   lat;//纬度
    double   lon;//经度
    double   alt;//高度
    float    sigma_lat;
    float    sigma_lon;
    float    sigma_alt;
    uint16_t GPS_Week;
    uint32_t GPS_Sec;	//周内秒(毫秒)
} ARM1_TO_KALAM_MIX_TypeDef;


COMMON_EXT GPSDataTypeDef hGPSData;						//GPS数据
COMMON_EXT uint8_t gCmdIndex;
COMMON_EXT uint8_t gCmdTypeIndex;
COMMON_EXT uint8_t gnssSynFlg;

int GNSS_Cmd_Kind_Parser(char* pData,uint16_t dataLen);
int GNSS_Cmd_Parser(char* pData);
void GNSS_Buff_Parser(char* pData, uint16_t nCmdIndex);
int gnss_RMC_Buff_Parser(char* pData);
int gnss_GGA_Buff_Parser(char* pData);
int gnss_VTG_Buff_Parser(char* pData);
int gnss_ZDA_Buff_Parser(char* pData);
int gnss_Heading_Buff_Parser(char* pData);
int gnss_BESTPOS_Buff_Parser(char * pData);
int gnss_BESTVEL_Buff_Parser(char * pData);
int gnss_TRA_Buff_Parser(char * pData);
int gnss_AGRIC_Buff_Parser(char * pData);
void gnss_Fetch_Data(void);
void gnss_fill_data(uint8_t* pData, uint16_t dataLen);
uint8_t gnss_isLocation(void);
uint8_t gnss_time_is_valid(void);

void gnss_config_movingbase(void);
void gnss_request_GGA(void);
void gnss_request_GSA(void);
void gnss_request_RMC(void);
void gnss_request_HDT(void);
void gnss_request_HEADING(void);
void gnss_request_OTG(void);
void gnss_request_ZDA(void);
void gnss_request_saveconfig(void);
uint8_t gnss_parse(uint8_t* pData, uint16_t dataLen);
void gnss_fill_rs422(RS422_FRAME_DEF* rs422);
GPS_AGRIC_TypeDef*  gnss_get_algorithm_dataPtr(void);

//INS 设置姿态和标准差
void gnss_set_posture(double pitch, double roll, double azimuth,
                      double pitchOffset, double rollOffset, double azimuthOffset);
//IMU 至从天线杆臂参数配置
void gnss_set_leverArm(double x, double y, double z,
                       double a, double b, double c);

//INS 输出位置偏移配置
void gnss_set_ins_offset(double xoffset, double yoffset, double zoffset);

#endif //____GNSS_H____
