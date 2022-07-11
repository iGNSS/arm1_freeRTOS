/*****************************************************************************************
***********************************************************************************
All rights reserved I-NAV 2022 2023
***********************************************************************************/

/***********************************************************************************
Modification history

|-----------+---------------+-------------------|
|Author          |    Date               |    Done                      |
|-----------+---------------+-------------------|
|DengWei       |  2022-6-8          | First Creation             |
|-----------+---------------+-------------------|  
***********************************************************************************/

#ifndef INS_H
#define INS_H

#include <math.h>
#include <stdlib.h>
#include "constant.h"
#include "inavgnss.h"

//??? 定义初始化角度偏差
#define INC_EB0    0.2
#define INC_EB1    0.2
#define INC_EB2    0.2



//惯导当前状态
typedef enum 
{
    NAV_INS_STATUS_IDLE =0,       
    NAV_INS_STATUS_START,			
    NAV_INS_STATUS_WAIT,	
    NAV_INS_STATUS_ROUTH_ALIGN,	
    NAV_INS_STATUS_KEEP,
    NAV_INS_STATUS_STOP
}ENavInsStatus ;

//静态检查枚举
typedef enum 
{
    MOTION_STATUS_UNKNOW =-1, //未知状态       
    MOTION_STATUS_STATIC	 =0, //静止状态			
    MOTION_STATUS_MOVING   =1, //移动状态
}EMotionStatus;

typedef struct KALMAN_T{
	double Rmax[NB], Rmin[NB];
	double beta, b;
	double Xk[NA];
	double Qk[NA * NA];
	double Hk[NB * NA];
	double Rk[NB * NB],RK[NB];
	double Pxk[NA * NA];
}KALMAN;

//大地数据
typedef struct EARTH_T{
	double sl, cl, tl;//sinL cosL tanL
	double RMh, RNh,clRNh;
	//东北天自转速度wnie[0] = 0.0, wnie[1] = wie * cosL, wnie[2] = wie * sinL;
	double wnie[3],wnen[3],wnin[3];
	double wnien[3];
	double g, gn[3];
	double gcc[3];
	
}EARTH;

// imu原始数据存储结构
typedef struct IMUDATA_T{
	float 		second;//时间用于和gps周内秒比较
	double 		gyro[3];
	double 		accm[3];
	float 		heading;
	float			pitch; 
	float			roll; 
	//保存上一次的数据
	double gyro_pre[3], accm_pre[3];
}IMUDATA;

//惯导结果
typedef struct INSRESULT_T{
	double ts, nts;//采样间隔
	double qnb[4],att[3],att_pre[3];//四元素、当前/上一个时刻经纬高程
	double vn[3];//当前速度
	double pos[3];//当前位置
	double eb[3], db[3];					// 陀螺的漂移和加速度计的偏差
	double an[3], fb[3], fn[3];//加速度、
	double web[3], wnb[3];
	double Mpv[3* 3];
	double learm[3];
}INSRESULT;


//惯导导航
typedef struct I_NAV_INS_T{
	ENavInsStatus	insStatus;
	EARTH			earth;
	IMUDATA 		imu;
	KALMAN 			kf;
	INSRESULT		ins; 
	double 			Fk[NA*NA];//kalman滤波F矩阵
	double 			dt;//惯导卫导时间差
}I_NAV_INS;


//静态检查结构体
typedef struct STATIC_DETECTION_T{
	double 		gyro[3][SAMPLE_SIZE];
	int 			index;							//当前缓存角速度索引号，缓存到达SAMPLE_SIZE个,进行1次静态检查
	int 			state;			 				//-1:初始状态；0：静态;1:运动
}STATIC_DETECTION;

//上位机设置的补偿量，目前包括杆臂补偿、航向角补偿
typedef struct COMPENSATE_PARAMS_T{
	double 		gnssArmLength[3];				//杆臂补偿
	float 		heading;						      //GNSS航向角补偿
}COMPENSATE_PARAMS;


#ifdef __cplusplus
extern "C" {
#endif
void InterfaceKalman();
void InitialNavIncParm();
unsigned int GetNavIncData(IMUDATA *p);
IMUDATA * GetNavIncImuPointer();
I_NAV_INS * GetNavIncPointer();
STATIC_DETECTION * GetStaticDetectPointer();
COMPENSATE_PARAMS * GetCompensateParmsPointer();
unsigned int StartCoarseAlign(I_NAV_INS * navins,  I_NAV_GNSS_RESULT *gnss);
void InitialStaticDetectParm();
void InsInit(INSRESULT *ins, double ts, double* qnb, I_NAV_GNSS_RESULT *pNAV_GNSS_RESULT,double *GnssArmLength,I_NAV_INS * navins) ;//惯导初始化
void KfInit(KALMAN* kf);  //卡尔曼滤波初始化
void InsUpdate(IMUDATA *imu,INSRESULT *ins,I_NAV_INS * navins) ;//惯导更新
void KfFk(INSRESULT * ins, double* Fk,I_NAV_INS * navins);//卡尔曼滤波、构建FK矩阵,计算一次状态更新矩阵
void static_detection(IMUDATA *imu,double *vn,double *acc, STATIC_DETECTION *pStaticDetect) ;//静态检测
void KfTimeUpdate(KALMAN* kf, double* Fk, double nts,int m); //kf时间更新
double difftimeInc2gnss(double inssecond, double gnsssecond);
void GnssInsFusion(INSRESULT* ins, I_NAV_GNSS_RESULT* gnss,KALMAN* kf ,double *lever ,double dt) ;
void KfFeedback(KALMAN* kf, INSRESULT* ins) ;
int StopNavigation();
//打印信息
void PrintOutGNSSMsg(I_NAV_GNSS_RESULT *p);
void PrintOutInsMsg(I_NAV_INS *p);

#ifdef __plusplus
}
#endif

#endif /* INS_H_ */
