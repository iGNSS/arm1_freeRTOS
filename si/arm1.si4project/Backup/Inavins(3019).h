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

//??? �����ʼ���Ƕ�ƫ��
#define INC_EB0    0.2
#define INC_EB1    0.2
#define INC_EB2    0.2



//�ߵ���ǰ״̬
typedef enum 
{
    NAV_INS_STATUS_IDLE =0,       
    NAV_INS_STATUS_START,			
    NAV_INS_STATUS_WAIT,	
    NAV_INS_STATUS_ROUTH_ALIGN,	
    NAV_INS_STATUS_KEEP,
    NAV_INS_STATUS_STOP
}ENavInsStatus ;

//��̬���ö��
typedef enum 
{
    MOTION_STATUS_UNKNOW =-1, //δ֪״̬       
    MOTION_STATUS_STATIC	 =0, //��ֹ״̬			
    MOTION_STATUS_MOVING   =1, //�ƶ�״̬
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

//�������
typedef struct EARTH_T{
	double sl, cl, tl;//sinL cosL tanL
	double RMh, RNh,clRNh;
	//��������ת�ٶ�wnie[0] = 0.0, wnie[1] = wie * cosL, wnie[2] = wie * sinL;
	double wnie[3],wnen[3],wnin[3];
	double wnien[3];
	double g, gn[3];
	double gcc[3];
	
}EARTH;

// imuԭʼ���ݴ洢�ṹ
typedef struct IMUDATA_T{
	float 		second;//ʱ�����ں�gps������Ƚ�
	double 		gyro[3];
	double 		accm[3];
	float 		heading;
	float			pitch; 
	float			roll; 
	//������һ�ε�����
	double gyro_pre[3], accm_pre[3];
}IMUDATA;

//�ߵ����
typedef struct INSRESULT_T{
	double ts, nts;//�������
	double qnb[4],att[3],att_pre[3];//��Ԫ�ء���ǰ/��һ��ʱ�̾�γ�߳�
	double vn[3];//��ǰ�ٶ�
	double pos[3];//��ǰλ��
	double eb[3], db[3];					// ���ݵ�Ư�ƺͼ��ٶȼƵ�ƫ��
	double an[3], fb[3], fn[3];//���ٶȡ�
	double web[3], wnb[3];
	double Mpv[3* 3];
	double learm[3];
}INSRESULT;


//�ߵ�����
typedef struct I_NAV_INS_T{
	ENavInsStatus	insStatus;
	EARTH			earth;
	IMUDATA 		imu;
	KALMAN 			kf;
	INSRESULT		ins; 
	double 			Fk[NA*NA];//kalman�˲�F����
	double 			dt;//�ߵ�����ʱ���
}I_NAV_INS;


//��̬���ṹ��
typedef struct STATIC_DETECTION_T{
	double 		gyro[3][SAMPLE_SIZE];
	int 			index;							//��ǰ������ٶ������ţ����浽��SAMPLE_SIZE��,����1�ξ�̬���
	int 			state;			 				//-1:��ʼ״̬��0����̬;1:�˶�
}STATIC_DETECTION;

//��λ�����õĲ�������Ŀǰ�����˱۲���������ǲ���
typedef struct COMPENSATE_PARAMS_T{
	double 		gnssArmLength[3];				//�˱۲���
	float 		heading;						      //GNSS����ǲ���
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
void InsInit(INSRESULT *ins, double ts, double* qnb, I_NAV_GNSS_RESULT *pNAV_GNSS_RESULT,double *GnssArmLength,I_NAV_INS * navins) ;//�ߵ���ʼ��
void KfInit(KALMAN* kf);  //�������˲���ʼ��
void InsUpdate(IMUDATA *imu,INSRESULT *ins,I_NAV_INS * navins) ;//�ߵ�����
void KfFk(INSRESULT * ins, double* Fk,I_NAV_INS * navins);//�������˲�������FK����,����һ��״̬���¾���
void static_detection(IMUDATA *imu,double *vn,double *acc, STATIC_DETECTION *pStaticDetect) ;//��̬���
void KfTimeUpdate(KALMAN* kf, double* Fk, double nts,int m); //kfʱ�����
double difftimeInc2gnss(double inssecond, double gnsssecond);
void GnssInsFusion(INSRESULT* ins, I_NAV_GNSS_RESULT* gnss,KALMAN* kf ,double *lever ,double dt) ;
void KfFeedback(KALMAN* kf, INSRESULT* ins) ;
int StopNavigation();
//��ӡ��Ϣ
void PrintOutGNSSMsg(I_NAV_GNSS_RESULT *p);
void PrintOutInsMsg(I_NAV_INS *p);

#ifdef __plusplus
}
#endif

#endif /* INS_H_ */
