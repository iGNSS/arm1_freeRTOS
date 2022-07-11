/*****************************************************************************************
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
#ifndef _INAV_GNSS_H
#define _INAV_GNSS_H


enum ENavGnssStatus 
{
    NAV_GNSS_STATUS_LOST =0,       	//ʧ��
    NAV_GNSS_STATUS_SPP=1,			//���㶨λ
    NAV_GNSS_STATUS_RTD=2,		//α����
    NAV_GNSS_STATUS_RTK_FIX=4,	      //RTK�̶���
    NAV_GNSS_STATUS_RTK_FLOAT=5,	//RTK�����
};


#include "constant.h"
// GNSSԭʼ����
typedef struct I_NAV_GNSS_RESULT_T {
	unsigned int	gpsweek; //gps���� 1980-1-6 ����ǰ���������� ��������ʱ�䣩 
	float 		gpssecond;//gps�������Ա����� 00:00:00 ����ǰ�������� ��������ʱ�䣩
	float 		heading;//ƫ���ǣ� 0~359.99�� 
	float			pitch;//�����ǣ� -90~90�� 
	float			roll;//����ǣ� -180~180�� 
	float			latitude;//γ�ȣ� -90~90�� 
	float			longitude;//���ȣ� -180~180��
	float			altitude;//�߶ȣ� ��λ�� �ף� 
	float			ve;//�����ٶȣ� ��λ�� ��/�룩 
	float 		vn;//�����ٶȣ� ��λ�� ��/�룩
	float			vu;//�����ٶȣ� ��λ�� ��/�룩
	float			baseline;//���߳��ȣ� ��λ�� �ף� 
	unsigned int	nsv;//������ 
	unsigned int	gnsslocatestatus;
	unsigned int	gnssstatus;//ϵͳ״̬��0�� ʧ���� ���㶨λ2�� α����4�� �ز���λ��֣� ���㣩5�� �ز���λ��֣� ���㣩
	double		utc;//UTC ʱ��

	//�Ƿ��ṩ׼ȷ��λ���ȶ������
	unsigned char supportposvelstd; //0��֧�֣�1֧��
	//������Ϣ
	//��λ����
	double		latstd;//γ�Ⱦ���
	double		logstd;//���Ⱦ���
	double		hstd;//�߳̾���
	//���پ���
	double		vestd;//�����ٶȾ���
	double		vnstd;//�����ٶȾ���
	double		vustd;//�����ٶȾ���

	//gnss������־
	unsigned int	gnssstartflag;// 1: �Ѿ����Ի�ȡgnss���
		
}I_NAV_GNSS_RESULT;


#ifdef __cplusplus
extern "C" {
#endif
void InitialGnssResultParm(void);
I_NAV_GNSS_RESULT *GetGnssResultPointer(void);

#ifdef __cplusplus
}
#endif

#endif
