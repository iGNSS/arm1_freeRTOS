/***********************************************************************************
This file DEFINED all header files of Constant parmameter
Application using constant parmameter should include this file first.
All rights reserved I-NAV 2022 2023
***********************************************************************************/

/***********************************************************************************
Modification history

|-----------+---------------+-------------------|
|Author          |    Date               |    Done                      |
|-----------+---------------+-------------------|
|DengWei       |  2022-6-2          | First Creation             |
|-----------+---------------+-------------------|  
***********************************************************************************/
/*****************************************************************
������:
1,�ṹ���ʹ�ô�д��ĸ+�»������
2. �ṹ�������ʹ��Сд��ĸ
3. ȫ�ֱ�������g_XXX��ʼ
4. ����ʹ������ĸ��д��ʽ
******************************************************************/


#ifndef _CONSTANT_H
#define _CONSTANT_H



#ifndef PI
#define PI        								(3.1415926535897932384626433832795) 
#endif 

#ifndef WIE
#define WIE									7.29211506e-5
#endif

#ifndef WGS84G0
#define WGS84G0								9.780318
#endif

#ifndef eps
#define eps                      						2.2204e-16
#endif

#define DEG2RAD								(PI/180.0)          /* deg to rad */
#define RAD2DEG     							(180.0/PI)          /* rad to deg */
//---------------------------------------------------------------------------------------------
#define F1									(   2.0 * 1)						// 2
#define F2									((F1)*2 * 2)						// 8
#define F3									((F2)*2 * 3)						// 48
#define F4									((F3)*2 * 4)						// 384
#define F5									((F4)*2 * 5)						// 3840
/****************************************************************************************/



#define ZIHR									1							//0:�رպ���Լ����1���򿪺���Լ��

#define Re									6378137.0						// WGS84����ϵ������
#define F									1/298.257							// ����
#define wie									7.2921151467e-5					// ��ת���ٶȣ�rad��
#define Rp									((1-(F))*(Re))						// �̰���
#define G0									9.7803267714						// ��������
#define cs_2									(2.0/3.0)						// ˫��������ϵ��
#define e									sqrt(2*(F)-(F)*(F))						// ��һƫ����
#define ep									(sqrt((Re)*(Re)-(Rp)*(Rp))/(Rp))			// �ڶ�ƫ����

#define DEG									((PI)/180.0)						// �Ƕ�ת����
#define DPH									((DEG)/3600.0)
#define UG									((G0)/1.0e6)

#define DPSH									((DEG)/60)						// rad/(sqrt(hour))
#define UGPSHZ								((UG)/1.0)						// ug/(sqrt(Hz))

//-----------------------------------------------------------------------------
#define NN									3						// 3��3�����ά��

#define SAMPLE_FREQ						100						//jiang  �������ʵ��������ò���ʱ��
#define SAMPLE_FREQ_GNSS					1									//GNSSƵ��
#define  SAMPLE_Ratio						SAMPLE_FREQ/SAMPLE_FREQ_GNSS //jiang,��Ҫ���ǣ�SAMPLE_Ratio ? SAMPLE_FREQ or SAMPLE_FREQ/SAMPLE_FREQ_GNSS
#define TS									1.0/SAMPLE_FREQ			// ���ݲ���

#define NA									15						// ״̬������ά��  +�˱�
#define NB									6						// ���ⷽ�̵�ά��

#define ALN									7

//--------------ZIHR----------------------------------------------
#define SAMPLE_SIZE							SAMPLE_FREQ							//��̬̽����������
#define VAR_THRESHSHOLD					5e-9								//������ֵ
#define ACCL_THRESHSHOLD					9.8								//���ٶȼƵ���ֵ
#define VN_3D_THRESHSHOLD					0.1								//�ٶȵ���ֵ=5*sqrt(sumsq(v));��ע��Ч�������ԣ�

//--------------------------------------jiang ���P������������´�����иĽ�����Ҫ����----------------------------
#define  MeaUpdatePSetting					0//����Ϊ1��ʹ�ù�ʽ��P, ����ʹ��ԭ�����㷨
//----------����Q�� jiang ��Ҫ����ʵ�ʹߵ��豸ʵ���������--------------------------------------------------------------
#define GYROBIAS							(30*(DPH))				// ������ƫ(deg/h)  10
#define GYROBIAS2 							(GYROBIAS*GYROBIAS)
#define ACCBIAS								(1000*(UG))				// ���ٶȼ���ƫ(ug)  1000
#define ACCBIAS2 							(ACCBIAS*ACCBIAS)
#define ANGNRANDOMWALK					(0.6*(DPSH))			// �Ƕ��������(deg/sqrt(h))//0.6
#define ANGNRANDOMWALK2 					(ANGNRANDOMWALK*ANGNRANDOMWALK)
#define VELRANDOMWALK						(10*(UGPSHZ))			// �ٶ��������(ug/sqrt(hz))
#define VELRANDOMWALK2 					(VELRANDOMWALK*VELRANDOMWALK)
#define ANGNR_RATERANDOMWALK			0.0						// �������������
#define ANGNR_RATERANDOMWALK2 			(ANGNR_RATERANDOMWALK*ANGNR_RATERANDOMWALK)
#define ACC_RANDOMWALK					0.0						// ���ٶ��������
#define ACC_RANDOMWALK2 					(ACC_RANDOMWALK*ACC_RANDOMWALK)
#define LEVER_VAR							1.0						// �˱�����
//״̬����λ����Ŀǰ��Ϊλ�����Զ����GNSS���㶨λ���Ժ���Խ�ģ����
#define  INS_POS_VAR						((5.0/Re)*(5.0/Re)) *9.0//jiang: ��Ҫ���Կ���3������Ƿ����
#define  INS_HEAD_VAR						5.5*5.5*9.0//jiang:
//-----------����P��-------------------------------------------------------------
#define DATT									(1*(DEG))
#define DATT_VAR							(DATT*DATT)
#define DVEL									1.0
#define DVEL_VAR							(DVEL*DVEL)
#define DPOS								(1/Re)
#define DPOS_VAR							(DPOS*DPOS)
//----------����R��--------------------------------------------------------------
#define VEL_VAR								(0.2*0.2)
#define VEL_VAR_F							(0.2/SAMPLE_Ratio)*(0.2/SAMPLE_Ratio)			//�ٶ������ʱ���й�ϵ��0.2m/s
//#define POS_VAR								((0.1/Re)*(0.1/Re))
#define HEAD_VAR							(5.5)
#define SPP_POS_VAR						((5.0/Re)*(5.0/Re))   //���㶨λλ�����
#define RTK_POS_VAR						((0.1/Re)*(0.1/Re))   //RTK��λλ�����





//#define WEEK2SECOND						604800.0;//һ����				
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef  __cplusplus
}
#endif

#endif	/*_CONSTANT_H*/
/*End***********************************************************************************/



