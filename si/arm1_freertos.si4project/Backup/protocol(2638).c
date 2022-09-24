#include "computerFrameParse.h"
#include "data_convert.h"
#include "serial.h"

#include "gnss.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"



union InsGnssData
{
	char  InsGnssDataChar[148*2];
	float InsGnssDataFloat[37*2];
}ins_gnss_data;

struct  InsGipotData
{
	uint16_t 	GPSWeek;		//�� 1980-1-6 ����ǰ������������������ʱ�䣩
	uint32_t 	GPSTime;		//�Ա����� 00:00:00 ����ǰ����������������ʱ�䣩
	double		Heading;		//ƫ���ǣ�0~359.99�� 
	double		Pitch;			//�����ǣ�-90~90��
	double		Roll;			//����ǣ�-180~180��
	double		Lattitude;  	//γ�ȣ�-90~90��
	double		Longitude;  	//���ȣ�-180~180��
	double		Altitude;  		//�߶ȣ���λ���ף�
	double		Ve;  			//�����ٶȣ���λ����/�룩  
	double		Vn;  			//�����ٶȣ���λ����/�룩
	double		Vu;  			//�����ٶȣ���λ����/�룩
	uint16_t	Baseline;  		//���߳��ȣ���λ���ף� 
	uint8_t		NSV;  			//������ 
	uint8_t		NavStatus;		//>0 ��ʾ�������������Ч
	uint8_t		GnssStatus;   	//ϵͳ״̬��0����ʼ��1�����㶨λ2��α����4���ز���λ��֣����㣩5���ز���λ��֣����㣩
	
}ins_gipot_data;
