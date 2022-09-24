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

__packed struct InsGipotData
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
	
}ins_gipot_data,ins_gpfpd_data;

__packed struct InsGrimuData
{
	uint16_t 	GPSWeek;		//�� 1980-1-6 ����ǰ������������������ʱ�䣩
	uint32_t 	GPSTime;		//�Ա����� 00:00:00 ����ǰ����������������ʱ�䣩
	double		GyroX; 			//������ X ����ٶȣ���/s��
	double		GyroY; 			//������ Y ����ٶȣ���/s��
	double		GyroZ; 			//������ Z ����ٶȣ���/s��
	double		AccelX;  		//���ٶȼ� X ����ٶȣ�m/s 2 ��
	double		AccelY;  		//���ٶȼ� Y ����ٶȣ�m/s 2 ��
	double		AccelZ;  		//���ٶȼ� Z ����ٶȣ�m/s 2 ��
	double		Temp;  			//�������¶ȼ��������棩
	
}ins_grimu_data;

typedef __packed struct
{
	char		head1;	  		//֡ͷ  0xAA
	char		head2;	  		//֡ͷ  0x44
	char		head3;	  		//֡ͷ  0x13
	uint8_t		frameLen;		//֡��  ������֡ͷ�� CRC У���֡����
	uint16_t	frameId;  		//֡��
	uint16_t 	GPSWeek;		//�� 1980-1-6 ����ǰ������������������ʱ�䣩
	uint32_t 	GPSMilliseconds;//�Ա����� 00:00:00 ����ǰ�ĺ���������������ʱ�䣩
}RawimuHeader;
__packed struct InsRawimuData 
{
	RawimuHeader	header;
	uint16_t 		GPSWeek;		//�� 1980-1-6 ����ǰ������������������ʱ�䣩
	uint32_t 		GPSTime;		//�Ա����� 00:00:00 ����ǰ����������������ʱ�䣩
	long  			IMUStatus; 		//״̬	//0X00000001 X ����״̬ 1������ 0���쳣
									//0X00000002 Y ����״̬ 1������ 0���쳣
									//0X00000004 Z ����״̬ 1������ 0���쳣
									//0X00000010 X �ӱ�״̬ 1������ 0���쳣
									//0X00000020 Y �ӱ�״̬ 1������ 0���쳣
									//0X00000040 Z �ӱ�״̬ 1������ 0���쳣
	long  			AccelZ; 		//�ӱ�Z���
	long  			AccelY; 		//�ӱ�Y���
	long  			AccelX; 		//�ӱ�X���
	long			GyroZ; 			//������Z���
	long			GyroY; 			//������Y���
	long			GyroX;			//������X���
	uint32_t		CRC32;			//CRC У��  32-bitCRC У��
}ins_rawimu_data;

