#include "computerFrameParse.h"
#include "data_convert.h"
#include "serial.h"

#include "gnss.h"
#include "frame_analysis.h"

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
} ins_gnss_data;

__packed struct InsGipotData
{
    uint16_t 	GPSWeek;		//�� 1980-1-6 ����ǰ������������������ʱ�䣩
    double 		GPSTime;		//�Ա����� 00:00:00 ����ǰ����������������ʱ�䣩
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

} ins_gipot_data,ins_gpfpd_data;

__packed struct InsGrimuData
{
    uint16_t 	GPSWeek;		//�� 1980-1-6 ����ǰ������������������ʱ�䣩
    double 		GPSTime;		//�Ա����� 00:00:00 ����ǰ����������������ʱ�䣩
    double		GyroX; 			//������ X ����ٶȣ���/s��
    double		GyroY; 			//������ Y ����ٶȣ���/s��
    double		GyroZ; 			//������ Z ����ٶȣ���/s��
    double		AccelX;  		//���ٶȼ� X ����ٶȣ�m/s 2 ��
    double		AccelY;  		//���ٶȼ� Y ����ٶȣ�m/s 2 ��
    double		AccelZ;  		//���ٶȼ� Z ����ٶȣ�m/s 2 ��
    double		Temp;  			//�������¶ȼ��������棩

} ins_grimu_data;

typedef __packed struct
{
    char		head1;	  		//֡ͷ  0xAA
    char		head2;	  		//֡ͷ  0x44
    char		head3;	  		//֡ͷ  0x13
    uint8_t		frameLen;		//֡��  ������֡ͷ�� CRC У���֡����
    uint16_t	frameId;  		//֡��
    uint16_t 	GPSWeek;		//�� 1980-1-6 ����ǰ������������������ʱ�䣩
    uint32_t 	GPSMilliseconds;//�Ա����� 00:00:00 ����ǰ�ĺ���������������ʱ�䣩
} RawimuHeader;
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
} ins_rawimu_data;

void protocol_fillGipotData(void* pagric, void* pheading)
{
    GPS_AGRIC_TypeDef* agric = (GPS_AGRIC_TypeDef*)pagric;
    GNSS_Heading_DataTypeDef* heading = (GNSS_Heading_DataTypeDef*)pheading;

    ins_gipot_data.GPSWeek = agric->header.gpsWn;
    ins_gipot_data.GPSTime = agric->header.gpsMs*0.001;
    ins_gipot_data.Heading = agric->Heading;
    ins_gipot_data.Pitch = agric->pitch;
    ins_gipot_data.Roll = agric->roll;
    ins_gipot_data.Lattitude = agric->lat;
    ins_gipot_data.Longitude = agric->lon;
    ins_gipot_data.Altitude = agric->alt;
    ins_gipot_data.Ve = agric->vE;
    ins_gipot_data.Vn = agric->vN;
    ins_gipot_data.Vu = agric->vZ;
    ins_gipot_data.Baseline = heading->baseLen;
    ins_gipot_data.NSV = agric->num_bd_star;
    ins_gipot_data.NSV += agric->num_gps_star;
    ins_gipot_data.NSV += agric->num_gal_star;
    ins_gipot_data.NSV += agric->num_glo_star;
    ins_gipot_data.GnssStatus = agric->rtk_status;
    ins_gipot_data.NavStatus = ins_gipot_data.GnssStatus;

    memcpy((void*)&ins_gpfpd_data.GPSWeek, (void*)&ins_gipot_data.GPSWeek, sizeof(struct InsGipotData));
}

void protocol_fillGrimuData(void* pagric)
{
    GPS_AGRIC_TypeDef* agric = (GPS_AGRIC_TypeDef*)pagric;

    ins_grimu_data.GPSWeek = agric->header.gpsWn;
    ins_grimu_data.GPSTime = agric->header.gpsMs*0.001;
    ins_grimu_data.GyroX   = imuParseData.gyroGrp[0];
    ins_grimu_data.GyroY   = imuParseData.gyroGrp[1];
    ins_grimu_data.GyroZ   = imuParseData.gyroGrp[2];
    ins_grimu_data.AccelX  = imuParseData.accelGrp[0];
    ins_grimu_data.AccelY  = imuParseData.accelGrp[1];
    ins_grimu_data.AccelZ  = imuParseData.accelGrp[2];
    ins_grimu_data.Temp    = imuParseData.sensorTemp;
}


