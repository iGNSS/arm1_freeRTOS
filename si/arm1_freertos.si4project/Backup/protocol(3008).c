#ifndef  __GOL_PROTOCOL_C__
#define  __GOL_PROTOCOL_C__

#include "computerFrameParse.h"
#include "protocol.h"
#include "serial.h"
#include "uartadapter.h"

#include "gnss.h"
#include "frame_analysis.h"



static uint8_t gnrmcSendBuff[200];
static uint16_t gnrmcSendLen = 0;
static uint8_t gnggaSendBuff[200];
static uint16_t gnggaSendLen = 0;

uint32_t NavDataSendedCnt=0;
uint8_t Cs,CsCharBuf[2];

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
    double		Baseline;  		//���߳��ȣ���λ���ף�
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
    double 			GPSTime;		//�Ա����� 00:00:00 ����ǰ����������������ʱ�䣩
    uint32_t  		IMUStatus; 		//״̬	//0X00000001 X ����״̬ 1������ 0���쳣
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

#define	RAWIMU_FRAME_LEN	( sizeof(__packed struct InsRawimuData) - sizeof(RawimuHeader) - sizeof(uint32_t) )
void protocol_fillRawimuData(void* pagric)
{
    GPS_AGRIC_TypeDef* agric = (GPS_AGRIC_TypeDef*)pagric;

	ins_rawimu_data.header.head1 			= 0xAA;
	ins_rawimu_data.header.head2 			= 0x44;
	ins_rawimu_data.header.head3 			= 0x13;
	ins_rawimu_data.header.frameLen 		= RAWIMU_FRAME_LEN;
	ins_rawimu_data.header.frameId 			= 325;
	ins_rawimu_data.header.GPSWeek 			= agric->header.gpsWn;
	ins_rawimu_data.header.GPSMilliseconds 	= agric->header.gpsMs;
	
    ins_rawimu_data.GPSWeek 				= agric->header.gpsWn;
    ins_rawimu_data.GPSTime 				= agric->header.gpsMs*0.001;
    ins_rawimu_data.IMUStatus   			= IMUStatus;
    ins_rawimu_data.AccelZ   				= imu_info.accelZ_l | (imu_info.accelZ_h<<8);//imuParseData.accelGrp[2];
    ins_rawimu_data.AccelY   				= imu_info.accelY_l | (imu_info.accelY_h<<8);//imuParseData.accelGrp[1];
    ins_rawimu_data.AccelX  				= imu_info.accelX_l | (imu_info.accelX_h<<8);//imuParseData.accelGrp[0];
    ins_rawimu_data.GyroZ  					= imu_info.gyroZ_l | (imu_info.gyroZ_h<<8);//imuParseData.gyroGrp[2];
    ins_rawimu_data.GyroY  					= imu_info.gyroY_l | (imu_info.gyroY_h<<8);//imuParseData.gyroGrp[1];
    ins_rawimu_data.GyroX    				= imu_info.gyroX_l | (imu_info.gyroX_h<<8);//imuParseData.gyroGrp[0];  
    ins_rawimu_data.CRC32 					= crc_block_data_calculate((uint32_t*)&ins_rawimu_data.AccelZ, 6);
        				
}

void protocol_crc32_init(void)
{
	rcu_periph_clock_enable(RCU_CRC);

    /* reset the CRC data register and calculate the CRC of the value */
    crc_data_register_reset();
}

/*****************************************************************************************
Function:HexToAscii
Input:none
Output:ret
Use:fpgaSem
Modify:fpgaSem
Note:
******************************************************************************************/
void HexToAscii( void * DestBuf, void * SrcBuf, int SrcLen )
{
    if ( ( DestBuf == NULL ) || ( SrcBuf == NULL ) )
    {
        return;
    }
    unsigned char * Dest = (unsigned char *) DestBuf;
    unsigned char * Src = (unsigned char *) SrcBuf;
    int DestPos = SrcLen << 1;
    int SrcPos = SrcLen;
    while ( --SrcPos >= 0 )
    {
        unsigned char HiHalf = Src[SrcPos] >> 4;
        unsigned char LoHalf = Src[SrcPos] & 0x0F;
        Dest[--DestPos] = ( HiHalf <= 9 ) ? ( HiHalf + '0' ) : ( HiHalf + 55 );
        Dest[--DestPos] = ( LoHalf <= 9 ) ? ( LoHalf + '0' ) : ( LoHalf + 55 );
    }

}

void protocol_gnrmcDataGet(uint8_t* pData, uint16_t* dataLen)
{
    uint16_t len = *dataLen;
    memcpy(gnrmcSendBuff, pData, len);
    gnrmcSendLen = len;
}

void protocol_gnggaDataGet(uint8_t* pData, uint16_t* dataLen)
{
    uint16_t len = *dataLen;
    memcpy(gnggaSendBuff, pData, len);
    gnggaSendLen = len;
}

void protocol_report(uint8_t channel)
{
    uint8_t i,frameType,dataMode;

    dataMode = comm_read_dataMode();
    frameType = comm_read_currentFrameType(channel);
    switch (frameType)
    {
    case cfg_format_GIPOT:
        if(dataMode==INS_DATA_MODE_1 || dataMode==INS_DATA_MODE_6)
        {
            sprintf(ins_gnss_data.InsGnssDataChar, "$GIPOT,%d,%.3f,%.3f,%.3f,%.3f,%.7f,%.7f,%.2f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d*%d\r\n", \
                    ins_gipot_data.GPSWeek,ins_gipot_data.GPSTime,ins_gipot_data.Heading,ins_gipot_data.Pitch, \
                    ins_gipot_data.Roll,ins_gipot_data.Lattitude,ins_gipot_data.Longitude,ins_gipot_data.Altitude, \
                    ins_gipot_data.Ve,ins_gipot_data.Vn,ins_gipot_data.Vu,ins_gipot_data.Baseline,ins_gipot_data.NSV,ins_gipot_data.NavStatus,ins_gipot_data.GnssStatus,66);
        }
        else if(dataMode==INS_DATA_MODE_2 || dataMode==INS_DATA_MODE_7)
        {
            sprintf(ins_gnss_data.InsGnssDataChar, "$GPFPD,%d,%.3f,%.3f,%.3f,%.3f,%.7f,%.7f,%.2f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d*%d\r\n", \
                    ins_gipot_data.GPSWeek,ins_gipot_data.GPSTime,ins_gipot_data.Heading,ins_gipot_data.Pitch, \
                    ins_gipot_data.Roll,ins_gipot_data.Lattitude,ins_gipot_data.Longitude,ins_gipot_data.Altitude, \
                    ins_gipot_data.Ve,ins_gipot_data.Vn,ins_gipot_data.Vu,ins_gipot_data.Baseline,ins_gipot_data.NSV,ins_gipot_data.NavStatus,ins_gipot_data.GnssStatus,66);
        }
        else if(dataMode==INS_DATA_MODE_3)
        {
            sprintf(ins_gnss_data.InsGnssDataChar, "$GRIMU,%d,%.3f,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.1f*%d\r\n",ins_grimu_data.GPSWeek,ins_grimu_data.GPSTime,ins_grimu_data.GyroX,ins_grimu_data.GyroY,ins_grimu_data.GyroZ,ins_grimu_data.AccelX,ins_grimu_data.AccelY,ins_grimu_data.AccelZ,ins_grimu_data.Temp,66);
        }
        else if(dataMode==INS_DATA_MODE_5 || dataMode==INS_DATA_MODE_9)
        {
            sprintf(ins_gnss_data.InsGnssDataChar,"$GRIMU,%d,%.3f,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.1f\n$GPFPD,%d,%.3f,%.3f,%.3f,%.3f,%.7f,%.7f,%.2f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d*%d\r\n",
                    ins_grimu_data.GPSWeek,ins_grimu_data.GPSTime,ins_grimu_data.GyroX,ins_grimu_data.GyroY,ins_grimu_data.GyroZ,ins_grimu_data.AccelX,ins_grimu_data.AccelY,ins_grimu_data.AccelZ,ins_grimu_data.Temp, \
                    ins_gipot_data.GPSWeek,ins_gipot_data.GPSTime,ins_gipot_data.Heading,ins_gipot_data.Pitch,ins_gipot_data.Roll,ins_gipot_data.Lattitude,ins_gipot_data.Longitude,ins_gipot_data.Altitude, \
                    ins_gipot_data.Ve,ins_gipot_data.Vn,ins_gipot_data.Vu,ins_gipot_data.Baseline,ins_gipot_data.NSV,ins_gipot_data.NavStatus,ins_gipot_data.GnssStatus,66);
        }
        else if(dataMode==INS_DATA_MODE_4 || dataMode==INS_DATA_MODE_8)
        {
            if(NavDataSendedCnt%2==1)
            {
                sprintf(ins_gnss_data.InsGnssDataChar, "$GIPOT,%d,%.3f,%.3f,%.3f,%.3f,%.7f,%.7f,%.2f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d*%d\r\n", \
                        ins_gipot_data.GPSWeek,ins_gipot_data.GPSTime,ins_gipot_data.Heading,ins_gipot_data.Pitch, \
                        ins_gipot_data.Roll,ins_gipot_data.Lattitude,ins_gipot_data.Longitude,ins_gipot_data.Altitude, \
                        ins_gipot_data.Ve,ins_gipot_data.Vn,ins_gipot_data.Vu,ins_gipot_data.Baseline,ins_gipot_data.NSV,ins_gipot_data.NavStatus,ins_gipot_data.GnssStatus,66);
            }
            else
            {
                sprintf(ins_gnss_data.InsGnssDataChar, "$GRIMU,%d,%.3f,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.1f*%d\r\n",ins_grimu_data.GPSWeek,ins_grimu_data.GPSTime,ins_grimu_data.GyroX,ins_grimu_data.GyroY,ins_grimu_data.GyroZ,ins_grimu_data.AccelX,ins_grimu_data.AccelY,ins_grimu_data.AccelZ,ins_grimu_data.Temp,66);
            }
        }

        for(i=1; ins_gnss_data.InsGnssDataChar[i]!='*'; i++)
        {
            Cs^=ins_gnss_data.InsGnssDataChar[i];
        }
        //����У���

        HexToAscii(CsCharBuf, &Cs, 1 );
        ins_gnss_data.InsGnssDataChar[i+1]=CsCharBuf[1];
        ins_gnss_data.InsGnssDataChar[i+2]=CsCharBuf[0];
        break;
    case cfg_format_GPRMC:
        memset((void*)ins_gnss_data.InsGnssDataChar,'\0',sizeof(ins_gnss_data.InsGnssDataChar));
        memcpy((void*)ins_gnss_data.InsGnssDataChar,(void*)gnrmcSendBuff,gnrmcSendLen);
        break;
    case cfg_format_GPGGA:
        memset((void*)ins_gnss_data.InsGnssDataChar,'\0',sizeof(ins_gnss_data.InsGnssDataChar));
        memcpy((void*)ins_gnss_data.InsGnssDataChar,(void*)gnggaSendBuff,gnggaSendLen);
        break;
    case cfg_format_RAWIMU:
    	memset((void*)ins_gnss_data.InsGnssDataChar,'\0',sizeof(ins_gnss_data.InsGnssDataChar));
        memcpy((void*)ins_gnss_data.InsGnssDataChar,(void*)&ins_rawimu_data.header.head1,sizeof(__packed struct InsRawimuData));
		break;
    default:
        break;
    }
    if(index_RS422 == channel)
    {
        Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, strlen(ins_gnss_data.InsGnssDataChar), (uint8_t*)ins_gnss_data.InsGnssDataChar);
    }
    else if(index_RS232A == channel)
    {

    }
    else if(index_RS232B == channel)
    {

    }
}
#endif

