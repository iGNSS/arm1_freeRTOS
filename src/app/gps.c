
#include "string.h"
#include "gps.h"




#define	GPS_RX_BUFFER_SIZE	100
#define	GPRMC_BUFFER_SIZE	100
#define	GPGGA_BUFFER_SIZE	100
#define	GPGSV_BUFFER_SIZE	100

static uint8_t 	gps_rx_buffer[GPS_RX_BUFFER_SIZE];
static uint8_t	gps_rx_count = 0;
uint8_t  gprmc_buffer[GPRMC_BUFFER_SIZE];
uint8_t  gpgga_buffer[GPGGA_BUFFER_SIZE];
uint8_t  gpgsv_buffer[GPGSV_BUFFER_SIZE];

typedef struct
{
    union
    {
        uint8_t  latitude_d;//度数
        uint8_t	 latitude_f;//分

        uint32_t lati;//纬度
    } latitude;
} GPS_STRUCT;

uint8_t gps_data_extract(uint8_t* pData,uint16_t len)
{
    uint8_t ret = INS_ERROR;
    uint16_t i;
    uint8_t  *p = pData;

    for(i=0; i<len; i++)
    {
        *p++;

        if('$' == *p)
        {
            gps_rx_count = 0;
            gps_rx_buffer[gps_rx_count] = *p;
            gps_rx_count++;
        }
        else if(0x0a == *p)
        {
            gps_rx_buffer[gps_rx_count] = *p;
            gps_rx_count++;

            if((0 == strncmp("$GPRMC", (char const *)gps_rx_buffer, 6)) //GPS
                    || (0 == strncmp("$GNRMC", (char const *)gps_rx_buffer, 6)) //GPS+BD
                    || (0 == strncmp("$BDRMC", (char const *)gps_rx_buffer, 6))) //BD
            {
                memcpy(gprmc_buffer, gps_rx_buffer, gps_rx_count);
                ret = INS_EOK;
            }
            else if((0 == strncmp("$GPGGA", (char const *)gps_rx_buffer, 6)) //GPS
                    || (0 == strncmp("$GNGGA", (char const *)gps_rx_buffer, 6)) //GPS+BD
                    || (0 == strncmp("$BDGGA", (char const *)gps_rx_buffer, 6))) //BD
            {

                memcpy(gpgga_buffer, gps_rx_buffer, gps_rx_count);
                ret = INS_EOK;
            }
            else if((0 == strncmp("$GPGSV", (char const *)gps_rx_buffer, 6)) //GPS
                    || (0 == strncmp("$GNGSV", (char const *)gps_rx_buffer, 6)) //GPS+BD
                    || (0 == strncmp("$BDGSV", (char const *)gps_rx_buffer, 6))) //BD
            {
                memcpy(gpgsv_buffer, gps_rx_buffer, gps_rx_count);
                ret = INS_EOK;
            }
        }
        else//中间字符
        {
            gps_rx_buffer[gps_rx_count] = *p;
            gps_rx_count++;
        }

    }

    return ret;
}

void gps_gprmc_parse(uint8_t* pData,uint16_t len)
{
    u8  i = 0;
    u8  j = 0;
    u8  k = 0;
    u8  l = 0;
    u8  m = 0;
    s8  z = 0;
    u8  temp;
    u8  flag = 0;
    u8  count = 0;
    u16 temp2;
    u16 temp3;

    for(i=0; i<len; i++)
    {
        temp = *(pData+i);
        if((0x0a == temp)||(0x0d == temp))
        {
            break;
        }
        else if('.' == temp)
        {
            l = i;//.号位置
        }
        else if(',' == temp)
        {
            k = i;//当前,号位置
            count++;
            switch(count)
            {
            case 2://��2������,����ʱ����
            {
                if(7 == (l-j))
                {
                    //����ʱ��,С������������
                    Position.Hour = (*(pBuffer+j+1)-0x30)*10+(*(pBuffer+j+2)-0x30);
                    Position.Minute = (*(pBuffer+j+3)-0x30)*10+(*(pBuffer+j+4)-0x30);
                    Position.Second = (*(pBuffer+j+5)-0x30)*10+(*(pBuffer+j+6)-0x30);
                }
                else if(1 == (k-j))//û������
                {

                }
                else
                {
                    flag = 1;//����
                }

                break;
            }
            case 3://��3������,������Ч��־
            {
                if(2 == (k-j))
                {
                    //������Ч��־
                    if('A' == *(pBuffer+j+1))
                    {
                        Position.Status = 1;
                    }
                    else if('V' == *(pBuffer+j+1))
                    {
                        Position.Status = 0;
                    }
                    else
                    {
                        flag = 1;//����
                    }

                }
                else if(1 == (k-j))//û������
                {

                }
                else
                {
                    flag = 1;
                }
                break;
            }
            case 4://��4������,����γ��ֵ����ȷ��0.0001�֣���ȡС�������λ
            {
                if((k > l)&&(5 == l-j))
                {
                    //����γ��ֵ
                    Position.Latitue_D = (*(pBuffer+j+1)-0x30)*10+(*(pBuffer+j+2)-0x30);
                    Position.Latitue_F = (*(pBuffer+j+3)-0x30)*10+(*(pBuffer+j+4)-0x30);
                    temp2 = 0;
                    //������δ����ڷֵ�С��λ����4λʱ����������,dxl,2013.6.27
                    for(m=l+1; m<l+5; m++)
                    {
                        temp3 = (*(pBuffer+m)-0x30);
                        for(z=0; z<l+4-m; z++)
                        {
                            temp3 = temp3 * 10;
                        }
                        temp2 += temp3;
                    }
                    Position.Latitue_FX = temp2;


                }
                else if(1 == (k-j))//û������
                {

                }
                else
                {
                    flag = 1;
                }
                break;
            }
            case 5://��5������,����γ�ȷ���
            {
                if(2 == (k-j))
                {
                    //����γ�ȷ���
                    if('N' == *(pBuffer+j+1))
                    {
                        Position.North = 1;

                    }
                    else if('S' == *(pBuffer+j+1))
                    {
                        Position.North = 0;

                    }
                    else
                    {
                        flag = 1;
                    }
                }
                else if(1 == (k-j))//û������
                {

                }
                else
                {
                    flag = 1;
                }
                break;
            }
            case 6://��6������,��������ֵ����ȷ��0.0001��
            {
                if((k > l)&&(6 == l-j))
                {
                    //��������ֵ
                    Position.Longitue_D = (*(pBuffer+j+1)-0x30)*100+(*(pBuffer+j+2)-0x30)*10+*(pBuffer+j+3)-0x30;
                    Position.Longitue_F = (*(pBuffer+j+4)-0x30)*10+(*(pBuffer+j+5)-0x30);
                    temp2 = 0;
                    for(m=l+1; m<l+5; m++)
                    {
                        temp3 = (*(pBuffer+m)-0x30);
                        for(z=0; z<l+4-m; z++)
                        {
                            temp3 = temp3 * 10;
                        }
                        temp2 += temp3;
                    }
                    Position.Longitue_FX = temp2;
                }
                else if(1 == (k-j))//û������
                {

                }
                else
                {
                    flag = 1;
                }
                break;
            }
            case 7://��7������,�������ȷ���
            {
                if(2 == (k-j))
                {
                    //�������ȷ���
                    if('E' == *(pBuffer+j+1))
                    {
                        Position.East = 1;

                    }
                    else if('W' == *(pBuffer+j+1))
                    {
                        Position.East = 0;

                    }
                    else
                    {
                        flag = 1;
                    }
                }
                else if(1 == (k-j))//û������
                {

                }
                else
                {
                    flag = 1;
                }
                break;
            }
            case 8://��8������,�����ٶȣ���ȷ��������С����������
            {
                if((k>l)&&(l>j))
                {
                    temp2 = 0;
                    for(m=j+1; m<l; m++)
                    {
                        temp3 = (*(pBuffer+m)-0x30);
                        for(z=0; z<l-1-m; z++)
                        {
                            temp3 = temp3 * 10;
                        }
                        temp2 += temp3;
                    }
                    Position.Speed = temp2;
                    Position.SpeedX = *(pBuffer+l+1)-0x30;
                    if(Position.SpeedX >= 10)
                    {
                        //��������
                        Position.SpeedX = 0;
                    }
                }
                else if(1 == (k-j))//û������
                {

                }
                else
                {
                    flag = 1;
                }
                break;
            }
            case 9://��9������,�������򣬾�ȷ��������С����������
            {
                if((k>l)&&(l>j))
                {
                    temp2 = 0;
                    for(m=j+1; m<l; m++)
                    {
                        temp3 = (*(pBuffer+m)-0x30);
                        for(z=0; z<l-1-m; z++)
                        {
                            temp3 = temp3 * 10;
                        }
                        temp2 += temp3;
                    }
                    Position.Course = temp2;
                }
                else if(1 == (k-j))//û������
                {
                    Position.Course = 0;
                }
                else
                {
                    flag = 1;
                }
                break;
            }
            case 10://��10������,����������
            {
                if(7 == (k-j))
                {
                    //����
                    Position.Date = (*(pBuffer+j+1)-0x30)*10+(*(pBuffer+j+2)-0x30);
                    Position.Month = (*(pBuffer+j+3)-0x30)*10+(*(pBuffer+j+4)-0x30);
                    Position.Year = (*(pBuffer+j+5)-0x30)*10+(*(pBuffer+j+6)-0x30);
                    //Position.Hour += 8;//��8Сʱ,�б��ⲻ��������Σ���Ϊ�б���ʱ��Уʱ��
                    //if(Position.Hour >= 24)//ʵ��ʹ��ʱ��8Сʱ����Уʱ������
                    //{
                    //Position.Hour -= 24;
                    //Position.Date++;
                    // }
                }
                else if(1 == (k-j))//û������
                {

                }
                else
                {
                    flag = 1;
                }
                break;
            }
            default :
                break;

            }
            j = i;//��һ�����ŵ�λ��
        }
        if(1 == flag)//�����⵽��������ǰ����
        {
            break;
        }

    }
    if(1 == flag)
    {
        GprmcParseErrorCount++;
    }
    else
    {
        GprmcParseErrorCount = 0;
    }
}

