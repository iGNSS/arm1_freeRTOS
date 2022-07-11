
#include "string.h"
#include "frame_analysis.h"
#include "m_queue.h"
#include "serial.h"
#include "exmc_sram.h"

//static uint8_t m_uart_queue_buffer[USART_SERIAL_RB_BUFSZ];
//static uint8_t m_uart_rx_buffer[USART_SERIAL_RB_BUFSZ];
//pQueue	m_queue = NULL;

RS422_FRAME_DEF	rs422_frame;

uint8_t xor_check(uint8_t *buf, uint16_t len)
{
    uint16_t i = 0;
    uint16_t x = 0;

    for(; i < len; i++)
    {
        x = x ^ (*(buf + i));
    }

    return x;
}

uint8_t sum_check(uint8_t *buf, uint16_t len )
{
    uint8_t checksum = 0, i;

    for (i = 0; i < len; i++)
    {
        checksum += buf[i];
    }

    checksum  = ~checksum;

    return checksum;

}
//参考PA-IMU-460.PDF
#define FRAME_HEADER_LSB                  0x7F
#define FRAME_HEADER_MSB                  0x80
uint8_t frame_fill_imu(uint8_t* pData)
{
    uint8_t  sum, calSum;
    uint8_t  data, data1;
    uint8_t  length = 0;
    uint8_t  ret = INS_ERROR;
    uint8_t  *p = pData;

    while( 1 )
    {
        data = *p++;
        data1 = *p++;
        length += 2;

        if((FRAME_HEADER_LSB == data) && (FRAME_HEADER_MSB == data1))
        {
            sum = sum_check(p, 21 );
            calSum = *(p + 20);

            if(sum == calSum)
            {
                rs422_frame.data_stream.accelX = *p++;
                rs422_frame.data_stream.accelX |= (*p++) << 8;
                rs422_frame.data_stream.accelY = *p++;
                rs422_frame.data_stream.accelY |= (*p++) << 8;
                rs422_frame.data_stream.accelZ = *p++;
                rs422_frame.data_stream.accelZ |= (*p++) << 8;

                rs422_frame.data_stream.gyroX = *p++;
                rs422_frame.data_stream.gyroX |= (*p++) << 8;
                rs422_frame.data_stream.gyroY = *p++;
                rs422_frame.data_stream.gyroY |= (*p++) << 8;
                rs422_frame.data_stream.gyroZ = *p++;
                rs422_frame.data_stream.gyroZ |= (*p++) << 8;

                rs422_frame.data_stream.roll = *p++;
                rs422_frame.data_stream.roll |= (*p++) << 8;
                rs422_frame.data_stream.pitch = *p++;
                rs422_frame.data_stream.pitch |= (*p++) << 8;
                rs422_frame.data_stream.azimuth = *p++;
                rs422_frame.data_stream.azimuth |= (*p++) << 8;

                ret = INS_EOK;
                break;
            }
        }

        if(length > RS422_FRAME_LENGTH)break;
    }

    return ret;
}

#define	GPS_RX_BUFFER_SIZE	100
#define	GPRMC_BUFFER_SIZE	100
#define	GPGGA_BUFFER_SIZE	100
#define	GPGSV_BUFFER_SIZE	100

static uint8_t 	gps_rx_buffer[GPS_RX_BUFFER_SIZE];
static uint8_t	gps_rx_count = 0;
uint8_t  gprmc_buffer[GPRMC_BUFFER_SIZE];
uint8_t  gprmc_bufferBusyFlag = 0;
uint8_t  gpgga_buffer[GPGGA_BUFFER_SIZE];
uint8_t  gpgga_bufferBusyFlag = 0;
uint8_t  gpgsv_buffer[GPGSV_BUFFER_SIZE];
uint8_t  gpgsv_bufferBusyFlag = 0;

void frame_fill_gnss(uint8_t* pData)
{
    uint8_t  *p = pData;

    while( 1 )
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
            }
            else if((0 == strncmp("$GPGGA", (char const *)gps_rx_buffer, 6)) //GPS
                    || (0 == strncmp("$GNGGA", (char const *)gps_rx_buffer, 6)) //GPS+BD
                    || (0 == strncmp("$BDGGA", (char const *)gps_rx_buffer, 6))) //BD
            {

                memcpy(gpgga_buffer, gps_rx_buffer, gps_rx_count);
            }
            else if((0 == strncmp("$GPGSV", (char const *)gps_rx_buffer, 6)) //GPS
                    || (0 == strncmp("$GNGSV", (char const *)gps_rx_buffer, 6)) //GPS+BD
                    || (0 == strncmp("$BDGSV", (char const *)gps_rx_buffer, 6))) //BD
            {
                memcpy(gpgsv_buffer, gps_rx_buffer, gps_rx_count);
            }
        }

    }
}

//数据封包
void frame_pack_and_send(uint8_t* pData)
{
    uint8_t xor;

    frame_fill_imu(pData);
    rs422_frame.data_stream.header[0] = FRAME_HEADER_L;
    rs422_frame.data_stream.header[1] = FRAME_HEADER_M;
    rs422_frame.data_stream.header[2] = FRAME_HEADER_H;

    xor = xor_check(rs422_frame.data_stream.header, sizeof(rs422_frame.data_stream) - 6 );
    rs422_frame.data_stream.xor_verify1 = xor;

    xor = xor_check(rs422_frame.data_stream.header, sizeof(rs422_frame.data_stream) - 1 );
    rs422_frame.data_stream.xor_verify2 = xor;

    exmc_sram_writebuffer_8(rs422_frame.fpga_cache, WRITE_READ_ADDR, BUFFER_SIZE, BANK0_REGION1);
}
#if 0
EventStatus frame_dispose_recvDataTask(void)
{
    uint8_t  checkxor, calCheckxor;
    uint8_t  flag, valid = 0;
    uint8_t  data;
    uint8_t  length = 0;
    uint8_t  tempLength = 0;
    uint8_t  *p = NULL;

    while( 1 )
    {
        data = QueueFrontAndOut2(m_queue, &flag);

        if(flag)
        {
            if((FRAME_HEADER_H == data) && (length == 0))
            {
                p = m_uart_rx_buffer;
                *p++ = data;
                length++;
            }
            else
            {
                if(p == NULL)
                {
                    continue;
                }

                if((FRAME_HEADER_M == data) && (length == 1))
                {
                    *p++ = data;
                    length++;
                }
                else if((FRAME_HEADER_L == data) && (length == 2))
                {
                    *p++ = data;
                    length++;
                    valid = 1;
                }
                else
                {
                    if(valid)
                    {
                        *p++ = data;
                        length++;

                        if(length > USART_SERIAL_RB_BUFSZ)
                        {
                            break;
                        }

                        checkxor = data;
                        calCheckxor = xor_check(m_uart_rx_buffer, length - 1);

                        if(checkxor == calCheckxor)
                        {
                            if(tempLength == 0)
                            {
                                tempLength = length;
                            }
                            else if((length - tempLength) == 4)
                            {
                                //数据解析
                                break;
                            }
                        }
                    }
                    else
                    {
                        length = 0;
                        p = NULL;
                    }
                }
            }
        }
        else
        {
            break;
        }
    }

    return ENABLE;
}

void frame_queue_init(void)
{
    m_queue = QueueCreate(USART_SERIAL_RB_BUFSZ, m_uart_queue_buffer);
}
#endif

