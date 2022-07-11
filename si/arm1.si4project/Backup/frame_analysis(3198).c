

#include "frame_analysis.h"
#include "m_queue.h"
#include "serial.h"

static uint8_t m_uart_queue_buffer[USART_SERIAL_RB_BUFSZ];
static uint8_t m_uart_rx_buffer[USART_SERIAL_RB_BUFSZ];
pQueue	m_queue = NULL;

RS422_FRAME_DEF	rs422_frame;

uint8_t xor_check(uint8_t *buf, uint8_t len)
{
    uint8_t i = 0;
    uint8_t x = 0;

    for(; i < len; i++)
    {
        x = x ^ (*(buf + i));
    }

    return x;
}
//数据帧校验
uint8_t frame_verify(void)
{
	uint8_t xor;

	xor = xor_check(rs422_frame.data_stream.header, sizeof(rs422_frame.data_stream)-6 );
	if(xor != rs422_frame.data_stream.xor_verify1)
	{
		return INS_ERROR;
	}
	xor = xor_check(rs422_frame.data_stream.header, sizeof(rs422_frame.data_stream)-1 );
	if(xor != rs422_frame.data_stream.xor_verify2)
	{
		return INS_ERROR;
	}

	return INS_EOK;
}

EventStatus frame_dispose_recvDataTask(void)
{
    uint8_t  frameLen,checkxor, calCheckxor;
    uint8_t  flag;
    uint8_t  data;
    uint8_t  length = 0;
    uint8_t  *p = NULL;

    while( 1 )
    {
        data = QueueFrontAndOut2(m_queue, &flag);
        if(flag)
        {
            if(FRAME_HEADER_H == data)
            {
                p = m_uart_rx_buffer;
                length = 0;
                *p++ = data;
                length++;
            }
            else
            {
                if(p == NULL)
                {
                    continue;
                }
                if(FRAME_HEADER_M == data)
                {
					
                }
                *p++ = data;
                length++;
                if(length > 4)
                {
                    if(length == (frameLen + 5))
                    {
                        checkxor = *(m_uart_rx_buffer + 4 + frameLen);
                        calCheckxor = xor_check(m_uart_rx_buffer, 4 + frameLen);

                        if(checkxor != calCheckxor)
                        {
                            break;
                        }
						
                        
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
    m_queue = QueueCreate(USART_SERIAL_RB_BUFSZ,m_uart_queue_buffer);
}

