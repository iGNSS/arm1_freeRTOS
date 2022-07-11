

#include "frame_analysis.h"

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

