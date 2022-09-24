#ifndef __PROTOCOL_H____
#define __PROTOCOL_H____

#undef COMMON_EXT
#ifdef  __GOL_PROTOCOL_C__
    #define COMMON_EXT
#else
    #define COMMON_EXT extern
#endif	

#include "gd32f4xx.h"
#include "config.h"


enum
{
	e_GyroX =  0,
	e_GyroY =  1,
	e_GyroZ =  2,
	e_AccelX = 4,
	e_AccelY = 5,
	e_AccelZ = 6
};

#define	IMUSTATUS_SET(x)	(IMUStatus |= ((uint32_t)0x01U<<(x)))

COMMON_EXT uint32_t  		IMUStatus;

COMMON_EXT void protocol_crc32_init(void);
COMMON_EXT void protocol_fillGipotData(void* pagric, void* pheading);
COMMON_EXT void protocol_fillGrimuData(void* pagric);
COMMON_EXT void protocol_fillRawimuData(void* pagric);
COMMON_EXT void protocol_report(uint8_t channel);
COMMON_EXT void protocol_gnrmcDataGet(uint8_t* pData, uint16_t* dataLen);
COMMON_EXT void protocol_gnggaDataGet(uint8_t* pData, uint16_t* dataLen);



#endif




