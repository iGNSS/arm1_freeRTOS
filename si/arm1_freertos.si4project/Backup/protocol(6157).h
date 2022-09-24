#ifndef __PROTOCOL_H____
#define __PROTOCOL_H____

#include "gd32f4xx.h"
#include "config.h"


void protocol_fillGipotData(void* pagric, void* pheading);
void protocol_fillGrimuData(void* pagric);
void protocol_report(uint8_t channel);
void protocol_gnrmcDataGet(uint8_t* pData, uint16_t* dataLen);
void protocol_gnggaDataGet(uint8_t* pData, uint16_t* dataLen);



#endif




