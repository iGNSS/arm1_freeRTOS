#ifndef _AUART_ADAPTER_H_
#define _AUART_ADAPTER_H_4

#include "stdbool.h"
#include "stm32f4xx.h"
#include "UartDefine.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void FUART_TxInit(EUartTxPort txPort, EUartBaudrate baudrate, EUartParitybits parityBits, EUartStopbits stopBits);
extern void FUART_RxInit(EUartRxPort rxPort, EUartBaudrate baudrate, EUartParitybits parityBits, EUartStopbits stopBits);
extern void FUART_SendData(EUartTxPort txPort, uint8_t* data, uint16_t size);
extern u16  FUART_RecvData(EUartRxPort rxPort, uint8_t* data, uint16_t size);


#ifdef __cplusplus
}
#endif


#endif
