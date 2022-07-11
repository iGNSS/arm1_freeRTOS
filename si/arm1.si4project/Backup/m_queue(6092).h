#ifndef __APP_USART_H__
#define __APP_USART_H__

//#include "config.h"
#include "ugdef.h"
#include "gd32f3x0.h"

#ifdef cplusplus
 extern "C" {
#endif 

#define FRAME_HEADER_MSB                    0x55
#define FRAME_HEADER_LSB                    0x56

#define USB_FRAME_HEADER_H                  0xFD
#define USB_FRAME_HEADER_M                  0xFD
#define USB_FRAME_HEADER_L                  0x02

#define NON_DONGLE_CMD_SERIAL_NO_SET        0x01
#define NON_DONGLE_CMD_BLE_NAME_SET         0x02
#define NON_DONGLE_CMD_TOUCH_MODE_SET       0x03
#define NON_DONGLE_CMD_LED_LIGHT_SET        0x04
#define NON_DONGLE_CMD_RTC_SET              0x05
#define NON_DONGLE_CMD_SLEEP_TIME_SET       0x06
#define NON_DONGLE_CMD_SERIAL_NO_GET        0xF1
#define NON_DONGLE_CMD_PAD_ATTR_GET         0xF3
#define NON_DONGLE_CMD_BLE_MAC_GET          0xF4
#define NON_DONGLE_CMD_BATT_GET             0xF5
#define NON_DONGLE_CMD_TOUCH_MODE_GET       0xF6
#define NON_DONGLE_CMD_LED_LIGHT_GET        0xF7
#define NON_DONGLE_CMD_BLE_FM_INFO_GET      0xF9


struct QueueRecord
{
    int Capacity;
    int Front;
    int Rear;
    int Size;
    uint8_t *Array;
};
typedef struct QueueRecord *    Queue;

unsigned char GetCheckSumHEX(unsigned char *buf, int len );
Queue QueueCreate(int MaxElements);
void QueueIn(uint8_t X);
void usart_queue_init(void);

#ifdef  configUse_wireless

EventStatus usart_dispose_recvDataTask(void); 
ug_err_t usart_data_analyze(ug_uint8_t *pBuffer, ug_int16_t bufferLen);

#endif

#ifdef cplusplus
}
#endif 

#endif









