#ifndef __USART_H__
#define __USART_H__

#include "config.h"
#include "ugdef.h"

#ifdef cplusplus
extern "C" {
#endif

#define COMBINED_FRAME_REPORT(data,num)             \
    do                                              \
    {                                               \
        if(e_work_mode_usb == get_report_mode())    \
        {                                           \
            usb_tx(data, num);                      \
        }                                           \
        else                                        \
        {                                           \
            REPORT_DATA_FRAME_FORM(data, num);      \
        }                                           \
    }while(0)

#ifdef  configUse_wireless

#define UART_ENABLE_IRQ(n)            NVIC_EnableIRQ((n))
#define UART_DISABLE_IRQ(n)           NVIC_DisableIRQ((n))

#define UG_USING_UART0


#define REPORT_DATA_FRAME_FORM(data,num)            \
    do                                              \
    {                                               \
        system_clear_sleep_count();                 \
        memmove(data+4,data,num);                   \
        data[0] = FRAME_HEADER_MSB;                 \
        data[1] = FRAME_HEADER_LSB;                 \
        data[2] = 0xf4;                             \
        data[3] = num;                              \
        data[4+num] = GetCheckSumHEX(data,4+num);   \
        gd32_usart_write(data, 5+num);              \
    }while(0)
#define BLE_TRANS_CMD_FORM(data,cmd,num)            \
    do                                              \
    {                                               \
        data[0] = FRAME_HEADER_MSB;                 \
        data[1] = FRAME_HEADER_LSB;                 \
        data[2] = cmd;                              \
        data[3] = num;                              \
        data[4+num] = GetCheckSumHEX(data,4+num);   \
        gd32_usart_write(data, 5+num);              \
    }while(0)
#define INQUIRY_BLE_STATUS(data,cmd)   BLE_TRANS_CMD_FORM(data,cmd,0)

    void        gd32_hw_usart_init(void);

    ug_size_t   gd32_usart_read(void *buffer, ug_size_t size);

    ug_size_t   gd32_usart_write(uint8_t *buffer, ug_size_t size);

    void        usart_tx_fifo_send(ug_uint8_t *buffer, ug_size_t size);

	void 		usart_transmit_dma(uint8_t *pData, uint16_t Size);
#else
#define REPORT_DATA_FRAME_FORM(data,num)
#define BLE_TRANS_CMD_FORM(data,cmd,num)
#define INQUIRY_BLE_STATUS(data,cmd)
#endif

#ifdef cplusplus
}
#endif

#endif




