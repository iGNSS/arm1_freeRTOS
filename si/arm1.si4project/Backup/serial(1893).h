
#ifndef __SERIAL_H__
#define __SERIAL_H__

#include "ugdef.h"
#include "usart.h"

#ifdef  configUse_wireless

#define BAUD_RATE_2400                  2400
#define BAUD_RATE_4800                  4800
#define BAUD_RATE_9600                  9600
#define BAUD_RATE_19200                 19200
#define BAUD_RATE_38400                 38400
#define BAUD_RATE_57600                 57600
#define BAUD_RATE_115200                115200
#define BAUD_RATE_230400                230400
#define BAUD_RATE_460800                460800
#define BAUD_RATE_921600                921600
#define BAUD_RATE_2000000               2000000
#define BAUD_RATE_3000000               3000000


#ifndef USART_SERIAL_RB_BUFSZ
#define USART_SERIAL_RB_BUFSZ           100
#endif

#define UG_SERIAL_EVENT_RX_IND          0x01    /* Rx indication */
#define UG_SERIAL_EVENT_TX_DONE         0x02    /* Tx complete   */
#define UG_SERIAL_EVENT_RX_DMADONE      0x03    /* Rx DMA transfer done */
#define UG_SERIAL_EVENT_TX_DMADONE      0x04    /* Tx DMA transfer done */
#define UG_SERIAL_EVENT_RX_TIMEOUT      0x05    /* Rx timeout    */

#define UG_SERIAL_DMA_RX                0x01
#define UG_SERIAL_DMA_TX                0x02

#define UG_SERIAL_RX_INT                0x01
#define UG_SERIAL_TX_INT                0x02

#define UG_SERIAL_ERR_OVERRUN           0x01
#define UG_SERIAL_ERR_FRAMING           0x02
#define UG_SERIAL_ERR_PARITY            0x03


/* Default config for serial_configure structure */
#define UG_SERIAL_CONFIG_DEFAULT                    \
{                                                   \
    BAUD_RATE_115200,       /* 115200 bits/s */     \
    USART_WL_8BIT,          /* 8 databits */        \
    USART_STB_1BIT,         /* 1 stopbit */         \
    USART_PM_NONE,          /* No parity  */        \
    USART_MSBF_LSB,         /* LSB first sent */    \
    USART_DINV_DISABLE,     /* Normal mode */       \
    USART_SERIAL_RB_BUFSZ,  /* Buffer size */       \
    0                                               \
}


struct serial_configure
{
    ug_uint32_t baud_rate;

    ug_uint32_t data_bits               :4;
    ug_uint32_t stop_bits               :2;
    ug_uint32_t parity                  :2;
    ug_uint32_t bit_order               :1;
    ug_uint32_t invert                  :1;
    ug_uint32_t bufsz                   :16;
    ug_uint32_t reserved                :4;
};


/*
 * Serial FIFO mode 
 */
typedef struct 
{
    /* software fifo */
    ug_uint8_t *buffer;

    ug_uint16_t put_index, get_index;

    ug_bool_t is_full;
}ug_serial_rx_fifo,ug_serial_tx_fifo;

struct ug_completion
{
    ug_uint32_t flag;

    /* suspended list */
    ug_list_t suspended_list;
};

struct ug_serial_tx_fifo
{
    struct ug_completion completion;
};

/* GD32 uart driver */
struct gd32_usart
{
    ug_uint32_t usart_device;
    IRQn_Type irq;
    ug_uint8_t irq_pre_priority;
    ug_uint8_t irq_sub_priority;
#if defined( configUse_usart_dma )
    struct gd32_usart_dma
    {
        /* dma stream */
        //dma_parameter_struct* dma_init_struct;
        /* dma channel */
        dma_channel_enum rx_ch;
        /* dma flag */
        uint32_t rx_flag;
        /* dma irq channel */
        uint8_t rx_irq_ch;
        /* setting receive len */
        ug_size_t setting_recv_len;
        /* last receive index */
        ug_size_t last_recv_index;
    } dma;
#endif
};

struct ug_serial_device
{
    struct gd32_usart *usart;
    const struct ug_usart_ops *ops;
    struct serial_configure   config;
    
    void *serial_rx;
    void *serial_tx;
};
typedef struct ug_serial_device ug_serial_t;

/**
 * uart operators
 */
struct ug_usart_ops
{
    ug_err_t (*configure)(struct ug_serial_device *serial);
    ug_err_t (*control)(struct ug_serial_device *serial, int cmd, void *arg);

    int (*putc)(struct ug_serial_device *serial, char c);
    int (*getc)(struct ug_serial_device *serial);

    ug_size_t (*dma_transmit)(struct ug_serial_device *serial, ug_uint8_t *buf, ug_size_t size, int direction);
};

void ug_hw_serial_isr(struct ug_serial_device *serial, int event);

ug_err_t ug_hw_serial_register(struct ug_serial_device *serial);

ug_size_t _serial_int_rx(struct ug_serial_device *serial, ug_uint8_t *data, int length);

ug_size_t _serial_int_tx(struct ug_serial_device *serial, const ug_uint8_t *data, int length);

ug_size_t _serial_read_tx_fifo(struct ug_serial_device *serial, ug_uint8_t *data, int length);

void _serial_write_tx_fifo(struct ug_serial_device *serial, ug_uint8_t *pdata, int length);

#endif

#endif


