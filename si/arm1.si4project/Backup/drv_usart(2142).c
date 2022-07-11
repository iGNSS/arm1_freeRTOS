#include "gd32f3x0.h"
#include "stdio.h"
#include "Gd32f3x0_usart.h"
#include "serial.h"
#include "timer.h"
#include "drv_pin.h"
#include "preferences.h"
#include "app_usart.h"
#ifdef  configUse_wireless

#define UART0_GPIO_TX       GPIO_PIN_9
#define UART0_GPIO_RX       GPIO_PIN_10
#define UART0_GPIO          GPIOA


static ug_err_t gd32_usart_configure(struct ug_serial_device *serial)
{
    struct gd32_usart *usart       = serial->usart;
    struct serial_configure cfg  = serial->config;

    // USART configure
    //usart_deinit(usart->usart_device);
    usart_baudrate_set(usart->usart_device, cfg.baud_rate);
    usart_word_length_set(usart->usart_device, cfg.data_bits);
    usart_parity_config(usart->usart_device, cfg.parity);
    usart_stop_bit_set(usart->usart_device, cfg.stop_bits);
    usart_data_first_config(usart->usart_device, cfg.bit_order);
    usart_invert_config(usart->usart_device, (usart_invert_enum)cfg.invert);
    usart_overrun_enable(usart->usart_device);
    usart_receive_config(usart->usart_device, USART_RECEIVE_ENABLE);
    usart_transmit_config(usart->usart_device, USART_TRANSMIT_ENABLE);

    //NVIC_Configuration(usart);

    usart_enable(usart->usart_device);

    return UG_EOK;
}

static ug_err_t gd32_usart_control(struct ug_serial_device *serial, int cmd, void *arg)
{
    struct gd32_usart* usart = serial->usart;
    ug_uint32_t ctrl_arg = (ug_uint32_t)(arg);

    switch (cmd)
    {
    case UG_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        UART_DISABLE_IRQ(usart->irq);
        /* disable interrupt */
        usart_interrupt_disable(usart->usart_device, USART_INT_RBNE);
        break;
    case UG_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        //UART_ENABLE_IRQ(usart->irq);
        nvic_irq_enable(usart->irq, usart->irq_pre_priority, usart->irq_sub_priority);
        /* enable interrupt */
        usart_interrupt_enable(usart->usart_device, USART_INT_RBNE);
        break;

    case UG_DEVICE_CTRL_CONFIG :
        if (ctrl_arg == UG_DEVICE_FLAG_DMA_RX)
        {
        }
        break;

    }

    return UG_EOK;
}

static int gd32_usart_putc(struct ug_serial_device *serial, char ch)
{
    struct gd32_usart *usart = serial->usart;

    while(!usart_flag_get(usart->usart_device, USART_FLAG_TBE));
    USART_TDATA(usart->usart_device) = (USART_TDATA_TDATA & ch);

    return 1;
}

static int gd32_usart_getc(struct ug_serial_device *serial)
{
    int data;

    struct gd32_usart *usart = serial->usart;

    data = -1;
    if (usart_flag_get(usart->usart_device, USART_FLAG_RBNE))
    {
        data = (ug_uint16_t)(GET_BITS(USART_RDATA(usart->usart_device), 0U, 8U));
    }

    return data;
}

#if defined(UG_USING_UART0)
static const struct ug_usart_ops gd32_usart_ops =
{
    gd32_usart_configure,
    gd32_usart_control,
    gd32_usart_putc,
    gd32_usart_getc
};

/* UART1 device driver structure */
struct gd32_usart usart0 =
{
    USART0,
    USART0_IRQn,
    0,
    0,
#if defined( configUse_usart_dma )
    {
        DMA_CH2,
        DMA_FLAG_FTF,
        DMA_Channel1_2_IRQn,
        0,
    }
#endif
};
struct ug_serial_device serial0 =
{
    &usart0,
    &gd32_usart_ops,
    UG_SERIAL_CONFIG_DEFAULT,
    UG_NULL,
    UG_NULL
};

#if defined( configUse_usart_dma )


typedef struct
{
    uint8_t  RX_flag:1;        //IDLE receive flag
    uint16_t RX_Size;          //receive length
    uint8_t  RX_pData[USART_SERIAL_RB_BUFSZ]; //DMA receive buffer
} USART_RECEIVETYPE;
//typedef struct
//{
//    uint8_t  TX_flag:1;        //IDLE tx flag
//    uint16_t TX_Size;          //tx length
//    uint8_t  TX_pData[USART_SERIAL_RB_BUFSZ]; //DMA tx buffer
//} USART_TRANSBYTE;
USART_RECEIVETYPE Usart_ReceiveStruct;
//USART_TRANSBYTE   Usart_TransStruct;

void usart_transmit_dma(uint8_t *pData, uint16_t Size)
{
    /*****************  DMA_TX_CONFIG   ****************/
    dma_parameter_struct dma_init_struct;
    /* enable the DMA clock */
    rcu_periph_clock_enable(RCU_DMA);
    /* configure the USART TX DMA channel */
    dma_deinit(DMA_CH1);
    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr = (uint32_t)pData;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = Size;
    dma_init_struct.periph_addr = (uint32_t) &USART_TDATA(USART0);
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA_CH1, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(DMA_CH1);
    dma_memory_to_memory_disable(DMA_CH1);
    /*****************  END  DMA_TX_CONFIG  ****************/
    /* enable the DMA channel1 and channel3 */
    dma_channel_enable(DMA_CH1);
    /* USART DMA enable for transmission */
    usart_dma_transmit_config(USART0, USART_DENT_ENABLE); //USARTÅäÖÃDMA·¢ËÍ
}

void usart_receive_dma(uint8_t *pData, uint16_t Size)
{
    /*****************  DMA_RX_CONFIG   ****************/
    dma_parameter_struct dma_init_struct;
    /* enable the DMA clock */
    rcu_periph_clock_enable(RCU_DMA);

    /* configure the USART RX DMA channel */
    dma_deinit(DMA_CH2);
    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)pData;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = Size;
    dma_init_struct.periph_addr = (uint32_t) &USART_RDATA(USART0);
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA_CH2, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(DMA_CH2);
    dma_memory_to_memory_disable(DMA_CH2);
    /*****************  END  DMA_RX_CONFIG  ****************/
    dma_channel_enable(DMA_CH2);
    usart_dma_receive_config(USART0, USART_DENR_ENABLE);  //USARTÅäÖÃDMA½ÓÊÕ
}

#endif

void usart_isr(struct ug_serial_device *serial)
{

    struct gd32_usart *usart = serial->usart;
    ug_serial_rx_fifo* rx_fifo;

    if(RESET != usart_interrupt_flag_get(usart->usart_device, USART_INT_FLAG_RBNE))
    {
        /* clear interrupt */
        usart_interrupt_flag_clear(usart->usart_device, USART_INT_FLAG_RBNE);
        usart_flag_clear(usart->usart_device, USART_FLAG_RBNE);
        ug_hw_serial_isr(serial, UG_SERIAL_EVENT_RX_IND);
    }
#if 0
    if (RESET != usart_interrupt_flag_get(usart->usart_device, USART_INT_FLAG_RBNE_ORERR))
    {
        usart_interrupt_flag_clear(usart->usart_device, USART_INT_FLAG_RBNE_ORERR);
        usart_data_receive(usart->usart_device);
        rx_fifo = (ug_serial_rx_fifo*) serial->serial_rx;
        rx_fifo->is_full = UG_FALSE;
        rx_fifo->put_index = rx_fifo->get_index = 0;
    }
    if (RESET != usart_interrupt_flag_get(usart->usart_device, USART_INT_FLAG_ERR_ORERR))
    {
        usart_interrupt_flag_clear(usart->usart_device, USART_INT_FLAG_ERR_ORERR);

        usart_data_receive(usart->usart_device);

        rx_fifo = (ug_serial_rx_fifo*) serial->serial_rx;
        rx_fifo->is_full = UG_FALSE;
        rx_fifo->put_index = rx_fifo->get_index = 0;
    }
#else
    if (RESET != usart_flag_get(usart->usart_device, USART_FLAG_ORERR))
    {
        usart_data_receive(usart->usart_device);
        usart_flag_clear(usart->usart_device, USART_FLAG_ORERR);

        rx_fifo = (ug_serial_rx_fifo*) serial->serial_rx;
        rx_fifo->is_full = UG_FALSE;
        rx_fifo->put_index = rx_fifo->get_index = 0;
    }
    if (RESET != usart_flag_get(usart->usart_device, USART_FLAG_NERR))
    {
        usart_data_receive(usart->usart_device);
        usart_flag_clear(usart->usart_device, USART_FLAG_NERR);

        rx_fifo = (ug_serial_rx_fifo*) serial->serial_rx;
        rx_fifo->is_full = UG_FALSE;
        rx_fifo->put_index = rx_fifo->get_index = 0;
    }
#endif
}

void USART0_IRQHandler(void)
{
#if defined (configUse_usart_queue_rev)
    uint8_t data;
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE))
    {
        usart_interrupt_flag_clear(USART0, USART_INT_FLAG_RBNE);
        /* receive data */
        data = (uint8_t)usart_data_receive(USART0);
        QueueIn(data);
    }
    else if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE_ORERR))
    {
        usart_data_receive(USART0);
        usart_interrupt_flag_clear(USART0, USART_INT_FLAG_RBNE_ORERR);
    }
#elif defined (configUse_usart_dma)
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_IDLE))
    {
        usart_interrupt_flag_clear(USART0, USART_INT_FLAG_IDLE);
        dma_channel_disable(DMA_CH2);
        Usart_ReceiveStruct.RX_Size =  USART_SERIAL_RB_BUFSZ - dma_transfer_number_get(DMA_CH2);
        if(Usart_ReceiveStruct.RX_Size)
        {
            usart_data_analyze(Usart_ReceiveStruct.RX_pData, Usart_ReceiveStruct.RX_Size);
        }
        usart_receive_dma(Usart_ReceiveStruct.RX_pData, USART_SERIAL_RB_BUFSZ);
    }

#else
    usart_isr(&serial0);
#endif
}

#endif

static void RCC_Configuration(void)
{
#ifdef UG_USING_UART0
    /* enable COM GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);
    // enable USART clock
    rcu_periph_clock_enable(RCU_USART0);

#endif

}

static void GPIO_Configuration(void)
{
#ifdef UG_USING_UART0
    gpio_mode_set(UART0_GPIO, GPIO_MODE_AF, GPIO_PUPD_PULLUP, UART0_GPIO_TX | UART0_GPIO_RX);
    gpio_output_options_set(UART0_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, UART0_GPIO_TX | UART0_GPIO_RX);
    gpio_af_set(UART0_GPIO, GPIO_AF_1, UART0_GPIO_TX | UART0_GPIO_RX);
#endif
}

ug_size_t gd32_usart_read(void *buffer,ug_size_t size)
{
    return _serial_int_rx(&serial0,buffer,size);

}

ug_size_t gd32_usart_write(uint8_t *buffer,ug_size_t size)
{
    return _serial_int_tx(&serial0,buffer,size);

}

void usart_tx_fifo_send(ug_uint8_t *buffer,ug_size_t size)
{
    struct gd32_usart *usart = serial0.usart;

    _serial_write_tx_fifo(&serial0,buffer, size);
    usart_interrupt_enable(usart->usart_device, USART_INT_TC);
}

void gd32_hw_usart_init(void)
{
    RCC_Configuration();
    GPIO_Configuration();

#if defined (configUse_usart_queue_rev)
    nvic_irq_enable(USART0_IRQn, 0, 0);

    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);

    usart_interrupt_enable(USART0, USART_INT_RBNE);
    usart_enable(USART0);

    usart_queue_init();
#elif defined(configUse_usart_dma)
    nvic_irq_enable(USART0_IRQn, 0, 0);

    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);

    usart_interrupt_enable(USART0, USART_INT_IDLE);
    usart_enable(USART0);

#else
    ug_hw_serial_register(&serial0);
#endif

}

#endif



