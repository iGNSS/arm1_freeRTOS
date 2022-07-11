#include "gd32f4xx.h"
#include "stdio.h"
#include "Gd32f4xx_usart.h"
#include "serial.h"


#define UART0_GPIO_TX       GPIO_PIN_9
#define UART0_GPIO_RX       GPIO_PIN_10
#define UART0_GPIO          GPIOA

static uint32_t gd32_usart_configure(struct serial_device *serial)
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
    //usart_overrun_enable(usart->usart_device);
    usart_receive_config(usart->usart_device, USART_RECEIVE_ENABLE);
    usart_transmit_config(usart->usart_device, USART_TRANSMIT_ENABLE);

    usart_enable(usart->usart_device);

    return INS_EOK;
}

static uint32_t gd32_usart_control(struct serial_device *serial, int cmd, void *arg)
{
    struct gd32_usart* usart = serial->usart;
    uint32_t ctrl_arg = (uint32_t)(arg);

    switch (cmd)
    {
    case DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        UART_DISABLE_IRQ(usart->irqn);
        /* disable interrupt */
        usart_interrupt_disable(usart->usart_device, USART_INT_RBNE);
        break;
    case DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        //UART_ENABLE_IRQ(usart->irqn);
        nvic_irq_enable(usart->irqn, usart->irq_pre_priority, usart->irq_sub_priority);
        /* enable interrupt */
        usart_interrupt_enable(usart->usart_device, USART_INT_RBNE);
        break;

    case DEVICE_CTRL_CONFIG :
        if (ctrl_arg == DEVICE_FLAG_DMA_RX)
        {
        }
        break;

    }

    return INS_EOK;
}

static int gd32_usart_putc(struct serial_device *serial, char ch)
{
    struct gd32_usart *usart = serial->usart;

	usart_data_transmit(usart->usart_device, ch);
	while((usart_flag_get(usart->usart_device, USART_FLAG_TC) == RESET));

    return 1;
}

static int gd32_usart_getc(struct serial_device *serial)
{
    int data;

    struct gd32_usart *usart = serial->usart;

    data = -1;
    if (usart_flag_get(usart->usart_device, USART_FLAG_RBNE) != RESET)
        data = usart_data_receive(usart->usart_device);

    return data;
}

#if defined(USING_UART4)
static const struct usart_ops gd32_usart_ops =
{
    gd32_usart_configure,
    gd32_usart_control,
    gd32_usart_putc,
    gd32_usart_getc
};

/* UART4 device driver structure */
struct gd32_usart usart4 =
{
    UART4,
    UART4_IRQn,
    0,
    0
};
struct serial_device serial4 =
{
    &usart4,
    &gd32_usart_ops,
    SERIAL_CONFIG_DEFAULT,
    NULL,
    NULL
};

void usart_isr(struct serial_device *serial)
{

    struct gd32_usart *usart = serial->usart;
    serial_rx_fifo* rx_fifo;

    if(RESET != usart_interrupt_flag_get(usart->usart_device, USART_INT_FLAG_RBNE))
    {
        /* clear interrupt */
        usart_interrupt_flag_clear(usart->usart_device, USART_INT_FLAG_RBNE);
        usart_flag_clear(usart->usart_device, USART_FLAG_RBNE);
        hw_serial_isr(serial, UG_SERIAL_EVENT_RX_IND);
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

        rx_fifo = (serial_rx_fifo*) serial->serial_rx;
        rx_fifo->is_full = INS_FALSE;
        rx_fifo->put_index = rx_fifo->get_index = 0;
    }
    if (RESET != usart_flag_get(usart->usart_device, USART_FLAG_NERR))
    {
        usart_data_receive(usart->usart_device);
        usart_flag_clear(usart->usart_device, USART_FLAG_NERR);

        rx_fifo = (serial_rx_fifo*) serial->serial_rx;
        rx_fifo->is_full = INS_FALSE;
        rx_fifo->put_index = rx_fifo->get_index = 0;
    }
#endif
}

void UART4_IRQHandler(void)
{
    usart_isr(&serial4);

}

#endif

static void RCC_Configuration(void)
{
#ifdef USING_UART4
    /* enable COM GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);
    // enable USART clock
    rcu_periph_clock_enable(RCU_UART4);

#endif

}

static void GPIO_Configuration(void)
{
#ifdef USING_UART4
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_12);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
    gpio_af_set(GPIOC, GPIO_AF_8, GPIO_PIN_12);

    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_2);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    gpio_af_set(GPIOD, GPIO_AF_8, GPIO_PIN_2);
#endif
}

uint32_t gd32_usart_read(void *buffer,uint32_t size)
{
    return _serial_int_rx(&serial4,buffer,size);

}

uint32_t gd32_usart_write(uint8_t *buffer,uint32_t size)
{
    return _serial_int_tx(&serial4,buffer,size);

}

void usart_tx_fifo_send(uint8_t *buffer,uint32_t size)
{
    struct gd32_usart *usart = serial4.usart;

    _serial_write_tx_fifo(&serial4,buffer, size);
    usart_interrupt_enable(usart->usart_device, USART_INT_TC);
}

void gd32_hw_usart_init(void)
{
    RCC_Configuration();
    GPIO_Configuration();

    hw_serial_register(&serial4);


}




