#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "Gd32f3x0_usart.h"
//#include "drv_pin.h"
#include "gpio.h"
#include "serial.h"
//#include "clock.h"
#include "ugservice.h"


#ifdef  configUse_wireless


/*
 * Serial interrupt routines
 */
ug_size_t _serial_int_rx(struct ug_serial_device *serial, ug_uint8_t *data, int length)
{
    int size;
    ug_serial_rx_fifo* rx_fifo;

    size = length;

    rx_fifo = (ug_serial_rx_fifo*) serial->serial_rx;
    configASSERT(rx_fifo != UG_NULL);

    /* read from software FIFO */
    while (length)
    {
        int ch;

        /* disable interrupt */
//        ug_hw_interrupt_disable();

        /* there's no data: */
        if ((rx_fifo->get_index == rx_fifo->put_index) && (rx_fifo->is_full == UG_FALSE))
        {
            /* no data, enable interrupt and break out */
//            ug_hw_interrupt_enable();
            break;
        }

        /* otherwise there's the data: */
        ch = rx_fifo->buffer[rx_fifo->get_index];
        rx_fifo->get_index += 1;
        if (rx_fifo->get_index >= serial->config.bufsz) rx_fifo->get_index = 0;

        if (rx_fifo->is_full == UG_TRUE)
        {
            rx_fifo->is_full = UG_FALSE;
        }

        /* enable interrupt */
//        ug_hw_interrupt_enable();

        *data = ch & 0xff;
        data ++;
        length --;
    }

    return size - length;
}

ug_size_t _serial_int_tx(struct ug_serial_device *serial, const ug_uint8_t *data, int length)
{
    int size;

    configASSERT(serial != UG_NULL);

    size = length;

    while (length)
    {
        serial->ops->putc(serial, *(char*)data);
        data ++;
        length --;
    }

    return size - length;
}

ug_size_t _serial_read_tx_fifo(struct ug_serial_device *serial, ug_uint8_t *data, int length)
{
    int size;
    ug_serial_tx_fifo* tx_fifo;

    size = length;

    tx_fifo = (ug_serial_tx_fifo*) serial->serial_tx;
    configASSERT(tx_fifo != UG_NULL);

    /* read from software FIFO */
    while (length)
    {
        int ch;

        /* disable interrupt */
//        ug_hw_interrupt_disable();

        /* there's no data: */
        if ((tx_fifo->get_index == tx_fifo->put_index) && (tx_fifo->is_full == UG_FALSE))
        {
            /* no data, enable interrupt and break out */
//            ug_hw_interrupt_enable();
            break;
        }

        /* otherwise there's the data: */
        ch = tx_fifo->buffer[tx_fifo->get_index];
        tx_fifo->get_index += 1;
        if (tx_fifo->get_index >= serial->config.bufsz) tx_fifo->get_index = 0;

        if (tx_fifo->is_full == UG_TRUE)
        {
            tx_fifo->is_full = UG_FALSE;
        }

        /* enable interrupt */
//        ug_hw_interrupt_enable();

        *data = ch & 0xff;
        data ++;
        length --;
    }

    return size - length;
}

void _serial_write_tx_fifo(struct ug_serial_device *serial, ug_uint8_t *pdata, int length)
{
    ug_serial_tx_fifo* tx_fifo;

    tx_fifo = (ug_serial_rx_fifo*)serial->serial_tx;
    configASSERT(tx_fifo != UG_NULL);

    /* disable interrupt */
//    ug_hw_interrupt_disable();

    while (length--)
    {
        tx_fifo->buffer[tx_fifo->put_index] = *pdata++;
        tx_fifo->put_index += 1;
        if (tx_fifo->put_index >= serial->config.bufsz) tx_fifo->put_index = 0;

        if (tx_fifo->put_index == tx_fifo->get_index)
        {
            tx_fifo->get_index += 1;
            tx_fifo->is_full = UG_TRUE;
            if (tx_fifo->get_index >= serial->config.bufsz) tx_fifo->get_index = 0;
        }
    }
    /* enable interrupt */
//    ug_hw_interrupt_enable();

}

static ug_err_t ug_serial_open(struct ug_serial_device *serial, ug_uint16_t oflag)
{
    ug_err_t result = UG_EOK;

    /* initialize the Rx/Tx structure according to open flag */
    if (serial->serial_rx == UG_NULL)
    {
        if (oflag & UG_DEVICE_FLAG_INT_RX)
        {
            ug_serial_rx_fifo* rx_fifo;

            rx_fifo = (ug_serial_rx_fifo*) malloc (sizeof(ug_serial_rx_fifo) +
                                                   serial->config.bufsz);
            configASSERT(rx_fifo != UG_NULL);
            rx_fifo->buffer = (ug_uint8_t*) (rx_fifo + 1);
            memset(rx_fifo->buffer, 0, serial->config.bufsz);
            rx_fifo->put_index = 0;
            rx_fifo->get_index = 0;
            rx_fifo->is_full = UG_FALSE;

            serial->serial_rx = rx_fifo;
            /* configure low level device */
            serial->ops->control(serial, UG_DEVICE_CTRL_SET_INT, (void *)UG_DEVICE_FLAG_INT_RX);
        }
        else
        {
            serial->serial_rx = UG_NULL;
        }
    }

    if (serial->serial_tx == UG_NULL)
    {
        if (oflag & UG_DEVICE_FLAG_INT_TX)
        {
            ug_serial_tx_fifo* tx_fifo;

            tx_fifo = (ug_serial_tx_fifo*) malloc (sizeof(ug_serial_tx_fifo) +
                                                   serial->config.bufsz);
            configASSERT(tx_fifo != UG_NULL);
            tx_fifo->buffer = (ug_uint8_t*) (tx_fifo + 1);
            memset(tx_fifo->buffer, 0, serial->config.bufsz);
            tx_fifo->put_index = 0;
            tx_fifo->get_index = 0;
            tx_fifo->is_full = UG_FALSE;

            serial->serial_tx = tx_fifo;
        }
        else
        {
            serial->serial_tx = UG_NULL;
        }
    }

    /* apply configuration */
    if (serial->ops->configure)
        result = serial->ops->configure(serial);

    return result;
}

/*
 * serial register
 */
ug_err_t ug_hw_serial_register(struct ug_serial_device *serial)
{
    configASSERT(serial != UG_NULL);
    struct gd32_usart *usart = serial->usart;

    usart_deinit(usart->usart_device);

    ug_serial_open(serial,UG_DEVICE_FLAG_INT_RX/* | UG_DEVICE_FLAG_INT_TX*/);

    return UG_EOK;
}

/* ISR for serial interrupt */
void ug_hw_serial_isr(struct ug_serial_device *serial, int event)
{
    switch (event & 0xff)
    {
    case UG_SERIAL_EVENT_RX_IND:
    {
        int ch = -1;
        ug_serial_rx_fifo* rx_fifo;

        /* interrupt mode receive */
        rx_fifo = (ug_serial_rx_fifo*)serial->serial_rx;
        configASSERT(rx_fifo != UG_NULL);

        while (1)
        {
            ch = serial->ops->getc(serial);
            if (ch == -1)
            {
                break;
            }

            /* disable interrupt */
//                ug_hw_interrupt_disable();

            rx_fifo->buffer[rx_fifo->put_index] = ch;
            rx_fifo->put_index += 1;
            if (rx_fifo->put_index >= serial->config.bufsz) rx_fifo->put_index = 0;

            /* if the next position is read index, discard this 'read char' */
            if (rx_fifo->put_index == rx_fifo->get_index)
            {
                rx_fifo->get_index += 1;
                rx_fifo->is_full = UG_TRUE;
                if (rx_fifo->get_index >= serial->config.bufsz) rx_fifo->get_index = 0;
            }

            /* enable interrupt */
//                ug_hw_interrupt_enable();
        }

        break;
    }
    case UG_SERIAL_EVENT_TX_DONE:
    {

        break;
    }
    }
}

#endif




