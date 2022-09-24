#ifndef __GOL_GNSSTASK_C__
#define __GOL_GNSSTASK_C__
#include "gd32f4xx.h"
#include "systick.h"
#include <stdio.h>
#include "config.h"
#include "drv_gpio.h"
#include "drv_usart.h"

#include "uartadapter.h"
#include "gnss.h"
#include "protocol.h"

#include "gnssTask.h"
#include "main.h"

#include "m_queue.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"



static uint8_t fpga_comm2_rxbuffer[GNSS_BUFFER_SIZE];
static uint8_t fpga_comm2_parse_rxbuffer[GNSS_BUFFER_SIZE];
static uint16_t gnss_comm2_len = 0;
static uint16_t gnss_comm2_parse_len = 0;

static uint8_t fpga_comm3_rxbuffer[GNSS_BUFFER_SIZE];
static uint8_t fpga_comm3_parse_rxbuffer[GNSS_BUFFER_SIZE];
static uint16_t gnss_comm3_len = 0;
static uint16_t gnss_comm3_parse_len = 0;

CirQueue_t comm2_queue  = NULL;
CirQueue_t comm3_queue  = NULL;

SemaphoreHandle_t xGnssComm2Semaphore  = NULL;
SemaphoreHandle_t xGnssComm3Semaphore  = NULL;

TaskHandle_t task_gnss_comm2_handler, task_gnss_comm3_handler;

extern EventGroupHandle_t xFwdogEventGroup;

void gnss_comm2_rx(void)
{
    gnss_comm2_len = Uart_RecvMsg(UART_RXPORT_COMPLEX_9, GNSS_BUFFER_SIZE, fpga_comm2_rxbuffer);
    if(Q_SUCCESS == EnCirQueue(fpga_comm2_rxbuffer, gnss_comm2_len,comm2_queue))
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        xSemaphoreGiveFromISR( xGnssComm2Semaphore, &xHigherPriorityTaskWoken );

        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

}

void gnss_comm2_parse(void)
{
    uint8_t temp;
    do
    {
        if(ByteDeCirQueue(&temp, comm2_queue))
        {
            if(('$' == temp) || ('#' == temp))
            {
                gnss_comm2_parse_len = 0;
                fpga_comm2_parse_rxbuffer[gnss_comm2_parse_len] = temp;
                gnss_comm2_parse_len++;
            }
            else if(0x0a == temp)
            {
                if(fpga_comm2_parse_rxbuffer[gnss_comm2_parse_len - 1] == 0x0d)//回车换行结尾
                {
                    fpga_comm2_parse_rxbuffer[gnss_comm2_parse_len] = temp;
                    gnss_comm2_parse_len++;

                    if((0 == strncmp("#AGRICA", (char const *)fpga_comm2_parse_rxbuffer, 7))
                            ||(0 == strncmp("#HEADINGA", (char const *)fpga_comm2_parse_rxbuffer, 9))
                            || (0 == strncmp("#BESTPOSA", (char const *)fpga_comm2_parse_rxbuffer, 9))
                            || (0 == strncmp("#BESTVELA", (char const *)fpga_comm2_parse_rxbuffer, 9))
                            ||(0 == strncmp("$GNRMC", (char const *)fpga_comm2_parse_rxbuffer, 6))
                            ||(0 == strncmp("$GNGGA", (char const *)fpga_comm2_parse_rxbuffer, 6))
                            || (0 == strncmp("$GNTRA", (char const *)fpga_comm2_parse_rxbuffer, 6))
                            || (0 == strncmp("$GPTRA", (char const *)fpga_comm2_parse_rxbuffer, 6))
                      )
                    {
#ifdef configUse_debug
                        //Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, gnss_comm2_parse_len, fpga_comm2_parse_rxbuffer);
                        //gd32_usart_write(fpga_comm2_parse_rxbuffer, gnss_comm2_parse_len);
#endif
                        if(0 == strncmp("$GNRMC", (char const *)fpga_comm2_parse_rxbuffer, 6))
                        {
                            protocol_gnrmcDataGet(fpga_comm2_parse_rxbuffer, &gnss_comm2_parse_len);
                        }
                        if(0 == strncmp("$GNGGA", (char const *)fpga_comm2_parse_rxbuffer, 6))
                        {
                            protocol_gnggaDataGet(fpga_comm2_parse_rxbuffer, &gnss_comm2_parse_len);
                        }
                        gnss_fill_data(fpga_comm2_parse_rxbuffer, gnss_comm2_parse_len);
                        gnss_comm2_parse_len = 0;
                        break;
                    }
                }
            }
            else
            {
                fpga_comm2_parse_rxbuffer[gnss_comm2_parse_len] = temp;
                gnss_comm2_parse_len++;
            }
        }
        else
        {
            break;
        }
    }
    while(1);
}


void gnss_comm3_rx(void)
{

    gnss_comm3_len = Uart_RecvMsg(UART_RXPORT_COMPLEX_10, GNSS_BUFFER_SIZE, fpga_comm3_rxbuffer);
    if(Q_SUCCESS == EnCirQueue(fpga_comm3_rxbuffer, gnss_comm3_len,comm3_queue))
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        xSemaphoreGiveFromISR( xGnssComm3Semaphore, &xHigherPriorityTaskWoken );

        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

void gnss_comm3_parse(void)
{
    uint8_t temp;
    do
    {
        if(ByteDeCirQueue(&temp, comm3_queue))
        {
            if(('$' == temp) || ('#' == temp))
            {
                gnss_comm3_parse_len = 0;
                fpga_comm3_parse_rxbuffer[gnss_comm3_parse_len] = temp;
                gnss_comm3_parse_len++;
            }
            else if(0x0a == temp)
            {
                if(fpga_comm3_parse_rxbuffer[gnss_comm3_parse_len - 1] == 0x0d)//回车换行结尾
                {
                    fpga_comm3_parse_rxbuffer[gnss_comm3_parse_len] = temp;
                    gnss_comm3_parse_len++;

                    if((0 == strncmp("#HEADINGA", (char const *)fpga_comm3_parse_rxbuffer, 9))
                            || (0 == strncmp("#BESTPOSA", (char const *)fpga_comm3_parse_rxbuffer, 9))
                            || (0 == strncmp("#BESTVELA", (char const *)fpga_comm3_parse_rxbuffer, 9))
                            ||(0 == strncmp("$GNRMC", (char const *)fpga_comm3_parse_rxbuffer, 6))
                            ||(0 == strncmp("$GNGGA", (char const *)fpga_comm3_parse_rxbuffer, 6))
                            || (0 == strncmp("$GNVTG", (char const *)fpga_comm3_parse_rxbuffer, 6))
                            || (0 == strncmp("$GNZDA", (char const *)fpga_comm3_parse_rxbuffer, 6))
                            || (0 == strncmp("$GPZDA", (char const *)fpga_comm3_parse_rxbuffer, 6))
                            || (0 == strncmp("$GNTRA", (char const *)fpga_comm3_parse_rxbuffer, 6))
                            || (0 == strncmp("$GPTRA", (char const *)fpga_comm3_parse_rxbuffer, 6))

                      )
                    {
#ifdef configUse_debug
                        //Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, gnss_comm3_parse_len, fpga_comm3_parse_rxbuffer);
                        //gd32_usart_write(fpga_comm3_parse_rxbuffer, gnss_comm3_parse_len);
#endif
                        if(0 == strncmp("$GNRMC", (char const *)fpga_comm3_parse_rxbuffer, 6))
                        {
                            protocol_gnrmcDataGet(fpga_comm3_parse_rxbuffer, &gnss_comm3_parse_len);
                        }
                        if(0 == strncmp("$GNGGA", (char const *)fpga_comm3_parse_rxbuffer, 6))
                        {
                            protocol_gnggaDataGet(fpga_comm3_parse_rxbuffer, &gnss_comm3_parse_len);
                        }
                        gnss_fill_data(fpga_comm3_parse_rxbuffer, gnss_comm3_parse_len);
                        gnss_comm3_parse_len = 0;
                        break;
                    }
                }
            }
            else
            {
                fpga_comm3_parse_rxbuffer[gnss_comm3_parse_len] = temp;
                gnss_comm3_parse_len++;
            }
        }
        else
        {
            break;
        }
    }
    while(1);

}

void gnss_comm2_task(void* arg)
{

    for( ;; )
    {
        if(pdTRUE == xSemaphoreTake( xGnssComm2Semaphore, portMAX_DELAY))//pdMS_TO_TICKS(100)
        {
            gnss_comm2_parse(); //send to fpga
            gnss_comm2_len = 0;
            xEventGroupSetBits(
                xFwdogEventGroup, /* The event group being updated. */
                TASK_GNSS_COMM2_BIT);
        }

    }
}

void gnss_comm3_task(void* arg)
{

    for( ;; )
    {
        if(pdTRUE == xSemaphoreTake( xGnssComm3Semaphore, portMAX_DELAY))//pdMS_TO_TICKS(100)
        {
            gnss_comm3_parse(); //send to fpga
            xEventGroupSetBits(
                xFwdogEventGroup, /* The event group being updated. */
                TASK_GNSS_COMM3_BIT);
        }

    }
}

void gnss_task_create(void)
{
    xTaskCreate( gnss_comm2_task,
                 "gnss_comm2_task",
                 configMINIMAL_STACK_SIZE,
                 ( void * ) NULL,
                 tskIDLE_PRIORITY + 6,
                 (TaskHandle_t*)&task_gnss_comm2_handler );
    xTaskCreate( gnss_comm3_task,
                 "gnss_comm3_task",
                 configMINIMAL_STACK_SIZE,
                 ( void * ) NULL,
                 tskIDLE_PRIORITY + 7,
                 (TaskHandle_t*)&task_gnss_comm3_handler );
}

void gnssTask_init(void)
{
    xGnssComm2Semaphore = xSemaphoreCreateBinary();
    configASSERT( xGnssComm2Semaphore );
    xGnssComm3Semaphore = xSemaphoreCreateBinary();
    configASSERT( xGnssComm3Semaphore );

    comm2_queue = CirQueueGenericCreate(GNSS_BUFFER_SIZE);
    configASSERT(comm2_queue);
    comm3_queue = CirQueueGenericCreate(GNSS_BUFFER_SIZE);
    configASSERT(comm3_queue);
}

#endif

