#ifndef __GOL_IMUTASK_C__
#define __GOL_IMUTASK_C__
#include "gd32f4xx.h"
#include "systick.h"
#include <stdio.h>
#include "config.h"

#include "drv_rtc.h"

#include "uartadapter.h"
#include "gnss.h"
#include "fwdog.h"
#include "imuTask.h"
#include "nav_task.h"

#include "m_queue.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"





static uint8_t fpga_comm5_rxbuffer[IMU_BUFFER_SIZE];
static uint16_t imu_comm5_len = 0;
TaskHandle_t task_imu_comm5_handler;


SemaphoreHandle_t xImuComm5Semaphore   = NULL;

extern EventGroupHandle_t xFwdogEventGroup;

void imu_comm5_rx(void)
{
    imu_comm5_len = Uart_RecvMsg(UART_RXPORT_COMPLEX_12, 23, fpga_comm5_rxbuffer);

    if(imu_comm5_len == 23)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        xSemaphoreGiveFromISR( xImuComm5Semaphore, &xHigherPriorityTaskWoken );

        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

}
uint16_t ffff;
void imu_comm5_task(void* arg)
{
    uint8_t ucTimeOneSecondChangeSecond;
    for( ;; )
    {
        if(pdTRUE == xSemaphoreTake( xImuComm5Semaphore, portMAX_DELAY))//pdMS_TO_TICKS(100)
        {
            frame_fill_imu(fpga_comm5_rxbuffer, imu_comm5_len); //send to fpga
            imu_notify();

            if(ucTimeOneSecondChangeSecond != RSOFT_RTC_SECOND)
            {
                ucTimeOneSecondChangeSecond = RSOFT_RTC_SECOND;
#ifdef configUse_debug
                //Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, imu_comm5_len, fpga_comm5_rxbuffer);
                ffff = (*(__IO uint16_t*)((uint32_t)0x60000220));
#endif
            }

            imu_comm5_len = 0;
            xEventGroupSetBits(
                xFwdogEventGroup, /* The event group being updated. */
                TASK_IMU_BIT);

        }
    }
}

void imu_task_create(void)
{
    xTaskCreate( imu_comm5_task,
                 "imu_comm5_task",
                 configMINIMAL_STACK_SIZE,
                 ( void * ) NULL,
                 tskIDLE_PRIORITY + 5,
                 (TaskHandle_t*)&task_imu_comm5_handler );
}

void imuTask_init(void)
{
    xImuComm5Semaphore = xSemaphoreCreateBinary();
    configASSERT( xImuComm5Semaphore );

}

#endif

