/*!
    \file    main.c
    \brief   led spark with systick

    \version 2016-08-15, V1.0.0, firmware for GD32F4xx
    \version 2018-12-12, V2.0.0, firmware for GD32F4xx
    \version 2020-09-30, V2.1.0, firmware for GD32F4xx
    \version 2022-03-09, V3.0.0, firmware for GD32F4xx
*/

/*
    Copyright (c) 2022, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f4xx.h"
#include "systick.h"
#include <stdio.h>
#include "exmc_sram.h"
#include "config.h"
#include "drv_gpio.h"
#include "drv_usart.h"
#include "drv_rtc.h"
#include "drv_timer.h"
//#include "public.h"
#include "pin_numbers_def.h"
#include "frame_analysis.h"
#include "uartadapter.h"
#include "gnss.h"
//#include "taskschedule.h"
#include "computerFrameParse.h"
#include "fwdog.h"
#include "fmc_operation.h"
#include "DRamAdapter.h"
#include "nav_task.h"
#include "Time_Unify.h"
#include "m_queue.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"


#define	ARM1_TO_ARM2_IO		GD32F450_PA12_PIN_NUM
#define	SYN_ARM_IO			GD32F450_PE2_PIN_NUM
#define	FPGA_TO_ARM1_INT	GD32F450_PA2_PIN_NUM

#define	ARM1_TO_ARM2_SYN 	gd32_pin_write(SYN_ARM_IO, PIN_LOW)


#define GNSS_BUFFER_SIZE            1024
#define IMU_BUFFER_SIZE             100

#define TASK_GNSS_COMM2_BIT ( 1 << 0 )
#define TASK_GNSS_COMM3_BIT ( 1 << 1 )
#define TASK_IMU_BIT 		( 1 << 2 )
#define ALL_SYNC_BITS 		( TASK_GNSS_COMM2_BIT | TASK_GNSS_COMM3_BIT | TASK_IMU_BIT )

/* SRAM */
uint8_t fpgaBuf[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

static uint8_t fpga_comm2_rxbuffer[GNSS_BUFFER_SIZE];
static uint8_t fpga_comm2_parse_rxbuffer[GNSS_BUFFER_SIZE];
static uint16_t gnss_comm2_len = 0;
static uint16_t gnss_comm2_parse_len = 0;

static uint8_t fpga_comm3_rxbuffer[GNSS_BUFFER_SIZE];
static uint8_t fpga_comm3_parse_rxbuffer[GNSS_BUFFER_SIZE];
static uint16_t gnss_comm3_len = 0;
static uint16_t gnss_comm3_parse_len = 0;

static uint8_t fpga_comm5_rxbuffer[IMU_BUFFER_SIZE];
static uint16_t imu_comm5_len = 0;


//static uint8_t comm2_fifo[GNSS_BUFFER_SIZE];
//static uint8_t comm3_fifo[GNSS_BUFFER_SIZE];
//CirQueue_t comm1_queue;
CirQueue_t comm2_queue;
CirQueue_t comm3_queue;


uint8_t fpga_syn = 0;//fpga同步
uint8_t gnss_comm2_ready = 0;
uint8_t gnss_comm3_ready = 0;
uint8_t imu_ready = 0;


uint8_t fpga_dram_wr[1000] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10,};
uint8_t fpga_dram_rd[1000];

TaskHandle_t task_start_handler;
TaskHandle_t task_gnss_comm2_handler, task_gnss_comm3_handler, task_imu_comm5_handler;

SemaphoreHandle_t xGnssComm2Semaphore  = NULL;
SemaphoreHandle_t xGnssComm3Semaphore  = NULL;
SemaphoreHandle_t xImuComm5Semaphore   = NULL;

QueueHandle_t xCommQueue;

EventGroupHandle_t xFwdogEventGroup = NULL;

extern TaskHandle_t task_imu_handler;
extern TaskHandle_t task_gnss_handler;

void syn_arm2(void)
{
    gd32_pin_write(SYN_ARM_IO, PIN_LOW);
    gd32_pin_write(SYN_ARM_IO, PIN_LOW);
    gd32_pin_write(SYN_ARM_IO, PIN_HIGH);
}

void fpga_dram_fill(void)
{
    uint16_t index = 0;

    for(index = 0; index < 1000; index++)
    {
        fpga_dram_wr[index] = index;
    }
}

void fpga_dram_test(void)
{
    fpga_dram_fill();
    DRam_Write(0, (uint16_t*)fpga_dram_wr, 256);
    DRam_Read(0, (uint16_t*)fpga_dram_rd, 256);
}

static void prvSetupHardware( void )
{
    SystemInit();

    SystemCoreClockUpdate();

    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    systick_config();

#ifdef	configUse_SEGGER_RTT
    SEGGER_RTT_Init();
    SEGGER_RTT_printf(0, "\r\nCK_SYS is %d", rcu_clock_freq_get(CK_SYS));
    SEGGER_RTT_printf(0, "\r\nCK_AHB is %d", rcu_clock_freq_get(CK_AHB));
    SEGGER_RTT_printf(0, "\r\nCK_APB1 is %d", rcu_clock_freq_get(CK_APB1));
    SEGGER_RTT_printf(0, "\r\nCK_APB2 is %d", rcu_clock_freq_get(CK_APB2));
#endif
}
#if 0
/////////////////////////////power down///////////////////////////////////////////
void sleep_int_install(void)
{
//    uint8_t i;
    irq_priority priority =
    {
        .nvic_irq_pre_priority = 0,
        .nvic_irq_sub_priority = 0
    };

    pin_irq_install(  SYN_ARM_IO, PIN_MODE_INPUT_PULLDOWN,
                      PIN_IRQ_MODE_RISING,
                      NULL,
                      NULL,
                      &priority);
}

void sleep_int_uninstall(void)
{

    gd32_pin_irq_enable(SYN_ARM_IO, PIN_IRQ_DISABLE);
}

void system_config_before_sleep(void)
{
    NVIC_DisableIRQ(SysTick_IRQn);
    timer_disable(TIMER0);
    timer_disable(TIMER1);
    timer_disable(TIMER13);
    //adc_disable(ADC0);
}

void system_config_after_wakeup(void)
{

    prvSetupHardware();


}

void system_halt(void)
{
    system_config_before_sleep();
    sleep_int_install();
    pmu_to_deepsleepmode(PMU_LDO_LOWPOWER, PMU_LOWDRIVER_ENABLE, WFI_CMD);
    sleep_int_uninstall();
    system_config_after_wakeup();

}
#endif

#if (configUse_COMM == COMM_MODE_RS422)
#include "nav_task.h"

static uint8_t fpga_comm1_rxbuffer[GNSS_BUFFER_SIZE];
//static uint8_t fpga_comm1_parse_rxbuffer[GNSS_BUFFER_SIZE];
static uint16_t rs422_comm1_len = 0;
//static uint16_t rs422_comm1_parse_len = 0;
//SemaphoreHandle_t xRs422Comm1Semaphore = NULL;

void rs422_comm1_rx(void)
{
	uint8_t status;
    rs422_comm1_len = Uart_RecvMsg(UART_RXPORT_COMPLEX_8, GNSS_BUFFER_SIZE, fpga_comm1_rxbuffer);
    if(rs422_comm1_len)
        //EnCirQueue(fpga_comm1_rxbuffer, rs422_comm1_len,comm1_queue);
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        status = 2;
        xQueueSendFromISR(xCommQueue, &status, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
extern EXPORT_RESULT  g_Export_Result;
void rs422_comm1_task(void* arg)
{
    uint8_t status;
    for( ;; )
    {
        if(pdTRUE == xQueueReceive( xCommQueue, &status, pdMS_TO_TICKS(100)))//portMAX_DELAY
        {
            if(1 == status)//串口发送至上位机
            {
                frame_pack_and_send(&g_Export_Result, &hGPSData);
                //frame_writeDram();
                //Oscilloscope();
            }
            else if(2 == status)//解析上位机数据
            {
                frameParse(fpga_comm1_rxbuffer, rs422_comm1_len);
                rs422_comm1_len = 0;
            }
        }

    }
}
#endif

void gnss_comm2_rx(void)
{
    gnss_comm2_len = Uart_RecvMsg(UART_RXPORT_COMPLEX_9, GNSS_BUFFER_SIZE, fpga_comm2_rxbuffer);
    EnCirQueue(fpga_comm2_rxbuffer, gnss_comm2_len,comm2_queue);
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        xSemaphoreGiveFromISR( xGnssComm2Semaphore, &xHigherPriorityTaskWoken );

        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }

}

void gnss_comm2_parse(uint8_t* pData, uint16_t dataLen)
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
                            || (0 == strncmp("#BESTPOSA", (char const *)fpga_comm2_parse_rxbuffer, 9))
                            || (0 == strncmp("#BESTVELA", (char const *)fpga_comm2_parse_rxbuffer, 9))
                            || (0 == strncmp("$GNTRA", (char const *)fpga_comm2_parse_rxbuffer, 6))
                            || (0 == strncmp("$GPTRA", (char const *)fpga_comm2_parse_rxbuffer, 6))
                      )
                    {
#ifdef configUse_debug
                        //Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, gnss_comm2_parse_len, fpga_comm2_parse_rxbuffer);
                        //gd32_usart_write(fpga_comm2_parse_rxbuffer, gnss_comm2_parse_len);
#endif
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
    EnCirQueue(fpga_comm3_rxbuffer, gnss_comm3_len,comm3_queue);
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        xSemaphoreGiveFromISR( xGnssComm3Semaphore, &xHigherPriorityTaskWoken );

        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}

void gnss_comm3_parse(uint8_t* pData, uint16_t dataLen)
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

//////////////////////////////////////////////////////////////////////////////////
void fpga_int_hdr(void *args)
{
    fpga_syn = 1;
#if (configUse_COMM == COMM_MODE_RS422)
    rs422_comm1_rx();
#endif
    gnss_comm2_rx();

    gnss_comm3_rx();

    imu_comm5_rx();
}

gtime_t tt;
void arm_syn_int_hdr(void *args)
{
    DRam_Read(0, (uint16_t*)&tt.time, 6);
}

void gpio_init(void)
{
    irq_priority priority =
    {
        .nvic_irq_pre_priority = 4,
        .nvic_irq_sub_priority = 0
    };

    //gd32_pin_mode(ARM1_TO_ARM2_IO, PIN_MODE_OUTPUT);
    gd32_pin_mode(SYN_ARM_IO, PIN_MODE_OUTPUT);

    pin_irq_install(  FPGA_TO_ARM1_INT, PIN_MODE_INPUT_PULLDOWN,
                      PIN_IRQ_MODE_RISING,
                      fpga_int_hdr,
                      NULL,
                      &priority);
    pin_irq_install(  ARM1_TO_ARM2_IO, PIN_MODE_INPUT,
                      PIN_IRQ_MODE_RISING_FALLING,
                      arm_syn_int_hdr,
                      NULL,
                      &priority);
}

void cir_queue_init(void)
{
    //comm1_queue = CirQueueGenericCreate(GNSS_BUFFER_SIZE);
    //configASSERT(comm1_queue);
    comm2_queue = CirQueueGenericCreate(GNSS_BUFFER_SIZE);
    configASSERT(comm2_queue);
    comm3_queue = CirQueueGenericCreate(GNSS_BUFFER_SIZE);
    configASSERT(comm3_queue);
}

static void peripherals_init(void)
{
    exmc_sram_init();

    // Uart
    //Uart_TxInit(UART_TXPORT_RS232_1, UART_BAUDRATE_115200BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_DEFAULT, UART_ENABLE);
    //Uart_RxInit(UART_RXPORT_RS232_1, UART_BAUDRATE_115200BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_DEFAULT, UART_ENABLE);

    Uart_TxInit(UART_TXPORT_COMPLEX_8, UART_BAUDRATE_230400BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS422, UART_ENABLE);
    Uart_RxInit(UART_RXPORT_COMPLEX_8, UART_BAUDRATE_230400BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS422, UART_ENABLE);

    Uart_TxInit(UART_TXPORT_COMPLEX_9, UART_BAUDRATE_115200BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS232, UART_ENABLE);
    Uart_RxInit(UART_RXPORT_COMPLEX_9, UART_BAUDRATE_115200BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS232, UART_ENABLE);

    Uart_TxInit(UART_TXPORT_COMPLEX_10, UART_BAUDRATE_115200BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS232, UART_ENABLE);
    Uart_RxInit(UART_RXPORT_COMPLEX_10, UART_BAUDRATE_115200BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS232, UART_ENABLE);

    Uart_TxInit(UART_TXPORT_COMPLEX_11, UART_BAUDRATE_115200BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS232, UART_ENABLE);
    Uart_RxInit(UART_RXPORT_COMPLEX_11, UART_BAUDRATE_115200BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS232, UART_ENABLE);

    Uart_TxInit(UART_TXPORT_COMPLEX_12, UART_BAUDRATE_921600BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS232, UART_ENABLE);
    Uart_RxInit(UART_RXPORT_COMPLEX_12, UART_BAUDRATE_921600BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS232, UART_ENABLE);

    frame_init();

    gd32_hw_usart_init();

    gpio_init();

    gd32_timer_init();

    rtc_configuration();

    cir_queue_init();

//    fwdog_init();
}

void gnss_comm2_task(void* arg)
{

    for( ;; )
    {
        if(pdTRUE == xSemaphoreTake( xGnssComm2Semaphore, portMAX_DELAY))//pdMS_TO_TICKS(100)
        {
            gnss_comm2_parse(fpga_comm2_rxbuffer, gnss_comm2_len); //send to fpga
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
            gnss_comm3_parse(fpga_comm3_rxbuffer, gnss_comm3_len); //send to fpga
            xEventGroupSetBits(
                xFwdogEventGroup, /* The event group being updated. */
                TASK_GNSS_COMM3_BIT);
        }

    }
}

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
#endif
            }

            imu_comm5_len = 0;
            xEventGroupSetBits(
                xFwdogEventGroup, /* The event group being updated. */
                TASK_IMU_BIT);

        }
    }
}

void fwdog_task(void* arg)
{
    EventBits_t uxBits;
    const TickType_t xTicksToWait = pdMS_TO_TICKS( 100 );

    while(1)
    {
        uxBits = xEventGroupWaitBits(
                     xFwdogEventGroup,
                     ALL_SYNC_BITS,
                     pdTRUE,
                     pdTRUE,
                     xTicksToWait );

        if( uxBits & ALL_SYNC_BITS == ALL_SYNC_BITS )
        {
            fwdog_feed();
        }
    }
}

void adjust_rtc(void)
{
    static uint8_t ucTimeOneSecondChangeHour = 0xff;
    static uint8_t rtcAdjInit = 0;

    //开机等待时间有效校准一次，之后每小时校准一次
    if(rtcAdjInit == 0)
    {
        if(INS_EOK == rtc_gnss_adjust_time())
        {
            rtcAdjInit = 1;
            rtc_update();
            ucTimeOneSecondChangeHour = RSOFT_RTC_HOUR;
        }
    }
    else
    {
        if(ucTimeOneSecondChangeHour != RSOFT_RTC_HOUR)
        {
            ucTimeOneSecondChangeHour = RSOFT_RTC_HOUR;

            rtc_gnss_adjust_time();
        }
    }
}

void start_task(void* arg)
{
    taskENTER_CRITICAL();
#if (configUse_COMM == COMM_MODE_RS422)
    xTaskCreate( rs422_comm1_task,
                 "rs422_comm1_task",
                 configMINIMAL_STACK_SIZE,
                 ( void * ) NULL,
                 tskIDLE_PRIORITY + 4,
                 NULL );
#endif
    xTaskCreate( imu_comm5_task,
                 "imu_comm5_task",
                 configMINIMAL_STACK_SIZE,
                 ( void * ) NULL,
                 tskIDLE_PRIORITY + 5,
                 (TaskHandle_t*)&task_imu_comm5_handler );
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

//    xTaskCreate( fwdog_task,
//                 "fwdog_task",
//                 configMINIMAL_STACK_SIZE,
//                 ( void * ) NULL,
//                 tskIDLE_PRIORITY + 3,
//                 NULL);
    xTaskCreate( nav_gnss_task,
                 "nav_gnss_task",
                 configMINIMAL_STACK_SIZE,
                 ( void * ) NULL,
                 tskIDLE_PRIORITY + 8,
                 (TaskHandle_t*)&task_gnss_handler );
    xTaskCreate( nav_imu_task,
                 "nav_task",
                 configMINIMAL_STACK_SIZE,
                 ( void * ) NULL,
                 tskIDLE_PRIORITY + 9,
                 (TaskHandle_t*)&task_imu_handler );
    vTaskDelete(task_start_handler);

    taskEXIT_CRITICAL();
}

int main(void)
{
    hw_interrupt_disable();

    xGnssComm2Semaphore = xSemaphoreCreateBinary();
    configASSERT( xGnssComm2Semaphore );
    xGnssComm3Semaphore = xSemaphoreCreateBinary();
    configASSERT( xGnssComm3Semaphore );
    xImuComm5Semaphore = xSemaphoreCreateBinary();
    configASSERT( xImuComm5Semaphore );
    //xnvaTaskSemaphore = xSemaphoreCreateBinary();
    //configASSERT( xnvaTaskSemaphore );
    xCommQueue = xQueueCreate(10, sizeof(uint8_t));
    configASSERT( xCommQueue );
    xFwdogEventGroup = xEventGroupCreate();
    configASSERT( xFwdogEventGroup );
    prvSetupHardware();
    peripherals_init();
//    fpga_dram_test();
#ifdef configUse_debug
    dbg_periph_enable(DBG_FWDGT_HOLD);
    dbg_periph_enable(DBG_WWDGT_HOLD);
#endif
    //printf("helloworld\r\n");
    xTaskCreate( start_task,
                 "start_task",
                 configMINIMAL_STACK_SIZE,
                 ( void * ) NULL,
                 tskIDLE_PRIORITY + 1,
                 (TaskHandle_t*)&task_start_handler );

    /* Start the scheduler. */
    vTaskStartScheduler();

    for(;;);
}

/**
  * @}
  */

void vApplicationIdleHook( void )
{
#if (configUse_COMM == COMM_MODE_RS232)
    comm_handle();
#endif
    rtc_update();
    adjust_rtc();
}

void vApplicationTickHook( void )
{

}

void OS_PreSleepProcessing(uint32_t ulExpectedIdleTime)
{


}

void OS_PostSleepProcessing(uint32_t ulExpectedIdleTime)
{


}

void vApplicationMallocFailedHook( void )
{

    taskDISABLE_INTERRUPTS();

    for( ;; );
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
    ( void ) pcTaskName;
    ( void ) pxTask;

    /* Run time stack overflow checking is performed if
    configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
    function is called if a stack overflow is detected. */
    taskDISABLE_INTERRUPTS();

    for( ;; );
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(UART4, (uint8_t)ch);

    while(RESET == usart_flag_get(UART4, USART_FLAG_TBE));

    return ch;
}

