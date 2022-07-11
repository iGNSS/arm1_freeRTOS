
#include "gd32f3x0.h"

#include "config.h"
#include "clock.h"
#include "taskschedule.h"
#include "app_usart.h"
#include "key.h"
#include "led.h"
#include "iqs.h"
#include "comm.h"
#include "system.h"
#include "scan.h"


static uint32_t	TimeTaskResumeSta[MAX_TIMETASK/32+1] = {0,};

extern EventStatus iqs5xx_ble_status(void);
extern EventStatus comm_repeat_report(void);
TIME_TASK MyTimerTask[MAX_TIMETASK] =
{
#if !defined(configUse_usart_dma)
#ifdef configUse_usart_queue_rev
    {ENABLE, DISABLE, 0, 4,      usart_dispose_recvDataTask},
#else
    {ENABLE, DISABLE, 0, 1, 	 usart_dispose_recvDataTask},
#endif
#endif
    {ENABLE, DISABLE, 20, 200,      system_1s_task},
#ifdef configUse_flex_button
    {ENABLE, DISABLE, 0, 10,     button_scan},
#else
    {ENABLE, DISABLE, 0, 2,      key_scan},
#endif
    {ENABLE, DISABLE, 0, 10,     led_flash_monitor},
    {ENABLE, DISABLE, 100,10,    system_judge_curr_mode},
#if defined( configUse_ESDTEST )
    //{ENABLE, 0, 400,    iqs6xx_err_scan},
#endif
    {DISABLE,DISABLE, 0, 10,     iqs5xx_clear_status},
    {DISABLE,DISABLE, 1, 1,      comm_repeat_report},
    {DISABLE,DISABLE, 0, 50,     key_longPressReset},
    //{DISABLE,DISABLE, 0, 500,     iqs5xx_ble_status}
};


/*******************************************************************************
* Function Name  : SetTimerTask
* Description    : Setup timer task .
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SetTimerTask(TIMERTASK task, uint32_t time)
{
    MyTimerTask[task].TaskTimerState = ENABLE;
    MyTimerTask[task].TaskExTimer = time;
    MyTimerTask[task].TaskTimerCount = time + ug_tick_get();
}

/*******************************************************************************
* Function Name  : ClrTimerTask
* Description    : Cancel timer task .
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ClrTimerTask(TIMERTASK task)
{
    MyTimerTask[task].TaskTimerState = DISABLE;
}

/*******************************************************************************
* Function Name  : ResumeTimerTask
* Description    : resume timer task .
* Input          : task MAX_TIMETASK: resume all task otherwise resume appointed task
* Output         : None
* Return         : None
*******************************************************************************/
void ResumeTimerTask(TIMERTASK task)
{
    uint8_t i,j,k;
    if (task > MAX_TIMETASK)return;
    if (task == MAX_TIMETASK)//恢复所有任务
    {
        for(i = 0; i < MAX_TIMETASK; i++)
        {
            j = i/32;
            k = i%32;
            if(BIT_CHECK(TimeTaskResumeSta[j],k))
            {
                BIT_CLR(TimeTaskResumeSta[j],k);
                MyTimerTask[i].TaskTimerState = ENABLE;
            }
        }
    }
    else
    {
        MyTimerTask[task].TaskTimerState = ENABLE;
    }
}

/*******************************************************************************
* Function Name  : SuspendTimerTask
* Description    : suspend timer task .
* Input          : task MAX_TIMETASK: suspend all task otherwise suspend appointed task
* Output         : None
* Return         : None
*******************************************************************************/
void SuspendTimerTask(TIMERTASK task)
{
    uint8_t i,j,k;
    if (task > MAX_TIMETASK)return;
    if (task == MAX_TIMETASK)//挂起所有任务
    {
        for(i = 0; i < MAX_TIMETASK; i++)
        {
            if(MyTimerTask[i].TaskTimerState == ENABLE)
            {
                j = i/32;
                k = i%32;
                BIT_SET(TimeTaskResumeSta[j],k);
                MyTimerTask[i].TaskTimerState = DISABLE;
            }
        }
    }
    else
    {
        MyTimerTask[task].TaskTimerState = DISABLE;
    }
}
uint8_t  task_status = 0;
uint16_t task_posi_count = 0;
uint16_t task_minus_count = 0;
#include "gpio.h"
/*******************************************************************************
* Function Name  : TimerTaskScheduler
* Description    : Schedule Timer task .
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void TimerTaskScheduler(void)
{
    uint8_t i;
    uint32_t nowtimer;
    EventStatus validflag;
    nowtimer = ug_tick_get();

    for(i = 0; i < MAX_TIMETASK; i++)
    {
        if(MyTimerTask[i].TaskTimerState != ENABLE)
        {
            continue;
        }
        if(MyTimerTask[i].TaskOverflow == ENABLE)
        {
            if(nowtimer >= MyTimerTask[i].TaskTimerCount)
            {
                MyTimerTask[i].TaskTimerCount = nowtimer + MyTimerTask[i].TaskExTimer;
                validflag = MyTimerTask[i].operate();
                MyTimerTask[i].TaskTimerState = validflag;
                MyTimerTask[i].TaskOverflow = DISABLE;
            }
        }
        else
        {
            if(nowtimer >= MyTimerTask[i].TaskTimerCount)
            {
                MyTimerTask[i].TaskTimerCount = nowtimer + MyTimerTask[i].TaskExTimer;
                validflag = MyTimerTask[i].operate();
                MyTimerTask[i].TaskTimerState = validflag;
                if(MyTimerTask[i].TaskTimerCount < nowtimer)
                    MyTimerTask[i].TaskOverflow = ENABLE;

            }
        }
    }
}


