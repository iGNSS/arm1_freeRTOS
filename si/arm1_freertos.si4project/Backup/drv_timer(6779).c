
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"

#include "stdio.h"
#include "config.h"
#include "drv_timer.h"
#include "nav_task.h"
#include "computerFrameParse.h"


extern QueueHandle_t xCommQueue;

#ifdef	configUse_SystemView

volatile uint32_t FreeRTOSRunTimeTicks;

void ConfigureTimeForRunTimeStats(void)
{
    FreeRTOSRunTimeTicks = 0;
}

void timer1_init(void)
{
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER1);

    rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);//200M

    nvic_irq_enable(TIMER1_IRQn, 5, 0);

    timer_deinit(TIMER1);

    /* TIMER3 configuration */
    timer_initpara.prescaler         = 200 - 1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 100;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;

    timer_init(TIMER1, &timer_initpara);
    /* enable TIMERI auto-reload shadow function */
    timer_auto_reload_shadow_enable( TIMER1 );
    timer_update_source_config(TIMER1, TIMER_UPDATE_SRC_REGULAR);
    /* clear the update interrupt bit */
    timer_interrupt_flag_clear( TIMER1, TIMER_INT_UP );
    /* enable the update interrupt */
    timer_interrupt_enable( TIMER1, TIMER_INT_UP);
    /* TIMERI counter enable */
    timer_enable( TIMER1);
}

void TIMER1_IRQHandler()
{
    if(SET == timer_interrupt_flag_get(TIMER1, TIMER_INT_UP))
    {
        timer_interrupt_flag_clear( TIMER1, TIMER_INT_UP );
        FreeRTOSRunTimeTicks++;
    }
}

#endif
/**************************************************************************************************/
/*                                                                                                */
/*                    					timer3_init                                               */
/*                                                                                                */
/**************************************************************************************************/
/**
 * @brief   timer3 initialization  1000Hz
 * @param   none
 *
 * @retval  None
 */
void timer3_init(void)
{
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER3);

    rcu_timer_clock_prescaler_config(RCU_TIMER_PSC_MUL4);//200M

    nvic_irq_enable(TIMER3_IRQn, 5, 0);

    timer_deinit(TIMER3);

    /* TIMER3 configuration */
    timer_initpara.prescaler         = 200 - 1;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = TIM_BASE_PERIOD;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;

    timer_init(TIMER3, &timer_initpara);
    /* enable TIMERI auto-reload shadow function */
    timer_auto_reload_shadow_enable( TIMER3 );
    timer_update_source_config(TIMER3, TIMER_UPDATE_SRC_REGULAR);
    /* clear the update interrupt bit */
    timer_interrupt_flag_clear( TIMER3, TIMER_INT_UP );
    /* enable the update interrupt */
    timer_interrupt_enable( TIMER3, TIMER_INT_UP);
    /* TIMERI counter enable */
    timer_enable( TIMER3);
}

void TIMER3_IRQHandler()
{
    uint8_t status;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(SET == timer_interrupt_flag_get(TIMER3, TIMER_INT_UP))
    {
        timer_interrupt_flag_clear( TIMER3, TIMER_INT_UP );
 
        status = 1;
        xQueueSendFromISR(xCommQueue, &status, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        
    }
}

void gd32_timer_init(void)
{
#ifdef	configUse_SystemView
    timer1_init();
#endif
    timer3_init();
}


