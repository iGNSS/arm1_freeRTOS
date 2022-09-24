
#include "gd32f4xx.h"
#include "fwdog.h"
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"


EventGroupHandle_t xFwdogEventGroup = NULL;

void fwdog_init(void)
{
	rcu_osci_on(RCU_IRC32K);
    
    /* wait till IRC32K is ready */
    while(SUCCESS != rcu_osci_stab_wait(RCU_IRC32K)){
    }
    
    /* confiure FWDGT counter clock: 32KHz(IRC32K) / 64 = 0.5 KHz */
    fwdgt_config(5*500,FWDGT_PSC_DIV64);
    
    /* After 2 seconds to generate a reset */
    fwdgt_enable();
}

void fwdog_feed(void)
{
    /* reload FWDGT counter */
    fwdgt_counter_reload();
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

void fwdog_task_create(void)
{
    xTaskCreate( fwdog_task,
                 "fwdog_task",
                 configMINIMAL_STACK_SIZE,
                 ( void * ) NULL,
                 tskIDLE_PRIORITY + 3,
                 NULL);
}

