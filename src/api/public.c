#include "public.h"
#include "clock.h"

enum 
{
    PUBLIC_TIMER_EIEXPAND,                  //
    PUBLIC_TIMER_TASK,                      //  
    PUBLIC_TIMER_TEST,                      //  
    PUBLIC_TIMER_OPERATE,                   //      
    PUBLIC_TIMERS_MAX
}STPUBLICTIME;
static SOFT_TIMER soft_PUBLICTimer[PUBLIC_TIMERS_MAX];

void PublicSetCycTimer(SOFT_TIMER *stTimer,uint32_t value,void (*function)(void))
{
    stTimer->enabled  = TIMER_ENABLE;
    stTimer->counter  = value+ins_tick_get();
    stTimer->interval = value;
    stTimer->operate = function;
}

void PublicSetOnceTimer(SOFT_TIMER *stTimer,uint32_t value,void (*function)(void))
{
    stTimer->enabled  = TIMER_ENABLE;
    stTimer->counter  = value+ins_tick_get();
    stTimer->interval = ONCE;
    stTimer->operate = function;
}

void PublicKillTimer(SOFT_TIMER *stTimer)
{
    stTimer->enabled  = TIMER_DISENABLE;
}

uint8_t PublicGetTimerEnable(SOFT_TIMER *stTimer)
{
    return (TIMER_ENABLE == stTimer->enabled)? 1 : 0;
}

void PublicKillTimerAll(SOFT_TIMER *stTimers,unsigned char maxtimes)
{
    unsigned char i;
    for(i = 0; i < maxtimes; i++)
    {
        stTimers[i].enabled = TIMER_DISENABLE;
    }
}

void PublicTimerHandler(SOFT_TIMER *stTimers,unsigned char maxtimes)
{
    unsigned char i;
    unsigned long timerVal;
    timerVal=ins_tick_get();
    for(i = 0; i < maxtimes; i++)
    {
        if(TIMER_ENABLE == stTimers[i].enabled)
        {
            if(stTimers[i].counter <= timerVal)//
            {
                if(ONCE == stTimers[i].interval)
                {
                    stTimers[i].enabled = TIMER_DISENABLE;
                }
                else
                {
                    stTimers[i].counter += stTimers[i].interval;
                    if(stTimers[i].counter< timerVal)
                        stTimers[i].counter = timerVal;
                }
                
                if(NULL != stTimers[i].operate)
                stTimers[i].operate();
            }
        }
    }
}

void Public_SetTestTimer(void (*function)(void),uint32_t time)
{
    PublicSetCycTimer(&soft_PUBLICTimer[PUBLIC_TIMER_TEST],time,function);
}

EventStatus PublicTimerTask(void)
{
    PublicTimerHandler(soft_PUBLICTimer,PUBLIC_TIMERS_MAX);
    
    return ENABLE;
}

