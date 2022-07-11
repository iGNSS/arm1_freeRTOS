
#include "config.h"
#include "clock.h"


static volatile uint32_t ins_tick = 0;

uint32_t ins_tick_get(void)
{
    /* return the global tick */
    return ins_tick;
}

/**
 * This function will set current tick
 */
void ins_tick_set(uint32_t tick)
{
    ins_tick = tick;
}

/**
 * This function will notify kernel there is one tick passed. Normally,
 * this function is invoked by clock ISR.
 */
void ins_tick_increase(void)
{

    /* increase the global tick */
    ++ ins_tick;
}

/**
 * This function will calculate the tick from millisecond.
 *
 * @param ms the specified millisecond
 *           - Negative Number wait forever
 *           - Zero not wait
 *           - Max 0x7fffffff
 *
 * @return the calculated tick
 */
int ins_tick_from_millisecond(int32_t ms)
{
    int tick;

    if (ms < 0)
        tick = INS_WAITING_FOREVER;
    else
        tick = (INS_TICK_PER_SECOND * ms + 999) / 1000;

    /* return the calculated tick */
    return tick;
}


