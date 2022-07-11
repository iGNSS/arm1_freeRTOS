
#include "gd32f3x0.h"
#include "config.h"
#include "clock.h"


static volatile uint32_t ug_tick = 0;

uint32_t ug_tick_get(void)
{
    /* return the global tick */
    return ug_tick;
}

/**
 * This function will set current tick
 */
void ug_tick_set(uint32_t tick)
{
    ug_tick = tick;
}

/**
 * This function will notify kernel there is one tick passed. Normally,
 * this function is invoked by clock ISR.
 */
void ug_tick_increase(void)
{

    /* increase the global tick */
    ++ ug_tick;
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
int ug_tick_from_millisecond(int32_t ms)
{
    int tick;

    if (ms < 0)
        tick = UG_WAITING_FOREVER;
    else
        tick = (UG_TICK_PER_SECOND * ms + 999) / 1000;

    /* return the calculated tick */
    return tick;
}


