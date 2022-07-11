#ifndef _CLOCK_H_
#define _CLOCK_H_		 

#define UG_WAITING_FOREVER              -1              /**< Block forever until get resource. */
#define UG_WAITING_NO                   0               /**< Non-block. */

uint32_t ug_tick_get(void);
void ug_tick_set(uint32_t tick);
void ug_tick_increase(void);
int ug_tick_from_millisecond(int32_t ms);

#endif
