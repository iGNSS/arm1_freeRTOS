#ifndef _CLOCK_H_
#define _CLOCK_H_		 

#define INS_WAITING_FOREVER              -1              /**< Block forever until get resource. */
#define INS_WAITING_NO                   0               /**< Non-block. */

uint32_t ins_tick_get(void);
void ins_tick_set(uint32_t tick);
void ins_tick_increase(void);
int ins_tick_from_millisecond(int32_t ms);

#endif
