#ifndef DRV_TIMER_H_INCLUDED
#define DRV_TIMER_H_INCLUDED


#ifdef __cplusplus
extern "C" {
#endif

#include "insdef.h"
#include "gd32f4xx.h"

  
#define TIM_BASE_PERIOD	1000

void gd32_timer_init(void);  
void CompensateTimeTicksClr(void);
uint32_t CompensateTimeTicksGet(void);  
  
  
#ifdef __cplusplus
}
#endif

#endif // gd32F20X_40X_TIMER_H_INCLUDED








