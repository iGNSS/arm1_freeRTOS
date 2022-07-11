#ifndef PUBLIC_H_INCLUDED
#define PUBLIC_H_INCLUDED


#ifdef __cplusplus
extern "C" {
#endif

#include "insdef.h"
#include "gd32f4xx.h"

#define TIMER_TRIGGER	0                     
#define ONCE			TIMER_TRIGGER    
#define TIMER_ENABLE	0xBA                
#define TIMER_DISENABLE	0x10		             



typedef struct sttimer
{
	uint32_t counter;					
	uint32_t interval;					
	uint8_t  enabled;					  
	void (*operate)(void);					
}SOFT_TIMER;
	


#define PUBLICSECS(x)	(uint32_t)((x) * INS_TICK_PER_SECOND)  

void PublicSetCycTimer(SOFT_TIMER *stTimer,uint32_t value,void (*function)(void));

void PublicSetOnceTimer(SOFT_TIMER *stTimer,uint32_t value,void (*function)(void));

void Public_SetTestTimer(void (*function)(void),uint32_t time);

EventStatus PublicTimerTask(void);

#ifdef __cplusplus
}
#endif

#endif 



