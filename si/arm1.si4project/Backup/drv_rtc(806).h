#ifndef DRV_RTC_H_INCLUDED
#define DRV_RTC_H_INCLUDED


#ifdef __cplusplus
extern "C" {
#endif

#include "insdef.h"
#include "gd32f4xx.h"

typedef struct
{
    uint16_t year;                                             
    uint8_t month;                                               
    uint8_t date;                                                   
    uint8_t hour;                                                            
    uint8_t minute;                                                             
    uint8_t second;                                                            
    uint32_t gpsTime;
    uint32_t gpsWeek;
}rtc_update_struct;

void rtc_configuration(void);

rtc_update_struct* rtc_update(void);

#ifdef __cplusplus
}
#endif

#endif // gd32F20X_40X_SPI_H_INCLUDED




