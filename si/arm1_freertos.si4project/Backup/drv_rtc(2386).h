#ifndef DRV_RTC_H_INCLUDED
#define DRV_RTC_H_INCLUDED

#undef COMMON_EXT
#ifdef  __GOL_RTC_C__
    #define COMMON_EXT
#else
    #define COMMON_EXT extern
#endif	

#ifdef __cplusplus
extern "C" {
#endif

#include "insdef.h"
#include "gd32f4xx.h"

COMMON_EXT uint8_t RAM[6];
#define RAM_SOFT_SECOND 0x00000000 
#define RAM_SOFT_MINUTE 0x00000001 
#define RAM_SOFT_HOUR 0x00000002 
#define RAM_SOFT_DATE 0x00000003 
#define RAM_SOFT_DAY 0x00000003 
#define RAM_SOFT_MONTH 0x00000004 
#define RAM_SOFT_YEAR 0x00000005 

#define RSOFT_RTC_SECOND    RAM[RAM_SOFT_SECOND]
#define RSOFT_RTC_MINUTE    RAM[RAM_SOFT_MINUTE]
#define RSOFT_RTC_HOUR      RAM[RAM_SOFT_HOUR]
#define RSOFT_RTC_DAY       RAM[RAM_SOFT_DAY]
#define RSOFT_RTC_MONTH     RAM[RAM_SOFT_MONTH]
#define RSOFT_RTC_YEAR      RAM[RAM_SOFT_YEAR]

typedef struct
{
    uint16_t year;                                             
    uint8_t month;                                               
    uint8_t date;                                                   
    uint8_t hour;                                                            
    uint8_t minute;                                                             
    uint8_t second;                                                            
    float gpsTime;
    uint32_t gpsWeek;
}rtc_update_struct;

COMMON_EXT void rtc_configuration(void);

COMMON_EXT rtc_update_struct* rtc_update(void);

COMMON_EXT uint8_t rtc_gnss_adjust_time(void) ;

#ifdef __cplusplus
}
#endif

#endif // gd32F20X_40X_SPI_H_INCLUDED




