/*****************************************************************************************
***********************************************************************************
All rights reserved I-NAV 2022 2023
***********************************************************************************/

/***********************************************************************************
Modification history

|-----------+---------------+-------------------|
|Author          |    Date               |    Done                      |
|-----------+---------------+-------------------|
|DengWei       |  2022-6-8          | First Creation             |
|-----------+---------------+-------------------|  
***********************************************************************************/
#ifndef _INAVLOG_H
#define _INAVLOG_H

#define  LOG_ITEM_MAX       (10*1024)                  /*每条日志信息最大长*/

#define INAVMD(ulLevel) (char*)(__FILE__), (unsigned int)(__LINE__), (ulLevel)

typedef enum 
{
    LOG_FATAL = 0,
    LOG_ERR,
    LOG_WARNING,
    LOG_INFO,
    LOG_DEBUG,  
    LOG_LEVEL_COUNT
}inav_log_level_t;


typedef struct _osa_time_info
{
     unsigned char   bHour;                                                        /* (0~23)  Hour in 24 format */
     unsigned char   bMinute;                                                      /* (0~59)  Minute in 60 format */
     unsigned char   bSecond;                                                      /* (0~59)  Second in 60 format */
     unsigned char   bSpare;
     unsigned char bMs;                                                             /* ms */
}osa_time_info;

typedef struct _osa_date_info
{
     unsigned short    wYear;                                                      /* Year in YYYY format for example 2002*/
     unsigned char     bMonth;                                                     /* (1~12) Month in MM format */
     unsigned char     bDay;                                                       /* (1~31) Day in DD format */
}osa_date_info;

typedef struct _osa_datetime_info
{
        osa_date_info  mstDate;
        osa_time_info mstTime;
}osa_datetime_info;

#ifdef __cplusplus
extern "C" {
#endif

void inav_set_loglevel(unsigned int loglevel);
unsigned int inav_get_loglevel(void);
void inav_log (const char *file, unsigned int line,unsigned int level, const char *chfr, ...);

#ifdef __cplusplus
}
#endif

#endif
