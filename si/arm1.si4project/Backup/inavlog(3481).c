/***********************************
Inavlog operation module

************************************/
#include "inavlog.h"
#include <stdio.h>


static unsigned int  osa_loglevel = LOG_WARNING;

#ifdef __cplusplus
extern "C" {
#endif

unsigned int inav_get_loglevel()
{
    return osa_loglevel;
}

void inav_set_loglevel(unsigned int level)
{

    if (level >= LOG_LEVEL_COUNT)
    {
        printf("Unsupport log level[%d] (Valid value [0, %d]).\r\r\n", level, LOG_LEVEL_COUNT - 1);
        printf("%d    : LOG_FATAL.\r\n", LOG_FATAL);
        printf("%d    : LOG_ERR.\r\n", LOG_ERR);
        printf("%d    : LOG_WARNING.\r\n", LOG_WARNING);
        printf("%d    : LOG_INFO.\r\n", LOG_INFO);
        printf("%d    : LOG_DEBUG.\r\n", LOG_DEBUG);            
        return;
    }    

    osa_loglevel = level;
    printf("level:%d \n", osa_loglevel);
    return;
}

int inav_datetime_get(osa_datetime_info *p)
{
	//填写从系统获取时间
	return 0;
}
/************************************************************************
 函数名:inav_log
 功能:  log 打印函数,面向应用接口
 输入:
          line         - 行号
          level        -级别
          chfr         - 打印格式化字符串
 输出:
 返回: 
************************************************************************/
void inav_log (unsigned int level, const char *chfr, ...)
{
    unsigned int    msg_curlen  = 0;
    unsigned int    msg_seglen = 0;
    //va_list ap;
    char  local_buf[LOG_ITEM_MAX + 1] = {0};
    char *fi = NULL;    
    unsigned int  fdlevel = LOG_ERR;
	//__DATE__, __TIME__
    osa_datetime_info dataTime;

    if (level >= LOG_LEVEL_COUNT)
    {
        return ;
    }

    fdlevel = osa_loglevel;
    

    //ERR级别以上强制输出
    if (level > LOG_ERR && level > fdlevel)
    {
        return;
    }
    
    inav_datetime_get(&dataTime );

#if 0	
    msg_seglen = vsnprintf(local_buf+msg_curlen,LOG_ITEM_MAX - msg_curlen,"%d_%02d_%02d %02d:%02d:%02d.%03d",
            dataTime.mstDate.wYear, dataTime.mstDate.bMonth, dataTime.mstDate.bDay,
            dataTime.mstTime.bHour, dataTime.mstTime.bMinute, dataTime.mstTime.bSecond, dataTime.mstTime.bMs);

   
   va_start(ap, chfr);
   vsnprintf(local_buf+msg_curlen, LOG_ITEM_MAX - msg_curlen,chfr, ap);
   va_end (ap); 
#endif	
   printf("%s\r\n", local_buf);
    //填写 发送到制定串口	
    //UartSend(local_buf);

    return ;
}




#ifdef __cplusplus
}
#endif

