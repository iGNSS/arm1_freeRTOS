/***********************************
Inavlog operation module

************************************/
#include "inavlog.h"
#include <stdio.h>
#include "drv_rtc.h"
#include "stdarg.h"
#include <string.h>

static unsigned int  osa_loglevel = LOG_WARNING;
char g_PrintBuff[1024*4]={0};

static char *osa_level_str[LOG_LEVEL_COUNT+1] = {"FATAL", "ERROR", "WARNING", "INFO", "DEBUG","UNKNOW"};

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

int inav_datetime_get(osa_datetime_info *p,rtc_update_struct* rtc_update_info)
{
	//填写从系统获取时间
	p->mstDate.wYear = rtc_update_info->year;
	p->mstDate.bMonth = rtc_update_info->month;
	p->mstDate.bDay = rtc_update_info->date;
	p->mstTime.bHour = rtc_update_info->hour;
	p->mstTime.bMinute = rtc_update_info->minute;
	p->mstTime.bSecond = rtc_update_info->second;
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
void inav_log (const char *file, unsigned int line,unsigned int level, const char *chfr, ...)
{
	va_list ap;
	unsigned int    msg_curlen  = 0;
	unsigned int    msg_seglen = 0;
	//char  local_buf[LOG_ITEM_MAX + 1] = {0};
  memset(g_PrintBuff,0,sizeof(g_PrintBuff));
	char *fi = NULL;    
	unsigned int  fdlevel = LOG_ERR;
	//__DATE__, __TIME__
	osa_datetime_info dataTime;

	memset(&dataTime,0,sizeof(dataTime));

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

	//去掉文件名路径
	if (file != NULL) 
	{
	    fi = strrchr (file, '/');
	    
	    if (fi == NULL)
	    {
	        fi = strrchr (file, '\\');
	    }
	    
	    if (fi != NULL)
	    {
	        fi++;
	    }
	    
	    if (fi == NULL)
	    {
	        fi = (char *)file;
	    }
	}    
    
    inav_datetime_get(&dataTime, rtc_update());

   	sprintf(g_PrintBuff,"%04x_%02x_%02x %02x:%02x:%02x.%03x|%s|",dataTime.mstDate.wYear, dataTime.mstDate.bMonth, dataTime.mstDate.bDay,
            dataTime.mstTime.bHour, dataTime.mstTime.bMinute, dataTime.mstTime.bSecond, dataTime.mstTime.bMs,
            osa_level_str[level]);


	msg_curlen = strlen(g_PrintBuff);
	if (NULL != fi)
	{
	    sprintf(g_PrintBuff+msg_curlen,"%s|", fi);   
	}
	
	msg_curlen = strlen(g_PrintBuff);
	//if (line >= 0)
	{
	    sprintf(g_PrintBuff+msg_curlen,"%d|", line);        
	}

   	msg_curlen = strlen(g_PrintBuff);
   	va_start(ap, chfr);
   	vsnprintf(g_PrintBuff+msg_curlen, LOG_ITEM_MAX - msg_curlen,chfr, ap);
   	va_end (ap); 

	msg_curlen = strlen(g_PrintBuff);
	sprintf(g_PrintBuff+msg_curlen,"\n");   
	
   	printf("%s", g_PrintBuff);
	//SEGGER_RTT_printf(0,"%s", g_PrintBuff);
    	//填写 发送到制定串口	
    	//UartSend(local_buf);
   memset(g_PrintBuff,0,sizeof(g_PrintBuff));
    return ;
}




#ifdef __cplusplus
}
#endif

