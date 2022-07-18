/*****************************************************************************************
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
#ifndef _INAV_GNSS_H
#define _INAV_GNSS_H


enum ENavGnssStatus 
{
    NAV_GNSS_STATUS_LOST =0,       	//Ê§Ëø
    NAV_GNSS_STATUS_SPP=1,			//µ¥µã¶¨Î»
    NAV_GNSS_STATUS_RTD=2,		//Î±¾à²î·Ö
    NAV_GNSS_STATUS_RTK_FIX=4,	      //RTK¹Ì¶¨½â
    NAV_GNSS_STATUS_RTK_FLOAT=5,	//RTK¸¡µã½â
};


#include "constant.h"
// GNSSÔ­Ê¼Êı¾İ
typedef struct I_NAV_GNSS_RESULT_T {
	unsigned int	gpsweek; //gpsÖÜ×Ô 1980-1-6 ÖÁµ±Ç°µÄĞÇÆÚÊı£¨ ¸ñÁÖÄáÖÎÊ±¼ä£© 
	float 		gpssecond;//gpsÖÜÄÚÃë×Ô±¾ÖÜÈÕ 00:00:00 ÖÁµ±Ç°µÄÃëÊı£¨ ¸ñÁÖÄáÖÎÊ±¼ä£©
	float 		heading;//Æ«º½½Ç£¨ 0~359.99£© 
	float			pitch;//¸©Ñö½Ç£¨ -90~90£© 
	float			roll;//ºá¹ö½Ç£¨ -180~180£© 
	float			latitude;//Î³¶È£¨ -90~90£© 
	float			longitude;//¾­¶È£¨ -180~180£©
	float			altitude;//¸ß¶È£¬ µ¥Î»£¨ Ã×£© 
	float			ve;//¶«ÏòËÙ¶È£¬ µ¥Î»£¨ Ã×/Ãë£© 
	float 		vn;//±±ÏòËÙ¶È£¬ µ¥Î»£¨ Ã×/Ãë£©
	float			vu;//ÌìÏòËÙ¶È£¬ µ¥Î»£¨ Ã×/Ãë£©
	float			baseline;//»ùÏß³¤¶È£¬ µ¥Î»£¨ Ã×£© 
	unsigned int	nsv;//ÎÀĞÇÊı 
	unsigned int	gnsslocatestatus;
	unsigned int	gnssstatus;//ÏµÍ³×´Ì¬£º0£º Ê§Ëø£º µ¥µã¶¨Î»2£º Î±¾à²î·Ö4£º ÔØ²¨ÏàÎ»²î·Ö£¨ ¶¨µã£©5£º ÔØ²¨ÏàÎ»²î·Ö£¨ ¸¡µã£©
	double		utc;//UTC Ê±¼ä

	//ÊÇ·ñÌá¹©×¼È·¶¨Î»¾«¶È¶¨ËÙÎó²î
	unsigned char supportposvelstd; //0²»Ö§³Ö£¬1Ö§³Ö
	//¾«¶ÈĞÅÏ¢
	//¶¨Î»¾«¶È
	double		latstd;//Î³¶È¾«¶È
	double		logstd;//¾­¶È¾«¶È
	double		hstd;//¸ß³Ì¾«¶È

	double hdgstddev;					/* èˆªå‘è§’æ ‡å‡†å·®, å•ä½ï¼š m*/
	double ptchstddev;
	
	double		vestd;//¶«ÏòËÙ¶È¾«¶È
	double		vnstd;//±±ÏòËÙ¶È¾«¶È
	double		vustd;//ÌìÏòËÙ¶È¾«¶È

	//gnssÆô¶¯±êÖ¾
	unsigned int	gnssstartflag;// 1: ÒÑ¾­¿ÉÒÔ»ñÈ¡gnss½á¹û
		
}I_NAV_GNSS_RESULT;


#ifdef __cplusplus
extern "C" {
#endif
void InitialGnssResultParm(void);
I_NAV_GNSS_RESULT *GetGnssResultPointer(void);

#ifdef __cplusplus
}
#endif

#endif
