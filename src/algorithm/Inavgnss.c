/***********************************
GNSS operation module

************************************/
#include "inavgnss.h"
#include <stdio.h>
#include <math.h>
#include "matrix.h"
#include "string.h"

I_NAV_GNSS_RESULT g_NAV_GNSS_RESULT;
#ifdef __cplusplus
extern "C" {
#endif
void InitialGnssResultParm(void)
{
	memset(&g_NAV_GNSS_RESULT,0,sizeof(I_NAV_GNSS_RESULT));
}

I_NAV_GNSS_RESULT *GetGnssResultPointer(void)
{
	return &g_NAV_GNSS_RESULT;
}



#ifdef __cplusplus
}
#endif

