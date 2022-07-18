/***********************************
CAN operation module

************************************/
#include "inavcan.h"
#include <stdio.h>
#include <math.h>
#include "matrix.h"
#include "string.h"

I_NAV_CAN_RESULT g_NAV_CAN_RESULT;

#ifdef __cplusplus
extern "C" {
#endif
void InitialCanResultParm()
{
	memset(&g_NAV_CAN_RESULT,0,sizeof(I_NAV_CAN_RESULT));
}

I_NAV_CAN_RESULT *GetCanResultPointer()
{
	return &g_NAV_CAN_RESULT;
}



#ifdef __cplusplus
}
#endif

