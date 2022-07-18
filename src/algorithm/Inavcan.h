/*****************************************************************************************
All rights reserved I-NAV 2022 2023
***********************************************************************************/

/***********************************************************************************
Modification history

|-----------+---------------+-------------------|
|Author          |    Date               |    Done                      |
|-----------+---------------+-------------------|
|DengWei       |  2022-6-10          | First Creation             |
|-----------+---------------+-------------------|  
***********************************************************************************/
#ifndef _INAV_CAN_H
#define _INAV_CAN_H
#include "constant.h"

// CAN原始数据
typedef struct I_NAV_CAN_RESULT_T {
	float 		second;//时间

		
}I_NAV_CAN_RESULT;


#ifdef __cplusplus
extern "C" {
#endif
void InitialCanResultParm(void);
I_NAV_CAN_RESULT *GetCanResultPointer(void);

#ifdef __cplusplus
}
#endif

#endif
