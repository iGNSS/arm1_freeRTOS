#ifndef IMUTASK_H_INCLUDED
#define IMUTASK_H_INCLUDED

#undef COMMON_EXT
#ifdef  __GOL_IMUTASK_C__
    #define COMMON_EXT
#else
    #define COMMON_EXT extern
#endif	

#ifdef __cplusplus
extern "C" {
#endif

#include "insdef.h"
#include "gd32f4xx.h"

#define IMU_BUFFER_SIZE             100

COMMON_EXT void imu_comm5_rx(void);
COMMON_EXT void imu_task_create(void);
COMMON_EXT void imuTask_init(void);

#ifdef __cplusplus
}
#endif

#endif 



