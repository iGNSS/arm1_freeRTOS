#ifndef GNSSTASK_H_INCLUDED
#define GNSSTASK_H_INCLUDED

#undef COMMON_EXT
#ifdef  __GOL_GNSSTASK_C__
#define COMMON_EXT
#else
#define COMMON_EXT extern
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "insdef.h"
#include "gd32f4xx.h"

#define GNSS_BUFFER_SIZE            1024

COMMON_EXT void gnss_comm2_rx(void);

COMMON_EXT void gnss_comm3_rx(void);

COMMON_EXT void gnss_comm2_task(void* arg);

COMMON_EXT void gnss_comm3_task(void* arg);

COMMON_EXT void gnss_cir_queue_init(void);

COMMON_EXT void gnssTask_init(void);

COMMON_EXT void gnss_task_create(void);

#ifdef __cplusplus
}
#endif

#endif



