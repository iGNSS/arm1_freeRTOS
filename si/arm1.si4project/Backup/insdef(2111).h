/*
 * Copyright (c) 2022-2028, INS Development Team
 *
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-05-25     ZWB      	   the first version
 *
 */

#ifndef __INS_DEF_H__
#define __INS_DEF_H__

/* include insconfig header to import configuration */
//#include <insconfig.h>

#ifdef __cplusplus
extern "C" {
#endif



typedef signed   char                   ins_int8_t;      /**<  8bit integer type */
typedef signed   short                  ins_int16_t;     /**< 16bit integer type */
typedef signed   int                    ins_int32_t;     /**< 32bit integer type */
typedef unsigned char                   ins_uint8_t;     /**<  8bit unsigned integer type */
typedef unsigned short                  ins_uint16_t;    /**< 16bit unsigned integer type */
typedef unsigned int                    ins_uint32_t;    /**< 32bit unsigned integer type */

typedef int                             ins_bool_t;      /**< boolean type */
typedef long                            ins_base_t;      /**< Nbit CPU related date type */
typedef unsigned long                   ins_ubase_t;     /**< Nbit unsigned CPU related data type */

typedef ins_base_t                      ins_err_t;       /**< Type for error number */
typedef ins_uint32_t                    ins_time_t;      /**< Type for time stamp */
typedef ins_uint32_t                    ins_tick_t;      /**< Type for tick count */
typedef ins_base_t                      ins_flag_t;      /**< Type for flags */
typedef ins_ubase_t                     ins_size_t;      /**< Type for size number */
typedef ins_ubase_t                     ins_dev_t;       /**< Type for device */
typedef ins_base_t                      ins_off_t;       /**< Type for offset */

/* boolean type definitions */
#define INS_TRUE                         1               /**< boolean true  */
#define INS_FALSE                        0               /**< boolean fails */

/**
 * @addtogroup Error
 */

/**@{*/

/* error code definitions */
#define INS_EOK                          0               /**< There is no error */
#define INS_ERROR                        1               /**< A generic error happens */
#define INS_ETIMEOUT                     2               /**< Timed out */
#define INS_EFULL                        3               /**< The resource is full */
#define INS_EEMPTY                       4               /**< The resource is empty */
#define INS_ENOMEM                       5               /**< No memory */
#define INS_ENOSYS                       6               /**< No system */
#define INS_EBUSY                        7               /**< Busy */
#define INS_EIO                          8               /**< IO error */
#define INS_EINTR                        9               /**< Interrupted system call */
#define INS_EINVAL                       10              /**< Invalid argument */

#ifdef __cplusplus
}
#endif


#endif
