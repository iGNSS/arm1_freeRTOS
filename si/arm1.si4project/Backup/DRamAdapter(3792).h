#ifndef _DRAM_ADAPTER_H_
#define _DRAM_ADAPTER_H_

#include "stdbool.h"
#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

extern bool DRam_Init(void);
extern bool DRam_Read(u32 addr, u16* data, u32 len);
extern bool DRam_Write(u32 addr, u16* data, u32 len);

#ifdef __cplusplus
}
#endif

#endif
