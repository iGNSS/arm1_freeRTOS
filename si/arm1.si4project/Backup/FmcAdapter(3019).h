
#ifndef __FMC_ADAPTER_H
#define __FMC_ADAPTER_H

#include "stm32f4xx.h"

typedef enum
{
    SPACE_COM,
    SPACE_RAM,
}EFpgaSpace;

extern void FMC_Init(void);

extern u16  FMC_ReadWord(EFpgaSpace space, u32 addr);
extern void FMC_WriteWord(EFpgaSpace space, u32 addr, u16 data);
extern void FMC_ReadBuffer(EFpgaSpace space, u32 addr, u16 *pBuffer, u32 size);
extern void FMC_WriteBuffer(EFpgaSpace space, u32 addr, u16 *pBuffer, u32 size);


#endif

