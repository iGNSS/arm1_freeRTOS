
#include "FmcAdapter.h"
#include "stm32f4xx_fmc.h"
#include "FPGADefine.h"


void FMC_GPIOConfig()
{
	GPIO_InitTypeDef GPIO_InitStructure;


	/* Enable GPIOs clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF |
		RCC_AHB1Periph_GPIOG | RCC_AHB1Periph_GPIOH | RCC_AHB1Periph_GPIOI, ENABLE);

	/* Common GPIO configuration */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	/* GPIOD configuration */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 |
		GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14 |
		GPIO_Pin_15;

	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* GPIOE configuration */
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_FMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
		GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;

	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* GPIOF configuration */
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource0, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource1, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource2, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource3, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource4, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource5, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource12, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource13, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource14, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource15, GPIO_AF_FMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
		GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 |
		GPIO_Pin_14 | GPIO_Pin_15;

	GPIO_Init(GPIOF, &GPIO_InitStructure);

	/* GPIOG configuration */
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource0, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource1, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource2, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource3, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource4, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource5, GPIO_AF_FMC);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_FMC);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
		GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_9;

	GPIO_Init(GPIOG, &GPIO_InitStructure);
}

void FMC_Init()
{
    
	FMC_NORSRAMInitTypeDef  FMC_NORSRAMInitStructure;
	FMC_NORSRAMTimingInitTypeDef  NORSRAMTimingStructure;

	/* GPIO configuration for FMC SRAM bank */
	FMC_GPIOConfig();

	/* Enable FMC clock */
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FMC, ENABLE);

	/* FMC Configuration ---------------------------------------------------------*/
	  /* SRAM Timing configuration */
	NORSRAMTimingStructure.FMC_AddressSetupTime = 8;
	NORSRAMTimingStructure.FMC_AddressHoldTime = 15;
	NORSRAMTimingStructure.FMC_DataSetupTime = 30;
	NORSRAMTimingStructure.FMC_BusTurnAroundDuration = 8;
	NORSRAMTimingStructure.FMC_CLKDivision = 15;
	NORSRAMTimingStructure.FMC_DataLatency = 0;
	NORSRAMTimingStructure.FMC_AccessMode = FMC_AccessMode_A;

	/* FMC SRAM control configuration */
	FMC_NORSRAMInitStructure.FMC_Bank = FMC_Bank1_NORSRAM1;
	FMC_NORSRAMInitStructure.FMC_DataAddressMux = FMC_DataAddressMux_Disable;
	FMC_NORSRAMInitStructure.FMC_MemoryType = FMC_MemoryType_SRAM;
	FMC_NORSRAMInitStructure.FMC_MemoryDataWidth = FMC_NORSRAM_MemoryDataWidth_16b;
	FMC_NORSRAMInitStructure.FMC_BurstAccessMode = FMC_BurstAccessMode_Disable;
	FMC_NORSRAMInitStructure.FMC_AsynchronousWait = FMC_AsynchronousWait_Disable;
	FMC_NORSRAMInitStructure.FMC_WaitSignalPolarity = FMC_WaitSignalPolarity_Low;
	FMC_NORSRAMInitStructure.FMC_WrapMode = FMC_WrapMode_Disable;
	FMC_NORSRAMInitStructure.FMC_WaitSignalActive = FMC_WaitSignalActive_BeforeWaitState;
	FMC_NORSRAMInitStructure.FMC_WriteOperation = FMC_WriteOperation_Enable;
	FMC_NORSRAMInitStructure.FMC_WaitSignal = FMC_WaitSignal_Disable;
	FMC_NORSRAMInitStructure.FMC_ExtendedMode = FMC_ExtendedMode_Disable;
	FMC_NORSRAMInitStructure.FMC_WriteBurst = FMC_WriteBurst_Disable;
	FMC_NORSRAMInitStructure.FMC_ContinousClock = FMC_CClock_SyncOnly;
	FMC_NORSRAMInitStructure.FMC_ReadWriteTimingStruct = &NORSRAMTimingStructure;
	FMC_NORSRAMInitStructure.FMC_WriteTimingStruct = &NORSRAMTimingStructure;

	/* SRAM configuration */
	FMC_NORSRAMInit(&FMC_NORSRAMInitStructure);

	/* Enable FMC Bank1_SRAM2 Bank */
	FMC_NORSRAMCmd(FMC_Bank1_NORSRAM1, ENABLE);
    
    /* Enable FMC Bank1_SRAM2 Bank */
	FMC_NORSRAMCmd(FMC_Bank1_NORSRAM2, ENABLE);

	/* Read FPGA version */
	while (FMC_ReadWord(SPACE_COM, 0x220 >> 1) != 0x5555);
}

void FMC_ReadBuffer(EFpgaSpace space, u32 addr, u16* pBuffer, u32 size)
{
	__IO u32 pointer = (u32)addr;
    __IO u32 base = (space == SPACE_COM)? FPGA_BASE_ADDR : FPGA_DRAM_ADDR;

	for (; size != 0; size--)
	{
		*pBuffer++ = *(__IO u16*) (base + ADDR_SHIFT(pointer));
		pointer++;
	}
}


void FMC_WriteBuffer(EFpgaSpace space, u32 addr, u16* pBuffer, u32 size)
{
	__IO u32 pointer = (u32)addr;
    __IO u32 base = (space == SPACE_COM)? FPGA_BASE_ADDR : FPGA_DRAM_ADDR;

	for (; size != 0; size--)
	{
		*(u16*)(base + ADDR_SHIFT(pointer)) = *pBuffer++;
		pointer++;
	}
}

u16 FMC_ReadWord(EFpgaSpace space, u32 addr)
{
    __IO u32 base = (space == SPACE_COM)? FPGA_BASE_ADDR : FPGA_DRAM_ADDR;
	return (*(__IO u16*)(base + ADDR_SHIFT(addr)));
}

void FMC_WriteWord(EFpgaSpace space, u32 addr, u16 data)
{
    __IO u32 base = (space == SPACE_COM)? FPGA_BASE_ADDR : FPGA_DRAM_ADDR;
	*(u16*)(base + ADDR_SHIFT(addr)) = data;
}



