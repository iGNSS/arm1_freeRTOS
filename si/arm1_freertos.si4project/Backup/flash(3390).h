/**
 *  @file: flash.h
 *
 *  @date: 2020/5/9
 *
 *  @author: aron566
 *
 *  @brief:Flash opt
 *  
 *  @version:v1.0
 */
#ifndef FLASH_OPT_H
#define FLASH_OPT_H
#ifdef __cplusplus ///<use C compiler
extern "C" {
#endif
/** Includes -----------------------------------------------------------------*/
#include <stdint.h> /**< nedd definition of uint8_t */
#include <stddef.h> /**< need definition of NULL    */
//#include <stdbool.h>/**< need definition of BOOL    */
#include <stdio.h>  /**< if need printf             */
#include <stdlib.h>
#include <string.h>
/** Private includes ---------------------------------------------------------*/
#include "gd32f4xx.h"
#include "HAL_Typedef.h"
/** Private defines ----------------------------------------------------------*/
/** @addtogroup FLASHEx_Private_Constants
  * @{
  */    
#define FLASH_TIMEOUT_VALUE       50000U /* 50 s */
/**
  * @}
  */
  
/** @addtogroup HAL库flash 扇区映射
  * @{
  */   
#define FLASH_SECTOR_0     CTL_SECTOR_NUMBER_0 /*!< Sector Number 0   */
#define FLASH_SECTOR_1     CTL_SECTOR_NUMBER_1 /*!< Sector Number 1   */
#define FLASH_SECTOR_2     CTL_SECTOR_NUMBER_2 /*!< Sector Number 2   */
#define FLASH_SECTOR_3     CTL_SECTOR_NUMBER_3 /*!< Sector Number 3   */
#define FLASH_SECTOR_4     CTL_SECTOR_NUMBER_4 /*!< Sector Number 4   */
#define FLASH_SECTOR_5     CTL_SECTOR_NUMBER_5 /*!< Sector Number 5   */
#define FLASH_SECTOR_6     CTL_SECTOR_NUMBER_6 /*!< Sector Number 6   */
#define FLASH_SECTOR_7     CTL_SECTOR_NUMBER_7 /*!< Sector Number 7   */
#define FLASH_SECTOR_8     CTL_SECTOR_NUMBER_8 /*!< Sector Number 8   */
#define FLASH_SECTOR_9     CTL_SECTOR_NUMBER_9 /*!< Sector Number 9   */
#define FLASH_SECTOR_10    CTL_SECTOR_NUMBER_10 /*!< Sector Number 10  */
#define FLASH_SECTOR_11    CTL_SECTOR_NUMBER_11 /*!< Sector Number 11  */
#define FLASH_SECTOR_12    CTL_SECTOR_NUMBER_12 /*!< Sector Number 12  */
#define FLASH_SECTOR_13    CTL_SECTOR_NUMBER_13 /*!< Sector Number 13  */
#define FLASH_SECTOR_14    CTL_SECTOR_NUMBER_14 /*!< Sector Number 14  */
#define FLASH_SECTOR_15    CTL_SECTOR_NUMBER_15 /*!< Sector Number 15  */
#define FLASH_SECTOR_16    CTL_SECTOR_NUMBER_16 /*!< Sector Number 16  */
#define FLASH_SECTOR_17    CTL_SECTOR_NUMBER_17 /*!< Sector Number 17  */
#define FLASH_SECTOR_18    CTL_SECTOR_NUMBER_18 /*!< Sector Number 18  */
#define FLASH_SECTOR_19    CTL_SECTOR_NUMBER_19 /*!< Sector Number 19  */
#define FLASH_SECTOR_20    CTL_SECTOR_NUMBER_20 /*!< Sector Number 20  */
#define FLASH_SECTOR_21    CTL_SECTOR_NUMBER_21 /*!< Sector Number 21  */
#define FLASH_SECTOR_22    CTL_SECTOR_NUMBER_22 /*!< Sector Number 22  */
#define FLASH_SECTOR_23    CTL_SECTOR_NUMBER_23 /*!< Sector Number 23  */  
#define FLASH_SECTOR_24    CTL_SECTOR_NUMBER_24 /*!< Sector Number 24  */  
#define FLASH_SECTOR_25    CTL_SECTOR_NUMBER_25 /*!< Sector Number 25  */  
#define FLASH_SECTOR_26    CTL_SECTOR_NUMBER_26 /*!< Sector Number 26  */  
#define FLASH_SECTOR_27    CTL_SECTOR_NUMBER_27 /*!< Sector Number 27  */  
#define FLASH_SECTOR_28	   CTL_SECTOR_NUMBER_28 /*!< Sector Number 28  */
#define FLASH_SECTOR_29	   CTL_SECTOR_NUMBER_29 /*!< Sector Number 29  */
#define FLASH_SECTOR_30	   CTL_SECTOR_NUMBER_30 /*!< Sector Number 30  */      
/** @defgroup FLASHEx_Type_Erase FLASH Type Erase
  * @{
  */ 
#define FLASH_TYPEERASE_SECTORS         0x00000000U  /*!< Sectors erase only          */
#define FLASH_TYPEERASE_MASSERASE       0x00000001U  /*!< Flash Mass erase activation */
/**
  * @}
  */
#define FLASH_BANK_1     1U /*!< Bank 1   */
#define FLASH_BANK_2     2U /*!< Bank 2   */
#define FLASH_BANK_BOTH  ((uint32_t)FLASH_BANK_1 | FLASH_BANK_2) /*!< Bank1 and Bank2  */  
      
/** @defgroup FLASHEx_Voltage_Range FLASH Voltage Range
  * @{
  */ 
#define FLASH_VOLTAGE_RANGE_1        0x00000000U  /*!< Device operating range: 1.8V to 2.1V                */
#define FLASH_VOLTAGE_RANGE_2        0x00000001U  /*!< Device operating range: 2.1V to 2.7V                */
#define FLASH_VOLTAGE_RANGE_3        0x00000002U  /*!< Device operating range: 2.7V to 3.6V                */
#define FLASH_VOLTAGE_RANGE_4        0x00000003U  /*!< Device operating range: 2.7V to 3.6V + External Vpp */
/**
  * @}
  */
      
/** @name 映射HAL库与GD标准库Flash编程方式
  * @{
  */ 
#define FLASH_TYPEPROGRAM_BYTE        0x00000000U  /*!< Program byte (8-bit) at a specified address           */
#define FLASH_TYPEPROGRAM_HALFWORD    0x00000001U  /*!< Program a half-word (16-bit) at a specified address   */
#define FLASH_TYPEPROGRAM_WORD        0x00000002U  /*!< Program a word (32-bit) at a specified address        */
#define FLASH_TYPEPROGRAM_DOUBLEWORD  0x00000003U  /*!< Program a double word (64-bit) at a specified address */
/**
  * @}
  */
/**
 * @name 映射HAL库与GD标准库Flash标志
 * @{
 */
#define FLASH_FLAG_EOP                 FMC_FLAG_END            /*!< FLASH End of Operation flag               */
#define FLASH_FLAG_OPERR               FMC_FLAG_OPERR          /*!< FLASH operation Error flag                */
#define FLASH_FLAG_WRPERR              FMC_FLAG_WPERR          /*!< FLASH Write protected error flag          */
#define FLASH_FLAG_PGAERR              FMC_FLAG_PGMERR         /*!< FLASH Programming Alignment error flag    */
#define FLASH_FLAG_PGPERR              0                       /*!< FLASH Programming Parallelism error flag  */
#define FLASH_FLAG_PGSERR              FMC_FLAG_PGSERR         /*!< FLASH Programming Sequence error flag     */
#define FLASH_FLAG_RDERR               FMC_FLAG_RDDERR         /*!< Read Protection error flag (PCROP)        */
#define FLASH_FLAG_BSY                 FMC_FLAG_BUSY           /*!< FLASH Busy flag                           */ 
/** @}*/
    
/** @defgroup FLASH_Error_Code FLASH Error Code
  * @brief    FLASH Error Code 
  * @{
  */ 
#define HAL_FLASH_ERROR_NONE         0x00000000U    /*!< No error                      */
#define HAL_FLASH_ERROR_RD           0x00000001U    /*!< Read Protection error         */
#define HAL_FLASH_ERROR_PGS          0x00000002U    /*!< Programming Sequence error    */
#define HAL_FLASH_ERROR_PGP          0x00000004U    /*!< Programming Parallelism error */
#define HAL_FLASH_ERROR_PGA          0x00000008U    /*!< Programming Alignment error   */
#define HAL_FLASH_ERROR_WRP          0x00000010U    /*!< Write protection error        */
#define HAL_FLASH_ERROR_OPERATION    0x00000020U    /*!< Operation Error               */
/**
  * @}
  */
      
/** Exported typedefines -----------------------------------------------------*/
/** 数据结构体*/
/**
  * @brief  FLASH Procedure structure definition
  */
typedef enum 
{
  FLASH_PROC_NONE = 0U, 
  FLASH_PROC_SECTERASE,
  FLASH_PROC_MASSERASE,
  FLASH_PROC_PROGRAM
} FLASH_ProcedureTypeDef;/** 
  * @brief  FLASH handle Structure definition  
  */
typedef struct
{
  volatile FLASH_ProcedureTypeDef ProcedureOnGoing;   /*Internal variable to indicate which procedure is ongoing or not in IT context*/
  
  volatile uint32_t               NbSectorsToErase;   /*Internal variable to save the remaining sectors to erase in IT context*/
  
  volatile uint8_t                VoltageForErase;    /*Internal variable to provide voltage range selected by user in IT context*/
  
  volatile uint32_t               Sector;             /*Internal variable to define the current sector which is erasing*/
  
  volatile uint32_t               Bank;               /*Internal variable to save current bank selected during mass erase*/
  
  volatile uint32_t               Address;            /*Internal variable to save address selected for program*/
  
  HAL_LockTypeDef             Lock;                   /* FLASH locking object                */  volatile uint32_t               ErrorCode;          /* FLASH error code                    */
  }FLASH_ProcessTypeDef;
  /**
  * @}
  *//**
  * @brief  FLASH Erase structure definition
  */
typedef struct
{
uint32_t TypeErase;   /*!< Mass erase or sector Erase.
                             This parameter can be a value of @ref FLASHEx_Type_Erase */  
uint32_t Banks;       /*!< Select banks to erase when Mass erase is enabled.
                             This parameter must be a value of @ref FLASHEx_Banks */  
uint32_t Sector;      /*!< Initial FLASH sector to erase when Mass erase is disabled
                             This parameter must be a value of @ref FLASHEx_Sectors */  
uint32_t NbSectors;   /*!< Number of sectors to be erased.
                             This parameter must be a value between 1 and (max number of sectors - value of Initial sector)*/  
uint32_t VoltageRange;/*!< The device voltage range which defines the erase parallelism
                             This parameter must be a value of @ref FLASHEx_Voltage_Range */
} FLASH_EraseInitTypeDef;
/** Exported constants -------------------------------------------------------*/
/** Exported macros-----------------------------------------------------------*/
/** Exported variables -------------------------------------------------------*/
/** Exported functions prototypes --------------------------------------------*/
FlagStatus __HAL_FLASH_GET_FLAG(uint32_t fmc_flag);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError);
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
uint32_t HAL_FLASH_GetError(void);
HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout);
HAL_StatusTypeDef FLASH_ProgramByte(uint32_t Address, uint8_t Data);
#ifdef __cplusplus ///<end extern c
}
#endif
#endif
/******************************** End of file *********************************/


