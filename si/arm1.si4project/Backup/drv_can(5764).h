#ifndef __DRV_CAN_H_
#define __DRV_CAN_H_
/* CAN Master Control Register bits */

#include "gd32f4xx.h"
#include "gd32f4xx_exti.h"
#include "insdef.h"

#define MCR_DBF      ((uint32_t)0x00010000) /* software master reset */

/* CAN Mailbox Transmit Request */
#define TMIDxR_TXRQ  ((uint32_t)0x00000001) /* Transmit mailbox request */

/* CAN Filter Master Register bits */
#define FMR_FINIT    ((uint32_t)0x00000001) /* Filter init mode */

/* Time out for INAK bit */
#define INAK_TIMEOUT        ((uint32_t)0x0000FFFF)
/* Time out for SLAK bit */
#define SLAK_TIMEOUT        ((uint32_t)0x0000FFFF)


#define USING_BXCAN0
#define USING_BXCAN1
#define RT_CAN_USING_HDR

#define RT_USING_PM

/* Flags in TSR register */
#define CAN_FLAGS_TSR              ((uint32_t)0x08000000)
/* Flags in RF1R register */
#define CAN_FLAGS_RF1R             ((uint32_t)0x04000000)
/* Flags in RF0R register */
#define CAN_FLAGS_RF0R             ((uint32_t)0x02000000)
/* Flags in MSR register */
#define CAN_FLAGS_MSR              ((uint32_t)0x01000000)
/* Flags in ESR register */
#define CAN_FLAGS_ESR              ((uint32_t)0x00F00000)

/* Mailboxes definition */
#define CAN_TXMAILBOX_0                   ((uint8_t)0x00)
#define CAN_TXMAILBOX_1                   ((uint8_t)0x01)
#define CAN_TXMAILBOX_2                   ((uint8_t)0x02)

#define CAN_MODE_MASK              ((uint32_t) 0x00000003)

enum CANBAUD
{
    CAN1MBaud   = 1000UL * 1000,/* 1 MBit/sec   */
    CAN500kBaud = 1000UL * 500, /* 500 kBit/sec */
    CAN250kBaud = 1000UL * 250, /* 250 kBit/sec */
    CAN125kBaud = 1000UL * 125, /* 125 kBit/sec */
    CAN100kBaud = 1000UL * 100, /* 100 kBit/sec */
    CAN50kBaud  = 1000UL * 50,  /* 50 kBit/sec  */
    CAN20kBaud  = 1000UL * 20,  /* 20 kBit/sec  */
    CAN10kBaud  = 1000UL * 10   /* 10 kBit/sec  */
};

#define CAN_MODE_NORMAL              0
#define CAN_MODE_LISEN               1
#define CAN_MODE_LOOPBACK            2
#define CAN_MODE_LOOPBACKANLISEN     3

#define CAN_MODE_PRIV                0x01
#define CAN_MODE_NOPRIV              0x00


typedef enum ECANCHANNEL
{
    CAN_1 = 0, // channel 1
	CAN_2,     // channel 2
}ECanChannel;

typedef enum ECANFRAMETYPE
{
    FrameType_Data = 0,  // data frame
    FrameType_Remote = 1, // remote frame
    
}ECanFrameType;

typedef enum ECANIDTYPE
{
    IDType_Standard = 0, // standard frame
    IDType_Extend = 1,   // extend frame
    
}ECanIdType;

typedef struct TFILTER
{
    ECanFrameType frameType;
    ECanIdType idType;
    uint8_t filterNo;
    uint32_t filterID;
}TFilter;

typedef struct TCANCONFIG
{
    ECanChannel ch;         
    uint8_t filterUsedNum;     
    TFilter filter[14];
    bool bUseInterrupt;
    
}TCanConfig;

typedef struct TCANDATA
{
    ECanFrameType frameType; 
    ECanIdType idType;
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
    
}TCanData;

int gd32_bxcan_init(void);

void CAN1_RX0_IRQHandler(void);
void CAN1_TX_IRQHandler(void);

#endif /*BXCAN_H_*/
