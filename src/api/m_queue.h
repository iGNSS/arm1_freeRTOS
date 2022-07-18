#ifndef __M_QUEUE_H__
#define __M_QUEUE_H__

#include "gd32f4xx.h"
#include "insdef.h"

#ifdef cplusplus
 extern "C" {
#endif 


typedef struct
{
	uint16_t		head;					//队列头
	uint16_t		tail;					//队列尾
	uint16_t		queueLen;				//队列长度
	uint16_t		queueCapacity;			//队列容量
	uint8_t			*buff;					//数据缓存指针
}CirQueue;

typedef CirQueue*	CirQueue_t;

typedef enum {Q_SUCCESS = 0u,Q_FAILED = !Q_SUCCESS}QueueStatus;


/*********************************************************************************************************
*										   IsCirQueueEmpty
*
* Description : 队列判空，如果长度=0则为空，这时没有元素可以出队
*
* Argument(s) : 循环队列指针
*
* Return(s)   : 队列状态
*
* Caller(s)   : none.
*
* Note(s)	  : none.
*********************************************************************************************************/
QueueStatus IsCirQueueEmpty(CirQueue_t xCirQueue);


/*********************************************************************************************************
*										   IsCirQueueFull
*
* Description : 队列判满，如果长度=容量，则此时不能有元素入队
*
* Argument(s) : 循环队列指针
*
* Return(s)   : 队列状态
*
* Caller(s)   : none.
*
* Note(s)	  : none.
*********************************************************************************************************/
QueueStatus IsCirQueueFull(CirQueue_t xCirQueue);

/*********************************************************************************************************
*										   GetCirQueueLength
*
* Description : 获取队列长度
*
* Argument(s) : 循环队列指针
*
* Return(s)   : 队列长度
*
* Caller(s)   : none.
*
* Note(s)	  : none.
*********************************************************************************************************/
int GetCirQueueLength(CirQueue_t xCirQueue);

/*********************************************************************************************************
*										   EnCirQueue
*
* Description : 多元素入队
*
* Argument(s) : pBuffer:待入队的数据指针 
*								BufferLen:入队数据长度
*								pQ:循环队列指针
*
* Return(s)   : 入队数据长度
*
* Caller(s)   : none.
*
* Note(s)	  : none.
*********************************************************************************************************/
QueueStatus EnCirQueue(uint8_t *pBuffer, uint32_t BufferLen,CirQueue_t xCirQueue);


/*********************************************************************************************************
*										   DeCirQueue
*
* Description : 元素出队
*
* Argument(s) : pBuffer:出队的数据指针 
*			   xCirQueue:循环队列指针
*
* Return(s)   : 出队列的数据长度(数据体长度( 2字节)+ 数据体的长度)
*
* Caller(s)   : none.
*
* Note(s)	  : none.
*********************************************************************************************************/
uint32_t DeCirQueue(uint8_t *pBuffer, uint32_t BufferLen, CirQueue_t xCirQueue);

uint32_t ByteDeCirQueue(uint8_t *pBuffer, CirQueue_t xCirQueue);

/*********************************************************************************************************
*										   CirQueueGenericCreate
*
* Description : 创建一个循环队列
*
* Argument(s) : uxQueueLength:队列的容量
*
* Return(s)   : 申请的队列指针
*
* Caller(s)   : none.
*
* Note(s)	  : none.
*********************************************************************************************************/
CirQueue_t CirQueueGenericCreate(const uint32_t uxQueueLength);

CirQueue_t CirQueueStaticCreate(const uint32_t uxQueueLength, uint8_t* pBuf);

#ifdef cplusplus
}
#endif 

#endif









