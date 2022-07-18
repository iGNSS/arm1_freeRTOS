#include "string.h"
#include "stdlib.h"
#include "m_queue.h"



/*********************************************************************************************************
*                                          IsCirQueueEmpty
*
* Description : 队列判空，如果长度=0则为空，这时没有元素可以出队
*
* Argument(s) : 循环队列指针
*
* Return(s)   : 队列状态
*
* Caller(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************/
QueueStatus IsCirQueueEmpty(CirQueue_t xCirQueue)
{
  if (xCirQueue->head == xCirQueue->tail)
  {
      return Q_SUCCESS;
  }
  return Q_FAILED;
}


/*********************************************************************************************************
*                                          IsCirQueueFull
*
* Description : 队列判满，如果长度=容量，则此时不能有元素入队
*
* Argument(s) : 循环队列指针
*
* Return(s)   : 队列状态
*
* Caller(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************/
QueueStatus IsCirQueueFull(CirQueue_t xCirQueue)
{
	
  if (xCirQueue->queueLen == xCirQueue->queueCapacity)
  {
      return Q_SUCCESS;
  }
	return Q_FAILED;
}

/*********************************************************************************************************
*                                          GetCirQueueLength
*
* Description : 获取队列长度
*
* Argument(s) : 循环队列指针
*
* Return(s)   : 队列长度
*
* Caller(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************/
int	GetCirQueueLength(CirQueue_t xCirQueue)
{
	return	xCirQueue->queueLen;
}

/*********************************************************************************************************
*                                          EnCirQueue
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
* Note(s)     : none.
*********************************************************************************************************/
QueueStatus EnCirQueue(uint8_t *pBuffer, uint32_t BufferLen,CirQueue_t xCirQueue)
{
	uint8_t* pBuf = pBuffer;
	uint16_t cut,mod;
	if((0 == BufferLen)||(NULL == pBuf))
	{
		return Q_FAILED;
	}

	if((xCirQueue->tail + BufferLen) > xCirQueue->queueCapacity)
	{
		cut = xCirQueue->queueCapacity - xCirQueue->tail;
		mod = (xCirQueue->tail + BufferLen) % xCirQueue->queueCapacity;
		memcpy(xCirQueue->buff + xCirQueue->tail,pBuf,cut);
		memcpy(xCirQueue->buff,pBuf+cut,mod);
	}
	else
	{
		memcpy(xCirQueue->buff + xCirQueue->tail,pBuf,BufferLen);
	}
	xCirQueue->tail += BufferLen;//队列尾部递增
	//因为队列是环形，所以tail需要通过取模来实现转回到0位置
	xCirQueue->tail = xCirQueue->tail % xCirQueue->queueCapacity;
	xCirQueue->queueLen += BufferLen;
	
	return Q_SUCCESS;
}

/*********************************************************************************************************
*                                          DeCirQueue
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
* Note(s)     : none.
*********************************************************************************************************/
uint32_t DeCirQueue(uint8_t *pBuffer, uint32_t BufferLen, CirQueue_t xCirQueue)
{
	uint8_t* pBuf = pBuffer;
	uint16_t len;
	if (Q_SUCCESS == IsCirQueueEmpty(xCirQueue))
	{
	  	return 0;
	}
	if(BufferLen > xCirQueue->queueLen)
	{
		len = xCirQueue->queueLen;
	}
	else
	{
		len = BufferLen;		
	}
	memcpy(pBuf,xCirQueue->buff+xCirQueue->head,len);
	xCirQueue->head += BufferLen;
	//因为队列是环形，所以head需要通过取模来实现转回到0位置
	xCirQueue->head = xCirQueue->head % xCirQueue->queueCapacity;
	xCirQueue->queueLen -= len; 
	
	return len;

}

uint32_t ByteDeCirQueue(uint8_t *pBuffer, CirQueue_t xCirQueue)
{
	uint8_t* pBuf = pBuffer;
	
	if (Q_SUCCESS == IsCirQueueEmpty(xCirQueue))
	{
	  	return 0;
	}
	
	*pBuf = *(xCirQueue->buff+xCirQueue->head);
	xCirQueue->head ++;
	//因为队列是环形，所以head需要通过取模来实现转回到0位置
	xCirQueue->head = xCirQueue->head % xCirQueue->queueCapacity;
	xCirQueue->queueLen --; 
	
	return 1;

}

/*********************************************************************************************************
*                                          CirQueueGenericCreate
*
* Description : 创建一个循环队列
*
* Argument(s) : uxQueueLength:队列的容量
*
* Return(s)   : 申请的队列指针
*
* Caller(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************/
CirQueue_t CirQueueGenericCreate(const uint32_t uxQueueLength)
{
	CirQueue_t pxCirQueue;

	pxCirQueue = (CirQueue_t)malloc(sizeof(CirQueue));
	if( pxCirQueue == NULL )
	{
		return NULL;
	}
	pxCirQueue->head = 0;
	pxCirQueue->tail = 0;
	pxCirQueue->queueLen = 0;
	pxCirQueue->queueCapacity = uxQueueLength;

	pxCirQueue->buff = (uint8_t*)malloc(uxQueueLength);
	if( pxCirQueue->buff == NULL )
	{
		return NULL;
	}
	
	return pxCirQueue;
}

CirQueue_t CirQueueStaticCreate(const uint32_t uxQueueLength, uint8_t* pBuf)
{
	CirQueue_t pxCirQueue;

	pxCirQueue = (CirQueue_t)malloc(sizeof(CirQueue));
	if( pxCirQueue == NULL )
	{
		return NULL;
	}
	pxCirQueue->head = 0;
	pxCirQueue->tail = 0;
	pxCirQueue->queueLen = 0;
	pxCirQueue->queueCapacity = uxQueueLength;

	pxCirQueue->buff = pBuf;

	return pxCirQueue;
}

