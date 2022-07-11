#ifndef __M_QUEUE_H__
#define __M_QUEUE_H__

#include "gd32f4xx.h"
#include "insdef.h"

#ifdef cplusplus
 extern "C" {
#endif 


struct QueueRecord
{
    int Capacity;
    int Front;
    int Rear;
    int Size;
    uint8_t *Array;
};
typedef struct QueueRecord *    pQueue;

pQueue QueueCreate(int MaxElements);
void QueueIn(uint8_t X);
void usart_queue_init(void);


#ifdef cplusplus
}
#endif 

#endif









