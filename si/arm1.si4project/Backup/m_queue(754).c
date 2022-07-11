


#ifdef configUse_usart_queue_rev

static uint8_t m_uart_queue_buffer[USART_SERIAL_RB_BUFSZ];
static uint8_t m_uart_rx_buffer[USART_SERIAL_RB_BUFSZ];
static Queue	m_queue = NULL;

int QueueIsEmpty( Queue Q )
{
    return Q->Size == 0;
}

int QueueIsFull( Queue Q )
{
    return Q->Size == Q->Capacity;
}

void QueueMakeEmpty(Queue Q)
{
    Q->Size = 0;
    Q->Front = 1;
    Q->Rear = 0;
}

Queue QueueCreate(int MaxElements)
{
    Queue Q;

    Q = (Queue)malloc( sizeof(struct QueueRecord));
    if( Q == NULL )
        return NULL;

    Q->Array = m_uart_queue_buffer;

    if( Q->Array == NULL )
        return NULL;

    Q->Capacity = MaxElements;

    QueueMakeEmpty( Q );

    return Q;
}

int QueueSucc(int Value, Queue Q)
{
    if( ++Value == Q->Capacity )
        Value = 0;
    return Value;
}

void QueueIn(uint8_t X)
{
    if(QueueIsFull(m_queue))
    {
        return ;
    }
    else
    {
        m_queue->Size++;
        m_queue->Rear = QueueSucc( m_queue->Rear, m_queue );
        m_queue->Array[ m_queue->Rear ] = X;
    }
}

void QueueInBuffer(uint8_t *pBuffer, int BufferLen, Queue Q)
{
    int i = 0;
    uint8_t X;

    if((Q->Capacity - Q->Size) > (BufferLen))
    {
        for(i=0; i<BufferLen; i++)
        {
            X = *(pBuffer+i);
            Q->Size++;
            Q->Rear = QueueSucc( Q->Rear, Q );
            Q->Array[ Q->Rear ] = X;
        }
    }
}

uint8_t QueueFrontAndOut2(Queue Q, uint8_t *pFlag)
{
    uint8_t X = 0;

    if(QueueIsEmpty(Q))
    {
        *pFlag = 0;
        return X;
    }

    Q->Size--;
    X = Q->Array[ Q->Front ];
    Q->Front = QueueSucc( Q->Front, Q );

    *pFlag = 1;
    return X;
}

int  QueueOutBuffer(uint8_t *pBuffer, Queue Q)
{
    int count = 0;
    uint8_t *p = NULL;

    if(NULL != p)
    {
        return 0;
    }

    p = pBuffer;

    while( 1 )
    {
        if(QueueIsEmpty(Q))
        {
            break;
        }
        else
        {
            Q->Size--;
            *p++ = Q->Array[ Q->Front ];
            Q->Front = QueueSucc( Q->Front, Q );
            count++;
        }

    }
    return count;
}

void usart_queue_init(void)
{
    m_queue = QueueCreate(USART_SERIAL_RB_BUFSZ);
}
#endif

