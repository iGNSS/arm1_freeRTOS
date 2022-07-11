

void LZM_PublicSetCycTimer(LZM_TIMER *stTimer,LZM_TIMER32 value,void (*function)(void))
{
    stTimer->enabled  = TIMER_ENABLE;
    stTimer->counter  = value+Timer_Val();
    stTimer->interval = value;
    stTimer->operate = function;
}
/************************************************************
** ��������: LZM_PublicSetOnceTimer
** ��������: ���ζ�ʱ
** ��ڲ���: stTimer:��ʱ��ָ�����
             value:��ʱֵ
             function:��Ҫִ�еĺ���
** ���ڲ���:
************************************************************/ 
void LZM_PublicSetOnceTimer(LZM_TIMER *stTimer,LZM_TIMER32 value,void (*function)(void))
{
    stTimer->enabled  = TIMER_ENABLE;
    stTimer->counter  = value+Timer_Val();
    stTimer->interval = ONCE;
    stTimer->operate = function;
}
/************************************************************
** ��������: LZM_PublicKillTimer
** ��������: �ر�ָ����ʱ��
** ��ڲ���: stTimer:��ʱ��ָ��
** ���ڲ���:
************************************************************/ 
void LZM_PublicKillTimer(LZM_TIMER *stTimer)
{
    stTimer->enabled  = TIMER_DISENABLE;
}
/************************************************************
** ��������: LZM_PublicGetTimerEnable
** ��������: ָ����ʱ���Ƿ���Ч
** ��ڲ���: stTimer:��ʱ��ָ��
** ���ڲ���:
************************************************************/ 
LZM_RET LZM_PublicGetTimerEnable(LZM_TIMER *stTimer)
{
    return (TIMER_ENABLE == stTimer->enabled)? 1 : 0;
}
/************************************************************
** ��������: LZM_PublicKillTimerAll
** ��������: �ر�����ָ����ʱ��
** ��ڲ���: stTimes:��ʱ������
             maxtimes:��ʱ�������
** ���ڲ���:
************************************************************/ 
void LZM_PublicKillTimerAll(LZM_TIMER *stTimers,unsigned char maxtimes)
{
    unsigned char i;
    for(i = 0; i < maxtimes; i++)
    {
        stTimers[i].enabled = TIMER_DISENABLE;
    }
}
/************************************************************
** ��������: LZM_PublicTimerHandler
** ��������: Ӧ�ó���ʱ���������
** ��ڲ���: stTimes:��ʱ������
             maxtimes:��ʱ�������
** ���ڲ���:
************************************************************/ 
void LZM_PublicTimerHandler(LZM_TIMER *stTimers,unsigned char maxtimes)
{
    unsigned char i;
    unsigned long timerVal;
    timerVal=Timer_Val();
    for(i = 0; i < maxtimes; i++)
    {
        if(TIMER_ENABLE == stTimers[i].enabled)
        {
            if(stTimers[i].counter <= timerVal)//
            {
                if(ONCE == stTimers[i].interval)//����
                {
                    stTimers[i].enabled = TIMER_DISENABLE;
                }
                else//�ظ���ʱ��
                {
                    stTimers[i].counter += stTimers[i].interval;
                    if(stTimers[i].counter< timerVal)
                        stTimers[i].counter = timerVal;
                }
                //�����ڸú��������ظ�����ö�ʱ��
                if(NULL != stTimers[i].operate)
                stTimers[i].operate();
            }
        }
    }
}

