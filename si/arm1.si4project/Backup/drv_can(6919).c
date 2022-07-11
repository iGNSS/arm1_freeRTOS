/*
说明: 本驱动完成了can控制器硬件抽象一 CAN Driver 注册	Can driver注册需要填充以下几个数据结构：
1，驱动采用中断模式
2,目前支持4组过滤器组，需要时可以增加或减少，扩展帧32位，标准帧16位
*/

#include <rthw.h>
#include <rtdevice.h>
#include <board.h>
#include <bxcan.h>
#include <gpio.h>


#ifdef RT_USING_CAN


#define BX_CAN_FMRNUMBER 28
#define BX_CAN2_FMRSTART 14

#define BX_CAN_MAX_FILTERS (BX_CAN_FMRNUMBER * 4)
#define BX_CAN_MAX_FILTER_MASKS BX_CAN_MAX_FILTERS
#define BX_CAN_FILTER_MAX_ARRAY_SIZE ((BX_CAN_MAX_FILTERS + 32 - 1) / 32)

struct stm_bxcanfiltermap
{
  rt_uint32_t id32mask_cnt;
  rt_uint32_t id32bit_cnt;
  rt_uint32_t id16mask_cnt;
  rt_uint32_t id16bit_cnt;
};
struct stm_bxcanfilter_masks
{
  rt_uint32_t id32maskm[BX_CAN_FILTER_MAX_ARRAY_SIZE];
  rt_uint32_t id32bitm[BX_CAN_FILTER_MAX_ARRAY_SIZE];
  rt_uint32_t id16maskm[BX_CAN_FILTER_MAX_ARRAY_SIZE];
  rt_uint32_t id16bitm[BX_CAN_FILTER_MAX_ARRAY_SIZE];
  rt_uint32_t id32maskshift[2];
  rt_uint32_t id32bitshift[2];
  rt_uint32_t id16maskshift[2];
  rt_uint32_t id16bitshift[2];
};

struct stm_bxcan
{
  CAN_TypeDef *reg;
  void *mfrbase;
  IRQn_Type sndirq;
  IRQn_Type rcvirq0;
  IRQn_Type rcvirq1;
  IRQn_Type errirq;
  struct stm_bxcanfilter_masks filtermask;
  rt_uint32_t alocmask[BX_CAN_FILTER_MAX_ARRAY_SIZE];
  rt_uint32_t filtercnt;
  const rt_uint32_t fifo1filteroff;
  const struct stm_bxcanfiltermap filtermap[2];
  rt_base_t	 canpwrpin;
};
struct stm_baud_rate_tab
{
  rt_uint32_t baud_rate;
  rt_uint32_t confdata;
};


#define BS1SHIFT 16
#define BS2SHIFT 20
#define RRESCLSHIFT 0
#define SJWSHIFT 24
#define BS1MASK ( (0x0F) << BS1SHIFT )
#define BS2MASK ( (0x07) << BS2SHIFT )
#define RRESCLMASK ( 0x3FF << RRESCLSHIFT )
#define SJWMASK ( 0x3 << SJWSHIFT )

#define MK_BKCAN_BAUD(SJW,BS1,BS2,PRES) \
  ((SJW << SJWSHIFT) | (BS1 << BS1SHIFT) | (BS2 << BS2SHIFT) | (PRES << RRESCLSHIFT))

static const struct stm_baud_rate_tab bxcan_baud_rate_tab[] =
{
  // PCLK1 = HCLK / 4 = 30 M
  {1000UL * 1000, 	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_7tq,  CAN_BS2_2tq, 3)},
  {1000UL * 500,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_7tq,  CAN_BS2_2tq, 6)},
  {1000UL * 250,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_9tq,  CAN_BS2_2tq, 10)},//1
  {1000UL * 125,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_9tq,  CAN_BS2_2tq, 20)},
  {1000UL * 100,	MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_7tq,  CAN_BS2_2tq, 30)},
  {1000UL * 50,		MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_7tq,  CAN_BS2_2tq, 60)},
  {1000UL * 20,		MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_12tq, CAN_BS2_2tq, 100)},
  {1000UL * 10,		MK_BKCAN_BAUD(CAN_SJW_2tq, CAN_BS1_12tq, CAN_BS2_2tq, 200)}
};

#define BAUD_DATA(TYPE,NO) \
  ((bxcan_baud_rate_tab[NO].confdata & TYPE##MASK) >> TYPE##SHIFT)

static rt_uint32_t bxcan_get_baud_index(rt_uint32_t baud)
{
  rt_uint32_t len, index, default_index;
  
  len = sizeof(bxcan_baud_rate_tab)/sizeof(bxcan_baud_rate_tab[0]);
  default_index = len;
  
  for(index = 0; index < len; index++)
  {
    if(bxcan_baud_rate_tab[index].baud_rate == baud)
      return index;
    
    if(bxcan_baud_rate_tab[index].baud_rate == 1000UL * 250)
      default_index = index;
  }
  
  if(default_index != len)
    return default_index;
  
  return 0;	
}


static void bxcan_init(CAN_TypeDef *pcan, rt_uint32_t baud, rt_uint32_t mode)
{
  CAN_InitTypeDef        CAN_InitStructure;
  
  rt_uint32_t baud_index = bxcan_get_baud_index(baud);
  
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = ENABLE;
  CAN_InitStructure.CAN_NART = ENABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = ENABLE;
  switch (mode)
  {
  case RT_CAN_MODE_NORMAL:
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    break;
  case RT_CAN_MODE_LISEN:
    CAN_InitStructure.CAN_Mode = CAN_Mode_Silent;
    break;
  case RT_CAN_MODE_LOOPBACK:
    CAN_InitStructure.CAN_Mode = CAN_Mode_LoopBack;
    break;
  case RT_CAN_MODE_LOOPBACKANLISEN:
    CAN_InitStructure.CAN_Mode = CAN_Mode_Silent_LoopBack;
    break;
  }
  CAN_InitStructure.CAN_SJW = BAUD_DATA(SJW, baud_index);
  CAN_InitStructure.CAN_BS1 = BAUD_DATA(BS1, baud_index);
  CAN_InitStructure.CAN_BS2 = BAUD_DATA(BS2, baud_index);
  CAN_InitStructure.CAN_Prescaler = BAUD_DATA(RRESCL, baud_index);
  
  CAN_Init(pcan, &CAN_InitStructure);
}
#ifdef USING_BXCAN1
static void bxcan1_hw_init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  
  //初始化 GPIO
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化 PA11,PA12
  
  //引脚复用映射配置
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1);//PA11 复用为 CAN1
  GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //PA12 复用为 CAN1
  
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
#endif
#ifdef USING_BXCAN2
static void bxcan2_hw_init(void)
{
  
  GPIO_InitTypeDef  GPIO_InitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;
  
  //初始化 GPIO
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12| GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化 PB12,PB13
  //引脚复用映射配置
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2);//PB12 复用为 CAN2
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2); //PB13 复用为 CAN2
  
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX1_IRQn;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = CAN2_TX_IRQn;
  NVIC_Init(&NVIC_InitStructure);
}
#endif
rt_inline rt_err_t bxcan_enter_init(CAN_TypeDef *pcan)
{
  uint32_t wait_ack = 0x00000000;
  
  pcan->MCR |= CAN_MCR_INRQ ;
  
  while (((pcan->MSR & CAN_MSR_INAK) != CAN_MSR_INAK) && (wait_ack != INAK_TIMEOUT))
  {
    wait_ack++;
  }
  if ((pcan->MSR & CAN_MSR_INAK) != CAN_MSR_INAK)
  {
    return RT_ERROR;
  }
  return RT_EOK;
}
rt_inline rt_err_t bxcan_exit_init(CAN_TypeDef *pcan)
{
  uint32_t wait_ack = 0x00000000;
  
  pcan->MCR &= ~(uint32_t)CAN_MCR_INRQ;
  
  while (((pcan->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) && (wait_ack != INAK_TIMEOUT))
  {
    wait_ack++;
  }
  if ((pcan->MSR & CAN_MSR_INAK) != CAN_MSR_INAK)
  {
    return RT_ERROR;
  }
  return RT_EOK;
}
static rt_err_t bxcan_set_mode(CAN_TypeDef *pcan, rt_uint32_t mode)
{
  if (bxcan_enter_init(pcan) != RT_EOK)
  {
    return RT_ERROR;
  }
  pcan->BTR &= ~(uint32_t)((uint32_t)0x03 << 30);
  switch (mode)
  {
  case RT_CAN_MODE_NORMAL:
    mode = CAN_Mode_Normal;
    break;
  case RT_CAN_MODE_LISEN:
    mode = CAN_Mode_Silent;
    break;
  case RT_CAN_MODE_LOOPBACK:
    mode = CAN_Mode_LoopBack;
    break;
  case RT_CAN_MODE_LOOPBACKANLISEN:
    mode = CAN_Mode_Silent_LoopBack;
    break;
  }
  pcan->BTR |= ~(uint32_t)(mode << 30);
  if (bxcan_exit_init(pcan) != RT_EOK)
  {
    return RT_ERROR;
  }
  return RT_EOK;
}
static rt_err_t bxcan_set_privmode(CAN_TypeDef *pcan, rt_uint32_t mode)
{
  if (bxcan_enter_init(pcan) != RT_EOK)
  {
    return RT_ERROR;
  }
  if (mode == ENABLE)
  {
    pcan->MCR |= CAN_MCR_TXFP;
  }
  else
  {
    pcan->MCR &= ~(uint32_t)CAN_MCR_TXFP;
  }
  if (bxcan_exit_init(pcan) != RT_EOK)
  {
    return RT_ERROR;
  }
  return RT_EOK;
}
static rt_err_t bxcan_set_baud_rate(CAN_TypeDef *pcan, rt_uint32_t baud)
{
  rt_uint32_t mode;
  
  rt_uint32_t baud_index = bxcan_get_baud_index(baud);
  
  if (bxcan_enter_init(pcan) != RT_EOK)
  {
    return RT_ERROR;
  }
  pcan->BTR = 0;
  mode = pcan->BTR & ((rt_uint32_t)0x03 << 30);
  pcan->BTR = (mode                         | \
    ((BAUD_DATA(SJW, baud_index)) << 24) | \
      ((BAUD_DATA(BS1, baud_index)) << 16) | \
        ((BAUD_DATA(BS2, baud_index)) << 20) | \
          (BAUD_DATA(RRESCL, baud_index) - 1));
  if (bxcan_exit_init(pcan) != RT_EOK)
  {
    return RT_ERROR;
  }
  return RT_EOK;
}


static rt_err_t bxmodifyfilter(struct stm_bxcan *pbxcan, struct rt_can_filter_item *pitem, rt_uint32_t actived)
{
  rt_int32_t fcase;
  //rt_err_t res;
  //rt_int32_t hdr, fbase, foff;
  rt_uint32_t ID[2];
  rt_uint32_t shift;
  rt_uint32_t thisid;
  rt_uint32_t thismask;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  CAN_FilterRegister_TypeDef *pfilterreg;
  
  if(!actived)return RT_EOK;
  //过滤器设置
  {
    fcase = (pitem->mode | (pitem->ide << 1));
    pfilterreg = &((CAN_FilterRegister_TypeDef *)pbxcan->mfrbase)[pbxcan->filtercnt];
    CAN_FilterInitStructure.CAN_FilterNumber = (pfilterreg - &CAN1->sFilterRegister[0]);
	
    if (pitem->mode)//屏蔽模式
    {
      CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    }
    else						//列表模式
    {
      CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
    }
    if(pitem->ide)//扩展帧
    {
      CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    }
    else//标准帧
    {
      CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit;
    }
    if(fcase & 0x02)//扩展帧
    {	
      shift = 3;
      thisid = (rt_uint32_t)pitem->id << shift;
      thismask = (rt_uint32_t)pitem->mask << shift;
      
      thisid |= CAN_ID_EXT;
      thismask |= CAN_ID_EXT;
      
      if (pitem->rtr)
      {
        thisid |= CAN_RTR_REMOTE;
        thismask |= CAN_RTR_REMOTE;
      }
	  if (pitem->mode)//屏蔽模式
	  {
		pbxcan->filtercnt ++;
	  }
	  else
	  {
	  	pbxcan->filtercnt += 2;
	  }
    }
    else		//标准帧
    {	
      shift = 5;
      thisid = (rt_uint32_t)pitem->id << shift;
      thismask = (rt_uint32_t)pitem->mask << shift;
      
      thisid |= CAN_ID_STD;
      thismask |= CAN_ID_STD;
      
      if (pitem->rtr)
      {
        thisid |= CAN_RTR_REMOTE;
        thismask |= CAN_RTR_REMOTE;
      }  
	  if (pitem->mode)//屏蔽模式
	  {
		pbxcan->filtercnt += 2;
	  }
	  else
	  {
	  	pbxcan->filtercnt += 4;
	  }
    }
    ID[0] = thisid;
    ID[1] = thismask;
    
    if(pitem->ide)
    {
      CAN_FilterInitStructure.CAN_FilterIdHigh = (ID[0] & 0xFFFF0000) >> 16;
      CAN_FilterInitStructure.CAN_FilterIdLow = ID[0] & 0x0000FFFF;
      CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (ID[1] & 0xFFFF0000) >> 16;
      CAN_FilterInitStructure.CAN_FilterMaskIdLow = ((ID[1]) & 0x0000FFFF);
    }
    else
    {
      CAN_FilterInitStructure.CAN_FilterIdHigh = ((ID[0]) & 0x0000FFFF);
      CAN_FilterInitStructure.CAN_FilterIdLow = ID[1] & 0x0000FFFF;
      CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (ID[0] & 0xFFFF0000) >> 16;
      CAN_FilterInitStructure.CAN_FilterMaskIdLow = (ID[1] & 0xFFFF0000) >> 16;
    }
    //这里can1只有放在fifo1才有效，锤子的
    if (CAN_FilterInitStructure.CAN_FilterNumber >= pbxcan->fifo1filteroff)
    {
      CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
    }
    else
    {
      CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO1;
    }
    if (ID[0] != 0xFFFFFFFF || ID[1] != 0xFFFFFFFF)
    {
      CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    }
    else
    {
      CAN_FilterInitStructure.CAN_FilterActivation = DISABLE;
    }
    
  }
  CAN_FilterInit(&CAN_FilterInitStructure);
  return RT_EOK;
}
#include <string.h>
static rt_err_t setfilter(struct rt_can_device *can, struct rt_can_filter_config *pconfig)
{

  struct stm_bxcan *pbxcan = (struct stm_bxcan *) can->parent.user_data;
  struct rt_can_filter_item *pitem = pconfig->items;
  rt_uint32_t count = pconfig->count;
  rt_err_t res;
  while (count)
  {
    res = bxmodifyfilter(pbxcan, pitem, pconfig->actived);
    if (res != RT_EOK)
    {
      return res;
    }
    pitem++;
    count--;
  }
 
  return RT_EOK;
}
static rt_err_t bxcan_pwr_configure(struct rt_can_device *can)
{
  rt_base_t pwrpin = ((struct stm_bxcan *) can->parent.user_data)->canpwrpin;
  
  rt_pin_mode(pwrpin, PIN_MODE_OUTPUT);
  rt_pin_write(pwrpin, PIN_HIGH);
  
  return RT_EOK;		
}
static rt_err_t configure(struct rt_can_device *can, struct can_configure *cfg)
{
  CAN_TypeDef *pbxcan;		
  
  pbxcan = ((struct stm_bxcan *) can->parent.user_data)->reg;
  assert_param(IS_CAN_ALL_PERIPH(pbxcan));
  
  if (pbxcan == CAN1)
  {
    bxcan1_hw_init();
    bxcan_init(pbxcan, cfg->baud_rate, can->config.mode);
    //bxcan1_filter_init(can);
  }
  else
  {
#ifdef USING_BXCAN2
    bxcan2_hw_init();
    bxcan_init(pbxcan, cfg->baud_rate, can->config.mode);
    //bxcan2_filter_init(can);
#endif
  }
  //CAN_WakeUp(pbxcan);
  bxcan_pwr_configure(can);
  
  return RT_EOK;
}
static rt_err_t control(struct rt_can_device *can, int cmd, void *arg)
{
  struct stm_bxcan *pbxcan;
  rt_uint32_t argval;
  NVIC_InitTypeDef  NVIC_InitStructure;
  
  pbxcan = (struct stm_bxcan *) can->parent.user_data;
  assert_param(pbxcan != RT_NULL);
  
  switch (cmd)
  {
  case RT_DEVICE_CTRL_CLR_INT:
    argval = (rt_uint32_t) arg;
    if (argval == RT_DEVICE_FLAG_INT_RX)
    {
      NVIC_DisableIRQ(pbxcan->rcvirq0);
      NVIC_DisableIRQ(pbxcan->rcvirq1);
      CAN_ITConfig(pbxcan->reg, CAN_IT_FMP0 , DISABLE);
      CAN_ITConfig(pbxcan->reg, CAN_IT_FF0 , DISABLE);
      CAN_ITConfig(pbxcan->reg, CAN_IT_FOV0 , DISABLE);
      CAN_ITConfig(pbxcan->reg, CAN_IT_FMP1 , DISABLE);
      CAN_ITConfig(pbxcan->reg, CAN_IT_FF1 , DISABLE);
      CAN_ITConfig(pbxcan->reg, CAN_IT_FOV1 , DISABLE);
    }
    else if (argval == RT_DEVICE_FLAG_INT_TX)
    {
      NVIC_DisableIRQ(pbxcan->sndirq);
      CAN_ITConfig(pbxcan->reg, CAN_IT_TME, DISABLE);
    }
    else if (argval == RT_DEVICE_CAN_INT_ERR)
    {
      CAN_ITConfig(pbxcan->reg, CAN_IT_BOF , DISABLE);
      CAN_ITConfig(pbxcan->reg, CAN_IT_LEC , DISABLE);
      CAN_ITConfig(pbxcan->reg, CAN_IT_ERR , DISABLE);
      NVIC_DisableIRQ(pbxcan->errirq);
    }
    break;
  case RT_DEVICE_CTRL_SET_INT:
    argval = (rt_uint32_t) arg;
    if (argval == RT_DEVICE_FLAG_INT_RX)
    {
      CAN_ITConfig(pbxcan->reg, CAN_IT_FMP0 , ENABLE);
      CAN_ITConfig(pbxcan->reg, CAN_IT_FF0 , ENABLE);
      CAN_ITConfig(pbxcan->reg, CAN_IT_FOV0 , ENABLE);
      CAN_ITConfig(pbxcan->reg, CAN_IT_FMP1 , ENABLE);
      CAN_ITConfig(pbxcan->reg, CAN_IT_FF1 , ENABLE);
      CAN_ITConfig(pbxcan->reg, CAN_IT_FOV1 , ENABLE);
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_InitStructure.NVIC_IRQChannel = pbxcan->rcvirq0;
      NVIC_Init(&NVIC_InitStructure);
      NVIC_InitStructure.NVIC_IRQChannel = pbxcan->rcvirq1;
      NVIC_Init(&NVIC_InitStructure);
    }
    else if (argval == RT_DEVICE_FLAG_INT_TX)
    {
      CAN_ITConfig(pbxcan->reg, CAN_IT_TME, ENABLE);
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_InitStructure.NVIC_IRQChannel = pbxcan->sndirq;
      NVIC_Init(&NVIC_InitStructure);
    }
    else if (argval == RT_DEVICE_CAN_INT_ERR)
    {
      CAN_ITConfig(pbxcan->reg, CAN_IT_BOF , ENABLE);
      CAN_ITConfig(pbxcan->reg, CAN_IT_LEC , ENABLE);
      CAN_ITConfig(pbxcan->reg, CAN_IT_ERR , ENABLE);
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_InitStructure.NVIC_IRQChannel = pbxcan->errirq;
      NVIC_Init(&NVIC_InitStructure);
    }
    break;
  case RT_CAN_CMD_SET_FILTER:
    return setfilter(can, (struct rt_can_filter_config *) arg);
  case RT_CAN_CMD_SET_MODE:
    argval = (rt_uint32_t) arg;
    if (argval != RT_CAN_MODE_NORMAL ||
        argval != RT_CAN_MODE_LISEN ||
          argval != RT_CAN_MODE_LOOPBACK ||
            argval != RT_CAN_MODE_LOOPBACKANLISEN)
    {
      return RT_ERROR;
    }
    if (argval != can->config.mode)
    {
      can->config.mode = argval;
      return bxcan_set_mode(pbxcan->reg, argval);
    }
    break;
  case RT_CAN_CMD_SET_BAUD:
    argval = (rt_uint32_t) arg;
    if (argval != CAN1MBaud &&
        argval != CAN800kBaud &&
          argval != CAN500kBaud &&
            argval != CAN250kBaud &&
              argval != CAN125kBaud &&
                argval != CAN100kBaud &&
                  argval != CAN50kBaud  &&
                    argval != CAN20kBaud  &&
                      argval != CAN10kBaud)
    {
      return RT_ERROR;
    }
    if (argval != can->config.baud_rate)
    {
      can->config.baud_rate = argval;
      return bxcan_set_baud_rate(pbxcan->reg, argval);
    }
    break;
  case RT_CAN_CMD_SET_PRIV:
    argval = (rt_uint32_t) arg;
    if (argval != RT_CAN_MODE_PRIV ||
        argval != RT_CAN_MODE_NOPRIV)
    {
      return RT_ERROR;
    }
    if (argval != can->config.privmode)
    {
      can->config.privmode = argval;
      return bxcan_set_privmode(pbxcan->reg, argval);
    }
    break;
  case RT_CAN_CMD_GET_STATUS:
    {
      rt_uint32_t errtype;
      errtype = pbxcan->reg->ESR;
      can->status.rcverrcnt = errtype >> 24;
      can->status.snderrcnt = (errtype >> 16 & 0xFF);
      can->status.errcode = errtype & 0x07;
      if (arg != &can->status)
      {
        rt_memcpy(arg, &can->status, sizeof(can->status));
      }
    }
    break;
  }
  
  return RT_EOK;
}
static int sendmsg(struct rt_can_device *can, const void *buf, rt_uint32_t boxno)
{
  CAN_TypeDef *pbxcan;
  struct rt_can_msg *pmsg = (struct rt_can_msg *) buf;
  
  pbxcan = ((struct stm_bxcan *) can->parent.user_data)->reg;
  assert_param(IS_CAN_ALL_PERIPH(pbxcan));
  
  pbxcan->sTxMailBox[boxno].TIR &= TMIDxR_TXRQ;
  if (pmsg->ide == RT_CAN_STDID)
  {
    assert_param(IS_CAN_STDID(pmsg->id));
    pbxcan->sTxMailBox[boxno].TIR |= ((pmsg->id << 21) | \
      (pmsg->rtr << 1));
  }
  else
  {
    assert_param(IS_CAN_EXTID(pmsg->id));
    pbxcan->sTxMailBox[boxno].TIR |= ((pmsg->id << 3) | \
      (pmsg->ide << 2) | \
        (pmsg->rtr << 1));
  }
  
  pmsg->len &= (uint8_t)0x0000000F;
  pbxcan->sTxMailBox[boxno].TDTR &= (uint32_t)0xFFFFFFF0;
  pbxcan->sTxMailBox[boxno].TDTR |= pmsg->len;
  
  pbxcan->sTxMailBox[boxno].TDLR = (((uint32_t)pmsg->data[3] << 24) |
                                    ((uint32_t)pmsg->data[2] << 16) |
                                      ((uint32_t)pmsg->data[1] << 8) |
                                        ((uint32_t)pmsg->data[0]));
  if (pmsg->len > 4)
  {
    pbxcan->sTxMailBox[boxno].TDHR = (((uint32_t)pmsg->data[7] << 24) |
                                      ((uint32_t)pmsg->data[6] << 16) |
                                        ((uint32_t)pmsg->data[5] << 8) |
                                          ((uint32_t)pmsg->data[4]));
  }
  pbxcan->sTxMailBox[boxno].TIR |= TMIDxR_TXRQ;
  
  return RT_EOK;
}
static int recvmsg(struct rt_can_device *can, void *buf, rt_uint32_t boxno)
{
  CAN_TypeDef *pbxcan;
  struct rt_can_msg *pmsg = (struct rt_can_msg *) buf;
  
  pbxcan = ((struct stm_bxcan *) can->parent.user_data)->reg;
  assert_param(IS_CAN_ALL_PERIPH(pbxcan));
  assert_param(IS_CAN_FIFO(boxno));
  pmsg->ide = ((uint8_t)0x04 & pbxcan->sFIFOMailBox[boxno].RIR) >> 2;
  if (pmsg->ide == CAN_Id_Standard)
  {
    pmsg->id = (uint32_t)0x000007FF & (pbxcan->sFIFOMailBox[boxno].RIR >> 21);
  }
  else
  {
    pmsg->id = (uint32_t)0x1FFFFFFF & (pbxcan->sFIFOMailBox[boxno].RIR >> 3);
  }
  
  pmsg->rtr = (uint8_t)((0x02 & pbxcan->sFIFOMailBox[boxno].RIR) >> 1);
  pmsg->len = (uint8_t)0x0F & pbxcan->sFIFOMailBox[boxno].RDTR;
  pmsg->data[0] = (uint8_t)0xFF & pbxcan->sFIFOMailBox[boxno].RDLR;
  pmsg->data[1] = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDLR >> 8);
  pmsg->data[2] = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDLR >> 16);
  pmsg->data[3] = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDLR >> 24);
  if (pmsg->len > 4)
  {
    pmsg->data[4] = (uint8_t)0xFF & pbxcan->sFIFOMailBox[boxno].RDHR;
    pmsg->data[5] = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDHR >> 8);
    pmsg->data[6] = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDHR >> 16);
    pmsg->data[7] = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDHR >> 24);
  }
  //pmsg->hdr = (uint8_t)0xFF & (pbxcan->sFIFOMailBox[boxno].RDTR >> 8);
  //if (boxno) pmsg->hdr += ((struct stm_bxcan *) can->parent.user_data)->fifo1filteroff * 4;
  pmsg->hdr = boxno;
  return RT_EOK;
}

static const struct rt_can_ops canops =
{
  configure,
  control,
  sendmsg,
  recvmsg,
};
#ifdef USING_BXCAN1
static struct stm_bxcan bxcan1data =
{
  CAN1,
  (void *) &CAN1->sFilterRegister[0],
  CAN1_TX_IRQn,
  CAN1_RX0_IRQn,
  CAN1_RX1_IRQn,
  CAN1_SCE_IRQn,
  {
    0,
  },
  {0, 0},
  0,
  14,
  {
    {
      0,
      0,
      2,
      24,
    },
    {
      0,
      0,
      2,
      24,
    },
  },
  PIN_NO_CAN_PWR,
};
struct rt_can_device bxcan1;
void CAN1_RX0_IRQHandler(void)
{
  rt_interrupt_enter();
  if (CAN1->RF0R & 0x03)
  {
    if ((CAN1->RF0R & CAN_RF0R_FOVR0) != 0)
    {
      CAN1->RF0R = CAN_RF0R_FOVR0;
      rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_RXOF_IND | 0 << 8);
    }
    else
    {
      rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_RX_IND | 0 << 8);
    }
    CAN1->RF0R |= CAN_RF0R_RFOM0;
  }
  rt_interrupt_leave();
}
void CAN1_RX1_IRQHandler(void)
{
  rt_interrupt_enter();
  if (CAN1->RF1R & 0x03)
  {
    if ((CAN1->RF1R & CAN_RF1R_FOVR1) != 0)
    {
      CAN1->RF1R = CAN_RF1R_FOVR1;
      rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_RXOF_IND | 1 << 8);
    }
    else
    {
      rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_RX_IND | 1 << 8);
    }
    CAN1->RF1R |= CAN_RF1R_RFOM1;
  }
  rt_interrupt_leave();
}
void CAN1_TX_IRQHandler(void)
{
  rt_uint32_t state;
  rt_interrupt_enter();
  if (CAN1->TSR & (CAN_TSR_RQCP0))
  {
    state =  CAN1->TSR & (CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0);
    CAN1->TSR |= CAN_TSR_RQCP0;
    if (state == (CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0))
    {
      rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_TX_DONE | 0 << 8);
    }
    else
    {
      rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_TX_FAIL | 0 << 8);
    }
  }
  if (CAN1->TSR & (CAN_TSR_RQCP1))
  {
    state =  CAN1->TSR & (CAN_TSR_RQCP1 | CAN_TSR_TXOK1 | CAN_TSR_TME1);
    CAN1->TSR |= CAN_TSR_RQCP1;
    if (state == (CAN_TSR_RQCP1 | CAN_TSR_TXOK1 | CAN_TSR_TME1))
    {
      rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_TX_DONE | 1 << 8);
    }
    else
    {
      rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_TX_FAIL | 1 << 8);
    }
  }
  if (CAN1->TSR & (CAN_TSR_RQCP2))
  {
    state =  CAN1->TSR & (CAN_TSR_RQCP2 | CAN_TSR_TXOK2 | CAN_TSR_TME2);
    CAN1->TSR |= CAN_TSR_RQCP2;
    if (state == (CAN_TSR_RQCP2 | CAN_TSR_TXOK2 | CAN_TSR_TME2))
    {
      rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_TX_DONE | 2 << 8);
    }
    else
    {
      rt_hw_can_isr(&bxcan1, RT_CAN_EVENT_TX_FAIL | 2 << 8);
    }
  }
  rt_interrupt_leave();
}
void CAN1_SCE_IRQHandler(void)
{
  rt_uint32_t errtype;
  errtype = CAN1->ESR;
  rt_interrupt_enter();
  if (errtype & 0x70 && bxcan1.status.lasterrtype == (errtype & 0x70))
  {
    switch ((errtype & 0x70) >> 4)
    {
    case RT_CAN_BUS_BIT_PAD_ERR:
      bxcan1.status.bitpaderrcnt++;
      break;
    case RT_CAN_BUS_FORMAT_ERR:
      bxcan1.status.formaterrcnt++;
      break;
    case RT_CAN_BUS_ACK_ERR:
      bxcan1.status.ackerrcnt++;
      break;
    case RT_CAN_BUS_IMPLICIT_BIT_ERR:
    case RT_CAN_BUS_EXPLICIT_BIT_ERR:
      bxcan1.status.biterrcnt++;
      break;
    case RT_CAN_BUS_CRC_ERR:
      bxcan1.status.crcerrcnt++;
      break;
    }
    bxcan1.status.lasterrtype = errtype & 0x70;
    CAN1->ESR &= ~0x70;
  }
  bxcan1.status.rcverrcnt = errtype >> 24;
  bxcan1.status.snderrcnt = (errtype >> 16 & 0xFF);
  bxcan1.status.errcode = errtype & 0x07;
  CAN1->MSR |= CAN_MSR_ERRI;
  rt_interrupt_leave();
}
#endif /*USING_BXCAN1*/

#ifdef USING_BXCAN2
static struct stm_bxcan bxcan2data =
{
  CAN2,
  (void *) &CAN1->sFilterRegister[BX_CAN2_FMRSTART],
  CAN2_TX_IRQn,
  CAN2_RX0_IRQn,
  CAN2_RX1_IRQn,
  CAN2_SCE_IRQn,
  {
    0,
  },
  {0, 0},
  0,
  14,
  {
    {
      0,
      0,
      2,
      24,
    },
    {
      0,
      0,
      2,
      24,
    },
  },
  PIN_NO_CAN_PWR,
};

struct rt_can_device bxcan2;
void CAN2_RX0_IRQHandler(void)
{
  rt_interrupt_enter();
  if (CAN2->RF0R & 0x03)
  {
    if ((CAN2->RF0R & CAN_RF0R_FOVR0) != 0)
    {
      CAN2->RF0R = CAN_RF0R_FOVR0;
      rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_RXOF_IND | 0 << 8);
    }
    else
    {
      rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_RX_IND | 0 << 8);
    }
    CAN2->RF0R |= CAN_RF0R_RFOM0;
  }
  rt_interrupt_leave();
}
void CAN2_RX1_IRQHandler(void)
{
  rt_interrupt_enter();
  if (CAN2->RF1R & 0x03)
  {
    if ((CAN2->RF1R & CAN_RF1R_FOVR1) != 0)
    {
      CAN2->RF1R = CAN_RF1R_FOVR1;
      rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_RXOF_IND | 1 << 8);
    }
    else
    {
      rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_RX_IND | 1 << 8);
    }
    CAN2->RF1R |= CAN_RF1R_RFOM1;
  }
  rt_interrupt_leave();
}
void CAN2_TX_IRQHandler(void)
{
  rt_uint32_t state;
  rt_interrupt_enter();
  if (CAN2->TSR & (CAN_TSR_RQCP0))
  {
    state =  CAN2->TSR & (CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0);
    CAN2->TSR |= CAN_TSR_RQCP0;
    if (state == (CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0))
    {
      rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_TX_DONE | 0 << 8);
    }
    else
    {
      rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_TX_FAIL | 0 << 8);
    }
  }
  if (CAN2->TSR & (CAN_TSR_RQCP1))
  {
    state =  CAN2->TSR & (CAN_TSR_RQCP1 | CAN_TSR_TXOK1 | CAN_TSR_TME1);
    CAN2->TSR |= CAN_TSR_RQCP1;
    if (state == (CAN_TSR_RQCP1 | CAN_TSR_TXOK1 | CAN_TSR_TME1))
    {
      rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_TX_DONE | 1 << 8);
    }
    else
    {
      rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_TX_FAIL | 1 << 8);
    }
  }
  if (CAN2->TSR & (CAN_TSR_RQCP2))
  {
    state =  CAN2->TSR & (CAN_TSR_RQCP2 | CAN_TSR_TXOK2 | CAN_TSR_TME2);
    CAN2->TSR |= CAN_TSR_RQCP2;
    if (state == (CAN_TSR_RQCP2 | CAN_TSR_TXOK2 | CAN_TSR_TME2))
    {
      rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_TX_DONE | 2 << 8);
    }
    else
    {
      rt_hw_can_isr(&bxcan2, RT_CAN_EVENT_TX_FAIL | 2 << 8);
    }
  }
  rt_interrupt_leave();
}
void CAN2_SCE_IRQHandler(void)
{
  rt_uint32_t errtype;
  errtype = CAN2->ESR;
  rt_interrupt_enter();
  if (errtype & 0x70 && bxcan2.status.lasterrtype == (errtype & 0x70))
  {
    switch ((errtype & 0x70) >> 4)
    {
    case RT_CAN_BUS_BIT_PAD_ERR:
      bxcan2.status.bitpaderrcnt++;
      break;
    case RT_CAN_BUS_FORMAT_ERR:
      bxcan2.status.formaterrcnt++;
      break;
    case RT_CAN_BUS_ACK_ERR:
      bxcan2.status.ackerrcnt++;
      break;
    case RT_CAN_BUS_IMPLICIT_BIT_ERR:
    case RT_CAN_BUS_EXPLICIT_BIT_ERR:
      bxcan2.status.biterrcnt++;
      break;
    case RT_CAN_BUS_CRC_ERR:
      bxcan2.status.crcerrcnt++;
      break;
    }
    bxcan2.status.lasterrtype = errtype & 0x70;
    CAN2->ESR &= ~0x70;
  }
  bxcan2.status.rcverrcnt = errtype >> 24;
  bxcan2.status.snderrcnt = (errtype >> 16 & 0xFF);
  bxcan2.status.errcode = errtype & 0x07;
  CAN2->MSR |= CAN_MSR_ERRI;
  rt_interrupt_leave();
}
#endif /*USING_BXCAN2*/
#if 0
void CAN1_ID_Filter_Set(u8 FilterNum, u32 filterId, u32 filterMaskId)
{
  
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  CAN_FilterInitStructure.CAN_FilterNumber          = FilterNum;      //指定了待初始化的过滤器，它的范围是14到27
  //CAN_FilterInitStructure.CAN_FilterMode            = CAN_FilterMode_IdList; //CAN_FilterMode_IdMask;//过滤器被初始化为标识符屏蔽位模式
  CAN_FilterInitStructure.CAN_FilterMode            = CAN_FilterMode_IdMask;//过滤器被初始化为标识符屏蔽位模式
  CAN_FilterInitStructure.CAN_FilterScale           = CAN_FilterScale_32bit;//给出了过滤器位宽
  
  CAN_FilterInitStructure.CAN_FilterIdHigh          = (((u32)filterId<<3)&0xFFFF0000)>>16;
  CAN_FilterInitStructure.CAN_FilterIdLow           = (((u32)filterId<<3)|0x0004)&0xFFFF;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh      = (((u32)filterMaskId<<3)&0xFFFF0000)>>16;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow       = (((u32)filterMaskId<<3)|0x0007)&0xFFFF;
  
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FilterFIFO0; //通过此滤波器信息包将被放在FIFO0里
  CAN_FilterInitStructure.CAN_FilterActivation      = ENABLE;    //使能/失能 过滤器
  CAN_FilterInit(&CAN_FilterInitStructure);
  
  
}
static void CAN1_Init_GPIO( void )
{
  
  GPIO_InitTypeDef   GPIO_InitStructure;
  
  //--------- 时钟初始化 --------------------------
  
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  
  //--------- 复用IO端口初始化 ----------------------------
  
  /* Connect PD0 to CAN1*/
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
  
  /* Connect PD1 to CAN1*/
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);
  
  /* Configure USART Tx as alternate function  */
  
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  /* Configure USART Rx as alternate function  */
  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  
}
static void CAN1_BaudRate_Set(u32 BandRateBuf)
{
  
  CAN_InitTypeDef        CAN_InitStructure;
  
  CAN_StructInit(&CAN_InitStructure);
  
  //------- CAN_MCR ------------
  CAN_InitStructure.CAN_TTCM = DISABLE; //使能/失能 时间触发通讯模式.0:时间触发通信模式关闭;
  CAN_InitStructure.CAN_ABOM = DISABLE;  //使能/失能 自动离线管理.    1:一旦监控到128次11个连续隐形位,自动退出离线状态;
  CAN_InitStructure.CAN_AWUM = ENABLE;  //使能/失能 自动唤醒模式.    1:硬件检测到CAN报文时自动离开休眠模式;
  CAN_InitStructure.CAN_NART = ENABLE; //使能/失能 非自动重传输模式.0:CAN硬件发送失败后会一直重发直到发送成功;
  CAN_InitStructure.CAN_RFLM = DISABLE; //使能/失能 能接收FIFO锁定模式.0:接收FIFO满了,下一条传入的报文将覆盖前一条;
  CAN_InitStructure.CAN_TXFP = ENABLE;  //使能/失能 发送FIFO优先级.    1:由发送请求的顺序(时间先后顺序)来决定优先级.
  
  //------- CAN_BTR ------------
  CAN_InitStructure.CAN_Mode =CAN_Mode_Normal ;//CAN硬件工作在正常模式,CAN_Mode_LoopBack,CAN_Mode_Normal
  CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;     //重新同步跳跃宽度1个时间单位
  
  CAN_InitStructure.CAN_BS1 = CAN_BS1_13tq;   //CAN_BS1_8tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;    //CAN_BS2_7tq;
  CAN_InitStructure.CAN_Prescaler = 6;//(波特率:250K, PCLK1:24MHz, CAN_BS1:CAN_BS1_13tq, CAN_BS2:CAN_BS2_2tq, CAN_Prescaler:6)
  
  
  CAN_Init(CAN1, &CAN_InitStructure);
  
  
}

static void CAN1_Init_Reg(void)
{
  
  /* CAN Periph clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  
  CAN_DeInit(CAN1);
  
  CAN1_BaudRate_Set(250000L);//设置CAN总线的 波特率(250KHz).
  
  CAN1_ID_Filter_Set( 1, 0x00000000, 0x00000000);//不屏蔽所有的
  
  //CanBus_SetIdentifie( 2, 0x18FFD117, 0x0000FFFF);
  
}
static void CAN1_Init_Interrupt( void)
{
  
  NVIC_InitTypeDef   NVIC_InitStructure;
  
  //----------中断初始化 ---------------------------------------
  /* Enable CAN RX0 interrupt IRQ channel */
  
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;		
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable CAN TX interrupt IRQ channel */
  
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;		
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);//初始时关闭CAN的接收中断
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);//初始时关闭CAN的接收中断
}

void CAN1_Init(void)
{
  
  CAN1_Init_GPIO();
  
  CAN1_Init_Reg();
  
  CAN1_Init_Interrupt();
  
}
void CAN_PowerCtrl_Init_GPIO( void )
{
  
  GPIO_InitTypeDef   GPIO_InitStructure;
  
  //-----------------CAN电源开关-----------------Weite
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
} 
#define CAN_PWR_ON()    GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_SET)
#endif
int stm32_bxcan_init(void)
{
#if 1  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG , ENABLE);
#ifdef USING_BXCAN1
  //使能相关时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能 PORTA 时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 , ENABLE);
  
  CAN_DeInit(CAN1);
  bxcan1.config.baud_rate = CAN250kBaud;
  bxcan1.config.msgboxsz = 16;
  bxcan1.config.sndboxnumber = 3;
  bxcan1.config.mode = RT_CAN_MODE_NORMAL;
  bxcan1.config.privmode = 0;
  bxcan1.config.ticks = 50;
#ifdef RT_CAN_USING_HDR
  bxcan1.config.maxhdr = BX_CAN2_FMRSTART * 4;
#endif
  rt_hw_can_register(&bxcan1, "bxcan1", &canops, &bxcan1data);
#endif
  
#ifdef USING_BXCAN2
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
#ifndef USING_BXCAN1
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 , ENABLE);
#endif
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
  CAN_DeInit(CAN2);
  bxcan2.config.baud_rate = CAN250kBaud;
  bxcan2.config.msgboxsz = 16;
  bxcan2.config.sndboxnumber = 3;
  bxcan2.config.mode = RT_CAN_MODE_NORMAL;
  bxcan2.config.privmode = 0;
  bxcan2.config.ticks = 50;
#ifdef RT_CAN_USING_HDR
  bxcan2.config.maxhdr = (BX_CAN_FMRNUMBER - BX_CAN2_FMRSTART) * 4;
#endif
  rt_hw_can_register(&bxcan2, "bxcan2", &canops, &bxcan2data);
#endif
  
#else
  NVIC_InitTypeDef  NVIC_InitStructure;
  //使能相关时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOB, ENABLE);//使能 PORTA 时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 , ENABLE);
  
  CAN_PowerCtrl_Init_GPIO( );
  CAN_PWR_ON();	//开CAN电源 
  CAN1_Init();
  
#endif
  return RT_EOK;
}
INIT_BOARD_EXPORT(stm32_bxcan_init);

#endif /*RT_USING_CAN*/
