
#include <drv_can.h>
#include <drv_gpio.h>



#define BX_CAN_FMRNUMBER 28
#define BX_CAN2_FMRSTART 14

#define BX_CAN_MAX_FILTERS (BX_CAN_FMRNUMBER * 4)
#define BX_CAN_MAX_FILTER_MASKS BX_CAN_MAX_FILTERS
#define BX_CAN_FILTER_MAX_ARRAY_SIZE ((BX_CAN_MAX_FILTERS + 32 - 1) / 32)

struct gd_bxcanfiltermap
{
    uint32_t id32mask_cnt;
    uint32_t id32bit_cnt;
    uint32_t id16mask_cnt;
    uint32_t id16bit_cnt;
};
struct gd_bxcanfilter_masks
{
    uint32_t id32maskm[BX_CAN_FILTER_MAX_ARRAY_SIZE];
    uint32_t id32bitm[BX_CAN_FILTER_MAX_ARRAY_SIZE];
    uint32_t id16maskm[BX_CAN_FILTER_MAX_ARRAY_SIZE];
    uint32_t id16bitm[BX_CAN_FILTER_MAX_ARRAY_SIZE];
    uint32_t id32maskshift[2];
    uint32_t id32bitshift[2];
    uint32_t id16maskshift[2];
    uint32_t id16bitshift[2];
};

struct gd_bxcan
{
    uint32_t can_periph;
    void *mfrbase;
    IRQn_Type sndirq;
    IRQn_Type rcvirq0;
    IRQn_Type rcvirq1;
    IRQn_Type errirq;
    struct gd_bxcanfilter_masks filtermask;
    uint32_t alocmask[BX_CAN_FILTER_MAX_ARRAY_SIZE];
    uint32_t filtercnt;
    const uint32_t fifo1filteroff;
    const struct gd_bxcanfiltermap filtermap[2];
    uint32_t	 canpwrpin;
};
struct gd_baud_rate_tab
{
    uint32_t baud_rate;
    uint32_t confdata;
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

static const struct gd_baud_rate_tab bxcan_baud_rate_tab[] =
{
    // PCLK1 = HCLK / 4 = 50 M
    {1000UL * 1000, 	MK_BKCAN_BAUD(CAN_BT_SJW_1TQ, CAN_BT_BS1_7TQ,  CAN_BT_BS2_2TQ, 5)},
    {1000UL * 500,	MK_BKCAN_BAUD(CAN_BT_SJW_1TQ, CAN_BT_BS1_7TQ,  CAN_BT_BS2_2TQ, 10)},
    {1000UL * 250,	MK_BKCAN_BAUD(CAN_BT_SJW_1TQ, CAN_BT_BS1_7TQ,  CAN_BT_BS2_2TQ, 20)},
    {1000UL * 125,	MK_BKCAN_BAUD(CAN_BT_SJW_1TQ, CAN_BT_BS1_7TQ,  CAN_BT_BS2_2TQ, 40)},
    {1000UL * 100,	MK_BKCAN_BAUD(CAN_BT_SJW_1TQ, CAN_BT_BS1_7TQ,  CAN_BT_BS2_2TQ, 50)},
    {1000UL * 50,		MK_BKCAN_BAUD(CAN_BT_SJW_1TQ, CAN_BT_BS1_7TQ,  CAN_BT_BS2_2TQ, 100)},
    {1000UL * 20,		MK_BKCAN_BAUD(CAN_BT_SJW_1TQ, CAN_BT_BS1_7TQ,  CAN_BT_BS2_2TQ, 250)},
    {1000UL * 10,		MK_BKCAN_BAUD(CAN_BT_SJW_1TQ, CAN_BT_BS1_7TQ,  CAN_BT_BS2_2TQ, 500)}
};

#define BAUD_DATA(TYPE,NO) \
    ((bxcan_baud_rate_tab[NO].confdata & TYPE##MASK) >> TYPE##SHIFT)

static uint32_t bxcan_get_baud_index(uint32_t baud)
{
    uint32_t len, index, default_index;

    len = sizeof(bxcan_baud_rate_tab) / sizeof(bxcan_baud_rate_tab[0]);
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


static void bxcan_init(uint32_t can_periph, uint32_t baud, uint32_t mode)
{
    can_parameter_struct can_parameter;

    uint32_t baud_index = bxcan_get_baud_index(baud);

    /* initialize CAN register */
    can_deinit(can_periph);

    /* initialize CAN parameters */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = DISABLE;
    can_parameter.auto_wake_up = ENABLE;
    can_parameter.auto_retrans = ENABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;

    switch (mode)
    {
        case GD_CAN_MODE_NORMAL:
            can_parameter.working_mode = CAN_NORMAL_MODE;
            break;

        case GD_CAN_MODE_LISEN:
            can_parameter.working_mode = CAN_SILENT_MODE;
            break;

        case GD_CAN_MODE_LOOPBACK:
            can_parameter.working_mode = CAN_LOOPBACK_MODE;
            break;

        case GD_CAN_MODE_LOOPBACKANLISEN:
            can_parameter.working_mode = CAN_SILENT_LOOPBACK_MODE;
            break;
    }

    can_parameter.resync_jump_width = BAUD_DATA(SJW, baud_index);
    can_parameter.time_segment_1 = BAUD_DATA(BS1, baud_index);
    can_parameter.time_segment_2 = BAUD_DATA(BS2, baud_index);
    can_parameter.prescaler = BAUD_DATA(RRESCL, baud_index);

    /* initialize CAN */
    can_init(can_periph, &can_parameter);
}

#ifdef USING_BXCAN0
static void bxcan0_hw_init(void)
{
    /* enable CAN clock */
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_GPIOA);

    /* configure CAN1 GPIO */
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_11);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_11);
    gpio_af_set(GPIOA, GPIO_AF_9, GPIO_PIN_11);

    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_af_set(GPIOA, GPIO_AF_9, GPIO_PIN_12);

    bxcan_init(CAN0, CAN500kBaud, CAN_MODE_NORMAL);
    /* configure CAN0 NVIC */
    //nvic_irq_enable(CAN0_RX0_IRQn,0,0);

    /* enable CAN receive FIFO0 not empty interrupt */
    //can_interrupt_enable(CAN0, CAN_INT_RFNE0);
}
#endif
#ifdef USING_BXCAN1
static void bxcan1_hw_init(void)
{
    /* enable CAN clock */
    rcu_periph_clock_enable(RCU_CAN1);
    rcu_periph_clock_enable(RCU_GPIOB);

    /* configure CAN2 GPIO */
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_12);
    gpio_af_set(GPIOB, GPIO_AF_9, GPIO_PIN_12);

    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_13);
    gpio_af_set(GPIOB, GPIO_AF_9, GPIO_PIN_13);

    bxcan_init(CAN1, CAN500kBaud, CAN_MODE_NORMAL);

    /* configure CAN1 NVIC */
    //nvic_irq_enable(CAN1_RX0_IRQn,1,0);

    /* enable CAN receive FIFO0 not empty interrupt */
    //can_interrupt_enable(CAN1, CAN_INT_RFNE0);
}

#endif

void bxcan_set_baud_rate(uint32_t can_periph, uint32_t baud)
{
    can_parameter_struct can_parameter;

    uint32_t baud_index = bxcan_get_baud_index(baud);

    /* initialize CAN parameters */
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = DISABLE;
    can_parameter.auto_wake_up = ENABLE;
    can_parameter.auto_retrans = ENABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;

    can_parameter.resync_jump_width = BAUD_DATA(SJW, baud_index);
    can_parameter.time_segment_1 = BAUD_DATA(BS1, baud_index);
    can_parameter.time_segment_2 = BAUD_DATA(BS2, baud_index);
    can_parameter.prescaler = BAUD_DATA(RRESCL, baud_index);

    /* initialize CAN */
    can_init(can_periph, &can_parameter);
}

void bxcan_pwr_configure(uint32_t pwrpin, uint32_t pinVal)
{
    gd32_pin_mode(pwrpin, PIN_MODE_OUTPUT);
    gd32_pin_write(pwrpin, pinVal);

}

//static int sendmsg(struct rt_can_device *can, const void *buf, rt_uint32_t boxno)

//static int recvmsg(struct rt_can_device *can, void *buf, rt_uint32_t boxno)

void Can_FormatFilter(TFilter* filter, uint8_t num, ECanFrameType frameType, ECanIdType idType, uint32_t id)
{
    filter->filterNo = num;
    filter->idType = idType;
    filter->frameType = frameType;
    filter->filterID = id;
}

void gd32_bxcan_filter_config(TCanConfig config)
{
    uint32_t can_periph;
    can_filter_parameter_struct can_filter;
    int i = 0;

    /* CAN filter init */
    if (config.ch == CAN_1)
    {
        can_periph = CAN0;
    }
    else
    {
        can_periph = CAN1;
    }

    if (config.filterUsedNum > 0)
    {
        can_filter.filter_mode = CAN_FILTERMODE_MASK;
        can_filter.filter_bits = CAN_FILTERBITS_32BIT;
        can_filter.filter_fifo_number = CAN_FIFO0;
        can_filter.filter_enable = ENABLE;

        for (i = 0; i < config.filterUsedNum; i++)
        {
            if (can_periph == CAN0)
            {
                can_filter.filter_number = config.filter[i].filterNo;
            }
            else
            {
                can_filter.filter_number = 14 + config.filter[i].filterNo;
            }

            can_filter.filter_list_high = 0x0000;
            can_filter.filter_list_low = 0x0000;
            can_filter.filter_mask_high = 0x0000;
            can_filter.filter_mask_low = 0x0000;

            if (config.filter[i].idType == IDType_Standard)
            {
                can_filter.filter_list_high = (config.filter[i].filterID << 5) & 0xFFFF;
                can_filter.filter_list_low = 0x0000;
                can_filter.filter_list_low &= ~(1 << 2);
                can_filter.filter_mask_high = 0xFFE0;
                can_filter.filter_mask_low = 0x0006;
            }
            else
            {
                can_filter.filter_list_high = (config.filter[i].filterID >> 13) & 0xFFFF;
                can_filter.filter_list_low = (config.filter[i].filterID << 3) & 0xFFFF;
                can_filter.filter_list_low |= (1 << 2);
                can_filter.filter_mask_high = 0xFFFF;
                can_filter.filter_mask_low = 0xFFFE;
            }

            if (config.filter[i].frameType == FrameType_Remote)
            {
                can_filter.filter_list_low |= (1 << 1);
            }
            else
            {
                can_filter.filter_list_low &= ~(1 << 1);
            }

        }
    }
    else
    {
        if (can_periph == CAN0)
        {
            can_filter.filter_number = 0;
        }
        else
        {
            can_filter.filter_number = 14;
        }

        can_filter.filter_mode = CAN_FILTERMODE_MASK;
        can_filter.filter_bits = CAN_FILTERBITS_32BIT;
        can_filter.filter_list_high = 0x0000;
        can_filter.filter_list_low = 0x0000;
        can_filter.filter_mask_high = 0x0000;
        can_filter.filter_mask_low = 0x0000;
        can_filter.filter_fifo_number = CAN_FIFO0;
        can_filter.filter_enable = ENABLE;

    }

    can_filter_init(&can_filter);

    if (config.bUseInterrupt)
    {
        if (can_periph == CAN0)
        {
            nvic_irq_enable(CAN0_RX0_IRQn, 0, 0);

            can_interrupt_enable(CAN0, CAN_INT_RFNE0);
        }
        else
        {
            nvic_irq_enable(CAN1_RX0_IRQn, 1, 0);

            can_interrupt_enable(CAN1, CAN_INT_RFNE0);
        }
    }
}

void gd32_bxcan_init(void)
{
    TCanConfig config;

    bxcan0_hw_init();
    bxcan1_hw_init();
    // Initialize CAN1
    config.ch = CAN_1;
    config.filterUsedNum = 2;
    config.bUseInterrupt = false;
    // Filter
    Can_FormatFilter(&config.filter[0], 0, FrameType_Data, IDType_Extend, 0x12);
    Can_FormatFilter(&config.filter[1], 1, FrameType_Data, IDType_Standard, 0x18);
    gd32_bxcan_filter_config(config);
    //Initialize CAN2
    config.ch = CAN_2;
    config.filterUsedNum = 3;
    config.bUseInterrupt = false;
    // Filter
    Can_FormatFilter(&config.filter[0], 0, FrameType_Data, IDType_Extend, 0x12);
    Can_FormatFilter(&config.filter[1], 1, FrameType_Data, IDType_Standard, 0x18);
    Can_FormatFilter(&config.filter[2], 2, FrameType_Data, IDType_Extend, 0x1ABB);
    gd32_bxcan_filter_config(config);
}


