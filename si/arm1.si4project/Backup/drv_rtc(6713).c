#include "drv_rtc.h"

#define RTC_CLOCK_SOURCE_IRC32K

uint8_t secondCount = 0;
//static uint8_t currSecond = 0;

rtc_parameter_struct rtc_initpara;
rtc_alarm_struct  rtc_alarm;

__IO uint32_t prescaler_a = 0, prescaler_s = 0;


void rtc_pre_config(void)
{
    #if defined (RTC_CLOCK_SOURCE_IRC32K)
          rcu_osci_on(RCU_IRC32K);
          rcu_osci_stab_wait(RCU_IRC32K);
          rcu_rtc_clock_config(RCU_RTCSRC_IRC32K);

          prescaler_s = 0x13F;
          prescaler_a = 0x63;
    #elif defined (RTC_CLOCK_SOURCE_LXTAL)
          rcu_osci_on(RCU_LXTAL);
          rcu_osci_stab_wait(RCU_LXTAL);
          rcu_rtc_clock_config(RCU_RTCSRC_LXTAL);

          prescaler_s = 0xFF;
          prescaler_a = 0x7F;
    #else
    #error RTC clock source should be defined.
    #endif /* RTC_CLOCK_SOURCE_IRC32K */

    rcu_periph_clock_enable(RCU_RTC);
    rtc_register_sync_wait();
}


void rtc_setup(void)
{
    /* setup RTC time value */
    rtc_initpara.factor_asyn = prescaler_a;
    rtc_initpara.factor_syn = prescaler_s;
    rtc_initpara.year = 0x21;
    rtc_initpara.day_of_week = RTC_THURSDAY;
    rtc_initpara.month = RTC_JAN;
    rtc_initpara.date = 0x12;
    rtc_initpara.display_format = RTC_24HOUR;
    rtc_initpara.am_pm = RTC_AM;

    rtc_initpara.hour   = 0x01;
    rtc_initpara.minute = 0x00;
    rtc_initpara.second = 0x00;

    /* RTC current time configuration */
    if(SUCCESS == rtc_init(&rtc_initpara))
    {
      
    }

}

void rtc_configuration(void)
{
    rcu_periph_clock_enable(RCU_PMU);
    pmu_backup_write_enable();

    rtc_pre_config();

    rtc_setup();


}

void rtc_alarm_setup(void)
{
    rtc_configuration();
    /* RTC alarm configuration */
    rtc_alarm_disable();
    rtc_alarm.alarm_mask = RTC_ALARM_DATE_MASK | RTC_ALARM_HOUR_MASK;
    rtc_alarm.weekday_or_date = RTC_ALARM_DATE_SELECTED;
    rtc_alarm.alarm_day = 0x24;
    rtc_alarm.am_pm = RTC_AM;

    rtc_alarm.alarm_hour    = 0x01;
    rtc_alarm.alarm_minute  = 0x00;
    rtc_alarm.alarm_second  = 0x02;

    rtc_alarm_config(&rtc_alarm);

    rtc_interrupt_enable(RTC_INT_ALARM0);
    rtc_alarm_enable();

    rtc_flag_clear(RTC_STAT_ALRM0F);
    exti_flag_clear(EXTI_17);
    rcu_all_reset_flag_clear();

    exti_init(EXTI_17, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    nvic_irq_enable(RTC_Alarm_IRQn, 0, 0);
}

void rtc_update(void)
{
    rtc_current_time_get(&rtc_initpara);
}


