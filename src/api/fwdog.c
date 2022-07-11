
#include "gd32f4xx.h"


void fwdog_init(void)
{
	rcu_osci_on(RCU_IRC32K);
    
    /* wait till IRC32K is ready */
    while(SUCCESS != rcu_osci_stab_wait(RCU_IRC32K)){
    }
    
    /* confiure FWDGT counter clock: 32KHz(IRC32K) / 64 = 0.5 KHz */
    fwdgt_config(5*500,FWDGT_PSC_DIV64);
    
    /* After 2 seconds to generate a reset */
    fwdgt_enable();
}

void fwdog_feed(void)
{
    /* reload FWDGT counter */
    fwdgt_counter_reload();
}

