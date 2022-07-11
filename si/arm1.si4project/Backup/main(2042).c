/*!
    \file    main.c
    \brief   led spark with systick

    \version 2016-08-15, V1.0.0, firmware for GD32F4xx
    \version 2018-12-12, V2.0.0, firmware for GD32F4xx
    \version 2020-09-30, V2.1.0, firmware for GD32F4xx
    \version 2022-03-09, V3.0.0, firmware for GD32F4xx
*/

/*
    Copyright (c) 2022, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include "gd32f4xx.h"
#include "systick.h"
#include <stdio.h>
#include "exmc_sram.h"
#include "config.h"
#include "drv_gpio.h"
#include "drv_usart.h"
#include "pin_numbers_def.h"
#include "frame_analysis.h"
#include "uartadapter.h"

#define	ARM1_TO_ARM2_IO	GD32F450_PA12_PIN_NUM
#define	SYN_ARM_IO		GD32F450_PE2_PIN_NUM

#define	ARM1_TO_ARM2_SYN gd32_pin_write(ARM1_TO_ARM2_IO, PIN_LOW)

/* SRAM */
uint8_t txbuffer[BUFFER_SIZE];
uint8_t rxbuffer[BUFFER_SIZE];

static void prvSetupHardware( void )
{
    SystemInit();

    SystemCoreClockUpdate();

    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    systick_config();

    #ifdef	configUse_SEGGER_RTT
    SEGGER_RTT_Init();
    SEGGER_RTT_printf(0, "\r\nCK_SYS is %d", rcu_clock_freq_get(CK_SYS));
    SEGGER_RTT_printf(0, "\r\nCK_AHB is %d", rcu_clock_freq_get(CK_AHB));
    SEGGER_RTT_printf(0, "\r\nCK_APB1 is %d", rcu_clock_freq_get(CK_APB1));
    SEGGER_RTT_printf(0, "\r\nCK_APB2 is %d", rcu_clock_freq_get(CK_APB2));
    #endif
}

static void peripherals_init(void)
{
    exmc_sram_init();

    // Uart
    //Uart_TxInit(UART_TXPORT_RS232_1, UART_BAUDRATE_115200BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_DEFAULT, UART_ENABLE);
    //Uart_RxInit(UART_RXPORT_RS232_1, UART_BAUDRATE_115200BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_DEFAULT, UART_ENABLE);
    
    Uart_TxInit(UART_TXPORT_COMPLEX_8, UART_BAUDRATE_115200BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS422, UART_ENABLE);
    Uart_RxInit(UART_RXPORT_COMPLEX_8, UART_BAUDRATE_115200BPS, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS422, UART_ENABLE);

    frame_init();

	gd32_hw_usart_init();
	
    gd32_pin_mode(ARM1_TO_ARM2_IO, PIN_MODE_OUTPUT);
}

void kalman(void)
{
	
}

/*!
    \brief    main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
	uint16_t len;
	
    irq_priority priority =
    {
        .nvic_irq_pre_priority = 1,
        .nvic_irq_sub_priority = 0
    };

    prvSetupHardware();

    peripherals_init();

    pin_irq_install(  SYN_ARM_IO, PIN_MODE_INPUT_PULLUP,
                      PIN_IRQ_MODE_FALLING,
                      NULL,
                      NULL,
                      &priority);

    for(;;)
    {
      //exmc_sram_readbuffer_8(rxbuffer, WRITE_READ_ADDR, BUFFER_SIZE, BANK0_REGION1);
      len = Uart_RecvMsg(UART_RXPORT_COMPLEX_8, BUFFER_SIZE, rxbuffer);
      kalman();//卡尔曼滤波
      frame_pack_and_send(rxbuffer,len);//send to fpga
      ARM1_TO_ARM2_SYN;	//syn call
    }
}

