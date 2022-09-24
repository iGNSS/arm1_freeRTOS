/*!
    \file    exmc_sram.c
    \brief   EXMC SRAM(ISSI IS61LV51216) driver

    \version 2016-08-15, V1.0.0, firmware for GD32F4xx
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
#include "exmc_sram.h"
#include "FmcAdapter.h"
#include "drv_gpio.h"
#include "pin_numbers_def.h"


#define Bank0_BASE_ADDR            ((uint32_t)0x60000000)
#define Bank0_ADDR_OFFSET          ((uint32_t)0x04000000)

#define	EXMC_NE1_PIN	GD32F450_PD7_PIN_NUM
#define	EXMC_NE2_PIN	GD32F450_PG9_PIN_NUM


/*!
    \brief      SRAM peripheral initialize
    \param[in]  nor_region: specify the nor flash region
    \param[out] none
    \retval     none
*/
void exmc_sram_init(void)
{
	exmc_norsram_parameter_struct		sram_init_struct;
	exmc_norsram_timing_parameter_struct read_timing;
	exmc_norsram_timing_parameter_struct write_timing;

	/* enable EXMC clock*/
	rcu_periph_clock_enable(RCU_EXMC);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOC);
	rcu_periph_clock_enable(RCU_GPIOD);
	rcu_periph_clock_enable(RCU_GPIOE);
	rcu_periph_clock_enable(RCU_GPIOF);
	rcu_periph_clock_enable(RCU_GPIOG);
	rcu_periph_clock_enable(RCU_GPIOH);

	/* common GPIO configuration */
	gpio_af_set(GPIOD, GPIO_AF_12, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 \
						|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15);
	gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 \
						|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15);
	gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 \
						|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15);
	
	
	gpio_af_set(GPIOE, GPIO_AF_12, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 \
						|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
	gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 \
						|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
	gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 \
						|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
	
	gpio_af_set(GPIOF, GPIO_AF_12, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 \
						|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
	gpio_mode_set(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 \
						|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
	gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 \
						|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
	
	gpio_af_set(GPIOG, GPIO_AF_12, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9);
	gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9);
	gpio_output_options_set(GPIOD,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_9);
	
	
	//读时序配置
	read_timing.asyn_access_mode		= EXMC_ACCESS_MODE_A;			//模式A，异步访问SRAM
	read_timing.asyn_address_setuptime	= 0;							//异步访问地址建立时间
	read_timing.asyn_address_holdtime	= 2;							//异步访问地址保持时间
	read_timing.asyn_data_setuptime		= 32;							//异步访问数据建立时间 160ns
	read_timing.bus_latency				= 0;							//同步/异步访问总线延时时间
	read_timing.syn_clk_division		= EXMC_SYN_CLOCK_RATIO_2_CLK;  	//同步访问时钟分频系数（从HCLK中分频）
	read_timing.syn_data_latency		= EXMC_DATALAT_2_CLK;			//同步访问中获得第1个数据所需要的等待延迟

	//写时序配置
	write_timing.asyn_access_mode		= EXMC_ACCESS_MODE_A;			//模式A，异步访问SRAM
	write_timing.asyn_address_setuptime	= 0;							//异步访问地址建立时间
	write_timing.asyn_address_holdtime	= 2;							//异步访问地址保持时间
	write_timing.asyn_data_setuptime	= 20;							//异步访问数据建立时间 100ns
	write_timing.bus_latency			= 0;							//同步/异步访问总线延时时间
	write_timing.syn_clk_division		= EXMC_SYN_CLOCK_RATIO_15_CLK; 	//同步访问时钟分频系数（从HCLK中分频）
	write_timing.syn_data_latency		= EXMC_DATALAT_2_CLK;			//同步访问中获得第1个数据所需要的等待延迟


	/* step 2 : configure SDRAM control registers */
	//Region1配置

	sram_init_struct.norsram_region		= EXMC_BANK0_NORSRAM_REGION0;	//Region0
	sram_init_struct.address_data_mux	= DISABLE;						//地址、数据总线多路复用
	sram_init_struct.memory_type		= EXMC_MEMORY_TYPE_SRAM;		//储存器类型为SRAM
	sram_init_struct.databus_width		= EXMC_NOR_DATABUS_WIDTH_16B;	//数据宽度16位
	sram_init_struct.burst_mode			= DISABLE;						//禁用突发访问
	sram_init_struct.nwait_config		= EXMC_NWAIT_CONFIG_BEFORE;		//等待输入配置
	sram_init_struct.nwait_polarity		= EXMC_NWAIT_POLARITY_HIGH;		//等待输入信号低电平有效
	sram_init_struct.wrap_burst_mode	= DISABLE;						//禁用包突发访问
	sram_init_struct.asyn_wait			= DISABLE;						//禁用异步等待
	sram_init_struct.extended_mode		= ENABLE;						//使能扩展模式
	sram_init_struct.memory_write		= ENABLE;						//使能写入外部存储器
	sram_init_struct.nwait_signal		= DISABLE;						//禁用等待输入信号
	sram_init_struct.write_mode			= EXMC_ASYN_WRITE;				//写入模式为异步写入
	sram_init_struct.read_write_timing	= &read_timing;					//读时序配置
	sram_init_struct.write_timing		= &write_timing;				//写时序配置
	//初始化Region1
	exmc_norsram_init(&sram_init_struct);

	//使能Region1
	exmc_norsram_enable(EXMC_BANK0_NORSRAM_REGION0);
	
	exmc_norsram_enable(EXMC_BANK0_NORSRAM_REGION1);
  
	/* Read FPGA version */
	while (FMC_ReadWord(SPACE_COM, 0x220 >> 1) != 0x5555);

}




