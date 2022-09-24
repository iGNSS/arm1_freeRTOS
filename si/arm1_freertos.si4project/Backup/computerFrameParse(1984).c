#include "computerFrameParse.h"
#include "data_convert.h"
#include "serial.h"
#include "uartadapter.h"

#include "gnss.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"

QueueHandle_t xCommQueue;

uint8_t fpga_data_read_flag;
AppSettingTypeDef hSetting;
AppSettingTypeDef hDefaultSetting =
{
    .saved = 1,
    .report_en = 0,
    .ProductID = 0x0001,
    .DeviceID  = 0x0001,
    .ChipID    =
    {
        0,0,0
    },
    //data
    {
	    {
	        {cfg_format_RAWIMU,6,100},
	        {cfg_format_RAWIMU,6,100},
	        {cfg_format_RAWIMU,6,100}
	    },
	    //系统工作状态
	    INS_BOOT_MODE_1,
	    INS_DATA_MODE_1,
	    //GNSS臂杆参数 3
	    1.0,
	    1.0,
	    1.0,
	    //航向角补偿 4
	    1.0,
	    //用户设置的坐标轴类型 5
	    INS_DATA_MODE_1,
	    //时间补偿 6
	    2,
	    //GNSS基线长度 7
	    0.8,
	},
    .ARM1_FW_Ver = 0x1,
    .ARM2_FW_Ver = 0x1,

    .FPGA_FW_Ver = 0x1
};

uint8_t setting_update = 0;

void comm_send_end_frame(uint16_t cmd)
{
    uint8_t  frameEnd[6];
    frameEnd[0] = 0xFA;
    frameEnd[1] = 0x55;
    frameEnd[2] = cmd >> 8;
    frameEnd[3] = cmd;
    frameEnd[4] = 0x00;
    frameEnd[5] = 0xFF;
#if (configUse_COMM == COMM_MODE_RS422)
    Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, 6, frameEnd);
#elif (configUse_COMM == COMM_MODE_RS232)
    gd32_usart_write(frameEnd, 6);
#endif
}
uint8_t frame_setting_is_update(void)
{
    return setting_update;
}

static const struct
{
    uint8_t index;
    uint32_t baudrate;
} comm_baud_table[] =
{
    {0, 0},
    {1, 9600},
    {2, 19200},
    {3, 38400},
    {4, 57600},
    {5, 115200},
    {6, 230400},
    {7, 460800},
    {8, 614400}
};

void frameParse(uint8_t* pData, uint16_t len)
{
    uint8_t id;
    uint8_t valid = 0;
    uint8_t* pDat = pData;
    uint16_t tCmd = 0;
    uint16_t dataLen = len;
    uint8_t  frameHead[3] = {0xAF, 0x55, 0xFA};

    //找到帧头
    do
    {
        if(0 == memcmp(pDat, frameHead, 3))
        {
            valid = 1;
            break;
        }

        pDat++;
        dataLen--;
    }
    while(dataLen > 0);

    if(valid)
    {
        pDat += 3;
        tCmd = ((*pDat) << 8) + (*(pDat + 1));
        pDat ++;
        pDat ++;

        switch(tCmd)
        {
        case CMD_SET_OUTPUT_FORMAT:				//串口输出模式
            if((*(pDat + 12)) == FRAME_END)
            {
                hSetting.settingData.serialFrameSetting[0].frameType = (*pDat);
                hSetting.settingData.serialFrameSetting[0].baudrate = (*(pDat + 1));
                hSetting.settingData.serialFrameSetting[0].freq = (*(pDat + 3) << 8) + (*(pDat + 2));

                hSetting.settingData.serialFrameSetting[1].frameType = (*(pDat + 4));
                hSetting.settingData.serialFrameSetting[1].baudrate = (*(pDat + 5));
                hSetting.settingData.serialFrameSetting[1].freq = (*(pDat + 7) << 8) + (*(pDat + 6));

                hSetting.settingData.serialFrameSetting[2].frameType = (*(pDat + 8));
                hSetting.settingData.serialFrameSetting[2].baudrate = (*(pDat + 9));
                hSetting.settingData.serialFrameSetting[2].freq = (*(pDat + 11) << 8) + (*(pDat + 10));
            }
            id = hSetting.settingData.serialFrameSetting[0].baudrate;
            Uart_TxInit(UART_TXPORT_COMPLEX_8, (EUartBaudrate)id, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS422, UART_ENABLE);
            Uart_RxInit(UART_RXPORT_COMPLEX_8, (EUartBaudrate)id, UART_PARITY_NONE, UART_STOPBIT_ONE, UART_RS422, UART_ENABLE);
            id = hSetting.settingData.serialFrameSetting[1].baudrate;
            gd32_usart_set_baud(comm_baud_table[id].baudrate);
            break;
#if 0
        case CMD_SET_SERIAL_CONFIG:				//设定串口配置
            if((*(pDat + 10)) == FRAME_END)
            {
                hSetting.comm_cfg.comX = *pDat++;
                hSetting.comm_cfg.baudRate = *pDat++;
                hSetting.comm_cfg.baudRate |= (*pDat++) << 8;
                hSetting.comm_cfg.baudRate |= (*pDat++) << 16;
                hSetting.comm_cfg.baudRate |= (*pDat++) << 24;
                hSetting.comm_cfg.data_bits = *pDat++;
                hSetting.comm_cfg.parity = *pDat++;
                hSetting.comm_cfg.stop_bits = *pDat++;
                hSetting.comm_cfg.mode = *pDat++;
                hSetting.comm_cfg.usr_type = *pDat++;
            }

            break;
#endif
        case CMD_SET_SAVE_CONFIG:				//存储参数
            if(*pDat== FRAME_END)
            {
                //保存参数
                comm_set_customPara();
            }

            break;

		case CMD_SET_RESUME_DEFAULT:				//恢复默认
            if(*pDat== FRAME_END)
            {
                //恢复默认参数
                comm_resume_defaultPara();
            }

            break;

        case CMD_SET_SAVE_ALL:				//全部应用
        //if((*(pDat + 42)) == FRAME_END)
        {
            //恢复默认参数
            memcpy((void*)&hSetting.settingData.serialFrameSetting[0].frameType, (void*)pDat, sizeof(SettingDataTypeDef));
        }

        break;
            
        case CMD_SET_READ_PARA:					//参数回读
            if(*pDat== FRAME_END)
            {
                //回读参数
                comm_para_ehco_rsp(tCmd);
            }

            break;

        case CMD_SET_SYS_MODE:					//设定系统工作模式
            if((*(pDat + 2)) == FRAME_END)
            {
                hSetting.settingData.datamode = (INS_DATA_ENUMTypeDef)(*(pDat + 1));
                hSetting.settingData.workmode = (INS_BOOT_MODE_ENUMTypeDef)(*(pDat));
            }

            break;

        case CMD_SET_REPORT_MODE:					//设定系统上报模式
            if((*(pDat + 1)) == FRAME_END)
            {
                hSetting.report_en = *pDat;
            }
            break;

        case CMD_SET_MECHAN_MIGRA:				//设置GNSS装配参数与基准点的坐标偏移
            if((*(pDat + 12)) == FRAME_END)
            {
                uint8_t tData[4] = {0};
                tData[0] = *(pDat + 0);
                tData[1] = *(pDat + 1);
                tData[2] = *(pDat + 2);
                tData[3] = *(pDat + 3);
                hSetting.settingData.gnssMechanicalMigration_x = hex2Float(tData);
                tData[0] = *(pDat + 4);
                tData[1] = *(pDat + 5);
                tData[2] = *(pDat + 6);
                tData[3] = *(pDat + 7);
                hSetting.settingData.gnssMechanicalMigration_y = hex2Float(tData);
                tData[0] = *(pDat + 8);
                tData[1] = *(pDat + 9);
                tData[2] = *(pDat + 10);
                tData[3] = *(pDat + 11);
                hSetting.settingData.gnssMechanicalMigration_z = hex2Float(tData);

                gnss_set_leverArm(hSetting.settingData.gnssMechanicalMigration_x,
                                  hSetting.settingData.gnssMechanicalMigration_x,
                                  hSetting.settingData.gnssMechanicalMigration_x,
                                  0.0, 0.0, 0.0);
            }

            break;

        case CMD_SET_COURSE_ANGLE:				//设置航向角补偿
            if((*(pDat + 4)) == FRAME_END)
            {
                uint8_t tData[4] = {0};
                tData[0] = *(pDat + 0);
                tData[1] = *(pDat + 1);
                tData[2] = *(pDat + 2);
                tData[3] = *(pDat + 3);
                hSetting.settingData.courseAngleCompensation = hex2Float(tData);
            }

            break;

        case CMD_SET_ZERO_OFFSET_TIME:			//设置静态测零偏时间
            if((*(pDat + 2)) == FRAME_END)
            {
                uint8_t tData[2] = {0};
                tData[0] = *(pDat + 0);
                tData[1] = *(pDat + 1);
                hSetting.settingData.timeCompensation = (tData[1] << 8) + tData[0];
            }

            break;

        case CMD_SET_USER_AXIS:					//设置用户设置的坐标轴与基准坐标轴的校正方式
            if((*(pDat + 1)) == FRAME_END)
            {
                hSetting.settingData.imuAxis = *pDat;
            }

            break;

        case CMD_SET_GNSS_BASELINE:				//设置GNSS基线长度
            if((*(pDat + 4)) == FRAME_END)
            {
                uint8_t tData[4] = {0};
                tData[0] = *(pDat + 0);
                tData[1] = *(pDat + 1);
                tData[2] = *(pDat + 2);
                tData[3] = *(pDat + 3);
                hSetting.settingData.gnssBaselineLength = hex2Float(tData);
                //((*(pDat+3)<<24)+(*(pDat+2)<<16)+(*(pDat+1)<<8)+(*pDat));
            }

        case CMD_SET_ALL:
            if((*(pDat + 61)) == FRAME_END)
            {

            }

            break;
		case 0x0064:
            if((*(pDat + 61)) == FRAME_END)
            {
							hSetting.settingData.gnssBaselineLength = *pDat;
            }

            break;
        default:
            break;
        }
        if(CMD_SET_READ_PARA != tCmd)
        	comm_send_end_frame(tCmd);
    }

}

static struct
{
    uint8_t  usart_rx_buffer[USART_SERIAL_RB_BUFSZ];
    uint16_t usart_rx_count;

} usart_rx_data = {0,};
#if (configUse_COMM == COMM_MODE_RS232)
void comm_handle(void)
{
    usart_rx_data.usart_rx_count = gd32_usart_read(usart_rx_data.usart_rx_buffer, USART_SERIAL_RB_BUFSZ);

    if(usart_rx_data.usart_rx_count)
    {
        frameParse(usart_rx_data.usart_rx_buffer, usart_rx_data.usart_rx_count);
        //gd32_usart_write(usart_rx_data.usart_rx_buffer, usart_rx_data.usart_rx_count);
    }
}
#endif
#if (configUse_COMM == COMM_MODE_RS422)
#include "nav_task.h"


void rs422_comm1_rx(void)
{
    uint8_t status;
    usart_rx_data.usart_rx_count = Uart_RecvMsg(UART_RXPORT_COMPLEX_8, USART_SERIAL_RB_BUFSZ, usart_rx_data.usart_rx_buffer);
    if(usart_rx_data.usart_rx_count)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        status = 2;
        xQueueSendFromISR(xCommQueue, &status, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
extern EXPORT_RESULT  g_Export_Result;
void rs422_comm1_task(void* arg)
{
    uint8_t status;
    xCommQueue = xQueueCreate(10, sizeof(uint8_t));
    configASSERT( xCommQueue );
    comm_store_init();
    for( ;; )
    {
        if(pdTRUE == xQueueReceive( xCommQueue, &status, pdMS_TO_TICKS(100)))//portMAX_DELAY
        {
            if(2 == status)//解析上位机数据
            {
                frameParse(usart_rx_data.usart_rx_buffer, usart_rx_data.usart_rx_count);
            }
            else if(1 == status)//串口发送至上位机
            {
                if(0 == hSetting.report_en)
                {
                    //frame_pack_and_send(&g_Export_Result, &hGPSData);
                    //frame_writeDram();
                    //Oscilloscope();
                }
            }
        }

    }
}

void rs422_task_create(void)
{
    xTaskCreate( rs422_comm1_task,
                 "rs422_comm1_task",
                 configMINIMAL_STACK_SIZE,
                 ( void * ) NULL,
                 tskIDLE_PRIORITY + 4,
                 NULL );
}

void rs422_comm1_init(void)
{
    xCommQueue = xQueueCreate(10, sizeof(uint8_t));
    configASSERT( xCommQueue );
}

#endif

//data save
#include "fmc_operation.h"
static uint16_t addr_offset_count = 0;
#define PARA_DEFAULT_ADDRESS			((uint32_t)0x08100000)
#define PARA_STORE_BASE_ADDRESS			((uint32_t)0x08104000)
#define PARA_STORE_ADDRESS_OFFSET		(sizeof(AppSettingTypeDef))
#define PARA_STORE_ADDRESS				(PARA_STORE_BASE_ADDRESS+PARA_STORE_ADDRESS_OFFSET*addr_offset_count)
#define FMC_SECTOR13_SIZE				(1024 * 16)

#define	comm_read_addrOffsetCount()		RTC_BKP1
#define	comm_write_addrOffsetCount()	RTC_BKP1 = addr_offset_count

void comm_set_defaultPara(void)
{
	fmc_erase_sector_by_address(PARA_DEFAULT_ADDRESS);
    fmc_write_8bit_data(PARA_DEFAULT_ADDRESS, sizeof(AppSettingTypeDef), (int8_t*)&hDefaultSetting.saved);
}

void comm_set_customPara(void)
{
    fmc_write_8bit_data(PARA_STORE_ADDRESS, sizeof(AppSettingTypeDef), (int8_t*)&hSetting.saved);
    comm_write_addrOffsetCount();
    addr_offset_count++;
    if((PARA_STORE_ADDRESS_OFFSET * addr_offset_count) >= FMC_SECTOR13_SIZE)
    {
		addr_offset_count = 0;//擦除扇区
		fmc_erase_sector_by_address(PARA_STORE_BASE_ADDRESS);
		comm_write_addrOffsetCount();
    }
    
}

void comm_resume_defaultPara(void)
{
    memcpy((void*)&hSetting.saved, (void*)&hDefaultSetting.saved, sizeof(AppSettingTypeDef));
    comm_set_customPara();
}

void comm_para_ehco_rsp(uint16_t cmd)
{
	uint8_t  frame[100];
    uint16_t len = sizeof(SettingDataTypeDef) + 6;

    frame[0] = 0xFA;
    frame[1] = 0x55;
    frame[2] = cmd >> 8;
    frame[3] = cmd;
    memcpy((void*)&frame[4], (void*)&hSetting.settingData.serialFrameSetting[0].frameType, sizeof(SettingDataTypeDef));
    frame[len-2] = 0x00;
    frame[len-1] = 0xFF;
    
#if (configUse_COMM == COMM_MODE_RS422)
    Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, len, frame);
#elif (configUse_COMM == COMM_MODE_RS232)
    gd32_usart_write(frameEnd, len);
#endif
}

void comm_store_init(void)
{
//    AppSettingTypeDef setting;
//    fmc_read_8bit_data(PARA_DEFAULT_ADDRESS, sizeof(AppSettingTypeDef), (int8_t*)&setting.saved);
//    if(1 != setting.saved)
//    {
//        comm_set_defaultPara();
//    }
	addr_offset_count = comm_read_addrOffsetCount();
    fmc_read_8bit_data(PARA_STORE_ADDRESS, sizeof(AppSettingTypeDef), (int8_t*)&hSetting.saved);
    if(1 != hSetting.saved)//还没有存储有效数据
    {    	
    	fmc_erase_sector_by_address(PARA_STORE_BASE_ADDRESS);
        comm_resume_defaultPara();
    }
}

