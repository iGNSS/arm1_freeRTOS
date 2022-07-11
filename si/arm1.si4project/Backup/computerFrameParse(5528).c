#include "computerFrameParse.h"
#include "data_convert.h"
#include "serial.h"

//#include "bsp_sys.h"

//INSDataTypeDef hINSData;
//FPGA_FRAME_DEF hINSFPGAData;
uint8_t fpga_data_read_flag;
AppSettingTypeDef hSetting;
AppSettingTypeDef hDefaultSetting;
uint8_t setting_update = 0;

#define	comm_send_end_frame()	\
    do \
    { \
        uint8_t  frameEnd[4] = {0xFA, 0x55, 0x00,0xFF}; \
        gd32_usart_write(frameEnd, 4); \
    }while(0)

uint8_t frame_setting_is_update(void)
{
    return setting_update;
}

void frameParse(uint8_t* pData, uint16_t len)
{
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
                    hSetting.serialFrameSetting[0].frameType = (*pDat);
                    hSetting.serialFrameSetting[0].baudrate = (*(pDat + 1));
                    hSetting.serialFrameSetting[0].freq = (*(pDat + 3) << 8) + (*(pDat + 2));

                    hSetting.serialFrameSetting[1].frameType = (*(pDat + 4));
                    hSetting.serialFrameSetting[1].baudrate = (*(pDat + 5));
                    hSetting.serialFrameSetting[1].freq = (*(pDat + 7) << 8) + (*(pDat + 6));

                    hSetting.serialFrameSetting[2].frameType = (*(pDat + 8));
                    hSetting.serialFrameSetting[2].baudrate = (*(pDat + 9));
                    hSetting.serialFrameSetting[2].freq = (*(pDat + 11) << 8) + (*(pDat + 10));
                }

                break;

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

                gd32_usart_set_baud(UART4, hSetting.comm_cfg.baudRate, hSetting.comm_cfg.parity, hSetting.comm_cfg.data_bits, hSetting.comm_cfg.stop_bits);
                break;

            case CMD_SET_SAVE_CONFIG:				//存储参数
                if((*(pDat + 2)) == FRAME_END)
                {
                    if((*pDat) == 1)
                    {
                        //保存参数
                        //save_flash();
                    }
                    else if((*pDat) == 1)
                    {
                        //恢复默认参数
                        memcpy(&hSetting, &hDefaultSetting, sizeof(AppSettingTypeDef));
                        //Sys_Soft_Reset();
                    }
                }

                break;

            case CMD_SET_READ_CONFIG:				//参数回读
                if((*(pDat + 2)) == FRAME_END)
                {
                    if((*(pDat + 2)) == 1)
                    {
                        //回读参数
                        //read_flash();
                        ///////////////////////////////////////////////////
                    }
                }

                break;

            case CMD_SET_SYS_MODE:					//设定系统工作模式
                if((*(pDat + 2)) == FRAME_END)
                {
                    hSetting.datamode = (INS_DATA_ENUMTypeDef)(*(pDat + 1));
                    hSetting.workmode = (INS_BOOT_MODE_ENUMTypeDef)(*(pDat));
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
                    hSetting.gnssMechanicalMigration_x = hex2Float(tData);
                    tData[0] = *(pDat + 4);
                    tData[1] = *(pDat + 5);
                    tData[2] = *(pDat + 6);
                    tData[3] = *(pDat + 7);
                    hSetting.gnssMechanicalMigration_y = hex2Float(tData);
                    tData[0] = *(pDat + 8);
                    tData[1] = *(pDat + 9);
                    tData[2] = *(pDat + 10);
                    tData[3] = *(pDat + 11);
                    hSetting.gnssMechanicalMigration_z = hex2Float(tData);
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
                    hSetting.courseAngleCompensation = hex2Float(tData);
                }

                break;

            case CMD_SET_ZERO_OFFSET_TIME:			//设置静态测零偏时间
                if((*(pDat + 2)) == FRAME_END)
                {
                    uint8_t tData[2] = {0};
                    tData[0] = *(pDat + 0);
                    tData[1] = *(pDat + 1);
                    hSetting.timeCompensation = (tData[1] << 8) + tData[0];
                }

                break;

            case CMD_SET_USER_AXIS:					//设置用户设置的坐标轴与基准坐标轴的校正方式
                if((*(pDat + 1)) == FRAME_END)
                {
                    hSetting.imuAxis = *pDat;
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
                    hSetting.gnssBaselineLength = hex2Float(tData);
                    //((*(pDat+3)<<24)+(*(pDat+2)<<16)+(*(pDat+1)<<8)+(*pDat));
                }

                break;

            default:
                break;
        }
    }

    comm_send_end_frame();
}

static struct
{
    uint8_t  usart_rx_buffer[USART_SERIAL_RB_BUFSZ];
    uint16_t usart_rx_count;

} usart_rx_data = {0,};

EventStatus usart_dispose_recvDataTask(void)
{
    static uint8_t   count = 0;
    uint32_t datalen;

    datalen = gd32_usart_read(usart_rx_data.usart_rx_buffer + usart_rx_data.usart_rx_count, USART_SERIAL_RB_BUFSZ - usart_rx_data.usart_rx_count);

    if(0 != datalen)
    {
        usart_rx_data.usart_rx_count += datalen;
        count = 0;

        if(usart_rx_data.usart_rx_count >= USART_SERIAL_RB_BUFSZ)
        {
            usart_rx_data.usart_rx_count = 0;
        }
    }
    else
    {
        count++;

        if(count >= 2)
        {
            count = 0;

            if(usart_rx_data.usart_rx_count >= 5)
            {
                frameParse(usart_rx_data.usart_rx_buffer, usart_rx_data.usart_rx_count);

                #ifdef configUse_SEGGER_RTT
                {
                    uint8_t i;

                    for(i = 0; i < usart_rx_data.usart_rx_count; i++)
                        SEGGER_RTT_printf(0, "%x ", usart_rx_data.usart_rx_buffer[i]);

                    SEGGER_RTT_printf(0, "\r\n");
                }
                #else
                gd32_usart_write(usart_rx_data.usart_rx_buffer, usart_rx_data.usart_rx_count);
                #endif
            }

            usart_rx_data.usart_rx_count = 0;
        }
    }

    return ENABLE;
}

