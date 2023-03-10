#include "gd32f4xx.h"
#include "string.h"
#include "stdlib.h"
#include "clock.h"
#include "systick.h"

#include "ymodem.h"
#include "fmc_operation.h"
#include "drv_usart.h"
#include "uartadapter.h"

/* Kernel includes. */
//#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
//#include "semphr.h"
#include "event_groups.h"

SemaphoreHandle_t xYmodemSemaphore   = NULL;

static uint32_t wr_pointer = 0;

static const uint16_t ccitt_table[256] =
{
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};
static uint16_t CRC16(unsigned char *q, int len)
{
    uint16_t crc = 0;

    while (len-- > 0)
        crc = (crc << 8) ^ ccitt_table[((crc >> 8) ^ *q++) & 0xff];
    return crc;
}
static struct rym_ctx *_rym_the_ctx;

ins_size_t ins_device_read(void       *buffer, ins_size_t   size)
{
        //return gd32_usart_read(buffer, size); 
        return Uart_RecvMsg(UART_RXPORT_COMPLEX_8, size, buffer);
}

ins_size_t ins_device_write(void       *buffer, ins_size_t   size)
{
        //return gd32_usart_write(buffer, size); 
        Uart_SendMsg(UART_TXPORT_COMPLEX_8, 0, size, buffer);
        return 1;
}


// we could only use global varible because we could not use
// rt_device_t->user_data(it is used by the serial driver)...
static struct rym_ctx *_rym_the_ctx;

//static ins_err_t _rym_rx_ind(ins_size_t size)
//{
//    return xSemaphoreGive(_rym_the_ctx->sem);
//}

/* SOH/STX + seq + payload + crc */
#define _RYM_SOH_PKG_SZ (1+2+128+2)
#define _RYM_STX_PKG_SZ (1+2+1024+2)
#define _RYM_PKG_SZ 	_RYM_SOH_PKG_SZ

static enum rym_code _rym_read_code(
        struct rym_ctx *ctx,
        ins_tick_t timeout)
{
    /* Fast path */
    if (ins_device_read(ctx->buf, 1) == 1)
        return (enum rym_code)(*ctx->buf);

    /* Slow path */
    do {
        ins_size_t rsz;

        /* No data yet, wait for one */
        if (xSemaphoreTake(ctx->sem, timeout) != INS_EOK)
            return RYM_CODE_NONE;

        /* Try to read one */
        rsz = ins_device_read(ctx->buf, 1);
        if (rsz == 1)
            return (enum rym_code)(*ctx->buf);
    } while (1);
}

/* the caller should at least alloc _RYM_STX_PKG_SZ buffer */
static ins_size_t _rym_read_data(
        struct rym_ctx *ctx,
        ins_size_t len)
{
    /* we should already have had the code */
    uint8_t *buf = ctx->buf;
    ins_size_t readlen = 0;

    do
    {
        readlen += ins_device_read(buf+readlen, len-readlen);
        if (readlen >= len)
            return readlen;
    } while (xSemaphoreTake(ctx->sem, RYM_WAIT_CHR_TICK) == INS_EOK);

    return readlen;
}

static ins_size_t _rym_putchar(struct rym_ctx *ctx, uint8_t code)
{
    ins_device_write(&code, sizeof(code));
    return 1;
}

static ins_err_t _rym_do_handshake(
        struct rym_ctx *ctx,
        int tm_sec)
{
//    enum rym_code code;
    ins_size_t i,len;
    uint16_t recv_crc, cal_crc;
	char* ch;
	
    ctx->stage = RYM_STAGE_ESTABLISHING;
    /* send C every second, so the sender could know we are waiting for it. */
    for (i = 0; i < tm_sec; i++)
    {
        _rym_putchar(ctx, RYM_CODE_C);
        len = _rym_read_data(ctx, _RYM_PKG_SZ);
        if (len == (_RYM_PKG_SZ))
            break;
    }
    if (i == tm_sec)
        return -RYM_ERR_TMO;

    ch = strstr((char*)&ctx->buf[3], ".bin");
	if(NULL != ch)
	{
		ctx->filesize = strtod(ch+5, NULL);
	}
    /* sanity check */
#if _RYM_PKG_SZ == _RYM_STX_PKG_SZ    
		if (ctx->buf[0] != RYM_CODE_STX || ctx->buf[1] != 0 || ctx->buf[2] != 0xFF)
			return -RYM_ERR_SEQ;
#else
		if (ctx->buf[0] != RYM_CODE_SOH || ctx->buf[1] != 0 || ctx->buf[2] != 0xFF)
			return -RYM_ERR_SEQ;
#endif

    recv_crc = (uint16_t)(*(ctx->buf+_RYM_PKG_SZ-2) << 8) | *(ctx->buf+_RYM_PKG_SZ-1);
    cal_crc = CRC16(ctx->buf+3, _RYM_PKG_SZ-5);
    if (recv_crc != cal_crc)
        return -RYM_ERR_CRC;

    /* congratulations, check passed. */
    if (ctx->on_begin && ctx->on_begin(ctx, ctx->buf+3, 128) != RYM_CODE_ACK)
        return -RYM_ERR_CAN;

    return INS_EOK;
}

static ins_err_t _rym_trans_data(
        struct rym_ctx *ctx,
        ins_size_t data_sz,
        enum rym_code *code)
{
    ins_size_t tsz = 1+2+data_sz+2;
    uint16_t recv_crc;

    /* seq + data + crc */
    ins_size_t i = _rym_read_data(ctx, tsz);
    if(ctx->buf[0] == RYM_CODE_EOT)
    	return INS_EOK;
    if(ctx->buf[0] == RYM_CODE_SOH)
    {
    	tsz = _RYM_SOH_PKG_SZ;
    	data_sz = 128;
	}
	if (i != tsz)
	    return -RYM_ERR_DSZ;

    if ((ctx->buf[1] + ctx->buf[2]) != 0xFF)
    {
        return -RYM_ERR_SEQ;
    }

    /* As we are sending C continuously, there is a chance that the
     * sender(remote) receive an C after sending the first handshake package.
     * So the sender will interpret it as NAK and re-send the package. So we
     * just ignore it and proceed. */
    if (ctx->stage == RYM_STAGE_ESTABLISHED && ctx->buf[1] == 0x00)
    {
        *code = RYM_CODE_NONE;
        return INS_EOK;
    }

    ctx->stage = RYM_STAGE_TRANSMITTING;

    /* sanity check */
    recv_crc = (uint16_t)(*(ctx->buf+tsz-2) << 8) | *(ctx->buf+tsz-1);
    if (recv_crc != CRC16(ctx->buf+3, data_sz))
        return -RYM_ERR_CRC;

    /* congratulations, check passed. */
    if (ctx->on_data)
        *code = ctx->on_data(ctx, ctx->buf+2, data_sz);
    else
        *code = RYM_CODE_ACK;
    return INS_EOK;
}

static ins_err_t _rym_do_trans(struct rym_ctx *ctx)
{
	fmc_erase_sector_by_address(USER_DAPSAVE_PAGE_ADDR);
	
    _rym_putchar(ctx, RYM_CODE_ACK);
    _rym_putchar(ctx, RYM_CODE_C);
    ctx->stage = RYM_STAGE_ESTABLISHED;
    ins_err_t err;
    enum rym_code code;
    ins_size_t i;
	#if _RYM_PKG_SZ == _RYM_STX_PKG_SZ 
	ins_size_t data_sz = 1024;
	#else
	ins_size_t data_sz = 128;
	#endif

    while (1)
    {

        err = _rym_trans_data(ctx, data_sz, &code);
        if (err != INS_EOK)
            return err;
        if(ctx->buf[0] == RYM_CODE_EOT)
        {
            	return INS_EOK;
        }
        switch (code)
        {
        case RYM_CODE_CAN:
            /* the spec require multiple CAN */
            for (i = 0; i < RYM_END_SESSION_SEND_CAN_NUM; i++) {
                _rym_putchar(ctx, RYM_CODE_CAN);
            }
            return -RYM_ERR_CAN;
        case RYM_CODE_ACK:
            _rym_putchar(ctx, RYM_CODE_ACK);
            break;
        default:
            // wrong code
            break;
        };
		if(ctx->buf[0] == RYM_CODE_SOH)
        {
        	data_sz = 128;
        }
        fmc_write_8bit_data(USER_DAPSAVE_PAGE_ADDR+wr_pointer, data_sz, (int8_t*)(ctx->buf+3));
        wr_pointer += data_sz;
    }
}


static ins_err_t _rym_do_fin(struct rym_ctx *ctx)
{
    enum rym_code code;
    uint16_t recv_crc;
    ins_size_t i;

    ctx->stage = RYM_STAGE_FINISHING;
    /* we already got one EOT in the caller. invoke the callback if there is
     * one. */
    if (ctx->on_end)
        ctx->on_end(ctx, ctx->buf+3, 128);

    _rym_putchar(ctx, RYM_CODE_NAK);
    code = _rym_read_code(ctx, RYM_WAIT_PKG_TICK);
    if (code != RYM_CODE_EOT)
        return -RYM_ERR_CODE;

    _rym_putchar(ctx, RYM_CODE_ACK);
    _rym_putchar(ctx, RYM_CODE_C);

    i = _rym_read_data(ctx, _RYM_SOH_PKG_SZ);
    if (i != _RYM_SOH_PKG_SZ)
        return -RYM_ERR_DSZ;
        
	if (ctx->buf[0] != RYM_CODE_SOH)
        return -RYM_ERR_CODE;
    /* sanity check
     *
     * TODO: multiple files transmission
     */
    if (ctx->buf[1] != 0 || ctx->buf[2] != 0xFF)
        return -RYM_ERR_SEQ;

    recv_crc = (uint16_t)(*(ctx->buf+_RYM_SOH_PKG_SZ-2) << 8) | *(ctx->buf+_RYM_SOH_PKG_SZ-1);
    if (recv_crc != CRC16(ctx->buf+3, _RYM_SOH_PKG_SZ-5))
        return -RYM_ERR_CRC;

    /* congratulations, check passed. */
    ctx->stage = RYM_STAGE_FINISHED;

    /* put the last ACK */
    _rym_putchar(ctx, RYM_CODE_ACK);

    return INS_EOK;
}


static ins_err_t _rym_do_recv(
        struct rym_ctx *ctx,
        int handshake_timeout)
{
    ins_err_t err;

    ctx->stage = RYM_STAGE_NONE;

	ctx->sem = xSemaphoreCreateBinary();
    if (ctx->sem == NULL)
        return -INS_ENOMEM;
        
    ctx->buf = pvPortMalloc(_RYM_STX_PKG_SZ);
    if (ctx->buf == NULL)
        return -INS_ENOMEM;

    err = _rym_do_handshake(ctx, handshake_timeout);
    if (err != INS_EOK)
        return err;

    err = _rym_do_trans(ctx);
    if (err != INS_EOK)
        return err;

    return _rym_do_fin(ctx);
}

ins_err_t rym_recv_on_device(
        struct rym_ctx *ctx,
        rym_callback on_begin,
        rym_callback on_data,
        rym_callback on_end,
        int handshake_timeout)
{
    ins_err_t res;

    configASSERT(_rym_the_ctx == 0);
    _rym_the_ctx = ctx;

    ctx->on_begin = on_begin;
    ctx->on_data  = on_data;
    ctx->on_end   = on_end;

    res = _rym_do_recv(ctx, handshake_timeout);

	if(INS_EOK == res)
	{
		uint8_t dapInfo[5];
		dapInfo[0] = 0x1;
		dapInfo[1] = ctx->filesize;
		dapInfo[2] = ctx->filesize>>8;
		dapInfo[3] = ctx->filesize>>16;
		dapInfo[4] = ctx->filesize>>24;
		fmc_erase_sector_by_address(USER_DAPFLAG_PAGE_ADDR);
		fmc_write_8bit_data(USER_DAPFLAG_PAGE_ADDR, 5, (int8_t*)dapInfo);
	}
    vPortFree(ctx->buf);
    _rym_the_ctx = NULL;

    return res;
}

static enum rym_code _rym_dummy_write(
        struct rym_ctx *ctx,
        uint8_t *buf,
        ins_size_t len)
{
    return RYM_CODE_ACK;
}

ins_err_t rym_null(void)
{
    struct rym_ctx rctx;

    return rym_recv_on_device(&rctx, NULL, _rym_dummy_write, NULL, 1000);
}

#ifdef configUse_debug
extern void imu_irq_unstall(void);
void rym_fm_update(void)
{

	do 
	 	{
			if(INS_EOK == rym_null())
			{
				RTC_BKP5 = CMD_BOOT;
				vTaskDelay(20);
				NVIC_SystemReset();
			}
			vTaskDelay(20);
	 	}while(1);
}
#endif
extern TaskHandle_t task_imu_handler;
extern TaskHandle_t task_gnss_handler;
extern TaskHandle_t task_imu_comm5_handler;
extern TaskHandle_t task_gnss_comm2_handler;
extern TaskHandle_t task_gnss_comm3_handler;
extern void fm_update_irq_unstall(void);
void ymodem_give_sem(void)
{
	xSemaphoreGive( xYmodemSemaphore);
}

void ymodem_task(void* arg)
{
    
    for( ;; )
    {
        if(pdTRUE == xSemaphoreTake( xYmodemSemaphore, portMAX_DELAY))//pdMS_TO_TICKS(100)
        {
        	vTaskSuspend( task_imu_comm5_handler );
        	vTaskSuspend( task_gnss_comm2_handler );
        	vTaskSuspend( task_gnss_comm3_handler );
        	vTaskSuspend( task_gnss_handler );
        	vTaskSuspend( task_imu_handler );
			fm_update_irq_unstall();
        	timer_disable(TIMER0);
			timer_disable(TIMER3);
            rym_fm_update();
        }
    }
}


void ymodem_task_create(void)
{
    xTaskCreate( ymodem_task,
                 "ymodem_task",
                 configMINIMAL_STACK_SIZE,
                 ( void * ) NULL,
                 tskIDLE_PRIORITY + 3,
                 NULL );
}

void ymodem_init(void)
{
    xYmodemSemaphore = xSemaphoreCreateBinary();
    configASSERT( xYmodemSemaphore );

}

