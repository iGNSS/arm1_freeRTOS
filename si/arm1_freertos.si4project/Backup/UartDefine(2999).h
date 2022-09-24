
#ifndef _UART_DEFINE_H_
#define _UART_DEFINE_H_

typedef unsigned short USHORT;
typedef unsigned char UCHAR;

typedef enum
{
	UART_BAUDRATE_2400BPS = 0,
	UART_BAUDRATE_4800BPS,
	UART_BAUDRATE_9600BPS,
	UART_BAUDRATE_19200BPS,
	UART_BAUDRATE_38400BPS,
	UART_BAUDRATE_57600BPS,
	UART_BAUDRATE_115200BPS,
	UART_BAUDRATE_230400BPS,
	UART_BAUDRATE_384000BPS,
	UART_BAUDRATE_460800BPS,
	UART_BAUDRATE_750000BPS,
	UART_BAUDRATE_921600BPS,
	UART_BAUDRATE_2500000BPS
}EUartBaudrate;

typedef enum
{
	UART_PARITY_NONE = 0,
	UART_PARITY_ODD = 0x0C,
	UART_PARITY_EVEN = 0x08
}EUartParitybits;

typedef enum
{
	UART_STOPBIT_ONE = 0,
	UART_STOPBIT_TWO = 2
}EUartStopbits;

typedef enum
{
	UART_RS232 = 0,
	UART_RS422,
    UART_RS485,
	UART_DEFAULT
}EUartMode;

typedef enum
{
	UART_DISABLE = 0,
	UART_ENABLE = 1
}EUartEnable;

typedef enum
{
	UART_RXPORT_RS232_1 = 0,
	UART_RXPORT_COMPLEX_8 = 1,
	UART_RXPORT_COMPLEX_9 = 2,
	UART_RXPORT_COMPLEX_10 = 3,
	UART_RXPORT_COMPLEX_11 = 4,
	UART_RXPORT_COMPLEX_12 = 5,
	UART_RXPORT_COMPLEX_13 = 6,
	UART_RXPORT_COMPLEX_14 = 7,
	UART_RXPORT_COMPLEX_15 = 8,
	UART_RXPORT_NULL = 255
}EUartRxPort;

typedef enum
{
	UART_TXPORT_RS232_1 = 0,
	UART_TXPORT_COMPLEX_8 = 1,
	UART_TXPORT_COMPLEX_9 = 2,
	UART_TXPORT_COMPLEX_10 = 3,
	UART_TXPORT_COMPLEX_11 = 4,
	UART_TXPORT_COMPLEX_12 = 5,
	UART_TXPORT_COMPLEX_13 = 6,
	UART_TXPORT_COMPLEX_14 = 7,
	UART_TXPORT_COMPLEX_15 = 8,
	UART_TXPORT_NULL = 255
}EUartTxPort;


#endif