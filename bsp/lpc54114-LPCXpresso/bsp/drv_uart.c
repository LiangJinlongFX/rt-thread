/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2013-05-18     Bernard      The first version for LPC40xx
 * 2014-12-16     RT_learning  The first version for LPC5410x
 * 2017-08-01     XiaoYang     The first version for LPC546xx
 * 2018-11-30     yangjie      The first version for LPC54114
 * fixme:
 * - 串口收发未加忙状态判断
 */

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

#include "board.h"
#include <chip.h>

static rt_err_t lpc_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    LPC_USART_T* uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (LPC_USART_T*)serial->parent.user_data;
		Chip_UART_SetFIFOTrigLevel(uart, 8, 0);
		Chip_UART_EnableFIFOInts(uart, UART_FIFOINT_RXLVL);
		//Chip_UART_IntEnable(uart, UART_INT_START);
		NVIC_EnableIRQ(USART0_IRQn);
    return RT_EOK;
}

static rt_err_t lpc_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    LPC_USART_T *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (LPC_USART_T *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        break;
    case RT_DEVICE_CTRL_SET_INT:
        /* enable rx irq */
        break;
    }

    return RT_EOK;
}

static int lpc_putc(struct rt_serial_device *serial, char c)
{
    LPC_USART_T *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (LPC_USART_T *)serial->parent.user_data;
		//while(!Chip_UART_GetStatus(uart)&UART_STAT_TXIDLE) {}
		// 使用 Chip_UART_SendByte 会发送异常
		Chip_UART_SendBlocking(uart, &c, 1);
    return 1;
}

static int lpc_getc(struct rt_serial_device *serial)
{
		int ch;
    LPC_USART_T *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (LPC_USART_T *)serial->parent.user_data;
    if(Chip_UART_GetFIFOStatus(uart)&UART_FIFOSTAT_RXNOTEMPTY)
		{
      Chip_UART_Read(uart, &ch, 1);
			return ch;
		}
    else
        return -1;
}

static const struct rt_uart_ops lpc_uart_ops =
{
    lpc_configure,
    lpc_control,
    lpc_putc,
    lpc_getc,
};

struct rt_serial_device serial0;

void FLEXCOMM0_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_hw_serial_isr(&serial0, RT_SERIAL_EVENT_RX_IND);
	
    /* leave interrupt */
    rt_interrupt_leave();
}

int rt_hw_uart_init(void)
{
    LPC_USART_T *uart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    /* 初始化串口句柄 */
    uart = LPC_USART0;
    serial0.ops    = &lpc_uart_ops;
    serial0.config = config;
    serial0.parent.user_data = uart;

    /* 底层初始化 */
    /* 将参数转换为底层API参数 */
    uint8_t datalen = 0x00;
    uint8_t parity = 0x00;
    uint8_t stop_bits = 0x00;
    switch(config.data_bits)
    {
        case DATA_BITS_7: datalen = UART_CFG_DATALEN_7; break;
        case DATA_BITS_8: datalen = UART_CFG_DATALEN_8; break;
        case DATA_BITS_9: datalen = UART_CFG_DATALEN_9; break;
        default: RT_ASSERT(FALSE); break;
    }
    switch(config.parity)
    {
        case PARITY_NONE: parity = UART_CFG_PARITY_NONE; break;
        case PARITY_ODD: parity = UART_CFG_PARITY_ODD; break;
        case PARITY_EVEN: parity = UART_CFG_PARITY_EVEN; break;
        default: RT_ASSERT(FALSE); break;
    }
    switch(config.stop_bits)
    {
        case STOP_BITS_1: stop_bits = UART_CFG_STOPLEN_1; break;
        case STOP_BITS_2: stop_bits = UART_CFG_STOPLEN_2; break;
        default: RT_ASSERT(FALSE); break;
    }
    /* 调用底层API进行初始化 */
    Chip_UART_Init(uart);
		Chip_UART_SetBaud(uart, config.baud_rate);
		Chip_UART_ConfigData(uart, datalen | parity | stop_bits);
		Chip_UART_Enable(uart);
		Chip_UART_TXEnable(uart);

    /* register UART0 device */
    rt_hw_serial_register(&serial0, "uart0",
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_STREAM,
                          uart);

    return 0;
}
INIT_BOARD_EXPORT(rt_hw_uart_init);
