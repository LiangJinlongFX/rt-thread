#ifndef _LPC5411X_BOARD_H
#define _LPC5411X_BOARD_H

#include <chip.h>


/** 调试用串口句柄 */
#define DEBUG_UART  LPC_USART0
#define USART0_FLEXCOMM         0

/* SPI0 (master) is FLEXCOMM 5	*/
#define SPI0_FLEXCOMM   				5      											

/** Main system clock rate in Hz for this board. Select a clock rate between
    12MHz and 96MHz for the main system (CPU) clock for this board. */
#define BOARD_MAINCLOCKRATE     (48000000)

/** External clock rate on the CLKIN pin in Hz for this board. If not used,
    set this to 0. Otherwise, set it to the exact rate in Hz this pin is
    being driven at. */
#define BOARD_EXTCLKINRATE      (0)

/** Set the BOARD_USECLKINSRC definition to (1) to use the external clock
    input pin as the PLL source. The BOARD_ECTCLKINRATE definition must
    be setup with the correct rate in the CLKIN pin. Set this to (0) to
    use the IRC for the PLL source. */
#define BOARD_USECLKINSRC       (0)


#ifdef __CC_ARM
extern int Image$$RW_IRAM1$$ZI$$Base;
extern int Image$$RW_IRAM1$$ZI$$Limit;
#define HEAP_BEGIN          (0x20010000)
#define HEAP_END            (0x20020000)
#elif __ICCARM__
#pragma section="HEAP"
#define HEAP_BEGIN  (__segment_end("HEAP"))
extern void __RTT_HEAP_END;
#define HEAP_END            (&__RTT_HEAP_END)
#else
extern int heap_start;
extern int heap_end;
#define HEAP_BEGIN          (&heap_start)
#define HEAP_END            (&heap_end)
#endif

void Board_SystemInit(void);
void Board_LED_RGB_Set(uint8_t led);
void Board_LED_RGB_Off(void);
#endif