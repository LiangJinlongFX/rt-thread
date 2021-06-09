/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2015-07-29     Arda.Fu      first implementation
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <chip.h>
#include "board.h"

#define LED4_PIN  30   /* PIO0_30 */

int main(void)
{
    /* user app entry */
    while(1)
		{
			rt_thread_delay(1000);
		}
}

/* Set up and initialize hardware prior to call to main */
void SystemInit(void)
{
#if defined(__CODE_RED)
	extern void(*const g_pfnVectors[]) (void);
	SCB->VTOR = (uint32_t) &g_pfnVectors;
#else
	extern void *__Vectors;
	SCB->VTOR = (uint32_t) &__Vectors;
#endif

#if defined(CORE_M4)
#if defined(__FPU_PRESENT) && __FPU_PRESENT == 1
	fpuInit();
#endif
#endif

#if !defined(__MULTICORE_M0SLAVE) && !defined(__MULTICORE_M4SLAVE)
#if defined(NO_BOARD_LIB)
	/* Chip specific SystemInit */
	Chip_SystemInit();
#else
	/* Board specific SystemInit */
	Board_SystemInit();
#endif
#endif
}
