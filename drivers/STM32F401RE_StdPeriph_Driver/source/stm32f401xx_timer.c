/*
 * stm32f401xx_timer.h
 *
 *  Created on: Jun 10, 2023
 *      Author: Huy Hoang
 */

#ifndef INC_STM32F401XX_GPIO_TIMER_H_
#define INC_STM32F401XX_GPIO_TIMER_H_

#include "../../../drivers/STM32F401RE_StdPeriph_Driver/inc/stm32f401xx_timer.h"

static volatile uint32_t g_MiliSecClock = 0;

/**
 * @func   TimerInit
 * @brief  None
 * @param  None
 * @retval None
 */
void TimerInit(void)
{
    RCC_ClocksTypeDef RCC_Clocks;

    RCC_GetClocksFreq(&RCC_Clocks);                     // get the frequency of the clock sources used by the microcontroller.
    SysTick_Config(RCC_Clocks.SYSCLK_Frequency / 1000); // config the SysTick timer, means it will generate an interrupt every 1 millisecond
    NVIC_SetPriority(SysTick_IRQn, 1);                  // sets the priority of the SysTick interrupt using NVIC_SetPriority
}

/**
 * @func   GetMilSecTick
 * @brief  None
 * @param  None
 * @retval None
 */
uint32_t GetMilSecTick(void)
{
	return g_MiliSecClock;
}

/**
 * @brief	Delay nMili miliseconds.
*/
void delay_ms(uint32_t nMili)
{
	uint32_t start;
	if(0xFFFFFFFFul - g_MiliSecClock <= nMili)
	{
		g_MiliSecClock = 0;
		start = g_MiliSecClock;
	}
	else
	{
		start = g_MiliSecClock;
	}

	while(g_MiliSecClock - start < nMili);
}

/**
 * @brief	SysTicks Interrupt Handler. Increse g_wMilSecTickTimer every one second.
 */
void SysTick_Handler(void)
{
	g_MiliSecClock++;
}

#endif /* INC_STM32F401XX_GPIO_DRIVER_H_ */