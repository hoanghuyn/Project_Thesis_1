/*
 * stm32f401xx_timer.h
 *
 *  Created on: Jun 10, 2023
 *      Author: Huy Hoang
 */

#ifndef DRIVERS_STM32F401RE_STDPERIPH_DRIVER_INC_STM32F401XX_TIMER_H_
#define DRIVERS_STM32F401RE_STDPERIPH_DRIVER_INC_STM32F401XX_TIMER_H_

#include <stdint.h>

#include "../../../drivers/CMSIS/Include/misc.h"
#include "../../../drivers/STM32F401RE_StdPeriph_Driver/inc/stm32f401re_rcc.h"
#include "../../../drivers/STM32F401RE_StdPeriph_Driver/inc/stm32f401xx.h"

void TimerInit(void);
uint32_t GetMilSecTick(void);
void delay_ms(uint32_t nMili);

#endif /* DRIVERS_STM32F401RE_STDPERIPH_DRIVER_INC_STM32F401XX_TIMER_H_ */
