/*
 * stm32f401xx_gpio_driver.c
 *
 *  Created on: Mar 16, 2023
 *      Author: Huy Hoang
 */

//#include "../../../drivers/STM32F401RE_StdPeriph_Driver/inc/stm32f401xx_gpio_driver.h"
#include "stm32f401xx.h"

/**
 * @func   GPIO_PeriClockControl
 * @brief  Enable/Disable Clock for the given GPIO
 * @param  *pGPIOx  Base address of GPIOx
 * @param  EnorDi   ENABLE or DISABLE_xx macros
 * @retval None
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE_xx)
    {
        if (pGPIOx == GPIOA_PERI)
        {
            GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB_PERI)
        {
            GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC_PERI)
        {
            GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD_PERI)
        {
            GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE_PERI)
        {
            GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOH_PERI)
        {
            GPIOH_PCLK_EN();
        }
    }
    else // (EnorDi == DISABLE_xx)
    {
        if (pGPIOx == GPIOA_PERI)
        {
            GPIOA_PCLK_DI();
        }
        else if (pGPIOx == GPIOB_PERI)
        {
            GPIOB_PCLK_DI();
        }
        else if (pGPIOx == GPIOC_PERI)
        {
            GPIOC_PCLK_DI();
        }
        else if (pGPIOx == GPIOD_PERI)
        {
            GPIOD_PCLK_DI();
        }
        else if (pGPIOx == GPIOE_PERI)
        {
            GPIOE_PCLK_DI();
        }
        else if (pGPIOx == GPIOH_PERI)
        {
            GPIOH_PCLK_DI();
        }
    }
}

/******************************** Init and De-init ****************************************/
/**
 * @func   GPIO_Init
 * @brief  Initializes GPIO port and pin
 * @param  *pGPIOHandle  Pointer to GPIO Handle structure
 * @retval None
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0;  // temp. registers

    // Enable the peripheral clock
    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE_xx);

    // 1. Configure the mode of the GPIO pin

    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        // The non interrupt mode
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing
        pGPIOHandle->pGPIOx->MODER |= temp;  // setting
    }
    else    
    {
        // Interrupt modes
        if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
        {
            // 1. Configure the FTSR (Falling Trigger Selection Register)
            EXTI_PERI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            // Clear the corresponding RTSR bit 
            EXTI_PERI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

        } else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
        {
            // 1. Configure the RTSR (Raising Trigger Selection Register)
            EXTI_PERI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); 
            // Clear the corresponding FTSR bit
            EXTI_PERI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

        } else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
        {
            // 1. Configure both FTSR and RTSR (Falling and Raising Trigger Selection Register)
            EXTI_PERI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI_PERI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        }

        // 2. Configure the GPIO port selection in SYSCFG_EXTICR
        uint8_t tempReg = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
        uint8_t tempPin = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
        uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
        SYSCFG_PCLK_EN();
        SYSCFG_PERI->EXTICR[tempReg] |= portcode << ( tempPin * 4);

    
        // 3. Enable the EXTI interrupt delivery using IMR (Interrupt Mask Register)
        EXTI_PERI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }

    // 2. Configure the speed of the GPIO pin
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;   // setting

    // 3. Configure the pupd (pull-up/pull-down) of the GPIO pin
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));    // clearing
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    // 4. Configure the optype (output type)
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUT)
    {
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
        pGPIOHandle->pGPIOx->OTYPER |= temp;
    }

    // 5. Configure the alt functionality
    if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
    {
        // Configure the alt function registers.
        uint8_t tempReg = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8);   // if Pin <= 7 (tempReg = 0); otherwise (tempReg = 1)
        uint8_t tempPin = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);   // determine pin to operate bitwise shift operator
        
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * tempPin));

        pGPIOHandle->pGPIOx->AFR[tempReg] &= ~(0xf << (4 * tempPin)); // clearing
        pGPIOHandle->pGPIOx->AFR[tempReg] |= temp;
    }
}

/**
 * @func   GPIO_DeInit
 * @brief  Deinitializes the registers of the given GPIO. Sending relavant registers
 *         back to its reset state.
 * @param  *pGPIOx  The base address of the peripheral
 * @retval None
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if (pGPIOx == GPIOA_PERI)
    {
        GPIOA_REG_RESET();
    } else if (pGPIOx == GPIOB_PERI)
    {
        GPIOB_REG_RESET();
    } else if (pGPIOx == GPIOC_PERI)
    {
        GPIOC_REG_RESET();
    } else if (pGPIOx == GPIOD_PERI)
    {
        GPIOD_REG_RESET();
    } else if (pGPIOx == GPIOE_PERI)
    {
        GPIOE_REG_RESET();
    } else if (pGPIOx == GPIOH_PERI)
    {
        GPIOH_REG_RESET();
    }
}

/******************************** Data read and write ************************************/
/**
 * @func   GPIO_ReadFromInputPin
 * @brief  Read value from the given input pin of the given GPIO
 * @param  *pGPIOx  The base address of the GPIO port want to read
 * @param  PinNumber  Number of the pin to read
 * @retval 0 or 1
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t value;
    // value = ( (pGPIOx->IDR & (1 << PinNumber)) ? 1 : 0);
    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
    
    return value;
}

/**
 * @func   GPIO_ReadFromInputPort
 * @brief  Read value from the given port of a GPIO
 * @param  *pGPIOx  The base address of the GPIO port want to read
 * @retval Content of input data register
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t value;

    value = (uint16_t)pGPIOx->IDR;

    return value;
}

/**
 * @func   		GPIO_WriteToOutputPin
 * @brief  		Write to an output pin
 * @param[in]  	*pGPIOx  The base address of the GPIO port to write
 * @param[in]  	PinNumber  The pin to write
 * @param[in]  	Value  Value to write: SET(1) or RESET(0)
 * @retval 		None
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if (Value == GPIO_PIN_SET)
    {
        // write 1 to the output data register at the bit field corresponding to the pin number
        pGPIOx->ODR |= (1 << PinNumber);
    }
    else
    {
        // write 0
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

/**
 * @func   		GPIO_WriteToOutputPort
 * @brief  		Write to an output port
 * @param[in]  	pGPIOx  The base address of the GPIO port to write
 * @param[in]  	Value  Write value: 16 bit value
 * @retval None
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR = Value;
}

/**
 * @func   		GPIO_ToggleOutputPin
 * @brief  		Toggle an output pin
 * @param[in]  	*pGPIOx  The base address of the GPIO port which contains the pin
 * @param[in]  	PinNumber  Number of the pin to toggle
 * @retval 		None
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR = pGPIOx->ODR ^ (1 << PinNumber);
}

/**
 * @brief    Check whether a button connected to a GPIO pin is pressed or not (active low state).
 *
 * @param[in] pGPIOx: Pointer to the GPIO peripheral register structure.
 * @param[in] PinNumber: Pin number of the GPIO pin connected to the button.
 * 
 * @return BTN_PRESSED (1) if the button is pressed and released, BTN_NOT_PRESSED (0) otherwise.
 */
_Bool GPIO_CheckButtonPressedActiveLow(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    if(GPIO_ReadFromInputPin(pGPIOx, PinNumber) == LOW)
    {
        delay_ms(20);
        if(GPIO_ReadFromInputPin(pGPIOx, PinNumber) == LOW)
        {
            while (GPIO_ReadFromInputPin(pGPIOx, PinNumber) == LOW);
            return BTN_PRESSED;
        }
    }
    return BTN_NOT_PRESSED;
}

/**
 * @brief    Check whether a button connected to a GPIO pin is pressed or not (active high state).
 *
 * @param[in] pGPIOx: Pointer to the GPIO peripheral register structure.
 * @param[in] PinNumber: Pin number of the GPIO pin connected to the button.
 *
 * @return BTN_PRESSED (1) if the button is pressed and released, BTN_NOT_PRESSED (0) otherwise.
 */
_Bool GPIO_CheckButtonPressedActiveHigh(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    if (GPIO_ReadFromInputPin(pGPIOx, PinNumber) == HIGH)
    {
        delay_ms(20);
        if (GPIO_ReadFromInputPin(pGPIOx, PinNumber) == HIGH)
        {
            while (GPIO_ReadFromInputPin(pGPIOx, PinNumber) == HIGH);
            return BTN_PRESSED;
        }
    }
    return BTN_NOT_PRESSED;
}

/******************************** IRQ Configuration and ISR handling ******************/

/**
 * @brief   	Config (enable/disable) the interrupt what you need
 *
 * @param[in]   IRQNumber:  value in @IRQNumber
 * @param[in]   EnorDi  :   ENABLE_xx/DISABLE_xx macro
 *
 * @return  	None
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE_xx)
	{
		if(IRQNumber < 32) // IRQ 0 to 31
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber >= 32 && IRQNumber < 64 ) // 32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )  // 64 to 95
		{
			//program ISER2 register
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
    else // (EnorDi == DISABLE_xx)
    {
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
}

/**
 * @func   GPIO_IRQInterruptConfig
 * @brief  Config the priority of given IRQ
 * @param  IRQNumber
 * @param  IRQPriority  Config the priority of this IRQ
 * @retval None
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
    uint8_t iprx = IRQNumber / 4;   // each register contains 4 IRQnumber
    uint8_t iprx_section = IRQNumber % 4;
    uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);

    *(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/**
 * @func   GPIO_IRQHandling
 * @brief  Clears the pending interrupt for a specific GPIO pin in the EXTI peripheral.
 * @param  PinNumber    The pin number for which the interrupt is to be handled and cleared.
 * @retval None
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
    // 1. Clear the EXTI Pending Register corresponding to the pin number
    if( EXTI_PERI->PR & (1 << PinNumber) )
    {
        // set to '1' to clear pended bit (Pending Register - EXTI_PR)
        EXTI_PERI->PR |= (1 << PinNumber);
    }
}
