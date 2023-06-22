/*
 * stm32f401xx_spi_driver.c
 *
 *  Created on: May 31, 2023
 *      Author: Huy Hoang
 */

#include "../../../drivers/STM32F401RE_StdPeriph_Driver/inc/stm32f401xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/**
 * @brief This function enables or disables the peripheral clock for the provided SPI peripheral (pSPIx)
 *
 * @param pSPIx Pointer to the SPI peripheral register structure.
 * @param EnorDi Control parameter to enable or disable the peripheral clock. Use ENABLE_xx to enable and DISABLE_xx to disable.
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE_xx)
    {
        if (pSPIx == SPI1_PERI)
        {
            SPI1_PCLK_EN();
        }
        else if (pSPIx == SPI2_PERI)
        {
            SPI2_PCLK_EN();
        }
        else if (pSPIx == SPI3_PERI)
        {
            SPI3_PCLK_EN();
        }
        else if (pSPIx == SPI4_PERI)
        {
            SPI4_PCLK_EN();
        }
    }
    else // (EnorDi == DISABLE_xx)
    {
        if (pSPIx == SPI1_PERI)
        {
            SPI1_PCLK_DI();
        }
        else if (pSPIx == SPI2_PERI)
        {
            SPI2_PCLK_DI();
        }
        else if (pSPIx == SPI3_PERI)
        {
            SPI3_PCLK_DI();
        }
        else if (pSPIx == SPI4_PERI)
        {
            SPI4_PCLK_DI();
        }
    }
}

/******************************************************************************
 * @fn      		  - SPIx_GPIOInits
 *
 * @brief             - Initializes GPIO pins for the specified SPI peripheral.
 *
 * @param             - pSPIx: Pointer to the SPI peripheral (SPI_RegDef_t)
 *
 * @return            - None
 */
void SPIx_GPIOInits(SPI_RegDef_t *pSPIx)
{
    GPIO_Handle_t SPIPins;

    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    if (pSPIx == SPI1_PERI)
    {
        SPIPins.pGPIOx = GPIOA_PERI;

        // NSS - PA4
        SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
        GPIO_Init(&SPIPins);

        // SCLK - PA5
        SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
        GPIO_Init(&SPIPins);

        // MISO - PA6
        SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
        GPIO_Init(&SPIPins);

        // MOSI - PA7
        SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
        GPIO_Init(&SPIPins);
    }
    else if (pSPIx == SPI2_PERI)
    {
        SPIPins.pGPIOx = GPIOB_PERI;

        // NSS - PB12
        SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
        GPIO_Init(&SPIPins);

        // SCLK - PB13
        SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
        GPIO_Init(&SPIPins);

        // MISO - PB14
        SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
        GPIO_Init(&SPIPins);

        // MOSI - PB15
        SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
        GPIO_Init(&SPIPins);
    }
    else if (pSPIx == SPI3_PERI)
    {
    	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 6;
        // NSS - PA15
        SPIPins.pGPIOx = GPIOA_PERI;
        SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
        GPIO_Init(&SPIPins);

        SPIPins.pGPIOx = GPIOB_PERI;

        // SCLK - PB3
        SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
        GPIO_Init(&SPIPins);

        // MISO - PB4
        SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
        GPIO_Init(&SPIPins);

        // MOSI - PB5
        SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
        GPIO_Init(&SPIPins);
    }
    else if (pSPIx == SPI4_PERI)
    {
        // STM32F401RE Nucleo board not provided pin avaiblable for SPI4
    }
}

/**
 * @brief Initializes the SPI peripheral with the provided configuration.
 *
 * @param pSPIHandle Pointer to the SPI handle structure.
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    // Peripheral clock enable
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE_xx);

    // First lets configure the SPI_CR1 register
    uint32_t temp_reg = 0;

    // 1. Configure the device mode
    temp_reg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

    // 2. Configure the bus config
    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        // BIDIMODE bit should be cleared
        temp_reg &= ~(1 << SPI_CR1_BIDIMODE);
    } else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        // BIDIMODE bit should be set
        temp_reg |= (1 << SPI_CR1_BIDIMODE);
    } else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_S_RXONLY)
    {
        // BIDIMODE bit should be cleared
        temp_reg &= ~(1 << SPI_CR1_BIDIMODE);
        // RxOnly bit must be set
        temp_reg |= (1 << SPI_CR1_RXONLY);
    }

    // 3. Configure the SPI Serial Clock Speed (baudrate)
    temp_reg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

    // 4. Configure the Data Frame Format
    temp_reg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

    // 5. Configure the CPOL (Clock Polarity)
    temp_reg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL; 

    // 6. Configure the CPHA (Clock Phase)
    temp_reg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

    // 7. Configure the SSM (Slave Select Management)
    temp_reg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

    // Write configuration value into SP1_CR1 register.
    pSPIHandle->pSPIx->CR1 = temp_reg;
}

/**
 * @brief This function resets the provided SPI peripheral (pSPIx) to its default state by
 * invoking the corresponding register reset function based on the given SPI peripheral.
 * 
 * @param pSPIx Pointer to the SPI peripheral register structure.
*/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
    if (pSPIx == SPI1_PERI) 
    {
        SPI1_REG_RESET();
    } else if (pSPIx == SPI2_PERI)
    {
        SPI2_REG_RESET();
    } else if (pSPIx == SPI3_PERI) 
    {
        SPI3_REG_RESET();
    } else if (pSPIx == SPI4_PERI) 
    {
        SPI4_REG_RESET();
    }
}

/**
 * @brief Checks the status of a specific flag in the status register of the SPI peripheral.
 *
 * @param pSPIx Pointer to the SPI peripheral register structure.
 * @param flag_name The name of the flag to check.
 * 
 * @return Returns FLAG_SET if the flag is set, FLAG_RESET otherwise.
 */
_Bool SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flag_name)
{
    if(pSPIx->SR & flag_name)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

/******************************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - Sends data over SPI in blocking mode (or polling based).
 *
 * @param[in]         - pSPIx: Pointer to the SPI peripheral register structure.
 * @param[in]         - pTxBuffer: Pointer to the transmit buffer containing the data to be sent.
 * @param[in]         - Len: Length of the data to be sent in bytes.
 *
 * @return            - None
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while(Len > 0)
    {
        // 1. Wait until TXE is set (Transmit buffer empty)
        while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

        // 2. Check the DFF bit in CR1
        if( (pSPIx->CR1 & (1 << SPI_CR1_DFF)) )  // if(16 bits DFF)
        {   
            // 1. Load the data into the Data Register
            pSPIx->DR = *((uint16_t*)pTxBuffer);    // *(pTxBuffer) is just dereferencing uint8 pointer type
            Len -= 2;
            (uint16_t*)pTxBuffer++;
        } else  // 8 bits DFF
        {
            pSPIx->DR = *pTxBuffer;
            Len--;
            pTxBuffer++;
        }
    }
}

/******************************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - Receive data via SPI.
 *
 * @param[in]         - pSPIx: Pointer to the SPI peripheral register structure.
 * @param[in]         - pRxBuffer: Pointer to the receive buffer containing the data to be sent.
 * @param[in]         - Len: Length of the data to be receive in bytes.
 *
 * @return            - None
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until RX buffer is non empty (RXNE is set)
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)  == FLAG_RESET);

        // 2. check the DFF bit in CR1
        if((pSPIx->CR1 & (1 << SPI_CR1_DFF)))
        {
            // 16 bit DFF
            // 1. load the data from DR to Rxbuffer address
            *((uint16_t*)pRxBuffer) = pSPIx->DR;
            Len -= 2;
            (uint16_t*)pRxBuffer++;
        }
        else
        {
            // 8 bit DFF
            *(pRxBuffer) = pSPIx->DR;
            Len--;
            pRxBuffer++;
        }
    }
}

/******************************************************************************
 * @fn      		  - SPI_SendDataIT
 *
 * @brief             - Sends data using SPI with interrupt-based transmission
 *
 * @param[in]         - pSPIHandle: Pointer to the SPI handle structure
 * @param[in]         - pTxBuffer: Pointer to the transmit buffer containing the data to be sent
 * @param[in]         - Len: Length of the data to be sent in bytes
 *
 * @return            - Current state of the SPI transmission
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1 . Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2.  Mark the SPI state as busy in transmission so that
		//    no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );

	}
	return state;
}

/******************************************************************************
 * @fn      		  - SPI_ReceiveDataIT
 *
 * @brief             - Initiates SPI data reception using interrupt mode
 *
 * @param[in]         - pSPIHandle: Pointer to the SPI handle structure
 * @param[in]         - pRxBuffer:  Pointer to the receive buffer where the received data will be stored
 * @param[in]         - Len: Length of the data to be received in bytes
 *
 * @return            - Current state of the SPI reception
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1 . Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2.  Mark the SPI state as busy in reception so that
		//    no other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );
	}
	return state;
}

/******************************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             - Enables or disables the SPI peripheral.
 *
 * @param[in]         - Pointer to the SPI peripheral register structure
 * @param[in]         - Enable or Disable command (ENABLE_xx or DISABLE_xx)
 *
 * @return            - None
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE_xx)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    } else  // EnOrDi == DISABLE_xx
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}

/******************************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             - Configures the SSI (Slave Select) signal of the SPI peripheral.
 *
 * @param[in]         - Pointer to the SPI peripheral register structure
 * @param[in]         - Enable or Disable command (ENABLE_xx or DISABLE_xx)
 *
 * @return            - None
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE_xx)
    {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    }
    else // EnOrDi == DISABLE_xx
    {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}

/******************************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             - Configures the SSOE signal of the SPI peripheral.
 *
 * @param[in]         - Pointer to the SPI peripheral register structure
 * @param[in]         - Enable or Disable command (ENABLE_xx or DISABLE_xx)
 *
 * @return            - None
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE_xx)
    {
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    }
    else // EnOrDi == DISABLE_xx
    {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}

/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 *
 * @brief             - Configures the interrupt for the specified IRQ number of the SPI peripheral
 *
 * @param[in]         - IRQNumber: IRQ number to configure
 * @param[in]         - EnorDi: Enable or disable the interrupt (ENABLE_xx or DISABLE_xx)
 *
 * @return            - None
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE_xx)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
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
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             - Configures the priority for the specified IRQ number of the SPI peripheral
 *
 * @param[in]         - IRQNumber: IRQ number to configure
 * @param[in]         - IRQPriority: Priority value to set for the IRQ
 *
 * @return            - None
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber % 4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*( NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );
}

/******************************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             - Handles the SPI interrupt events
 *
 * @param             - pHandle: Pointer to the SPI handle structure
 *
 * @return            - None
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	//first lets check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if( temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	// check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	// check for ovr flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
		//handle ovr error
		spi_ovr_err_interrupt_handle(pHandle);
	}    
}

// some helper function implementations

/******************************************************************************
 * @fn      		  - spi_txe_interrupt_handle
 *
 * @brief             - Handles the TXE (Transmit Buffer Empty) interrupt for SPI
 *
 * @param             - pHandle: Pointer to the SPI handle structure
 *
 * @return            - None
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    // check the DFF bit in CR1
    if ((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)))
    {
        // 16 bit DFF
        // 1. load the data in to the DR
        pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);
        pSPIHandle->TxLen -= 2;
        (uint16_t *)pSPIHandle->pTxBuffer++;
    }
    else
    {
        // 8 bit DFF
        pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
        pSPIHandle->TxLen--;
        pSPIHandle->pTxBuffer++;
    }

    if (!pSPIHandle->TxLen)
    {
        // TxLen is zero , so close the spi transmission and inform the application that
        // TX is over.

        // this prevents interrupts from setting up of TXE flag
        SPI_CloseTransmission(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
    }
}

/******************************************************************************
 * @fn      		  - spi_rxne_interrupt_handle
 *
 * @brief             - Handles the SPI RXNE (Receive Buffer Not Empty) interrupt
 *
 * @param             - pHandle: Pointer to the SPI handle structure
 *
 * @return            - None
 */
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    // do rxing as per the dff
    if (pSPIHandle->pSPIx->CR1 & (1 << 11))
    {
        // 16 bit
        *((uint16_t *)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen -= 2;
        (uint16_t*)pSPIHandle->pRxBuffer++;
    }
    else
    {
        // 8 bit
        *(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
        pSPIHandle->RxLen--;
        pSPIHandle->pRxBuffer++;
    }

    if (!pSPIHandle->RxLen)
    {
        // reception is complete
        SPI_CloseReception(pSPIHandle);
        SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
    }
}

/******************************************************************************
 * @fn      		  - spi_ovr_err_interrupt_handle
 *
 * @brief             - Handles the SPI overrun error interrupt
 *
 * @param             - pHandle: Pointer to the SPI handle structure
 *
 * @return            - None
 */
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
    uint8_t temp;
    // 1. clear the ovr flag
    if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
    {
        temp = pSPIHandle->pSPIx->DR;
        temp = pSPIHandle->pSPIx->SR;
    }
    (void)temp;     // avoid unused variable warning
    // 2. inform the application
    SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

/******************************************************************************
 * @fn      		  - SPI_CloseTransmission
 *
 * @brief             - Closes the SPI transmission and resets relevant parameters
 *
 * @param             - pHandle: Pointer to the SPI handle structure
 *
 * @return            - None
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

/******************************************************************************
 * @fn      		  - SPI_CloseReception
 *
 * @brief             - Closes the SPI reception by resetting relevant parameters
 *
 * @param             - pHandle: Pointer to the SPI handle structure
 *
 * @return            - None
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
    pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    pSPIHandle->pRxBuffer = NULL;
    pSPIHandle->RxLen = 0;
    pSPIHandle->RxState = SPI_READY;
}

/******************************************************************************
 * @fn      		  - SPI_ClearOVRFlag
 *
 * @brief             - Clears the overrun (OVR) flag in the SPI status register
 *
 * @param             - pSPIx: Pointer to the SPI peripheral register structure
 *
 * @return            - None
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
    uint8_t temp;
    temp = pSPIx->DR;
    temp = pSPIx->SR;
    (void)temp;     // avoid unused variable warning
}

/**
 * @brief Default SPI application event callback function.
 * This is a weak implementation that can be overridden by the user application.
 * It is called by the SPI driver to notify the application about specific SPI events.
 *
 * @param[in]   pSPIHandle Pointer to the SPI handle structure
 * @param[in]   AppEv      SPI application event, This parameter can be a value of @Possible_SPI_Application_events.
 * 
 * @return      None
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
    // This is a weak implementation . the user application may override this function.
}

/******************************************************************************/
/*                          PERSONAL PURPOSE FUNCTION                         */
/******************************************************************************/
void SPI2_master_FD_init(SPI_Handle_t *SPI_handle)
{
    SPI_handle->pSPIx = SPI2_PERI;
    SPI_handle->SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI_handle->SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI_handle->SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV256;
    SPI_handle->SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI_handle->SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI_handle->SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI_handle->SPIConfig.SPI_SSM = SPI_SSM_DI;

    SPI_Init(SPI_handle);
}

void SPI2_slave_FD_init(SPI_Handle_t *SPI_handle)
{
    SPI_handle->pSPIx = SPI2_PERI;
    SPI_handle->SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI_handle->SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_SLAVE;
    SPI_handle->SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV256;
    SPI_handle->SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI_handle->SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI_handle->SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI_handle->SPIConfig.SPI_SSM = SPI_SSM_DI;

    SPI_Init(SPI_handle);
}

void SPI1_master_FD_init(SPI_Handle_t *SPI_handle)
{
    SPI_handle->pSPIx = SPI1_PERI;
    SPI_handle->SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI_handle->SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI_handle->SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV256;
    SPI_handle->SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI_handle->SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI_handle->SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI_handle->SPIConfig.SPI_SSM = SPI_SSM_DI;

    SPI_Init(SPI_handle);
}

void SPI1_slave_FD_init(SPI_Handle_t *SPI_handle)
{
    SPI_handle->pSPIx = SPI1_PERI;
    SPI_handle->SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI_handle->SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_SLAVE;
    SPI_handle->SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV256;
    SPI_handle->SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI_handle->SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI_handle->SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI_handle->SPIConfig.SPI_SSM = SPI_SSM_DI;

    SPI_Init(SPI_handle);
}
