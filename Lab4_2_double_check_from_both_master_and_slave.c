/*******************************************************************************
 * File name: Lab4_1_SPI2_master_SPSP_slave_communication.c
 *
 * Description: This is the Lab 4.2 exercise in the IOT_303x course. The purpose
 * of this exercise is to establish communication between two SPI peripherals,
 * SPI1 and SPI2, on the STM32F401RE board. The different between Lab 4.1 and Lab
 * 4.2 is the 4.2 Lab require double check from Master whether Slave receive right
 * data and handle it or not.
 *
 * Author: Nguyen Huy Hoang
 *
 * Last Changed By:  $Author: Nguyen Huy Hoang $
 * Revision:         $Revision: 1.0 $
 * Last Changed:     $Date: $Jun 18, 2023
 *
 ******************************************************************************/
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include<stdio.h>
#include<string.h>
#include "stm32f401xx.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
#define SLAVE_DATA_CHECK        0x4C
#define MASTER_DATA_CHECK       0x4D
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/
SPI_Handle_t SPI1handle;
SPI_Handle_t SPI3handle;
/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
void SPI1_Inits(void)
{
	SPI1handle.pSPIx = SPI1_PERI;
	SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_SLAVE;
	SPI1handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32; // APB1 = APB2/2
	SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI1handle);
}

void SPI3_Inits(void)
{
    SPI3handle.pSPIx = SPI3_PERI;
    SPI3handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI3handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI3handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32;
    SPI3handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI3handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI3handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI3handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

    SPI_Init(&SPI3handle);
}

void Button_Led_Inits(void)
{
    GPIO_Handle_t GPIO_Btn, GPIO_Led;

    // this is btn gpio configuration
    GPIO_Btn.pGPIOx = GPIOC_PERI;
    GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIO_Btn);

    // this is led gpio configuration
    GPIO_Led.pGPIOx = GPIOA_PERI;
    GPIO_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    GPIO_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GPIO_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&GPIO_Led);	// Led Green 0

    GPIO_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
    GPIO_Init(&GPIO_Led);	// Led Green 1

    GPIO_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
    GPIO_Init(&GPIO_Led);		// Led RED 0
}

/******************************************************************************/
int main(void)
{
	uint8_t dummy_read = 0xff;
	uint8_t dummy_write = 0xff;
	uint8_t g_master_send = SLAVE_DATA_CHECK;
	uint8_t g_slave_send = MASTER_DATA_CHECK;
	volatile uint8_t g_slave_rcvd = 0;
	volatile uint8_t g_master_rcvd = 0;

	TimerInit();

    SPIx_GPIOInits(SPI1_PERI);  // Slave-GPIO Inits
    SPIx_GPIOInits(SPI3_PERI);  // Master-GPIO Inits

    SPI1_Inits();
    SPI3_Inits();

    SPI_SSOEConfig(SPI3_PERI, ENABLE_xx);

    Button_Led_Inits();

//    initialise_monitor_handles();   // use print debug
    SPI_IRQInterruptConfig(IRQ_NO_SPI1, ENABLE_xx);		// Enable NVIC for interrupts coming from SPI1
    SPI_IRQInterruptConfig(IRQ_NO_SPI3, ENABLE_xx);		// Enable NVIC for interrupts coming from SPI1


    while(1)
    {
        if(GPIO_CheckButtonPressedActiveLow(GPIOC_PERI, GPIO_PIN_NO_13) == BTN_PRESSED)
        {
            SPI_PeripheralControl(SPI1_PERI, ENABLE_xx);
        	SPI_PeripheralControl(SPI3_PERI, ENABLE_xx);

            // Master send data
            while(SPI_SendDataIT(&SPI3handle,&g_master_send,1) == SPI_BUSY_IN_TX);		// Master send data
            while(SPI_ReceiveDataIT(&SPI3handle, &dummy_read, 1) == SPI_BUSY_IN_RX);	// Master read dummy data

            // Slave receive data
            while(SPI_ReceiveDataIT(&SPI1handle,&g_slave_rcvd,1) == SPI_BUSY_IN_RX);  // Slave receive data

            // confirm SPI is not busy
            while(SPI_GetFlagStatus(SPI3_PERI,SPI_BUSY_FLAG));
            while(SPI_GetFlagStatus(SPI1_PERI,SPI_BUSY_FLAG));

            SPI_PeripheralControl(SPI1_PERI, DISABLE_xx);
        	SPI_PeripheralControl(SPI3_PERI, DISABLE_xx);
        }

		if(g_slave_rcvd == SLAVE_DATA_CHECK)
		{
			for(uint8_t i=0; i < 10; i++)
			{
				GPIO_ToggleOutputPin(GPIOA_PERI, GPIO_PIN_NO_0);
				GPIO_ToggleOutputPin(GPIOA_PERI, GPIO_PIN_NO_11);
				delay_ms(500);
			}

			GPIO_WriteToOutputPin(GPIOA_PERI, GPIO_PIN_NO_0, LOW);
            GPIO_WriteToOutputPin(GPIOA_PERI, GPIO_PIN_NO_11, LOW);

            g_slave_rcvd = 0;

            SPI_PeripheralControl(SPI1_PERI, ENABLE_xx);
        	SPI_PeripheralControl(SPI3_PERI, ENABLE_xx);

			while(SPI_SendDataIT(&SPI1handle,&g_slave_send,1) == SPI_BUSY_IN_TX);		// Slave send data
			while(SPI_ReceiveDataIT(&SPI1handle, &dummy_read, 1) == SPI_BUSY_IN_RX);	// Slave read dummy data

            // Master receive data
            while(SPI_SendDataIT(&SPI3handle,&dummy_write,1) == SPI_BUSY_IN_TX);		// Master send dummy byte to fetch data
            while(SPI_ReceiveDataIT(&SPI3handle, &g_master_rcvd, 1) == SPI_BUSY_IN_RX);	// Master read data

            // confirm SPI is not busy
            while(SPI_GetFlagStatus(SPI3_PERI,SPI_BUSY_FLAG));
            while(SPI_GetFlagStatus(SPI1_PERI,SPI_BUSY_FLAG));

            SPI_PeripheralControl(SPI1_PERI, DISABLE_xx);
        	SPI_PeripheralControl(SPI3_PERI, DISABLE_xx);

            g_slave_rcvd = 0;
        }
		if(g_master_rcvd == MASTER_DATA_CHECK)
		{
			// Toggle Led RED 1 time
			GPIO_WriteToOutputPin(GPIOA_PERI, GPIO_PIN_NO_1, HIGH);
			delay_ms(500);
			GPIO_WriteToOutputPin(GPIOA_PERI, GPIO_PIN_NO_1, LOW);
			delay_ms(500);

			g_master_rcvd = 0;
		}
    }
    return 0;
}

void SPI1_IRQHandler(void)
{
    SPI_IRQHandling(&SPI1handle);
}

void SPI3_IRQHandler(void)
{
    SPI_IRQHandling(&SPI3handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
    // Nothing to do
}
