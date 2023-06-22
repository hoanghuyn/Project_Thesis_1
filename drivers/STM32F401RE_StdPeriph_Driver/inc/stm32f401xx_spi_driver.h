/*
 * stm32f401xx_spi_driver.h
 *
 *  Created on: May 31, 2023
 *      Author: Huy Hoang
 */

#ifndef DRIVERS_STM32F401RE_STDPERIPH_DRIVER_INC_STM32F401XX_SPI_DRIVER_H_
#define DRIVERS_STM32F401RE_STDPERIPH_DRIVER_INC_STM32F401XX_SPI_DRIVER_H_

#include "../../../drivers/STM32F401RE_StdPeriph_Driver/inc/stm32f401xx.h"

/*
 *  Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;		/*!< This parameter can be a value of @SPI_DeviceMode */
	uint8_t SPI_BusConfig;		/*!< Bus configuration. This parameter can be a value of @SPI_BusConfig */
	uint8_t SPI_SclkSpeed;		/*!< Serical Clock Speed. This parameter can be a value of @SPI_SclkSpeed */
	uint8_t SPI_DFF;			/*!< Data frame format. This parameter can be a value of @SPI_DFF */
	uint8_t SPI_CPOL;			/*!< Clock Polarity. This parameter can be a value of @SPI_CPOL */
	uint8_t SPI_CPHA;			/*!< Clock Phase. This parameter can be a value of @SPI_CPHA */
	uint8_t SPI_SSM;			/*!< Software slave management. This parameter can be a value of @SPI_SSM */
} SPI_Config_t;

/*
 *Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t 	*pSPIx;     /*!< This holds the base address of SPIx(x:1,2,3,4) peripheral >*/
	SPI_Config_t 	SPIConfig;
	uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* !< To store Tx len > */
	uint32_t 		RxLen;		/* !< To store Rx len > */
	uint8_t 		TxState;	/* !< To store Tx state.This parameter can be a value of @SPI_application_states > */
	uint8_t 		RxState;	/* !< To store Rx state.This parameter can be a value of @SPI_application_states > */
} SPI_Handle_t;

/**
 * @SPI_application_states
 */
#define SPI_READY 					0
#define SPI_BUSY_IN_RX 				1
#define SPI_BUSY_IN_TX 				2

/*
 * @Possible_SPI_Application_events
 */
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4

/**
 * @SPI_DeviceMode	SPI_CR1[bit2], refer Reference Manual page 602
 */
#define SPI_DEVICE_MODE_SLAVE		0
#define SPI_DEVICE_MODE_MASTER		1	

/**
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD			1	// Full-duplex
#define SPI_BUS_CONFIG_HD			2	// Half-duplex
#define SPI_BUS_CONFIG_S_RXONLY		3	// Simplex with Rx Only

/**
 * @SPI_SclkSpeed	SPI_CR1[5:3] = BR[2:0], refer Reference Manual page 602.
 */
#define SPI_SCLK_SPEED_DIV2		0
#define SPI_SCLK_SPEED_DIV4		1
#define SPI_SCLK_SPEED_DIV8		2
#define SPI_SCLK_SPEED_DIV16	3
#define SPI_SCLK_SPEED_DIV32	4
#define SPI_SCLK_SPEED_DIV64	5
#define SPI_SCLK_SPEED_DIV128	6
#define SPI_SCLK_SPEED_DIV256	7

/**
 * @SPI_DFF		SPI_CR1[11] = DFF, refer RM page 602.
 */
#define SPI_DFF_8BITS	0
#define SPI_DFF_16BITS	1

/**
 * @SPI_CPOL	SPI_CR1[1] = CPOL, refer RM page 604.
 */
#define SPI_CPOL_LOW	0	// CK to 0 when idle
#define SPI_CPOL_HIGH	1	// CK to 1 when idle

/**
 * @SPI_CPHA	SPI_CR1[0] = CPHA, refer RM page 604.
 */
#define SPI_CPHA_LOW	0	// The first clock transition is the first data capture edge
#define SPI_CPHA_HIGH	1	// The second clock transition is the first data capture edge

/**
 * @SPI_SSM
 */
#define SPI_SSM_DI	0	// Software slave management disabled
#define SPI_SSM_EN	1	// Software slave management enabled

/*
 * SPI related status flags definitions
 */

#define SPI_TXE_FLAG    ( 1 << SPI_SR_TXE)		/*!< Give the masking info of the TXE field in the SR register >*/
#define SPI_RXNE_FLAG   ( 1 << SPI_SR_RXNE)		/*!< Give the masking info of the RXNE field in the SR register >*/
#define SPI_BUSY_FLAG   ( 1 << SPI_SR_BSY)		/*!< Give the masking info of the BSY field in the SR register >*/

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

/* Peripheral Clock setup *****************************************************************/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/* Init and De-init ***********************************************************************/

void SPIx_GPIOInits(SPI_RegDef_t *pSPIx);
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/* Data Send and Receive ******************************************************************/

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/* IRQ Configuration and ISR handling *****************************************************/

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/* Other Peripheral Control APIs **********************************************************/

_Bool SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flag_name);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/* Application callback *******************************************************************/

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

/******************************************************************************/
/*                          PERSONAL PURPOSE FUNCTION                         */
/******************************************************************************/
void SPI2_master_FD_init(SPI_Handle_t *SPI2handle);

#endif /* DRIVERS_STM32F401RE_STDPERIPH_DRIVER_INC_STM32F401XX_SPI_DRIVER_H_ */
