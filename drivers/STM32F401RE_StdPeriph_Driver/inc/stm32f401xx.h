/*
 * stm32f401xx.h
 *
 *  Created on: Mar 15, 2023
 *      Author: Huy Hoang (py)
 */
#define HELLO 	0
#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stddef.h>
#include <stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex M4 Processor NVIC ISERx register Addresses
 * Based on Table 4-2 NVIC register summary (Cortex-M4 Devices Generic User Guide, page 219)
 */

#define NVIC_ISER0          ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__vo uint32_t*)0xE000E10C )

/*
 * ARM Cortex M4 Processor NVIC ICERx register Addresses
 */

#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18C)

/*
 * Cortex Mx Processor Priority Register Address Calculation
 * Based on Table 4-2 NVIC register summary (Cortex-M4 Devices Generic User Guide, page 219)
 */
#define NVIC_PR_BASE_ADDR 	((__vo uint32_t*)0xE000E400)


#define NO_PR_BITS_IMPLEMENTED      4   // 4 HIGHER bits implemented in 8 bits

/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000ul	/* Base address of FLASH memory*/
#define SRAM1_BASEADDR			0x20000000ul	/* Base address of SRAM1*/
#define ROM_BASEADDR			0x1FFF0000ul	/* Base address of System Memory*/
#define SRAM 					SRAM1_BASEADDR  /* Base address of SRAM*/

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR			0x40000000ul	/* Base address of Peripherals*/
#define APB1PERIPH_BASEADDR		0x40000000ul	/* Base address of Peripherals are hanging on APB1 bus*/
#define APB2PERIPH_BASEADDR		0x40010000ul	/* Base address of Peripherals are hanging on APB2 bus*/
#define AHB1PERIPH_BASEADDR		0x40020000ul	/* Base address of Peripherals are hanging on AHB1 bus*/
#define AHB2PERIPH_BASEADDR		0x50000000ul	/* Base address of Peripherals are hanging on AHB2 bus*/

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000ul)	/* Base address of GPIOA peripheral*/
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400ul)	/* Base address of GPIOB peripheral*/
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800ul)	/* Base address of GPIOC peripheral*/
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00ul)	/* Base address of GPIOD peripheral*/
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000ul)	/* Base address of GPIOE peripheral*/
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00ul)	/* Base address of GPIOH peripheral*/

#define CRC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3000ul)	/* Base address of Cyclic Redundancy Check peripheral*/
#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x3800ul)	/* Base address of Reset and Clock Control peripheral*/
#define FLASH_INTERFACE_BASEADDR	(AHB1PERIPH_BASEADDR + 0x3C00ul)	/* Base address of Flash interface peripheral*/

#define DMA1_BASEADDR				(AHB1PERIPH_BASEADDR + 0x6000ul)	/* Base address of DMA1 (Direct memory access 1) peripheral*/
#define DMA2_BASEADDR				(AHB1PERIPH_BASEADDR + 0x6400ul)	/* Base address of DMA2 (Direct memory access 2) peripheral*/

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400ul)	/* Base address of I2C1 peripheral*/
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800ul)	/* Base address of I2C2 peripheral*/
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00ul)	/* Base address of I2C3 peripheral*/

#define I2S2EXT_BASEADDR			(APB1PERIPH_BASEADDR + 0x3400ul)	/* Base address of I2S2ext peripheral*/
#define I2S3EXT_BASEADDR			(APB1PERIPH_BASEADDR + 0x4000ul)	/* Base address of I2S3ext peripheral*/

#define IWDG_BASEADDR				(APB1PERIPH_BASEADDR + 0x3000ul)	/* Base address of IWDG peripheral*/
#define PWR_BASEADDR				(APB1PERIPH_BASEADDR + 0x7000ul)	/* Base address of PWR peripheral*/
#define RTC_AND_BKP_BASEADDR		(APB1PERIPH_BASEADDR + 0x2800ul)	/* Base address of RTC & BKP peripheral*/

#define SPI2_I2S2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800ul)	/* Base address of SPI2/I2S2 peripheral*/
#define SPI3_I2S3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00ul)	/* Base address of SPI3/I2S3 peripheral*/
#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800ul)	/* Base address of SPI2/I2S2 peripheral*/
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00ul)	/* Base address of SPI3/I2S3 peripheral*/

#define TIM2_BASEADDR				(APB1PERIPH_BASEADDR + 0x0000ul)	/* Base address of TIM2 peripheral*/
#define TIM3_BASEADDR				(APB1PERIPH_BASEADDR + 0x0400ul)	/* Base address of TIM3 peripheral*/
#define TIM4_BASEADDR				(APB1PERIPH_BASEADDR + 0x0800ul)	/* Base address of TIM4 peripheral*/
#define TIM5_BASEADDR				(APB1PERIPH_BASEADDR + 0x0C00ul)	/* Base address of TIM5 peripheral*/

#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400ul)	/* Base address of USART2 peripheral*/
#define WWDG_BASEADDR				(APB1PERIPH_BASEADDR + 0x2C00ul)	/* Base address of WWDG peripheral*/


/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define ADC1_BASEADDR				(APB2PERIPH_BASEADDR + 0x2000ul)	/* Base address of ADC1 peripheral*/

#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00ul)	/* Base address of EXTI peripheral*/

#define SDIO_BASEADDR				(APB2PERIPH_BASEADDR + 0x2C00ul)	/* Base address of SDIO peripheral*/

#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000ul)	/* Base address of SPI1 peripheral*/
#define SPI4_BASEADDR				(APB2PERIPH_BASEADDR + 0x3400ul)	/* Base address of SPI4 peripheral*/

#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800ul)	/* Base address of SYSCFG peripheral*/

#define TIM1_BASEADDR				(APB2PERIPH_BASEADDR + 0x0000ul)	/* Base address of TIM1 peripheral*/
#define TIM10_BASEADDR				(APB2PERIPH_BASEADDR + 0x4400ul)	/* Base address of TIM10 peripheral*/
#define TIM11_BASEADDR				(APB2PERIPH_BASEADDR + 0x4800ul)	/* Base address of TIM11 peripheral*/
#define TIM9_BASEADDR				(APB2PERIPH_BASEADDR + 0x4000ul)	/* Base address of TIM9 peripheral*/

#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000ul)	/* Base address of USART1 peripheral*/
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400ul)	/* Base address of USART6 peripheral*/

/**********************************peripheral register definition structures **********************************/

/*
 * peripheral register definition structure for GPIO
 */
typedef struct
{
    __vo uint32_t MODER;            /*!< GPIO port mode register >                  Address offset: 0x00 */
    __vo uint32_t OTYPER;           /*!< GPIO port output type register >           Address offset: 0x04 */
    __vo uint32_t OSPEEDR;          /*!< GPIO port output speed register >          Address offset: 0x08 */
    __vo uint32_t PUPDR;            /*!< GPIO port pull-up/pull-down register >     Address offset: 0x0C */
    __vo uint32_t IDR;              /*!< GPIO port input data register  >           Address offset: 0x10 */
    __vo uint32_t ODR;              /*!< GPIO port output data register >           Address offset: 0x14 */
    __vo uint32_t BSRR;             /*!< GPIO port bit set/reset register >         Address offset: 0x18 */
    __vo uint32_t LCKR;             /*!< GPIO port configuration lock register  >   Address offset: 0x1C */
                                    /*AFR[0]: GPIO alternate function low register, Address offset: 0x20
                                      AFR[1]: GPIO alternate function low register, Address offset: 0x24 */
    __vo uint32_t AFR[2];
} GPIO_RegDef_t;

/*
 * peripheral register definition structure for RCC
 */
typedef struct
{
    __vo uint32_t CR;            /*!< RCC clock control register >                                  Address offset: 0x00 */
    __vo uint32_t PLLCFGR;       /*!< RCC PLL configuration register     				            Address offset: 0x04 */
    __vo uint32_t CFGR;          /*!< RCC clock configuration register     				            Address offset: 0x08 */
    __vo uint32_t CIR;           /*!< RCC clock interrupt register     					            Address offset: 0x0C */
    __vo uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register     			            Address offset: 0x10 */
    __vo uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register     			            Address offset: 0x14 */
    uint32_t      RESERVED0[2];  /*!< Reserved, 0x18-0x1C                                                                */
    __vo uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register     			            Address offset: 0x20 */
    __vo uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register    			            Address offset: 0x24 */
    uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                                */
    __vo uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock enable register 		            Address offset: 0x30 */
    __vo uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock enable register                     Address offset: 0x34 */
    uint32_t      RESERVED2[2];  /*!< Reserved, 0x38-0x3C                                                                */
    __vo uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register                     Address offset: 0x40 */
    __vo uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register                     Address offset: 0x44 */
    uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                                */
    __vo uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register   Address offset: 0x50 */
    __vo uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register   Address offset: 0x54 */
    uint32_t      RESERVED4[2];  /*!< Reserved, 0x58-0x5C                                                                */
    __vo uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register   Address offset: 0x60 */
    __vo uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register   Address offset: 0x64 */
    uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                                */
    __vo uint32_t BDCR;          /*!< RCC Backup domain control register     						Address offset: 0x70 */
    __vo uint32_t CSR;           /*!< RCC clock control & status register     						Address offset: 0x74 */
    uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                                */
    __vo uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register     			Address offset: 0x80 */
    __vo uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register     						Address offset: 0x84 */
    uint32_t      RESERVED7;     /*!< Reserved, 0x88                                                                     */
    __vo uint32_t DCKCFGR;       /*!< RCC Dedicated Clocks Configuration Register     				Address offset: 0x8C */

} RCC_RegDef_t;

/**
 * @brief register definition structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;        /*!< SPI control register 1 (not used in I2S mode)        Address offset: 0x00 */
    __vo uint32_t CR2;        /*!< SPI control register 1 (not used in I2S mode)        Address offset: 0x04 */
    __vo uint32_t SR;         /*!< SPI status register                                  Address offset: 0x08 */
    __vo uint32_t DR;         /*!< SPI data register                                    Address offset: 0x0C */
    __vo uint32_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode)   Address offset: 0x10 */
    __vo uint32_t RXCRCR;     /*!< SPI RX CRC register (not used in I2S mode)           Address offset: 0x14 */
    __vo uint32_t TXCRCR;     /*!< SPI TX CRC register (not used in I2S mode)           Address offset: 0x18 */
    __vo uint32_t I2SCFGR;    /*!< SPI_I2S configuration register                       Address offset: 0x1C */
    __vo uint32_t I2SPR;      /*!< SPI_I2S prescaler register                           Address offset: 0x20 */
} SPI_RegDef_t;

/**
 * @brief peripheral register definition structure for SYSCFG
 */
typedef struct 
{
    __vo uint32_t MEMRMP;       /*!< SYSCFG memory remap register,                          Address offset: 0x00 */
    __vo uint32_t PMC;          /*!< SYSCFG peripheral mode configuration register,         Address offset: 0x04 */
    __vo uint32_t EXTICR[4];    /*!< SYSCFG external interrupt configuration register 1,2,3,4,    Address offset: 0x08-0x014*/
    __vo uint32_t RESERVED[2];  /*!< Reserved,                                              Address offset: 0x18-0x1F */
    __vo uint32_t CMPCR;        /*!< Compensation cell control register,                    Address offset: 0x20 */
} SYSCFG_RegDef_t;


/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;    /*!< Interrupt mask register (EXTI_IMR)               Address offset: 0x00 */
	__vo uint32_t EMR;    /*!< Event mask register (EXTI_EMR)                   Address offset: 0x04 */
	__vo uint32_t RTSR;   /*!< Rising trigger selection register (EXTI_RTSR)    Address offset: 0x08 */
    __vo uint32_t FTSR;   /*!< Falling trigger selection register (EXTI_FTSR)   Address offset: 0x0C */
    __vo uint32_t SWIER;  /*!< Software interrupt event register (EXTI_SWIER)   Address offset: 0x10 */
    __vo uint32_t PR;     /*!< Pending register (EXTI_PR)                       Address offset: 0x14 */
} EXTI_RegDef_t;

/*
 * peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t)
 */
#define GPIOA_PERI      ((GPIO_RegDef_t *)GPIOA_BASEADDR) /*!<Type casted value to GPIO_RegDef_t pointer type of GPIOA_BASEADDR>*/
#define GPIOB_PERI      ((GPIO_RegDef_t *)GPIOB_BASEADDR) /*!<Type casted value to GPIO_RegDef_t pointer type of GPIOB_BASEADDR>*/
#define GPIOC_PERI      ((GPIO_RegDef_t *)GPIOC_BASEADDR) /*!<Type casted value to GPIO_RegDef_t pointer type of GPIOC_BASEADDR>*/
#define GPIOD_PERI      ((GPIO_RegDef_t *)GPIOD_BASEADDR) /*!<Type casted value to GPIO_RegDef_t pointer type of GPIOD_BASEADDR>*/
#define GPIOE_PERI      ((GPIO_RegDef_t *)GPIOE_BASEADDR) /*!<Type casted value to GPIO_RegDef_t pointer type of GPIOE_BASEADDR>*/
#define GPIOH_PERI      ((GPIO_RegDef_t *)GPIOH_BASEADDR) /*!<Type casted value to GPIO_RegDef_t pointer type of GPIOH_BASEADDR>*/


#define RCC_PERI             ((RCC_RegDef_t *)RCC_BASEADDR)
#define EXTI_PERI            ((EXTI_RegDef_t *)EXTI_BASEADDR)
#define SYSCFG_PERI          ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI1_PERI            ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2_PERI            ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3_PERI            ((SPI_RegDef_t *)SPI3_BASEADDR)
#define SPI4_PERI            ((SPI_RegDef_t *)SPI4_BASEADDR)


#define I2C1_PERI            ((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2_PERI            ((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3_PERI            ((I2C_RegDef_t *)I2C3_BASEADDR)

#define USART1_PERI          ((USART_RegDef_t *)USART1_BASEADDR)
#define USART2_PERI          ((USART_RegDef_t *)USART2_BASEADDR)
#define USART6_PERI          ((USART_RegDef_t *)USART6_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN() (RCC_PERI->AHB1ENR |= (1 << 0))      /*!< GPIOA Peripheral clock enable>*/
#define GPIOB_PCLK_EN() (RCC_PERI->AHB1ENR |= (1 << 1))      /*!< GPIOB Peripheral clock enable>*/
#define GPIOC_PCLK_EN() (RCC_PERI->AHB1ENR |= (1 << 2))      /*!< GPIOC Peripheral clock enable>*/
#define GPIOD_PCLK_EN() (RCC_PERI->AHB1ENR |= (1 << 3))      /*!< GPIOD Peripheral clock enable>*/
#define GPIOE_PCLK_EN() (RCC_PERI->AHB1ENR |= (1 << 4))      /*!< GPIOE Peripheral clock enable>*/
#define GPIOH_PCLK_EN() (RCC_PERI->AHB1ENR |= (1 << 7))      /*!< GPIOH Peripheral clock enable>*/

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() (RCC_PERI->APB1ENR |= (1 << 21))      /*!< I2C1 Peripheral clock enable>*/
#define I2C2_PCLK_EN() (RCC_PERI->APB1ENR |= (1 << 22))      /*!< I2C2 Peripheral clock enable>*/
#define I2C3_PCLK_EN() (RCC_PERI->APB1ENR |= (1 << 23))      /*!< I2C3 Peripheral clock enable>*/

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN() (RCC_PERI->APB2ENR |= (1 << 12))      /*!< SPI1 Peripheral clock enable>*/
#define SPI2_PCLK_EN() (RCC_PERI->APB1ENR |= (1 << 14))      /*!< SPI2 Peripheral clock enable>*/
#define SPI3_PCLK_EN() (RCC_PERI->APB1ENR |= (1 << 15))      /*!< SPI3 Peripheral clock enable>*/
#define SPI4_PCLK_EN() (RCC_PERI->APB2ENR |= (1 << 13))      /*!< SPI4 Peripheral clock enable>*/

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCCK_EN() (RCC_PERI->APB2ENR |= (1 << 4))     /*!< USART1 Peripheral Clock ENABLE> */    
#define USART2_PCCK_EN() (RCC_PERI->APB1ENR |= (1 << 17))    /*!< USART2 Peripheral Clock ENABLE> */        
#define USART6_PCCK_EN() (RCC_PERI->APB2ENR |= (1 << 5))     /*!< USART6 Peripheral Clock ENABLE> */

/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN() (RCC_PERI->APB2ENR |= (1 << 14))    /*!< System configuration controller clock enable> */

/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_DI() (RCC_PERI->AHB1ENR &= ~(1 << 0))     /*!< GPIOA Peripheral clock disable>*/
#define GPIOB_PCLK_DI() (RCC_PERI->AHB1ENR &= ~(1 << 1))     /*!< GPIOB Peripheral clock disable>*/
#define GPIOC_PCLK_DI() (RCC_PERI->AHB1ENR &= ~(1 << 2))     /*!< GPIOC Peripheral clock disable>*/
#define GPIOD_PCLK_DI() (RCC_PERI->AHB1ENR &= ~(1 << 3))     /*!< GPIOD Peripheral clock disable>*/
#define GPIOE_PCLK_DI() (RCC_PERI->AHB1ENR &= ~(1 << 4))     /*!< GPIOE Peripheral clock disable>*/
#define GPIOH_PCLK_DI() (RCC_PERI->AHB1ENR &= ~(1 << 7))     /*!< GPIOH Peripheral clock disable>*/

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI() (RCC_PERI->APB1ENR &= ~(1 << 21))     /*!< I2C1 Peripheral clock DISABLE>*/
#define I2C2_PCLK_DI() (RCC_PERI->APB1ENR &= ~(1 << 22))     /*!< I2C2 Peripheral clock DISABLE>*/
#define I2C3_PCLK_DI() (RCC_PERI->APB1ENR &= ~(1 << 23))     /*!< I2C3 Peripheral clock DISABLE>*/

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI() (RCC_PERI->APB2ENR &= ~(1 << 12))     /*!< SPI1 Peripheral clock DISABLE>*/
#define SPI2_PCLK_DI() (RCC_PERI->APB1ENR &= ~(1 << 14))     /*!< SPI2 Peripheral clock DISABLE>*/
#define SPI3_PCLK_DI() (RCC_PERI->APB1ENR &= ~(1 << 15))     /*!< SPI3 Peripheral clock DISABLE>*/
#define SPI4_PCLK_DI() (RCC_PERI->APB2ENR &= ~(1 << 13))     /*!< SPI4 Peripheral clock DISABLE>*/

/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCCK_DI() (RCC_PERI->APB2ENR &= ~(1 << 4))    /*!< USART1 Peripheral Clock DISABLE> */
#define USART2_PCCK_DI() (RCC_PERI->APB1ENR &= ~(1 << 17))   /*!< USART2 Peripheral Clock DISABLE> */
#define USART6_PCCK_DI() (RCC_PERI->APB2ENR &= ~(1 << 5))    /*!< USART6 Peripheral Clock DISABLE> */

/*
 * Clock Disable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI() (RCC_PERI->APB2ENR &= ~(1 << 14))   /*!< System configuration controller clock DISABLE> */

/*
 *  Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()               do{ (RCC_PERI->AHB1RSTR |= (1 << 0)); (RCC_PERI->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC_PERI->AHB1RSTR |= (1 << 1)); (RCC_PERI->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC_PERI->AHB1RSTR |= (1 << 2)); (RCC_PERI->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC_PERI->AHB1RSTR |= (1 << 3)); (RCC_PERI->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC_PERI->AHB1RSTR |= (1 << 4)); (RCC_PERI->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC_PERI->AHB1RSTR |= (1 << 7)); (RCC_PERI->AHB1RSTR &= ~(1 << 7)); }while(0)

#define GPIO_BASEADDR_TO_CODE(x)  ( (x == GPIOA_PERI) ? 0 :\
                                    (x == GPIOB_PERI) ? 1 :\
                                    (x == GPIOC_PERI) ? 2 :\
                                    (x == GPIOD_PERI) ? 3 :\
                                    (x == GPIOE_PERI) ? 4 :\
                                    (x == GPIOH_PERI) ? 7 : -1 )

/*
 *  Macros to reset SPIx peripherals
 */
#define SPI1_REG_RESET()               do{ (RCC_PERI->APB2RSTR |= (1 << 12)); (RCC_PERI->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()               do{ (RCC_PERI->APB1RSTR |= (1 << 14)); (RCC_PERI->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()               do{ (RCC_PERI->APB1RSTR |= (1 << 15)); (RCC_PERI->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()               do{ (RCC_PERI->APB2RSTR |= (1 << 13)); (RCC_PERI->APB2RSTR &= ~(1 << 13)); }while(0)


/**
 * @IRQNumber
 * IRQ(Interrupt Request) Numbers of STM32F401x MCU
 * Based on Table 38. Vector table... (RM0368, page 203)
 */
#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4         84

#define IRQ_NO_I2C1_EV      31
#define IRQ_NO_I2C1_ER      32
#define IRQ_NO_I2C2_EV      33
#define IRQ_NO_I2C2_ER      34
#define IRQ_NO_I2C3_EV      72
#define IRQ_NO_I2C3_ER      73

#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART6	    71

/*
 * macros for all the possible priority levels
 */

#define NVIC_IRQ_PRI0       0
#define NVIC_IRQ_PRI1       1
#define NVIC_IRQ_PRI2       2
#define NVIC_IRQ_PRI3       3
#define NVIC_IRQ_PRI4       4
#define NVIC_IRQ_PRI5       5
#define NVIC_IRQ_PRI6       6
#define NVIC_IRQ_PRI7       7
#define NVIC_IRQ_PRI8       8
#define NVIC_IRQ_PRI9       9
#define NVIC_IRQ_PRI10      10
#define NVIC_IRQ_PRI11      11
#define NVIC_IRQ_PRI12      12
#define NVIC_IRQ_PRI13      13
#define NVIC_IRQ_PRI14      14
#define NVIC_IRQ_PRI15      15

/**
 * @GenericMacros
 */

#define ENABLE_xx 			1
#define DISABLE_xx 			0
#define SET_xx 				ENABLE_xx
#define RESET_xx 			DISABLE_xx
#define GPIO_PIN_SET        ENABLE_xx
#define GPIO_PIN_RESET      DISABLE_xx
#define FLAG_RESET          DISABLE_xx
#define FLAG_SET 			ENABLE_xx
#define ON   				ENABLE_xx
#define OFF 				DISABLE_xx
#define HIGH   				ENABLE_xx
#define LOW 				DISABLE_xx
#define BTN_PRESSED         ENABLE_xx
#define BTN_NOT_PRESSED     DISABLE_xx

/******************************************************************************
                Bit position definitions of SPI peripheral
 *****************************************************************************/

/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0  // Clock phase
#define SPI_CR1_CPOL      				 1  // Clock polarity
#define SPI_CR1_MSTR     				 2  // Master selection
#define SPI_CR1_BR   					 3  // BR[2:0]: Baud rate contro
#define SPI_CR1_SPE     				 6  // SPI enable
#define SPI_CR1_LSBFIRST   			 	 7  // Frame format
#define SPI_CR1_SSI     				 8  // Internal slave select
#define SPI_CR1_SSM      				 9  // Software slave management
#define SPI_CR1_RXONLY      		 	10  // Receive only
#define SPI_CR1_DFF     			 	11  // Data frame format
#define SPI_CR1_CRCNEXT   			 	12  // CRC transfer next
#define SPI_CR1_CRCEN   			 	13  // Hardware CRC calculation enable
#define SPI_CR1_BIDIOE     			 	14  // Output enable in bidirectional mode
#define SPI_CR1_BIDIMODE      			15  // Bidirectional data mode enable

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0   // Rx buffer DMA enable
#define SPI_CR2_TXDMAEN				 	1   // Tx buffer DMA enable
#define SPI_CR2_SSOE				 	2   // SS output enable
#define SPI_CR2_FRF						4   // Frame format
#define SPI_CR2_ERRIE					5   // Error interrupt enable
#define SPI_CR2_RXNEIE				 	6   // RX buffer not empty interrupt enable
#define SPI_CR2_TXEIE					7   // Tx buffer empty interrupt enable


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0   // Receive buffer not empty
#define SPI_SR_TXE				 		1   // Transmit buffer empty
#define SPI_SR_CHSIDE				 	2   // Channel side
#define SPI_SR_UDR					 	3   // Underrun flag
#define SPI_SR_CRCERR				 	4   // CRC error flag
#define SPI_SR_MODF					 	5   // Mode fault
#define SPI_SR_OVR					 	6   // Overrun flag
#define SPI_SR_BSY					 	7   // Busy flag
#define SPI_SR_FRE					 	8   // Frame format error



#include "../../../drivers/STM32F401RE_StdPeriph_Driver/inc/stm32f401xx_gpio_driver.h"
#include "../../../drivers/STM32F401RE_StdPeriph_Driver/inc/stm32f401xx_spi_driver.h"
#include "../../../drivers/STM32F401RE_StdPeriph_Driver/inc/stm32f401xx_timer.h"


#endif /* INC_STM32F401XX_H_ */
