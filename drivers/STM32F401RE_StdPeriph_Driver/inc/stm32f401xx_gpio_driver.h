/*
 * stm32f401xx_gpio_driver.h
 *
 *  Created on: Mar 16, 2023
 *      Author: Huy Hoang
 */
/******************************************************************************/
/*                              INCLUDE FILES                                 */
/******************************************************************************/
#include "../../../drivers/STM32F401RE_StdPeriph_Driver/inc/stm32f401xx.h"
/******************************************************************************/
/*                     EXPORTED TYPES and DEFINITIONS                         */
/******************************************************************************/
/*
 * This is a Configuration structure for a GPIO pin
 */
typedef struct
{
    uint8_t GPIO_PinNumber;         /*!< possible values from @GPIO_PIN_NUMBERS >*/
    uint8_t GPIO_PinMode;           /*!< possible values from @GPIO_PIN_MODES >*/
    uint8_t GPIO_PinSpeed;          /*!< possible values from @GPIO_PIN_SPEED >*/
    uint8_t GPIO_PinPuPdControl;    /*!< possible values from @GPIO_PIN_PUPD >*/
    uint8_t GPIO_PinOPType;         /*!< possible values from @GPIO_PIN_OPTYPE >*/
    uint8_t GPIO_PinAltFunMode;     /*!< NOT HANDLE YET >*/
} GPIO_PinConfig_t;

/*
 * This is a Handle structure for a GPIO pin
 */
typedef struct
{
    GPIO_RegDef_t *pGPIOx;           /*!< This holds the base address of the GPIO port to which the pin belongs >*/
    GPIO_PinConfig_t GPIO_PinConfig; /*!< This holds GPIO pin configuration settings >*/
} GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 */
#define GPIO_PIN_NO_0  				0
#define GPIO_PIN_NO_1  				1
#define GPIO_PIN_NO_2  				2
#define GPIO_PIN_NO_3  				3
#define GPIO_PIN_NO_4  				4
#define GPIO_PIN_NO_5  				5
#define GPIO_PIN_NO_6  				6
#define GPIO_PIN_NO_7  				7
#define GPIO_PIN_NO_8  				8
#define GPIO_PIN_NO_9  				9
#define GPIO_PIN_NO_10  			10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12  			12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT     4   // Mode: Interrupt Falling Edge Trigger
#define GPIO_MODE_IT_RT     5   // Mode: Interrupt Falling Raising Trigger
#define GPIO_MODE_IT_RFT    6   // Mode: Interrupt Raising and Falling Edge Trigger

/*
 * @GPIO_PIN_OPTYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP   0     // Mode: Output Push-pull
#define GPIO_OP_TYPE_OD   1     // Mode: Output Open drain

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPOI_SPEED_HIGH			3

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull up AND pull down configuration macros
 */
#define GPIO_NO_PUPD   		0   // No pull-up/pull-down resistor
#define GPIO_PIN_PU			1   // Pull-up resistor
#define GPIO_PIN_PD			2   // Pull-down resistor
/******************************************************************************/
/*                              PRIVATE DATA                                  */
/******************************************************************************/

/******************************************************************************/
/*                              EXPORTED DATA                                 */
/******************************************************************************/

/******************************************************************************/
/*                            PRIVATE FUNCTIONS                               */
/******************************************************************************/

/******************************************************************************/
/*                            EXPORTED FUNCTIONS                              */
/******************************************************************************/
/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/******************************** Init and De-init ****************************************/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/******************************** Data read and write ************************************/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
_Bool GPIO_CheckButtonPressedActiveLow(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
_Bool GPIO_CheckButtonPressedActiveHigh(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/******************************** IRQ Configuration and ISR handling ******************/

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);
