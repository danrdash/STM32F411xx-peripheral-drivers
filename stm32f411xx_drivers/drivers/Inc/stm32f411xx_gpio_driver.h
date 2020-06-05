/*
 * stm32f411xx_gpio_driver.h
 *
 *  Created on: 25 May 2020
 *      Author: Dan
 */

#ifndef STM32F411XX_GPIO_DRIVER_H_
#define STM32F411XX_GPIO_DRIVER_H_

#include "stm32f411xx.h"

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode; //possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed; //possible values from @GPIO_SPEED_XX
	uint8_t GPIO_PinPuPdControl; //possible values from @GPIO_PUPD_MODES
	uint8_t GPIO_PinOPType; //POSSIBLE VALUES FROM @GPIO_OP_TYPE_XX
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;
typedef struct
{
	// pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx; // THIS HOLDS THE BASE ADDRESS OF THE GPIO PORT TO WHICH THE PIN BELONGS
	GPIO_PinConfig_t GPIO_PinConfig; //this holds the GPIO pin config settings
}GPIO_Handle_t;



/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
*/

#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15

//@GPIO_PIN_MODES
// GPIO Pin Possible Modes

#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT		4 //INTERRUPT FALLING EDGE TRIGGER
#define GPIO_MODE_IT_RT		5 // INTERRUPT RISING EDGE TRIGGER
#define GPIO_MODE_IT_RFT	6 // INTERRUPT RISING FALLING EDGE TRIGGER

//@GPIO_OP_TYPE_XX
// GPIO Pin Possible Modes

#define GPIO_OP_TYPE_PP 	0
#define GPIO_OP_TYPE_OD 	1

//@GPIO_SPEED_XX
// GPIO Pin Possible Output speeds

#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

//@GPIO_PUPD_MODES
// GPIO PUPD configurations macros

#define GPIO_NO_PUPD 		0
#define GPIO_PIN_PU 		1
#define GPIO_PIN_PD 		2


// API Declarations


	// Peripheral Clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t En_or_Di);

	// Init and Deinit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


	// Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_Handle_t *pGPIOHandle, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_Handle_t *pGPIOHandle);
void GPIO_WriteToOutputPin(GPIO_Handle_t *pGPIOHandle, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_Handle_t *pGPIOHandle, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_Handle_t *pGPIOHandle, uint8_t PinNumber);

	// IRQ configuration and ISR handling.
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En_or_Di);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber );

#endif /* STM32F411XX_GPIO_DRIVER_H_ */
