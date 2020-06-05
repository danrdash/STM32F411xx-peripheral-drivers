/*
 * stm32f411xx.h
 *
 *  Created on: May 25, 2020
 *      Author: Dan
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stddef.h>
#include <stdint.h>
#define __vo volatile
#define __weak __attribute__((weak))

// ARM Cortex Mx processor NVIC ISERx and ICERx register addresses.

#define NVIC_ISER0 ( (__vo uint32_t*)0xE000E100)
#define NVIC_ISER1 ( (__vo uint32_t*)0xE000E104)
#define NVIC_ISER2 ( (__vo uint32_t*)0xE000E108)
#define NVIC_ISER3 ( (__vo uint32_t*)0xE000E10C)

#define NVIC_ICER0 ( (__vo uint32_t*)0xE000E180)
#define NVIC_ICER1 ( (__vo uint32_t*)0xE000E184)
#define NVIC_ICER2 ( (__vo uint32_t*)0xE000E188)
#define NVIC_ICER3 ( (__vo uint32_t*)0xE000E18C)

// ARM Cortex Mx PROCESSOR NVIC IRQPri register addresses

#define NVIC_PR_BASE_ADDR  ( (__vo uint32_t*)0xE000E400)

// NVIC Priority macros
#define NVIC__IRQ_PRIO1	1
#define NVIC__IRQ_PRIO2	2
#define NVIC__IRQ_PRIO3	3
#define NVIC__IRQ_PRIO4	4
#define NVIC__IRQ_PRIO5	5
#define NVIC__IRQ_PRIO6	6
#define NVIC__IRQ_PRIO7	7
#define NVIC__IRQ_PRIO8	8
#define NVIC__IRQ_PRIO9	9
#define NVIC__IRQ_PRIO10	10
#define NVIC__IRQ_PRIO11	11
#define NVIC__IRQ_PRIO12	12
#define NVIC__IRQ_PRIO13	13
#define NVIC__IRQ_PRIO14	14
#define NVIC__IRQ_PRIO15	15

#define NO_PR_BITS_IMPLEMENTED 4


// Base addresses of flashs and SRAM Memories

#define FLASH_BASEADDR  		0x0800000U  //base address of flash memory
#define SRAM1_BASEADDR  		0x2000000U  // base address of SRAM1 Memory block
#define ROM_BASEADDR			0x1FFF0000U // SYSTEM memory (ROM ) base address
#define SRAM 					SRAM1_BASEADDR //quality of life macro.


// AHBx and APBx bus peripheral base addresses

#define PERIPH_BASE 			0x40000000U
#define APB1PERIPH_BASE 		PERIPH_BASE
#define APB2PERIPH_BASE 		0x40010000U
#define AHB1PERIPH_BASE 		0x40020000U
#define AHB2PERIPH_BASE 		0x50000000U


// Base addresses of peripherals which are hanging on AHB1 bus.

#define GPIOA_BASEADDR 			(AHB1PERIPH_BASE +  0x0000U)
#define GPIOB_BASEADDR 			(AHB1PERIPH_BASE +  0x0400U)
#define GPIOC_BASEADDR 			(AHB1PERIPH_BASE +  0x0800U)
#define GPIOD_BASEADDR 			(AHB1PERIPH_BASE +  0x0C00U)
#define GPIOE_BASEADDR 			(AHB1PERIPH_BASE +  0x1000U)
#define GPIOH_BASEADDR 			(AHB1PERIPH_BASE +  0x1C00U)

#define RCC_BASEADDR 			(AHB1PERIPH_BASE +  0x3800U)
// Base addresses of peripherals which are hanging on APB1 bus.

#define USART2_BASEADDR 			(APB1PERIPH_BASE +  0x4400U)
#define SPI2_BASEADDR 				(APB1PERIPH_BASE +  0x3800U)
#define SPI3_BASEADDR 				(APB1PERIPH_BASE +  0x3C00U)
#define I2C1_BASEADDR				(APB1PERIPH_BASE +  0x5400U)
#define I2C2_BASEADDR				(APB1PERIPH_BASE +  0x5800U)
#define I2C3_BASEADDR				(APB1PERIPH_BASE +  0x5C00U)

// Base addresses of peripherals which are hanging on APB2 bus.

#define USART1_BASEADDR 			(APB2PERIPH_BASE +  0x1000U)
#define USART6_BASEADDR 			(APB2PERIPH_BASE +  0x1400U)
#define SPI1_BASEADDR 				(APB2PERIPH_BASE +  0x3000U)
#define SPI4_BASEADDR 				(APB2PERIPH_BASE +  0x3400U)
#define SPI5_BASEADDR 				(APB2PERIPH_BASE +  0x5000U)
#define EXTI_BASEADDR 				(APB2PERIPH_BASE +  0x3C00U)
#define SYSCFG_BASEADDR				(APB2PERIPH_BASE +  0X3800U)

// Peripheral register definition structures //
// Note: Registers of peripheral are specific to MCU.

typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
} GPIO_RegDef_t;


typedef struct
{
	__vo uint32_t RCC_CR;
	__vo uint32_t RCC_PLLCFGR;
	__vo uint32_t RCC_CFGR;
	__vo uint32_t RCC_CIR;
	__vo uint32_t RCC_AHB1RSTR;
	__vo uint32_t RCC_AHB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t RCC_APB1RSTR;
	__vo uint32_t RCC_APB2RSTR;
	uint32_t RESERVED2[2];
	__vo uint32_t RCC_AHB1ENR;
	__vo uint32_t RCC_AHB2ENR;
	uint32_t RESERVED3[2];
	__vo uint32_t RCC_APB1ENR;
	__vo uint32_t RCC_APB2ENR;
	uint32_t RESERVED4[2];
	__vo uint32_t RCC_AHB1LPENR;
	__vo uint32_t RCC_AHB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t RCC_APB1LPENR;
	__vo uint32_t RCC_APB2LPENR;
	uint32_t RESERVED6[2];
	__vo uint32_t RCC_BDCR;
	__vo uint32_t RCC_CSR;
	uint32_t RESERVED7[2];
	__vo uint32_t RCC_SSCGR;
	__vo uint32_t RCC_PLLI2SCFGR;
	uint32_t RESERVED8;
	__vo uint32_t RCC_DCKCFGR;
} RCC_RegDef_t;

typedef struct
{
	__vo uint32_t EXTI_IMR;
	__vo uint32_t EXTI_EMR;
	__vo uint32_t EXTI_RTSR;
	__vo uint32_t EXTI_FTSR;
	__vo uint32_t EXTI_SWIER;
	__vo uint32_t EXTI_PR;
} EXTI_RegDef_t;

typedef struct
{
	__vo uint32_t SYSCFG_MEMRMP;
	__vo uint32_t SYSCFG_PMC;
	__vo uint32_t SYSCFG_EXTICR[4];
	uint32_t RESERVED[2];
	__vo uint32_t SYSCFG_CMPCR;
} SYSCFG_RegDef_t;

// Macros for SPIx Peripheral definitions

typedef struct
{
	__vo uint32_t SPI_CR1;
	__vo uint32_t SPI_CR2;
	__vo uint32_t SPI_SR;
	__vo uint32_t SPI_DR;
	__vo uint32_t SPI_CRCPR;
	__vo uint32_t SPI_RXCRCPR;
	__vo uint32_t SPI_TXCRCPR;
	__vo uint32_t SPI_I2SCFGR;
	__vo uint32_t SPI_I2SPR;
} SPI_RegDef_t;

// Macros for I2Cx Peripheral Defienitions

typedef struct
{
	__vo uint32_t I2C_CR1;
	__vo uint32_t I2C_CR2;
	__vo uint32_t I2C_OAR1;
	__vo uint32_t I2C_OAR2;
	__vo uint32_t I2C_DR;
	__vo uint32_t I2C_SR1;
	__vo uint32_t I2C_SR2;
	__vo uint32_t I2C_CCR;
	__vo uint32_t I2C_TRISE;
	__vo uint32_t I2C_FLTR;
} I2C_RegDef_t;
// Peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t )

#define GPIOA ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*)GPIOH_BASEADDR)


#define RCC  ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1 ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2 ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3 ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4 ((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5 ((SPI_RegDef_t*)SPI5_BASEADDR)

#define I2C1 ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2 ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3 ((I2C_RegDef_t*)I2C3_BASEADDR)

// irq iNTERRUPT REQUESTS NUMBERS DEFINITION

#define IRQ_NO_EXTI0 6
#define IRQ_NO_EXTI1 7
#define IRQ_NO_EXTI2 8
#define IRQ_NO_EXTI3 9
#define IRQ_NO_EXTI4 10
#define IRQ_NO_EXTI9_5 23
#define IRQ_NO_EXTI15_10 40


#define IRQ_NO_SPI1		35
#define IRQ_NO_SPI2		43
#define IRQ_NO_SPI3		51
#define IRQ_NO_SPI4		84
#define IRQ_NO_SPI5		85
#define IRQ_NO_I2C1_EV  31
#define IRQ_NO_I2C1_ER	32


//**ENABLE MACROS**//

// Clock Enable Macros for GPIOx peripherals

#define GPIOA_PCLK_EN() 	(RCC->RCC_AHB1ENR |= ( 1 << 0))
#define GPIOB_PCLK_EN() 	(RCC->RCC_AHB1ENR |= ( 1 << 1))
#define GPIOC_PCLK_EN() 	(RCC->RCC_AHB1ENR |= ( 1 << 2))
#define GPIOD_PCLK_EN() 	(RCC->RCC_AHB1ENR |= ( 1 << 3))
#define GPIOE_PCLK_EN() 	(RCC->RCC_AHB1ENR |= ( 1 << 4))
#define GPIOH_PCLK_EN() 	(RCC->RCC_AHB1ENR |= ( 1 << 7))



// Clock Enable Macros for SPIx peripherals

#define SPI1_PCLK_EN()  (RCC->RCC_APB2ENR |= (1 << 12 ))
#define SPI4_PCLK_EN()  (RCC->RCC_APB2ENR |= (1 << 13 ))
#define SPI2_PCLK_EN()  (RCC->RCC_APB1ENR |= (1 << 14 ))
#define SPI3_PCLK_EN()  (RCC->RCC_APB1ENR |= (1 << 15 ))
#define SPI5_PCLK_EN()  (RCC->RCC_APB2ENR |= ( 1 << 20 ))

// Clock Enable Macros for USARTx peripheral

#define USART1_PCLK_EN()  (RCC->APB2ENR |= (1 << 4 ))
#define USART6_PCLK_EN()  (RCC->APB2ENR |= (1 << 5 ))
#define USART2_PCLK_EN()  (RCC->APB1ENR |= (1 << 17 ))

// Clock Enable Macros for SYSCFG
#define I2C1_PCLK_EN()  (RCC->RCC_APB1ENR |= (1 << 21 ))
#define I2C2_PCLK_EN()  (RCC->RCC_APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()  (RCC->RCC_APB1ENR |= (1 << 23 ))

// Clock Enable Macros for I2Cx peripheral

#define SYSCFG_PCLK_EN()  (RCC->RCC_APB2ENR |= (1 <<14 ))

//*DISABLE MACROS**//

// Clock Disable Macros for GPIOx peripherals

#define GPIOA_PCLK_DI() 	(RCC->RCC_AHB1ENR &= ~( 1 << 0))
#define GPIOB_PCLK_DI()	 	(RCC->RCC_AHB1ENR &= ~( 1 << 1))
#define GPIOC_PCLK_DI() 	(RCC->RCC_AHB1ENR &= ~( 1 << 2))
#define GPIOD_PCLK_DI() 	(RCC->RCC_AHB1ENR &= ~( 1 << 3))
#define GPIOE_PCLK_DI() 	(RCC->RCC_AHB1ENR &= ~( 1 << 4))
#define GPIOH_PCLK_DI() 	(RCC->RCC_AHB1ENR &= ~( 1 << 7))

// Clock Disable Macros for I2Cx peripherals

#define I2C1_PCLK_DI() 		(RCC->RCC_APB1ENR &= ~( 1 << 21 ))
#define I2C2_PCLK_DI() 		(RCC->RCC_APB1ENR &= ~( 1 << 22 ))
#define I2C3_PCLK_DI() 		(RCC->RCC_APB1ENR &= ~( 1 << 23 ))

// Clock Disable Macros for SPIx peripherals

#define SPI1_PCLK_DI()  	(RCC->RCC_APB2ENR &= ~( 1 << 12 ))
#define SPI4_PCLK_DI() 		(RCC->RCC_APB2ENR &= ~( 1 << 13 ))
#define SPI2_PCLK_DI()  	(RCC->RCC_APB1ENR &= ~( 1 << 14 ))
#define SPI3_PCLK_DI()  	(RCC->RCC_APB1ENR &= ~( 1 << 15 ))
#define SPI5_PCLK_DI()  	(RCC->RCC_APB2ENR &= ~( 1 << 20 ))


// Clock Disable Macros for USARTx peripheral

#define USART1_PCLK_DI()  	(RCC->APB2ENR &= ~( 1 << 4 ))
#define USART6_PCLK_DI()  	(RCC->APB2ENR &= ~( 1 << 5 ))


// Clock Enable Macros for SYSCFG
#define SYSCFG_PCLK_DI()  (RCC->RCC_APB2ENR &= ~(1 <<14 ))

// Macros to reset GPIOx peripherals

#define GPIOA_REG_RESET()   do{(RCC->RCC_AHB1RSTR |= ( 1<< 0));  (RCC->RCC_AHB1RSTR&= ~( 1<< 0)); } while(0)
#define GPIOB_REG_RESET()   do{(RCC->RCC_AHB1RSTR |= ( 1<< 1));  (RCC->RCC_AHB1RSTR&= ~( 1<< 1)); } while(0)
#define GPIOC_REG_RESET()   do{(RCC->RCC_AHB1RSTR |= ( 1<< 2));  (RCC->RCC_AHB1RSTR&= ~( 1<< 2)); } while(0)
#define GPIOD_REG_RESET()   do{(RCC->RCC_AHB1RSTR |= ( 1<< 3));  (RCC->RCC_AHB1RSTR&= ~( 1<< 3)); } while(0)
#define GPIOE_REG_RESET()   do{(RCC->RCC_AHB1RSTR |= ( 1<< 4));  (RCC->RCC_AHB1RSTR&= ~( 1<< 4)); } while(0)
#define GPIOH_REG_RESET()   do{(RCC->RCC_AHB1RSTR |= ( 1<< 7));  (RCC->RCC_AHB1RSTR&= ~( 1<< 7)); } while(0)

// Macros to reset SPIx Peripherals
#define SPI1_REG_RESET()   do{(RCC->RCC_APB2RSTR |= ( 1<< 12));  (RCC->RCC_APB2RSTR&= ~( 1<< 12)); } while(0)
#define SPI2_REG_RESET()   do{(RCC->RCC_APB1RSTR |= ( 1<< 14));  (RCC->RCC_APB1RSTR&= ~( 1<< 14)); } while(0)
#define SPI3_REG_RESET()   do{(RCC->RCC_APB1RSTR |= ( 1<< 15));  (RCC->RCC_APB1RSTR&= ~( 1<< 15)); } while(0)
#define SPI4_REG_RESET()   do{(RCC->RCC_APB2RSTR |= ( 1<< 13));  (RCC->RCC_APB2RSTR&= ~( 1<< 13)); } while(0)
#define SPI5_REG_RESET()   do{(RCC->RCC_APB2RSTR |= ( 1<< 20));  (RCC->RCC_APB2RSTR&= ~( 1<< 20)); } while(0)


// MAcros to reset I2Cx Peripherals
#define I2C1_REG_RESET()   do{(RCC->RCC_APB1RSTR |= ( 1<< 21));  (RCC->RCC_APB2RSTR&= ~( 1<< 21)); } while(0)
#define I2C2_REG_RESET()   do{(RCC->RCC_APB1RSTR |= ( 1<< 22));  (RCC->RCC_APB1RSTR&= ~( 1<< 22)); } while(0)
#define I2C3_REG_RESET()   do{(RCC->RCC_APB1RSTR |= ( 1<< 23));  (RCC->RCC_APB1RSTR&= ~( 1<< 23)); } while(0)


// RETURNS PORT CODE FOR GIVEN gpioX BASE ADDRESS

#define GPIO_BASEADDR_TO_CODE(x) ((x==GPIOA) ? 0:\
								 (x==GPIOB) ? 1:\
								 (x==GPIOC) ? 2:\
								 (x==GPIOD) ? 3:\
								 (x==GPIOE) ? 4:\
								 (x==GPIOH) ? 7:0)
// Generic macros

#define ENABLE 		1
#define DISABLE 	0
#define SET 		ENABLE
#define RESET 		DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET
#define FLAG_RESET 	RESET
#define FLAG_SET 	SET


// bit position macros for SPI peripheral

#define SPI_CR1_CPHA 			0
#define SPI_CR1_CPOL 			1
#define SPI_CR1_MSTR 			2
#define SPI_CR1_BR 				3
#define SPI_CR1_SPE 			6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY 			10
#define SPI_CR1_DFF 			11
#define SPI_CR1_CRCNEXT 		12
#define SPI_CR1_CRCEN 			13
#define SPI_CR1_BIDIOE 			14
#define SPI_CR1_BIDIMODE 		15

#define SPI_CR2_RXDMAEN			 0
#define SPI_CR2_TXDMAENN		 1
#define SPI_CR2_SSOE 			 2
#define SPI_CR2_FRF				 4
#define SPI_CR2_ERRIE			 5
#define SPI_CR2_RXNEIE			 6
#define SPI_CR2_TXEIE 			 7

#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8


/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_spi_driver.h"
#include "stm32f411xx_i2c_driver.h"
#endif /* INC_STM32F411XX_H_ */
