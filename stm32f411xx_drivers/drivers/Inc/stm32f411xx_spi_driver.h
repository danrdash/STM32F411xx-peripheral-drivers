/*
 * stm32f411xx_spi_driver.h
 *
 *  Created on: 29 May 2020
 *      Author: Dan
 */

#ifndef INC_STM32F411XX_SPI_DRIVER_H_
#define INC_STM32F411XX_SPI_DRIVER_H_

#include "stm32f411xx.h"

typedef struct
{
	uint8_t SPI_DeviceMode; // @Device Mode
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_PinConfig_t;


typedef struct
{
	// pointer to hold the base address of the GPIO peripheral
	SPI_RegDef_t *pSPIx; // THIS HOLDS THE BASE ADDRESS OF THE GPIO PORT TO WHICH THE PIN BELONGS
	SPI_PinConfig_t SPI_PinConfig; //this holds the GPIO pin config settings
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
} SPI_Handle_t;

// Various PinConfig Macros
// Spi interrupt macros

#define SPI_READY			0
#define SPI_BUSY_IN_RX		1
#define SPI_BUSY_IN_TX		2

//possible SPI application events
#define SPI_EVENT_TX_COMPLT 1
#define SPI_EVENT_RX_COMPLT 2
#define SPI_EVENT_OVR_ERR	3
#define SPI_EVENT_CRC_ERR	4

//@Device Mode
#define SPI_DEVICE_MODE_MASTER 	1
#define SPI_DEVICE_MODE_SLAVE 	0

// @SPI_Bus_Config
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD			 		2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		 3

// @SPI_SclkSpeed

#define SPI_SCLKSPEED_DIV2	0
#define SPI_SCLKSPEED_DIV4  1
#define SPI_SCLKSPEED_DIV8  2
#define SPI_SCLKSPEED_DIV16  3
#define SPI_SCLKSPEED_DIV32  4
#define SPI_SCLKSPEED_DIV64 5
#define SPI_SCLKSPEED_DIV128 6
#define SPI_SCLKSPEED_DIV256  7

// @SPI_DFF

#define SPI_DFF_8BITS 0
#define SPI_DFF_16BITS 1

// @ SPI_CPOL

#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW 0


// @ SPI_CPHA

#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW 0

// @SPI_SSM

#define SPI_SSM_EN  	1
#define SPI_SSM_DI 		0

// SPI related status flags definitions

#define SPI_TXE_FLAG  		(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG  		(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG		(1 << SPI_SR_BSY)
// Peripheral Clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t En_or_Di);

// Init and Deinit
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


// data send and recieve
// blocking based APIs non interrupt based.
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName );
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
//non blocking based APIs
uint8_t SPI_SendDataIT(SPI_Handle_t*pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);


// IRQ configuration and ISR handling.
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En_or_Di);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle );

//oTHER pERIPHERAL cONTROL api
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t En_or_Di);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t En_or_Di );
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t En_or_Di);
void SPI_ClearOVRFlag(SPI_Handle_t *pSPIHandle);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

//application callback
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);


#endif /* INC_STM32F411XX_SPI_DRIVER_H_ */
