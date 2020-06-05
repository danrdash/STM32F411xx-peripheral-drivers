/*
 * stm32f411xx_spi_driver.c
 *
 *  Created on: 29 May 2020
 *      Author: Dan
 */

#include "stm32f411xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
#define LOW 0

// Peripheral Clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}
			else if (pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}
			else if (pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}
			else if (pSPIx == SPI4)
			{
				SPI4_PCLK_EN();
			}
			else if (pSPIx == SPI5)
			{
				SPI5_PCLK_EN();
			}
		}
	else
	{
		if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}
			else if (pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}
			else if (pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}
			else if (pSPIx == SPI4)
			{
				SPI4_PCLK_DI();
			}
			else if (pSPIx == SPI5)
			{
				SPI5_PCLK_DI();
			}
	}
}

// Init and Deinit
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// configure SPI_CR1 Register
	uint32_t tempreg= 0;
	// configure device mode
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_DeviceMode << SPI_CR1_MSTR;
	// confgiure the bus config
	if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode should be clearead
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
	}else if( pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode should be set
		tempreg |= ~(1<<SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// bidi mode shoudl be clearead
		//rx only bit set
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		tempreg |= (1<<SPI_CR1_RXONLY);
	}
	//configure the SclkSpeed
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_SclkSpeed << SPI_CR1_BR;
	//configre the dff
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_DFF << SPI_CR1_DFF;
	//configure the CPOL
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_CPOL<< SPI_CR1_CPOL;
	//configure the CPHA
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_CPHA<< SPI_CR1_CPHA;
	//configure the SSM
	tempreg |= pSPIHandle->SPI_PinConfig.SPI_SSM<< SPI_CR1_SSM;

	pSPIHandle->pSPIx->SPI_CR1 = tempreg;


}
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	else if (pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
	else if (pSPIx == SPI5)
	{
		SPI5_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName )
{
	if(pSPIx->SPI_SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

// data send and recieve
// blocking based APIs non interrupt based.

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) //this is a blocking call. polling based cased.
{
	while(Len > 0 )
	{
		//wait until Txe IS SET
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//check the dfff bit in CR1
		if(pSPIx->SPI_CR1 & (1 <<SPI_CR1_DFF))
		{
			//16 bit dff, load data in the DR
			pSPIx->SPI_DR = *((uint16_t*)pTxBuffer);
			Len--; Len--;
			(uint16_t*)pTxBuffer++; //increase buffer by 2
		}
		else
		{
			//8bit dff
			pSPIx->SPI_DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}

}
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0 )
		{
			//wait until Txe IS SET
			while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

			//check the dfff bit in CR1
			if(pSPIx->SPI_CR1 & (1 <<SPI_CR1_DFF))
			{
				//16 bit dff, load data in the DR
				*((uint16_t*)pRxBuffer) = pSPIx->SPI_DR;
				Len--; Len--;
				(uint16_t*)pRxBuffer++; //increase buffer by 2
			}
			else
			{
				//8bit dff
				*pRxBuffer = pSPIx->SPI_DR;
				//pSPIx->SPI_DR |= *pTxBuffer;
				Len--;
				pRxBuffer++;
			}
		}
}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint8_t state = pSPIHandle->TxState;

	if(state!= SPI_BUSY_IN_TX){
	//1 SAVE THE TX BUFFER ADDRESS AND LEN INFO
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = Len;

	//2 MARK TEH SPI STATE AS BUSY IN TRANSMISSION SO THAT NO OTHER CODE CAN TAKE OVER SAME SPI PERIPHERAL UNTIL TRANMSISSION IS OVER
	pSPIHandle->TxState = SPI_BUSY_IN_TX;
	//3 ENABLE THE TXEIE CONTROL BIT TO GET INTERRUPT WHENEVER TXE FLAG IS SET TO SR
	pSPIHandle->pSPIx->SPI_CR2 |= (1<< SPI_CR2_TXEIE);
	//4 Data Transmission will be handles by the ISR code.
	}
	return state;
}
uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state!= SPI_BUSY_IN_RX){
	//1 SAVE THE TX BUFFER ADDRESS AND LEN INFO
	pSPIHandle->pRxBuffer = pRxBuffer;
	pSPIHandle->RxLen = Len;

	//2 MARK TEH SPI STATE AS BUSY IN TRANSMISSION SO THAT NO OTHER CODE CAN TAKE OVER SAME SPI PERIPHERAL UNTIL TRANMSISSION IS OVER
	pSPIHandle->RxState = SPI_BUSY_IN_RX;
	//3 ENABLE THE TXEIE CONTROL BIT TO GET INTERRUPT WHENEVER TXE FLAG IS SET TO SR
	pSPIHandle->pSPIx->SPI_CR2 |= (1<< SPI_CR2_RXNEIE);
	//4 Data Transmission will be handles by the ISR code.
	}
	return state;
}

// IRQ configuration and ISR handling.
void SPI_IRQInterruptConfig(uint8_t IRQNumber,  uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		if(IRQNumber <=31)
		{
				//program ISER0 reg
			*NVIC_ISER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber >31 && IRQNumber <64)
		{
			*NVIC_ISER1 |= (1<<(IRQNumber % 32));
		}
		else if(IRQNumber >=64 && IRQNumber <96)
		{
			*NVIC_ISER2 |= (1<<(IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <=31)
		{
				//program ISER0 reg
			*NVIC_ICER0 |= (1<<IRQNumber);
		}
		else if(IRQNumber >31 && IRQNumber <64)
		{
			*NVIC_ICER1 |= (1<<(IRQNumber % 32));
		}
		else if(IRQNumber >=64 && IRQNumber <96)
		{
			*NVIC_ICER2 |= (1<<(IRQNumber % 64));
		}
	}
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// first lets find out the IPR register
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR +iprx) |= (IRQPriority << shift_amount);
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle )
{
	uint8_t temp1,temp2;
		//first lets check for TXE
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 <<SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_TXEIE);
	if(temp1 && temp2)
	{
		//Handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//RXE check
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 <<SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_RXNEIE);
	if(temp1 && temp2)
	{
		//Handle TXE
		spi_rxne_interrupt_handle(pSPIHandle);
	}
	//check for ovr flag
	temp1 = pSPIHandle->pSPIx->SPI_SR & (1 <<SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->SPI_CR2 & (1 << SPI_CR2_ERRIE);
	if(temp1 && temp2)
		{
			//Handle TXE
			spi_ovr_err_interrupt_handle(pSPIHandle);
		}
}

// OTHER PERIPHERAL CONTROL APIS
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t En_or_Di)
{
	if( En_or_Di == ENABLE)

		{
			pSPIx->SPI_CR2 |= ( 1 << SPI_CR2_SSOE);
		}
		else
		{
			pSPIx->SPI_CR2 &= ~( 1 << SPI_CR2_SSOE);
		}
}
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t En_or_Di)
{
	if( En_or_Di == ENABLE)

		{
			pSPIx->SPI_CR1 |= ( 1 << SPI_CR1_SSI);
		}
		else
		{
			pSPIx->SPI_CR1 &= ~( 1 << SPI_CR1_SSI);
		}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t En_or_Di )
{
	if( En_or_Di == ENABLE)

	{
		pSPIx->SPI_CR1 |= ( 1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~( 1 << SPI_CR1_SPE);
	}
}

//helper functions implementation

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//check the dfff bit in CR1
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 <<SPI_CR1_DFF))
	{
		//16 bit dff, load data in the DR
		pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--; pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++; //increase buffer by 2
	}
	else
	{
		//8bit dff
		pSPIHandle->pSPIx->SPI_DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}
	if(pSPIHandle->TxLen == LOW)
	{
		//txlen is zero, so close the spi transmission and inform the app that tx is over
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_COMPLT);
	}

}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->SPI_CR1 & (1 <<SPI_CR1_DFF))
		{
			//16 bit dff, load data in the DR
		 *((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR;
			pSPIHandle->RxLen--; pSPIHandle->RxLen--;
			(uint16_t*)pSPIHandle->pRxBuffer++; //increase buffer by 2
		}
		else
		{
			//8bit dff
			*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->SPI_DR ;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer++;
		}
		if(pSPIHandle->RxLen == LOW)
		{
			//txlen is zero, so close the spi transmission and inform the app that tx is over
			SPI_CloseReception(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLT);
		}

}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp;
	//clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->SPI_DR;
		temp = pSPIHandle->pSPIx->SPI_SR;
	}
	(void)temp;
	//infor the application
			SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){

	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_TXEIE); // prevents interrupts from settings up of the TXE flag
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen =0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_RXNEIE); // prevents interrupts from settings up of the TXE flag
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen =0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_Handle_t *pSPIHandle){
	uint8_t temp;
	temp = pSPIHandle->pSPIx->SPI_DR;
	temp = pSPIHandle->pSPIx->SPI_SR;
	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	//this is a weak implementation the application may override this function
}

