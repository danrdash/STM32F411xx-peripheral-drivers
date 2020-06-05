/*
 * stm32f411xx_i2c_driver.c
 *
 *  Created on: 2 Jun 2020
 *      Author: Dan
 */


/*
 * Peripheral Clock setup
 */

#include "stm32f411xx.h"

uint16_t AHB_PreScalar[8] = {2,4,8,16,64,128,256,512};
uint16_t APB_PreScalar[4] = {2,4,8,16};


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr);

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);







static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 <<I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr <<1;
	SlaveAddr &= ~(1);
	pI2Cx->I2C_DR = SlaveAddr;
}
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr <<1;
	SlaveAddr |= (1);
	pI2Cx->I2C_DR = SlaveAddr;
}
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= ( 1<< I2C_CR1_STOP);
}
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyRead;
	//check for dev mode
	if(pI2CHandle->pI2Cx->I2C_SR2 & ( 1<< I2C_SR2_MSL))
	{
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize==1)
			{
				//disable ACK
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
				dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
				dummyRead = pI2CHandle->pI2Cx->I2C_SR2;
				(void)dummyRead;
			}
		}
		else
		{
			dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
			dummyRead = pI2CHandle->pI2Cx->I2C_SR2;
			(void)dummyRead;
		}
	}
	else
	{
		//device is in slave mode
		dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
		dummyRead = pI2CHandle->pI2Cx->I2C_SR2;
		(void)dummyRead;

	}
}
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}

	}
else
{
	if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
}
}

/*
 * Init and De-init
 */
uint32_t RCC_GetPLLOutputClock(void)
{
	uint32_t PLLOut=0;
	return PLLOut;
}

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;
	uint8_t clksrc,temp,ahbp,temp2,apbp;
	clksrc = RCC->RCC_CFGR >>2 &  0X3;
	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}
	else if(clksrc ==2)
	{
		SystemClk= RCC_GetPLLOutputClock();
	}


	temp = (RCC->RCC_CFGR >>4) & 0xF;
	if (temp< 8)
		ahbp=1;
	else
	{
		ahbp = AHB_PreScalar[temp-8];
	}
	temp2 = (RCC->RCC_CFGR >> 10) & 0x7;
	if (temp2 < 4)
		apbp=1;
	else
	{
		apbp = APB_PreScalar[temp2-4];
	}

	pclk1 = (SystemClk / ahbp) /apbp;
	return pclk1;
}
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg=0;
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->I2C_CR1 = tempreg;
	//configure the freq field of CR2
	tempreg = 0;
	tempreg = RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->I2C_CR2 = (tempreg & 0x3F);
	//program the device own address

	tempreg = pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	pI2CHandle->pI2Cx->I2C_OAR1 = (tempreg & (0xFE));
	pI2CHandle->pI2Cx->I2C_OAR1 |= (1<< 14);


	//ccr calculations
	uint16_t ccr_value =0;
	tempreg=0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//standard mode
		ccr_value = RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		tempreg |= (ccr_value & 0xFFF);
	}else
	{
		//fast mode
		tempreg |= (1<< I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		else if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_16_9)
		{
			ccr_value = RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		tempreg |= (ccr_value & 0xFFF);
	}
		pI2CHandle->pI2Cx->I2C_CCR = tempreg;

		//configure TRISE
		if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
			{
				//standard mode

				tempreg = (RCC_GetPLLOutputClock()/1000000U) +1;
			}else
			{
				//fast mode
				tempreg = ((RCC_GetPLLOutputClock()*300)/100000000U) +1;
			}
				pI2CHandle->pI2Cx->I2C_TRISE = (tempreg & 0x3F);

}
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
		{
			I2C1_REG_RESET();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_REG_RESET();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_REG_RESET();
		}

}


/*
 * Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{ // generate start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
//	confirm that start generation is completed by checking the SB flag in the SR1
//	note: until SB is clearead SCL will be stretched
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)));
	//send the address of the slave with r/nw bit set to w(0) ( total 8 bits)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);
	// confirm that address phase is completed by checking the ADDR flag in teh SR1
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));
	//clear the ADDR flag according to it's software sequence., scl will be stretched until then
	I2C_ClearADDRFlag(pI2CHandle);
	//send the data until len bcomes 0
	while(Len >0)

	{
		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));
		pI2CHandle->pI2Cx->I2C_DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}
	//when len becomes zero wait for TXE and BTF flag before generating the stop condition.
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)));
	//generate stop condition
	if(Sr == I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	//1. Generate the START condition

	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)));

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);
	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));
	//procedure to read only 1 byte from slave

	if( Len == 1)
	{
		//Disable Acking
		I2C_ManageAcking(I2C1, DISABLE);

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until  RXNE becomes 1
		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));

		//generate STOP condition
		if(Sr == I2C_DISABLE_SR)
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);


		//read data in to buffer
		*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
		Len--;

	}
	if( Len > 1)
	{
    //procedure to read data from slave when Len > 1

		//clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
		//read the data until Len becomes zero
		while(Len >0)
		{
			//wait until RXNE becomes 1
			while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));
			//if last 2 bytes remain
			if(Len == 2)
			{
				//Disable Acking
				I2C_ManageAcking(I2C1, DISABLE);

				//generate STOP condition
				if(Sr == I2C_DISABLE_SR  )
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}



			//read the data from data register in to buffer
			*pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
			//increment the buffer address
			pRxBuffer++;
			Len--;
		}
	}




	//re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE){
		I2C_ManageAcking(I2C1, ENABLE);
	}


}
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX)) // if bus is not busy
	{
		pI2CHandle->pTxBuffer = pTxbuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1<< I2C_CR2_ITERREN);


	}

	return busystate;

}
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)

{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);


		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1<< I2C_CR2_ITERREN);

	}

	return busystate;
}


void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}
}
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;
}


void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data)
{
	pI2C->I2C_DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	 return (uint8_t) pI2C->I2C_DR;
}

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t En_or_Di)
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// first lets find out the IPR register
		uint8_t iprx = IRQNumber/4;
		uint8_t iprx_section = IRQNumber % 4;
		uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);
		*(NVIC_PR_BASE_ADDR +iprx) |= (IRQPriority << shift_amount);

}

void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->RxSize == 1)
		{
				*pI2CHandle->pRxBuffer=pI2CHandle->pI2Cx->I2C_DR;
				pI2CHandle->pRxBuffer++;
				pI2CHandle->RxLen--;
		}
		if(pI2CHandle->RxSize > 1)
		{
			if(pI2CHandle->RxLen==2)
			{
				//clear the ack bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
			}

				//READ dr
			*pI2CHandle->pRxBuffer=pI2CHandle->pI2Cx->I2C_DR;
			pI2CHandle->pRxBuffer++;
			pI2CHandle->RxLen--;
		}
		if(pI2CHandle->RxLen==0)
		{
			//close the I2C Data andnotify the app

			//1 generate the stop
			if(pI2CHandle->Sr ==I2C_DISABLE_SR)
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			//2 close the i2c rx
			I2C_CloseReceiveData(pI2CHandle);
			//notify app
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
		}

}


void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	//Interrupt handling for both master and slave mode of a device
	volatile uint32_t temp1=0, temp2=0,temp3=0, I2C_CR2Reg=0, temp4=0;
	volatile uint32_t I2C_CR2_ITEVTENReg=0;
	//1. Handle for interrupt generated by SB Event
	I2C_CR2Reg = pI2CHandle->pI2Cx->I2C_CR2;
	I2C_CR2_ITEVTENReg = (1 << I2C_CR2_ITEVTEN);
	temp1 = I2C_CR2Reg & I2C_CR2_ITEVTENReg;

	temp4 = pI2CHandle->pI2Cx->I2C_CR2 & ( 1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->I2C_CR2 & ( 1 << I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<< I2C_SR1_SB);
	if(temp1 && temp3)
	{
		//handle SB bit interrupt , WON'T BE EXECUTED IN SLAVE MODE
		//in this block lets excute the address phase.
				if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
					I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
				if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
					I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
	}
	temp3=0;
	//2. Handle for interrupt generated by ADDR EVent
	//note: when master mode: address is sent, when slave mode: address matched with own address.
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<< I2C_SR1_ADDR);
	if(temp1 && temp3)
		{
			//handle ADDR bit interrupt
			I2C_ClearADDRFlag(pI2CHandle);

		}
	temp3=0;
	//3 Handle for interrupt generated by BTF(byte transfer finished) event
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<< I2C_SR1_BTF);
	if(temp1 && temp3)
		{
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			//make sure that TXCE is also set
			if(pI2CHandle->pI2Cx->I2C_SR1 & (1<< I2C_SR1_TXE))
			{
				//btf and txe are set
				if(pI2CHandle->TxLen == RESET)
				{
						//1. generate stop condition
						if(pI2CHandle->Sr ==I2C_DISABLE_SR)
							I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

						//2. reset all member elements of the handle structure.
						I2C_CloseSendData(pI2CHandle);

						//3. notify the app about transmission complete
						I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}

			}
		}
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
			 	 //make sure that RXNE is also set
				; //we don't close com
			}
		}
	temp3=0;
	//4 Handle for interrupt generated by STOPF: Stop detecting flag is applicable only slave mode.
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<< I2C_SR1_STOPF);
	if(temp1 && temp3)
		{
			//handle STOPF bit interrupt
			//clear the STOPF - read sr1 and write to cr1
			pI2CHandle->pI2Cx->I2C_CR1 |= 0x0000;
			//notify the app that stop is detected
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);

		}
	temp3=0;

	//5 Handle for interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<< I2C_SR1_TXE);
	if(((temp1 && temp2 && temp3) == SET))
	{
		//handle TXE event
		//write to dr register
		if( pI2CHandle->TxRxState == I2C_BUSY_IN_TX && pI2CHandle->TxLen > 0 && (pI2CHandle->pI2Cx->I2C_SR2 & ( 1<< I2C_SR2_MSL)))
		{
			pI2CHandle->pI2Cx->I2C_DR = *(pI2CHandle->pTxBuffer);
			pI2CHandle->pTxBuffer++;
			pI2CHandle->TxLen--;
		}
	}
	temp3 =0;
	//6 handle for interrupt generated by RXNE Event
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & ( 1<< I2C_SR1_RXNE);
	if(((temp1 && temp2 && temp3) == SET))
	{
		if( pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
		{	//i2c is maste r
				if( pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
				{
					I2C_MasterHandleRXNEInterrupt(pI2CHandle);
				}
		}

	}
}
/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the code also define these macros in the driver
						header file
						#define I2C_ERROR_BERR  3
						#define I2C_ERROR_ARLO  4
						#define I2C_ERROR_AF    5
						#define I2C_ERROR_OVR   6
						#define I2C_ERROR_TIMEOUT 7

 */

void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->I2C_CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_ARLO);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_AF);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_OVR);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_TIMEOUT);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}



/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)

	{
		pI2Cx->I2C_CR1 |= ( 1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->I2C_CR1 &= ~( 1 << I2C_CR1_PE);
	}
}
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
{
	if(pI2Cx->I2C_SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)

		{
			pI2Cx->I2C_CR1|= ( 1 << I2C_CR1_ACK);
		}
		else
		{
			pI2Cx->I2C_CR1 &= ~( 1 << I2C_CR1_ACK);
		}
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{

}

/*
 * Application callback
 */




