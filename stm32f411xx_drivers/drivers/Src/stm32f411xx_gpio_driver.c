/*
 * stm32f411xx_gpio_driver.c
 *
 *  Created on: 25 May 2020
 *      Author: Dan
 */


#include "stm32f411xx_gpio_driver.h"

/** @fn - GPIO_PeriClockControl
  * @brief -    Enable or disable GPIO Peripheral clock
  * @param  - GPIO_RegDef_t *pGPIOx handle on GPIO configuration.  uint8_t En_or_Di enable or disable byte.
  * @retval None
  * @note - .
  */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t En_or_Di)
{
	if(En_or_Di == ENABLE)
	{
		if(pGPIOx ==GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx==GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx==GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx==GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx==GPIOE)
		{
			GPIOE_PCLK_EN();
		}

		else if (pGPIOx==GPIOH)
		{
			GPIOH_PCLK_EN();
		}

	}
	else
	{
		if(pGPIOx ==GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx==GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx==GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx==GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx==GPIOE)
		{
			GPIOE_PCLK_DI();
		}

		else if (pGPIOx==GPIOH)
		{
			GPIOH_PCLK_DI();
		}

	}
}

	// Init and Deinit
/** @fn - GPIO_Init
  * @brief -    Initialize GPIO Port
  * @param  - GPIO_RegDef_t *pGPIOx handle on GPIO configuration.
  * @retval None
  * @note - .
  */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	volatile uint32_t temp=0; //temp variable.
	//1. Configure the mode of gpio pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER&= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear bit
		pGPIOHandle->pGPIOx->MODER|=temp;
		temp=0;
	}
	else
	{
		// TODO // //interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// 1. CONFIGURE the FTSR

			EXTI->EXTI_FTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->EXTI_RTSR &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1. conffigure the RTSR

			EXTI->EXTI_RTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->EXTI_FTSR &= ~(1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			//1. configure the FRTSR

			EXTI->EXTI_FTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
			EXTI->EXTI_RTSR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
		}
		//2. CONFIGURE THE gpio PORT SELECTION IN syscfg_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->SYSCFG_EXTICR[temp1] |= portcode << (temp2 *4);

		//3. enable the EXTI interrupt delivery using IMR
		EXTI->EXTI_IMR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	}
	//2. configure the speed
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR&= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear bit
		pGPIOHandle->pGPIOx->OSPEEDR|=temp;
		temp=0;


	//3. configure te pupd settings
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->PUPDR&= ~(0x3 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clear bit
		pGPIOHandle->pGPIOx->PUPDR= (uint32_t)((pGPIOHandle->pGPIOx->PUPDR) | temp);
		temp=0;
	//4. configure the optype
		temp  = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->OTYPER&= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clear bit
		pGPIOHandle->pGPIOx->OTYPER|=temp;

		temp =0;
	//5. configure the alt functionality
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
		{
			uint8_t temp1, temp2;
			temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /8;
			temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
			pGPIOHandle->pGPIOx->AFR[temp1] &= ~((uint8_t)0xF<<( 4 * temp2) ); //clearing
			pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle -> GPIO_PinConfig.GPIO_PinAltFunMode <<( 4 * temp2) );

		}

}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx ==GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx==GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx==GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx==GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx==GPIOE)
	{
		GPIOE_REG_RESET();
	}

	else if (pGPIOx==GPIOH)
	{
		GPIOH_REG_RESET();
	}

}


	// Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_Handle_t *pGPIOHandle, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOHandle->pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_Handle_t *pGPIOHandle)
{
	uint16_t value;
	value = (uint16_t)(pGPIOHandle->pGPIOx->IDR);
	return value;
}
void GPIO_WriteToOutputPin(GPIO_Handle_t *pGPIOHandle, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
		{
			pGPIOHandle->pGPIOx->ODR |= (1 << PinNumber);
		}
	else
		{
			pGPIOHandle->pGPIOx->ODR &= ~(1 << PinNumber);
		}
}
void GPIO_WriteToOutputPort(GPIO_Handle_t *pGPIOHandle, uint16_t Value)
{
	pGPIOHandle->pGPIOx->ODR |= Value;
}
void GPIO_ToggleOutputPin(GPIO_Handle_t *pGPIOHandle, uint8_t PinNumber)
{
	pGPIOHandle->pGPIOx->ODR ^= (1<< PinNumber);
}

	// IRQ configuration and ISR handling.
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,  uint8_t En_or_Di)
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// first lets find out the IPR register
	uint8_t iprx = IRQNumber/4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR +iprx) |= (IRQPriority << shift_amount);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear EXTI pr bit.
	if(EXTI->EXTI_PR & (1 << PinNumber))
	{
		// INCASE OF PR YOU CLEAR BY TOUCHING THE PIN NUMBER BIT
		EXTI->EXTI_PR |= (1<<PinNumber);
	}

}



