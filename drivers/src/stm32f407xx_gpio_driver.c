/*
 * stm32f407xx_gpio_driver.c
 *
 *
 *      Author: Halim
 */

#include <stm32f407xx.h>

/**********			**********			**********			**********			**********			**********/

/*
 * 												Peripheral Clock Setup
 */

/*************************************************************************************************************
 * @brief Enables or disables the peripheral clock for a given GPIO port.
 *
 * @param pGPIOx: Pointer to the GPIO port register definition structure.
 * @param EnOrDi: Enable or disable the peripheral clock for the given GPIO port.
 *                0: Disable the peripheral clock.
 *                1: Enable the peripheral clock.
 *
 * @return 			-			none.
 *
 * @note 			-			The macro definitions GPIOA_PCLK_EN() and GPIOA_PCLK_DI() are defined in stm32f407xx.h.
 *************************************************************************************************************/

void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        if (pGPIOx == GPIOA)
        {
        	GPIOA_PCLK_EN();
        }
        else if (pGPIOx == GPIOB)
        {
        	GPIOB_PCLK_EN();
        }
        else if (pGPIOx == GPIOC)
        {
        	GPIOC_PCLK_EN();
        }
        else if (pGPIOx == GPIOD)
        {
        	GPIOD_PCLK_EN();
        }
        else if (pGPIOx == GPIOE)
        {
        	GPIOE_PCLK_EN();
        }
        else if (pGPIOx == GPIOF)
        {
        	GPIOF_PCLK_EN();
        }
        else if (pGPIOx == GPIOG)
        {
        	GPIOG_PCLK_EN();
        }
        else if (pGPIOx == GPIOH)
        {
        	GPIOH_PCLK_EN();
        }
        else if (pGPIOx == GPIOI)
        {
        	GPIOI_PCLK_EN();
        }

    }
    else
    {
        if (pGPIOx == GPIOA)
        {
        	GPIOA_PCLK_DI();
        }
        else if (pGPIOx == GPIOB)
        {
        	GPIOB_PCLK_DI();
        }
        else if (pGPIOx == GPIOC)
        {
        	GPIOC_PCLK_DI();
        }
        else if (pGPIOx == GPIOD)
        {
        	GPIOD_PCLK_DI();
        }
        else if (pGPIOx == GPIOE)
        {
        	GPIOE_PCLK_DI();
        }
        else if (pGPIOx == GPIOF)
        {
        	GPIOF_PCLK_DI();
        }
        else if (pGPIOx == GPIOG)
        {
        	GPIOG_PCLK_DI();
        }
        else if (pGPIOx == GPIOH)
        {
        	GPIOH_PCLK_DI();
        }
        else if (pGPIOx == GPIOI)
        {
        	GPIOI_PCLK_DI();
        }


    }
}

/**********			**********			**********			**********			**********			**********/

/*
 * 											initialization & De-initialization
 */

/*************************************************************************************************************
 * @brief Initializes a GPIO pin with the specified configuration.
 *
 * @param pGPIOHandle: Pointer to the GPIO handle structure containing the configuration for the GPIO pin.
 *
 * @return 			-			none.
 *
 * @note 			-			none.
 *************************************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp=0; 		//temp register value

	// Enable the peripheral clock

	GPIO_PeriClkControl(pGPIOHandle->pGPIOX, ENABLE);

	//1. Configure the mode of GPIO
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber	) );
		pGPIOHandle->pGPIOX->MODER &= ~(0x3);
		pGPIOHandle->pGPIOX->MODER |= temp;

	}
	else
	{
		//  interrupt mode

		//	1. Configure the mode (FTSR, RTSR, FTSR & RTSR)
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |=  ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding RTSR bit
			EXTI->RTSR &=  ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			EXTI->RTSR |=  ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding FTSR bit
			EXTI->FTSR &=  ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			EXTI->RTSR |=  ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding FTSR bit
			EXTI->FTSR |=  ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}


		//	2. Configure the GPIO port selection in SYSCFG_EXTICR
		//uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
		//uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4 ;
		//uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOX);
		//SYSCFG_PCLK_EN();
		//SYSCFG->EXTICR[temp1] ( portcode << (temp2 * 4) );


		//	3. Enable EXTI interrupt delivery using IMR

		EXTI->IMR |=  ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);



	}

	temp = 0;

	//2. Configure the speed
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber	) );
	pGPIOHandle->pGPIOX->OSPEEDR &= ~(0x3);
	pGPIOHandle->pGPIOX->OSPEEDR |= temp;


	temp = 0;

	//3. Configure the pull-up/pull-down settings
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber	) );
	pGPIOHandle->pGPIOX->PUPDR &= ~(0x3);
	pGPIOHandle->pGPIOX->PUPDR |= temp;

	temp = 0;

	//4. Configure the output type
	temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << ( 1 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber	) );
	pGPIOHandle->pGPIOX->OTYPER &= ~(0x1);
	pGPIOHandle->pGPIOX->OTYPER |= temp;

	temp = 0;

	//5. Configure the alternative functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTEN)
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8 )
		{
			temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber	) );
			pGPIOHandle->pGPIOX->AFRL &= ~(0xF);
			pGPIOHandle->pGPIOX->AFRL |= temp;

		}
		else
		{
			temp = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8)	) );
			pGPIOHandle->pGPIOX->AFRH &= ~(0xF);
			pGPIOHandle->pGPIOX->AFRH |= temp;
		}

		temp = 0;
	}




}

/*************************************************************************************************************
 * @brief Resets the GPIO port to its default state.
 *
 * @param pGPIOx: Pointer to the GPIO port to be reset.
 *
 * @return 			-			none.
 *
 * @note 			-			none.
 *************************************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
    	GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPIOB)
    {
    	GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
    	GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
    	GPIOD_REG_RESET();
    }
    else if (pGPIOx == GPIOE)
    {
    	GPIOE_REG_RESET();
    }
    else if (pGPIOx == GPIOF)
    {
    	GPIOF_REG_RESET();
    }
    else if (pGPIOx == GPIOG)
    {
    	GPIOG_REG_RESET();
    }
    else if (pGPIOx == GPIOH)
    {
    	GPIOH_REG_RESET();
    }
    else if (pGPIOx == GPIOI)
    {
    	GPIOI_REG_RESET();
    }

}




/**********			**********			**********			**********			**********			**********/

/*
 * 													Data Read & Write
 */

/*************************************************************************************************************
 * @brief Reads the input value of a specific GPIO pin.
 *
 * @param pGPIOx: Pointer to the GPIO port of the pin to be read.
 * @param PinNumber: Pin number of the pin to be read.
 *
 * @return The input value of the specified GPIO pin (0 or 1).
 *
 * @note 			-			none.
 *************************************************************************************************************/
uint8_t GPIO_ReadFromInPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value ;
	value = (uint8_t )((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;

}




/*************************************************************************************************************
 * @brief Reads the input value of the entire GPIO port.
 *
 * @param pGPIOx: Pointer to the GPIO port to be read.
 *
 * @return The input value of the GPIO port (16-bit integer).
 *
 * @note 			-			none.
 *************************************************************************************************************/
uint16_t GPIO_ReadFromInPort(GPIO_RegDef_t *pGPIOx)
{
		uint16_t value ;
		value = (uint16_t )pGPIOx->IDR ;
		return value;

}




/*************************************************************************************************************
 * @brief Writes a value to a specific GPIO output pin.
 *
 * @param pGPIOx: Pointer to the GPIO port of the pin to be written.
 * @param PinNumber: Pin number of the pin to be written.
 * @param Value: Value to be written to the specified GPIO pin (0 or 1).
 *
 * @return 			-			none.
 *
 * @note 			-			none.
 *************************************************************************************************************/
void GPIO_WriteToOutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber , uint8_t Value )
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);

	}
	else
	{
		pGPIOx->ODR &= (0 << PinNumber);
	}


}




/*************************************************************************************************************
 * @brief Writes a value to the entire GPIO output port.
 *
 * @param pGPIOx: Pointer to the GPIO port to be written.
 * @param Value: Value to be written to the GPIO port (16-bit integer).
 *
 * @return 			-			none.
 *
 * @note 			-			none.
 *************************************************************************************************************/
void GPIO_WriteToOutPort(GPIO_RegDef_t *pGPIOx , uint16_t Value)
{
	pGPIOx->ODR &= ~(0xFFFF);
	pGPIOx->ODR |= Value;
}

/*************************************************************************************************************
 * @brief Toggles the value of a specific GPIO output pin.
 *
 * @param pGPIOx: Pointer to the GPIO port of the pin to be toggled.
 * @param PinNumber: Pin number of the pin to be toggled.
 *
 * @return 			-			none.
 *
 * @note 			-			none.
 *************************************************************************************************************/
void GPIO_ToggleOutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

	pGPIOx->ODR ^= (1<< PinNumber);

}




/**********			**********			**********			**********			**********			**********/

/*
 * 													IRQ Configuration and ISR Handling
 */

/*************************************************************************************************************
 * @brief Configures the interrupt settings for a specific GPIO pin.
 *
 * @param IRQNum: IRQ number to be configured.
 * @param IRQPrio: Priority to be set for the interrupt.
 * @param EnOrDi: Enable or disable the interrupt (1 = enable, 0 = disable).
 *
 * @return 			-			none.
 *
 * @note 			-			none.
 *************************************************************************************************************/
void GPIO_IRQIntConfig(uint8_t IRQNum, uint8_t EnOrDi )
{
	if(EnOrDi == ENABLE)
	{
		if(IRQNum <= 31)
		{
			//Program ISER0 Register
			*NVIC_ISER0  |= (1 << IRQNum );
		}
		else if(IRQNum > 31 && IRQNum < 64)
		{
			//Program ISER1 Register
			*NVIC_ISER1  |= (1 << IRQNum % 32);
		}
		else if(IRQNum > 64 && IRQNum < 96)
		{
			//Program ISER2 Register
			*NVIC_ISER2  |= (1 << IRQNum % 32);
		}
	}
	else
	{
		if(IRQNum <= 31)
		{
			//Program ICER0 Register
			*NVIC_ICER0  |= (1 << IRQNum );
		}
		else if(IRQNum > 31 && IRQNum < 64)
		{
			//Program ICER1 Register
			*NVIC_ICER1  |= (1 << IRQNum % 32 );

		}
		else if(IRQNum > 64 && IRQNum < 96)
		{
			//Program ICER2 Register
			*NVIC_ICER2  |= (1 << IRQNum % 32 );
		}
	}


}




/*************************************************************************************************************
 * @brief Configures the priority of a given GPIO Interrupt request (IRQ).
 * This function sets the priority of a given GPIO IRQ using the Interrupt Priority Register (IPR) of the Nested Vectored Interrupt Controller (NVIC).
 *
 * @param IRQNum: The IRQ number for which the priority needs to be configured.
 * @param IRQPriority: The priority level to be assigned to the IRQ.
 *
 * @return 			-			none.
 *
 * @note 			-			none.
 *************************************************************************************************************/
void GPIO_IRQPriorityConfig (uint8_t IRQNum, uint8_t IRQPriority)
{
	// IPR register
	uint8_t iprx = IRQNum / 4;
	uint8_t iprx_section = IRQNum % 4 ;
	uint8_t shift_amount =  (8 * iprx_section) + (8-NO_PR_BITS_IMP);

	*(NVIC_PR_BASEADDR + (iprx)) |= 	IRQPriority << shift_amount;

}




/*************************************************************************************************************
 * @brief Handles the interrupt for a specific GPIO pin.
 *
 * @param PinNumber: Pin number of the GPIO pin for which the interrupt was triggered.
 *
 * @return 			-			none.
 *
 * @note 			-			none.
 *************************************************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR &= ~(1 << PinNumber))
	{
		// Clear
		EXTI->PR |= (1 << PinNumber);

	}

}
