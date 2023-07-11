/*
 * stm32f407xx_spi_driver.c
 *
 *
 *      Author: Halim
 */
#include <stm32f407xx.h>

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*********************************************************************
 *
 */


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIX , uint32_t FlagName)
{
	if(pSPIX->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}



/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIX , uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIX->CR1 |= (1<< SPI_CR1_SPE);
	}
	else
	{
		pSPIX->CR1 &= ~(1<< SPI_CR1_SPE);
	}

}




void SPI_SSIConfig(SPI_RegDef_t *pSPIX , uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIX->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIX->CR1 &= ~(1<< SPI_CR1_SPE);
	}

}




void SPI_SSOEConfig(SPI_RegDef_t *pSPIX , uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIX->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIX->CR2 &= ~(1<< SPI_CR2_SSOE);
	}

}




/**********			**********			**********			**********			**********			**********/

/*
 * 												Peripheral Clock Setup
 */

/*************************************************************************************************************
 * @brief Enables or disables the peripheral clock for a specific SPI module.
 *
 * @param pSPIX: Pointer to the SPI module's register definition structure.
 * @param EnOrDi: Enable or disable the peripheral clock.
 *                0: Disable the peripheral clock.
 *                1: Enable the peripheral clock.
 *
 * @return 			-			none.
 *
 * @note 			-			The macro definitions SPIx_PCLK_EN() and SPIx_PCLK_DI() are defined in stm32f407xx.h.
 *************************************************************************************************************/

void SPI_PeriClkControl(SPI_RegDef_t *pSPIX, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        if (pSPIX == SPI1)
        {
        	SPI1_PCLK_EN();
        }
        else if (pSPIX == SPI2)
        {
        	SPI2_PCLK_EN();
        }
        else if (pSPIX == SPI3)
        {
        	SPI3_PCLK_EN();
        }
        else if (pSPIX == SPI4)
        {
        	SPI4_PCLK_EN();
        }

    }

    else
    {
        if (pSPIX == SPI1)
        {
        	SPI1_PCLK_DI();
        }
        else if (pSPIX == SPI2)
        {
        	SPI2_PCLK_DI();
        }
        else if (pSPIX == SPI3)
        {
        	SPI3_PCLK_DI();
        }
        else if (pSPIX == SPI4)
        {
        	SPI4_PCLK_DI();
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
void SPI_Init(SPI_Handle_t *pSPIHandle )
{
	// Configure the SPI_CR1 register
	uint32_t tempreg = 0;

	// Peripheral clock enable
	SPI_PeriClkControl(pSPIHandle->pSPIX, ENABLE);

	//1. Configure the mode of SPI (Master, Slave)

	tempreg |= pSPIHandle->SPI_PinConfig.SPI_DeviceMode << SPI_CR1_MSTR;



	//2. Configure bus types of SPI
	if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

	}
	else if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// BIDI mode should be set
		tempreg |=  (1 << SPI_CR1_BIDIMODE);

	}
	else if(pSPIHandle->SPI_PinConfig.SPI_BusConfig == SPI_BUS_CONFIG_S_RX)
	{
		// BIDI mode should be cleared and RXONLY should be set
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		tempreg |=  (1 << SPI_CR1_RXONLY);
	}

	//3. Configure SPI SCLK speeds
	tempreg |= (pSPIHandle->SPI_PinConfig.SPI_SclkSpeed  << SPI_CR1_BR);


	//4. Select the data frame format (DFF)
	tempreg |=  (pSPIHandle->SPI_PinConfig.SPI_DFF << SPI_CR1_DFF);


	//5. Configure the CPOL
	tempreg |=  (pSPIHandle->SPI_PinConfig.SPI_CPOL << SPI_CR1_CPOL);


	//6. Configure the CPHA
	tempreg |=  (pSPIHandle->SPI_PinConfig.SPI_CPHA << SPI_CR1_CPHA);



	pSPIHandle->pSPIX->CR1 = tempreg;

}


/*************************************************************************************************************
 * @brief Resets the GPIO port to its default state.
 *
 * @param pSPIX: Pointer to the GPIO port to be reset.
 *
 * @return 			-			none.
 *
 * @note 			-			none.
 *************************************************************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIX)
{
    if (pSPIX == SPI1)
    {
    	SPI1_REG_RESET();
    }
    else if (pSPIX == SPI2)
    {
    	SPI2_REG_RESET();
    }
    else if (pSPIX == SPI3)
    {
    	SPI3_REG_RESET();
    }
    else if (pSPIX == SPI4)
    {
    	SPI4_REG_RESET();
    }

}



/**********			**********			**********			**********			**********			**********/

/*
 * 													Data Send & Receive
 */

/*************************************************************************************************************
 * @brief Sends data over SPI using the provided SPI handle and data buffer.
 *
 * @param pSPIX: Pointer to the SPI peripheral handle structure.
 * @param pTxBuffer: Pointer to the data buffer containing the data to be sent over SPI.
 * @param Len: The length of the data buffer.
 *
 * @return			-			none.
 *
 * @note 			-			This is blocking call.
 *************************************************************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIX , uint8_t *pTxBuffer , uint32_t Len)
{
	while(Len > 0)
	{
		//1. Wait until TXE is set
		while(	SPI_GetFlagStatus(pSPIX,SPI_TXE_FLAG) == FLAG_RESET	);

		//2. Check the data frame format (DFF)
		if(pSPIX->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16-bit data frame format is selected
			// 1. load the data in data register DR
			pSPIX->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			// 8-bit data frame format is selected
			pSPIX->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}

}


/*************************************************************************************************************
@brief Receives data from the SPI port.

@param pSPIX: Pointer to the SPI port from which data is to be received.

@param pRxBuffer: Pointer to the buffer where received data will be stored.

@param Len: Number of data bytes to be received.

@return			-			none.

@note This function waits until the specified number of data bytes is received from the SPI port.
 *************************************************************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIX, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			//1. wait until RXNE is set
			while(SPI_GetFlagStatus(pSPIX,SPI_RXNE_FLAG)  == (uint8_t)FLAG_RESET );

			//2. check the DFF bit in CR1
			if( (pSPIX->CR1 & ( 1 << SPI_CR1_DFF) ) )
			{
				//16 bit DFF
				//1. load the data from DR to Rxbuffer address
				 *((uint16_t*)pRxBuffer) = pSPIX->DR ;
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}else
			{
				//8 bit DFF
				*(pRxBuffer) = pSPIX->DR ;
				Len--;
				pRxBuffer++;
			}
		}

}



/**********			**********			**********			**********			**********			**********/

/*
 * 													IRQ Configuration and ISR Handling
 */

	/*************************************************************************************************************
	 * @brief Configures the interrupt settings for a specific IRQ number.
	 *
	 * @param IRQNum: IRQ number to be configured.
	 * @param EnOrDi: Enable or disable the interrupt (1 = enable, 0 = disable).
	 *
	 * @return None.
	 *
	 * @note None.
	 *************************************************************************************************************/
	void SPI_IRQIntConfig(uint8_t IRQNum, uint8_t EnOrDi )
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
	 * @brief Handles the SPI interrupt.
	 *
	 * @param pSPIHandle: Pointer to the SPI handle structure.
	 *
	 * @return None.
	 *
	 * @note None.
	 *************************************************************************************************************/
	void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
	{

		uint8_t temp1 , temp2;
		//first lets check for TXE
		temp1 = pSPIHandle->pSPIX->SR & ( 1 << SPI_SR_TXE);
		temp2 = pSPIHandle->pSPIX->CR2 & ( 1 << SPI_CR2_TXEIE);

		if( temp1 && temp2)
		{
			//handle TXE
			spi_txe_interrupt_handle(pSPIHandle);
		}

		// check for RXNE
		temp1 = pSPIHandle->pSPIX->SR & ( 1 << SPI_SR_RXNE);
		temp2 = pSPIHandle->pSPIX->CR2 & ( 1 << SPI_CR2_RXNEIE);

		if( temp1 && temp2)
		{
			//handle RXNE
			spi_rxne_interrupt_handle(pSPIHandle);
		}

		// check for ovr flag
		temp1 = pSPIHandle->pSPIX->SR & ( 1 << SPI_SR_OVR);
		temp2 = pSPIHandle->pSPIX->CR2 & ( 1 << SPI_CR2_ERRIE);

		if( temp1 && temp2)
		{
			//handle ovr error
			spi_ovr_err_interrupt_handle(pSPIHandle);
		}


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
	void SPI_IRQPriorityConfig (uint8_t IRQNum, uint8_t IRQPriority)
	{
		//1. first lets find out the ipr register
		uint8_t iprx = IRQNum / 4;
		uint8_t iprx_section  = IRQNum %4 ;

		uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMP) ;

		*(  NVIC_PR_BASEADDR + iprx ) |=  ( IRQPriority << shift_amount );

	}




	uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
	{
		uint8_t state = pSPIHandle->TxState;

		if(state != SPI_BUSY_IN_TX)
		{
			//1 . Save the Tx buffer address and Len information in some global variables
			pSPIHandle->pTxBuffer = pTxBuffer;
			pSPIHandle->TxLen = Len;
			//2.  Mark the SPI state as busy in transmission so that
			//    no other code can take over same SPI peripheral until transmission is over
			pSPIHandle->TxState = SPI_BUSY_IN_TX;

			//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
			pSPIHandle->pSPIX->CR2 |= ( 1 << SPI_CR2_TXEIE );

		}


		return state;
	}





	uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
	{
		uint8_t state = pSPIHandle->RxState;

		if(state != SPI_BUSY_IN_RX)
		{
			//1 . Save the Rx buffer address and Len information in some global variables
			pSPIHandle->pRxBuffer = pRxBuffer;
			pSPIHandle->RxLen = Len;
			//2.  Mark the SPI state as busy in reception so that
			//    no other code can take over same SPI peripheral until reception is over
			pSPIHandle->RxState = SPI_BUSY_IN_RX;

			//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
			pSPIHandle->pSPIX->CR2 |= ( 1 << SPI_CR2_RXNEIE );

		}


		return state;

	}



	//some helper function implementations

	static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
	{
		// check the DFF bit in CR1
		if( (pSPIHandle->pSPIX->CR1 & ( 1 << SPI_CR1_DFF) ) )
		{
			//16 bit DFF
			//1. load the data in to the DR
			pSPIHandle->pSPIX->DR =   *((uint16_t*)pSPIHandle->pTxBuffer);
			pSPIHandle->TxLen--;
			pSPIHandle->TxLen--;
			(uint16_t*)pSPIHandle->pTxBuffer++;
		}else
		{
			//8 bit DFF
			pSPIHandle->pSPIX->DR =   *pSPIHandle->pTxBuffer;
			pSPIHandle->TxLen--;
			pSPIHandle->pTxBuffer++;
		}

		if(! pSPIHandle->TxLen)
		{
			//TxLen is zero , so close the spi transmission and inform the application that
			//TX is over.

			//this prevents interrupts from setting up of TXE flag
			SPI_CloseTransmisson(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
		}

	}


	static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
	{
		//do rxing as per the dff
		if(pSPIHandle->pSPIX->CR1 & ( 1 << 11))
		{
			//16 bit
			*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIX->DR;
			pSPIHandle->RxLen -= 2;
			pSPIHandle->pRxBuffer++;
			pSPIHandle->pRxBuffer++;

		}else
		{
			//8 bit
			*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIX->DR;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer++;
		}

		if(! pSPIHandle->RxLen)
		{
			//reception is complete
			SPI_CloseReception(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
		}

	}


	static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
	{

		uint8_t temp;
		//1. clear the ovr flag
		if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
		{
			temp = pSPIHandle->pSPIX->DR;
			temp = pSPIHandle->pSPIX->SR;
		}
		(void)temp;
		//2. inform the application
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

	}


	void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
	{
		pSPIHandle->pSPIX->CR2 &= ~( 1 << SPI_CR2_TXEIE);
		pSPIHandle->pTxBuffer = NULL;
		pSPIHandle->TxLen = 0;
		pSPIHandle->TxState = SPI_READY;

	}

	void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
	{
		pSPIHandle->pSPIX->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
		pSPIHandle->pRxBuffer = NULL;
		pSPIHandle->RxLen = 0;
		pSPIHandle->RxState = SPI_READY;

	}



	void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIX)
	{
		uint8_t temp;
		temp = pSPIX->DR;
		temp = pSPIX->SR;
		(void)temp;

	}



	__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
	{

		//This is a weak implementation . the user application may override this function.
	}







