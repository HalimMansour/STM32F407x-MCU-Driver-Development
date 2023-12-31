/*
 * stm32f407xx_spi_driver.h
 *
 *
 *      Author: Halim
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include <stm32f407xx.h>

/*
 * 	SPI Peripheral Configuration Structure
 */

typedef struct
{

	uint8_t SPI_DeviceMode; 	 	/* !< possible value from @SPI_DeviceModes > */
	uint8_t SPI_BusConfig;	 		/* !< possible value from @SPI_BUS_CONFIG > */
	uint8_t SPI_SclkSpeed;			/* !< possible value from @SPI_SclkSpeeds > */
	uint8_t SPI_DFF;				/* !< possible value from @SPI_DFF > */
	uint8_t SPI_CPOL;				/* !< possible value from @SPI_CPOL > */
	uint8_t SPI_CPHA;				/* !< possible value from @SPI_CPHA > */
	uint8_t SPI_SSM;				/* !< possible value from @SPI_SSM > */

}SPI_PinConfig_t;




/*
 * 	SPI Peripheral Handle Structure
 */

typedef struct
{
	SPI_RegDef_t 		*pSPIX;   /*!< This holds the base address of SPIx(x:0,1,2) peripheral >*/
	SPI_PinConfig_t 	SPI_PinConfig;
	uint8_t 			*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 			*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 			TxLen;		/* !< To store Tx len > */
	uint32_t 			RxLen;		/* !< To store Tx len > */
	uint8_t 			TxState;	/* !< To store Tx state > */
	uint8_t 			RxState;	/* !< To store Rx state > */
}SPI_Handle_t;



/*
 * SPI application states
 */
#define SPI_READY 					0
#define SPI_BUSY_IN_RX 				1
#define SPI_BUSY_IN_TX 				2


/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4


/*
 * 	@SPI_DeviceModes
 *  SPI possible modes
 */

#define SPI_MODE_MASTER				 	1
#define SPI_MODE_SLAVE				 	0




/*
 * 	@SPI_BUS_CONFIG
 *  SPI possible bus types
 */

#define SPI_BUS_CONFIG_FD			 		1		// Full-duplex
#define SPI_BUS_CONFIG_HD			 		2		// Half-duplex
#define SPI_BUS_CONFIG_S_RX			 		3		// Simplex Receive Only




/*
 * 	@SPI_SclkSpeeds
 *  SPI possible SCLK speeds
 */

#define SPI_SCLK_SPEED_DIV2				 	0
#define SPI_SCLK_SPEED_DIV4				 	1
#define SPI_SCLK_SPEED_DIV8				 	2
#define SPI_SCLK_SPEED_DIV16			 	3
#define SPI_SCLK_SPEED_DIV32			 	4
#define SPI_SCLK_SPEED_DIV64			 	5
#define SPI_SCLK_SPEED_DIV128			 	6
#define SPI_SCLK_SPEED_DIV256			 	7




/*
 * 	@SPI_DFF
 *  SPI possible data frame format
 */

#define SPI_DFF_8BITS					 	0
#define SPI_DFF_16BITS					 	1




/*
 * 	@SPI_CPOL
 *  SPI clock polarity
 */

#define SPI_CPOL_HIGH					 	1
#define SPI_CPOL_LOW					 	0




/*
 * 	@SPI_CPHA
 *  SPI clock phase
 */

#define SPI_CPHA_HIGH					 	1
#define SPI_CPHA_LOW					 	0




/*
 * 	@SPI_SSM
 *  SPI software slave management
 */

#define SPI_SSM_EN					 	1
#define SPI_SSM_DI					 	0




/*
 *  SPI related status flags definitions
 */
#define SPI_TXE_FLAG 		(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG 		(1 << SPI_SR_RXNE)
#define SPI_BSY_FLAG 		(1 << SPI_SR_BSY)
#define SPI_CHSIDE_FLAG 	(1 << SPI_SR_CHSIDE)
#define SPI_CRCEER_FLAG 	(1 << SPI_SR_CRCEER)
#define SPI_FRE_FLAG 		(1 << SPI_SR_FRE)
#define SPI_MODF_FLAG 		(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG 		(1 << SPI_SR_OVR)
#define SPI_UDR_FLAG 		(1 << SPI_SR_UDR)


/*************************************************************************************************************************
 *
 * 										APIs supported by this driver
 * 					For more information about the APIs you can check the function definitions
 *
 *************************************************************************************************************************/

/*
 * 	Peripheral Clock Setup
 */
void SPI_PeriClkControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);




/*
 * 	initialization & De-initialization
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);




/*
 * 	Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx , uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle , uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);



/*
 * 	IRQ Configuration and ISR Handling
 */
void SPI_IRQIntConfig(uint8_t IRQNum, uint8_t EnOrDi );
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
void SPI_IRQPriorityConfig (uint8_t IRQNum, uint8_t IRQPriority);



/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx , uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx , uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx , uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
