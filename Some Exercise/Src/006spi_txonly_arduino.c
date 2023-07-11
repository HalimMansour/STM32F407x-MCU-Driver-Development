/*s
 * 006spi_txonly_arduino.c
 *
 *
 *      Author: Halim
 */

#include "string.h"
#include "stm32f407xx.h"

// Alternate function mode : AF5
// PB15 - - - >	SPI2_MOSI
// PB14 - - - >	SPI2_MISO
// PB13 - - - >	SPI2_SCK
// PB12 - - - >	SPI2_NSS


void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOX = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTEN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_H;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;


	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	//GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIX = SPI2;
	SPI2Handle.SPI_PinConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_PinConfig.SPI_DeviceMode = SPI_MODE_MASTER;
	SPI2Handle.SPI_PinConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //2 MHz
	SPI2Handle.SPI_PinConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_PinConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_PinConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_PinConfig.SPI_SSM = SPI_SSM_DI; // Hardware slave management enable for NSS

	SPI_Init(&SPI2Handle);
}


void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioBtn;
	// Button GPIO Configuration
	GpioBtn.pGPIOX = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_H;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	//					-----
	GPIO_Init(&GpioBtn);
}

void delay(void)
{
	for( uint32_t i = 0 ; i < 500000 ; i++);
}


int main(void)
{
	char user_data[] = "Hello";

	GPIO_ButtonInit();

	// Initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInits();

	// Initialize the SPI2 peripheral parameters
	SPI2_Inits();

	while(1)
	{
		while(! GPIO_ReadFromInPin(GPIOA, GPIO_PIN_NO_0));

		delay();

		// Enable SPI2 SSOE (Slave Select Output Enable)
		SPI_SSOEConfig(SPI2,ENABLE);

		// Enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,ENABLE);

		// first send length information
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1 );

		// Send Data
		SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));

		// Check if SPI is not busy
		while( SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

		// Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);
	}

	return 0;

}


