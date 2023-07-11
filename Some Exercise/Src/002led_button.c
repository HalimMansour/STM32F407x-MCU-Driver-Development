/*
 * 002led_button.c
 *
 *
 *      Author: Halim
 */


#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"


#define HIGH			1
#define LOW				0
#define BTN_PRESSED		HIGH




void delay(void)
{
	for( uint32_t i = 0 ; i < 500000 ; i++);
}


int main(void)
{
	GPIO_Handle_t GpioBtn,GpioLed;


	// LED GPIO Configuration
	GpioLed.pGPIOX = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_H;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	//					-----
	GPIO_PeriClkControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);



	// Button GPIO Configuration
	GpioBtn.pGPIOX = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_H;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	//					-----
	GPIO_PeriClkControl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);


	while(1)
	{
		if(GPIO_ReadFromInPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED )
		{
			delay();
			GPIO_ToggleOutPin(GPIOD, GPIO_PIN_NO_12);
		}

	}

	return 0;
}
