/*
 * 004button_interrupt.c
 *
 *
 *      Author: Halim
 */


#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "string.h"

#define HIGH			1
#define LOW				0
#define BTN_PRESSED		LOW



void delay(void)
{
	for( uint32_t i = 0 ; i < 500000/2 ; i++);
}


int main(void)
{
	GPIO_Handle_t GpioBtn,GpioLed;
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioBtn,0,sizeof(GpioBtn));

	// LED GPIO Configuration
	GpioLed.pGPIOX = GPIOD	;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_L;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	//					-----
	GPIO_PeriClkControl(GPIOD, ENABLE);
	GPIO_Init(&GpioLed);



	// Button GPIO Configuration
	GpioBtn.pGPIOX = GPIOD;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_SPEED_H;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	//					-----
	GPIO_PeriClkControl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);


	// IRQ Configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15);
	GPIO_IRQIntConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1);


	return 0;
}

void EXTI9_5_IRQHandler()
{
	delay();			// 200ms Delay
	GPIO_IRQHandling(GPIO_PIN_NO_5);
	GPIO_ToggleOutPin(GPIOD, GPIO_PIN_NO_12);


}
