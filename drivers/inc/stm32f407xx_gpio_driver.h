/*
 * stm32f407xx_gpio_driver.h
 *
 *
 *      Author: Halim
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include <stm32f407xx.h>


/*
 * 	GPIO Peripheral Configuration Structure
 */

typedef struct
{

	uint8_t GPIO_PinNumber; 	 	/* !< possible value from @GPIO_PIN_NUMBERS > */
	uint8_t GPIO_PinMode;	 		/* !< possible value from @GPIO_PIN_MODES > */
	uint8_t GPIO_PinSpeed;			/* !< possible value from @GPIO_PIN_OUT_SPEEDS > */
	uint8_t GPIO_PinPuPdControl;	/* !< possible value from @GPIO_PIN_PUPD > */
	uint8_t GPIO_PinOPType;			/* !< possible value from @GPIO_PIN_OUT_TYPES > */
	uint8_t GPIO_PinAltFunMode;


}GPIO_PinConfig_t;



/*
 * 	GPIO Peripheral Handle Structure
 */

typedef struct
{

	GPIO_RegDef_t *pGPIOX; 					// Pointer to hold the base address of GPIO
	GPIO_PinConfig_t GPIO_PinConfig;		// Holds GPIO pin configuration settings

}GPIO_Handle_t;



/*
 * 	@GPIO_PIN_NUMBERS
 *  GPIO pin numbers
 */

#define GPIO_PIN_NO_0				 	0
#define GPIO_PIN_NO_1				 	1
#define GPIO_PIN_NO_2				 	2
#define GPIO_PIN_NO_3				 	3
#define GPIO_PIN_NO_4				 	4
#define GPIO_PIN_NO_5				 	5
#define GPIO_PIN_NO_6				 	6
#define GPIO_PIN_NO_7				 	7
#define GPIO_PIN_NO_8				 	8
#define GPIO_PIN_NO_9				 	9
#define GPIO_PIN_NO_10				 	10
#define GPIO_PIN_NO_11				 	11
#define GPIO_PIN_NO_12				 	12
#define GPIO_PIN_NO_13				 	13
#define GPIO_PIN_NO_14				 	14
#define GPIO_PIN_NO_15				 	15






/*
 * 	@GPIO_PIN_MODES
 *  GPIO pin possible modes
 */

#define GPIO_MODE_IN				 	0
#define GPIO_MODE_OUT					1
#define GPIO_MODE_ALTEN					2
#define GPIO_MODE_ANALOG				3
#define GPIO_MODE_IT_FT					4		// falling edge trigger
#define GPIO_MODE_IT_RT					5		// rising edge trigger
#define GPIO_MODE_IT_RFT				6		// rising falling edge trigger




/*
 * 	@GPIO_PIN_OUT_TYPES
 *  GPIO pin possible output types
 */

#define GPIO_OP_TYPE_PP			 		0		// push-pull
#define GPIO_OP_TYPE_OD			 		1		// open-drain




/*
 * 	@GPIO_PIN_OUT_SPEEDS
 *  GPIO pin possible output speeds
 */

#define GPIO_OP_SPEED_L			 		0		// Low
#define GPIO_OP_SPEED_M			 		1		// Medium
#define GPIO_OP_SPEED_H			 		2		// High
#define GPIO_OP_SPEED_VH		 		3		// Very high




/*
 *  @GPIO_PIN_PUPD
 *  GPIO pin pull-up/pull-down configuration macros
 */

#define GPIO_NO_PUPD			 		0		//  No pull-up, pull-down
#define GPIO_PIN_PU					 	1		//  Pull-up
#define GPIO_PIN_PD					 	2		//  Pull-down









/*************************************************************************************************************************
 *
 * 										APIs supported by this driver
 * 					For more information about the APIs you can check the function definitions
 *
 *************************************************************************************************************************/

/*
 * 	Peripheral Clock Setup
 */
void GPIO_PeriClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

/*
 * 	initialization & De-initialization
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * 	Data Read & Write
 */
uint8_t GPIO_ReadFromInPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber , uint8_t Value );
void GPIO_WriteToOutPort(GPIO_RegDef_t *pGPIOx , uint16_t Value);
void GPIO_ToggleOutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * 	IRQ Configuration and ISR Handling
 */
void GPIO_IRQIntConfig(uint8_t IRQNum, uint8_t EnOrDi );
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig (uint8_t IRQNum, uint8_t IRQPriority);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
