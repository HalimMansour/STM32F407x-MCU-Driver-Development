/*
 * stm32f407xx.h
 *
 *  Created on: Apr 13, 2023
 *      Author: Halim
 */


#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stddef.h>
#include<stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/* * * * * * * * * * * * * * * * * * * * * * * * * Start: Processor Specific Details * * * * * * * * * * * * * * * * * * * * * * * * */

/*
 *  ARM Cortex Mx processor NVIC ISERx register addresses
 */

#define NVIC_ISER0							(__vo uint32_t*) 0xE000E100
#define NVIC_ISER1							(__vo uint32_t*) 0xE000E104
#define NVIC_ISER2							(__vo uint32_t*) 0xE000E108
#define NVIC_ISER3							(__vo uint32_t*) 0xE000E10C
#define NVIC_ISER4							(__vo uint32_t*) 0xE000E110
#define NVIC_ISER5							(__vo uint32_t*) 0xE000E114
#define NVIC_ISER6							(__vo uint32_t*) 0xE000E118
#define NVIC_ISER7							(__vo uint32_t*) 0xE000E11C


/*
 *  ARM Cortex Mx processor NVIC ICERx register addresses
 */

#define NVIC_ICER0							(__vo uint32_t*) 0xE000E180
#define NVIC_ICER1							(__vo uint32_t*) 0xE000E184
#define NVIC_ICER2							(__vo uint32_t*) 0xE000E188
#define NVIC_ICER3							(__vo uint32_t*) 0xE000E18C
#define NVIC_ICER4							(__vo uint32_t*) 0xE000E190
#define NVIC_ICER5							(__vo uint32_t*) 0xE000E194
#define NVIC_ICER6							(__vo uint32_t*) 0xE000E198
#define NVIC_ICER7							(__vo uint32_t*) 0xE000E19C


/*
 *  ARM Cortex Mx processor priority register addresses calculation
 */

#define NVIC_PR_BASEADDR					(__vo uint32_t*) 0xE000E400


#define NO_PR_BITS_IMP						4

/* * * * * * * * * * * * * * * * * * * * * * * * * End: Processor Specific Details * * * * * * * * * * * * * * * * * * * * * * * * */



// Define base addresses of memory sections

#define FLASH_BASEADDR						 0x08000000U
#define ROM_BASEADDR						 0x1FFF0000U					//	System Memory
#define SRAM1_BASEADDR						 0x20000000U
#define SRAM2_BASEADDR						 0x2001C000U
#define SRAM								 SRAM1_BASEADDRADDR


// Define base addresses of various bus domains

#define PERIPH_BASEADDR						 0x40000000U
#define APB1_BASEADDR						 PERIPH_BASEADDR					// 0x40000000U
#define APB2_BASEADDR						 (PERIPH_BASEADDR + 0x10000) 		// 0x40010000U
#define AHB1_BASEADDR						 (PERIPH_BASEADDR + 0x20000) 		// 0x40020000U
#define AHB2_BASEADDR						 (PERIPH_BASEADDR + 0x10000000) 	// 0x50000000U
#define AHB3_BASEADDR						 (PERIPH_BASEADDR + 0x60000000) 	// 0xA0000000U



// Define base addresses of peripherals hanging on AHB1 bus
// GPIO

#define GPIO_BASEADDR						 AHB1_BASEADDR						// 0x40020000U
#define GPIOA_BASEADDR						 (GPIO_BASEADDR + 0x00000000)		// 0x40020000U
#define GPIOB_BASEADDR						 (GPIO_BASEADDR + 0x00000400)		// 0x40020400U
#define GPIOC_BASEADDR						 (GPIO_BASEADDR + 0x00000800)		// 0x40020800U
#define GPIOD_BASEADDR						 (GPIO_BASEADDR + 0x00000C00)		// 0x40020C00U
#define GPIOE_BASEADDR						 (GPIO_BASEADDR + 0x00001000)		// 0x40021000U
#define GPIOF_BASEADDR						 (GPIO_BASEADDR + 0x00001400)		// 0x40021400U
#define GPIOG_BASEADDR						 (GPIO_BASEADDR + 0x00001800)		// 0x40021800U
#define GPIOH_BASEADDR						 (GPIO_BASEADDR + 0x00001C00)		// 0x40021C00U
#define GPIOI_BASEADDR						 (GPIO_BASEADDR + 0x00002000)		// 0x40022000U
#define GPIOJ_BASEADDR						 (GPIO_BASEADDR + 0x00002400)		// 0x40022400U
#define GPIOK_BASEADDR						 (GPIO_BASEADDR + 0x00002800)		// 0x40022800U


//Define base addresses of RCC

#define RCC_BASEADDR						 (AHB1_BASEADDR + 0x3800)			// 0x40023800U


// Define base addresses of peripherals hanging on APB1 bus

#define SPI2_BASEADDR						 (APB1_BASEADDR + 0x00003800)		// 0x40003800U
#define SPI3_BASEADDR						 (APB1_BASEADDR + 0x00003C00)		// 0x40003C00U

#define USART2_BASEADDR						 (APB1_BASEADDR + 0x00004400)		// 0x40004400U
#define USART3_BASEADDR						 (APB1_BASEADDR + 0x00004800)		// 0x40004800U
#define UART4_BASEADDR						 (APB1_BASEADDR + 0x00004C00)		// 0x40004800U
#define UART5_BASEADDR						 (APB1_BASEADDR + 0x00005000)		// 0x40004800U

#define I2C1_BASEADDR						 (APB1_BASEADDR + 0x00005400)		// 0x40005400U
#define I2C2_BASEADDR						 (APB1_BASEADDR + 0x00005800)		// 0x40005800U
#define I2C3_BASEADDR						 (APB1_BASEADDR + 0x00005C00)		// 0x40005C00U


// Define base addresses of peripherals hanging on APB2 bus

#define USART1_BASEADDR						 (APB2_BASEADDR + 0x00001000)		// 0x40011000U
#define USART6_BASEADDR						 (APB2_BASEADDR + 0x00001400)		// 0x40011400U
#define SPI1_BASEADDR						 (APB2_BASEADDR + 0x00003000)		// 0x40013000U
#define SPI4_BASEADDR						 (APB2_BASEADDR + 0x00003400)		// 0x40013400U
#define SYSCFG_BASEADDR						 (APB2_BASEADDR + 0x00003800)		// 0x40013800U
#define EXTI_BASEADDR						 (APB2_BASEADDR + 0x00003C00)		// 0x40013C00U



//-----------------------------------------------------------------------------------------------------------


// Peripheral register definition structure for GPIO

typedef struct
{
	__vo uint32_t MODER;		// Configure the I/O direction mode.								   offset 0x00
	__vo uint32_t OTYPER;		// Configure the output type of the I/O port.						   offset 0x04
	__vo uint32_t OSPEEDR;		// Configure the I/O output speed.									   offset 0x08
	__vo uint32_t PUPDR;		// Configure the I/O pull-up or pull-down 							   offset 0x0C
	__vo uint32_t IDR;			// read-only, contain the input value of the corresponding I/O port.   offset 0x10
	__vo uint32_t ODR;			// can be individually set and reset by writing to the GPIOx_BSRR 	   offset 0x14
	__vo uint32_t BSRR;			// BRy: Port x reset bit y 	 BSy: Port x set bit y 					   offset 0x18
	__vo uint32_t LCKR;			// This register is used to lock the configuration 					   offset 0x1C
	__vo uint32_t AFRL;			// Alternate function selection for port x bit y (y = 0..7)			   offset 0x20
	__vo uint32_t AFRH;			//  Alternate function selection for port x bit y (y = 8..15)		   offset 0x24

}GPIO_RegDef_t;



// Peripheral register definition structure for SPI

typedef struct
{
	__vo uint32_t CR1;			// This register contains various control bits that configure the SPI module's behavior.	 	offset 0x00
	__vo uint32_t CR2;			// This register contains additional control bits that configure the SPI module's behavior. 	offset 0x04
	__vo uint32_t SR;			// This register contains status bits that indicate the current state of the SPI module.		offset 0x08
	__vo uint32_t DR;			// This register holds the data to be transmitted or the data received from the slave device.	offset 0x0C
	__vo uint32_t CRCPR;		// This register holds the polynomial value used for CRC calculation.						    offset 0x10
	__vo uint32_t RXCRCR;		// This register holds the computed CRC value for the received data.						    offset 0x14
	__vo uint32_t TXCRCR;		// This register holds the computed CRC value for the transmitted data. 					    offset 0x18
	__vo uint32_t I2SCFGR;		// This register contains control bits that configure the I2S interface. 					    offset 0x1C
	__vo uint32_t I2SPR;		// This register holds the prescaler value used to generate the I2S clock signal.     		    offset 0x20

}SPI_RegDef_t;


// Peripheral register definition structure for RCC

typedef struct
{
	__vo uint32_t RC;			// RCC clock control register
	__vo uint32_t PLLCFGR;		// RCC PLL configuration register
	__vo uint32_t CFGR;			// RCC clock configuration register
	__vo uint32_t CIR;			// RCC clock interrupt register
	__vo uint32_t AHB1RSTR;		// RCC AHB1 peripheral reset register
	__vo uint32_t AHB2RSTR;		// RCC AHB2 peripheral reset register
	__vo uint32_t AHB3RSTR;		// RCC AHB3 peripheral reset register
	uint32_t REV0;				// Reserved, not used
	__vo uint32_t APB1RSTR;		// RCC APB1 peripheral reset register
	__vo uint32_t APB2RSTR;		// RCC APB2 peripheral reset register
	uint32_t REV1;				// Reserved, not used
	uint32_t REV2;				// Reserved, not used
	__vo uint32_t AHB1ENR;		// RCC AHB1 peripheral clock enable register
	__vo uint32_t AHB2ENR;		// RCC AHB2 peripheral clock enable register
	__vo uint32_t AHB3ENR;		// RCC AHB3 peripheral clock enable register
	uint32_t REV3;				// Reserved, not used
	__vo uint32_t APB1ENR;		// RCC APB1 peripheral clock enable register
	__vo uint32_t APB2ENR; 		// RCC APB2 peripheral clock enable register
	uint32_t REV4;				// Reserved, not used
	uint32_t REV5;				// Reserved, not used
	__vo uint32_t AHB1LPENR;	// RCC AHB1 peripheral clock enable in low power mode register
	__vo uint32_t AHB2LPENR;	// RCC AHB2 peripheral clock enable in low power mode register
	__vo uint32_t AHB3LPENR;	// RCC AHB3 peripheral clock enable in low power mode register
	uint32_t REV6;				// Reserved, not used
	__vo uint32_t APB1LENR;		// RCC APB1 peripheral clock enable in low power mode register
	__vo uint32_t APB2LENR;		// RCC APB2 peripheral clock enable in low power mode register
	uint32_t REV7;				// Reserved, not used
	uint32_t REV8;				// Reserved, not used
	__vo uint32_t BDCR;			// RCC Backup domain control register
	__vo uint32_t CSR;			// RCC clock control & status register
	uint32_t REV9;				// Reserved, not used
	uint32_t REV10;				// Reserved, not used
	__vo uint32_t SSCGR;		// RCC spread spectrum clock generation register
	__vo uint32_t PLLI2SCFGR;	// RCC PLLI2S configuration register

} RCC_RegDef_t;



// Peripheral register definition structure for EXTI

typedef struct
{
	__vo uint32_t IMR;			// Interrupt mask on line x.								  		   offset 0x00
	__vo uint32_t EMR;			// Event mask on line x.											   offset 0x04
	__vo uint32_t RTSR;			// Rising trigger event configuration bit of line x.				   offset 0x08
	__vo uint32_t FTSR;			// Falling trigger event configuration bit of line x. 				   offset 0x0C
	__vo uint32_t SWIER;		// Software Interrupt on line x.   									   offset 0x10
	__vo uint32_t PR;			// Pending bit													 	   offset 0x14

}EXTI_RegDef_t;


// Peripheral register definition structure for EXTI

typedef struct
{
	__vo uint32_t MEMRMP;		// 																  	   offset 0x00
	__vo uint32_t PMC;			// 																	   offset 0x04
	__vo uint32_t EXTICR[4];	// 																	   offset 0x08 - 0x14
	__vo uint32_t REV1[2];		// 																 	   offset 0x18 - 0x1C
	__vo uint32_t CMPCR;		// 																 	   offset 0x20
	__vo uint32_t REV2[2];		// 																 	   offset 0x24 - 0x28
	__vo uint32_t CFGR;			// 																 	   offset 0x2C

}SYSCFG_RegDef_t;


// Peripheral definitions (Peripheral base addresses type casted to xxx_RegDef_t)

//GPIO
#define GPIOA				((GPIO_RegDef_t *) GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t *) GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t *) GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t *) GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t *) GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t *) GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegDef_t *) GPIOG_BASEADDR)
#define GPIOH				((GPIO_RegDef_t *) GPIOH_BASEADDR)
#define GPIOI				((GPIO_RegDef_t *) GPIOI_BASEADDR)


//SPI
#define SPI1				((SPI_RegDef_t *) SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t *) SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t *) SPI3_BASEADDR)
#define SPI4				((SPI_RegDef_t *) SPI4_BASEADDR)


#define RCC					((RCC_RegDef_t *) RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t *) EXTI_BASEADDR)
//#define SYSCFG				((SYSCFG_RegDef_t *) SYSCFG_BASEADDR)

//-----------------------------------------------------------------------------------------------------------


/*
 * Clock Enable Macro for GPIOx Peripherals
 */

#define GPIOA_PCLK_EN()	  ( RCC->AHB1ENR |= (1<<0) )
#define GPIOB_PCLK_EN()	  ( RCC->AHB1ENR |= (1<<1) )
#define GPIOC_PCLK_EN()	  ( RCC->AHB1ENR |= (1<<2) )
#define GPIOD_PCLK_EN()	  ( RCC->AHB1ENR |= (1<<3) )
#define GPIOE_PCLK_EN()	  ( RCC->AHB1ENR |= (1<<4) )
#define GPIOF_PCLK_EN()	  ( RCC->AHB1ENR |= (1<<5) )
#define GPIOG_PCLK_EN()	  ( RCC->AHB1ENR |= (1<<6) )
#define GPIOH_PCLK_EN()	  ( RCC->AHB1ENR |= (1<<7) )
#define GPIOI_PCLK_EN()	  ( RCC->AHB1ENR |= (1<<8) )




/*
 * Clock Enable Macro for SYSCFG Peripherals
 */


#define SYSCFG_PCLK_EN()	  ( RCC->APB2ENR |= (1<<14) )



/*
 * Clock Enable Macro for I2Cx Peripherals
 */

#define I2C1_PCLK_EN()	  ( RCC->APB1ENR |= (1<<21) )
#define I2C2_PCLK_EN()	  ( RCC->APB1ENR |= (1<<22) )
#define I2C3_PCLK_EN()	  ( RCC->APB1ENR |= (1<<23) )


/*
 * Clock Enable Macro for SPIx Peripherals
 */

#define SPI1_PCLK_EN()	  ( RCC->APB2ENR |= (1<<12) )
#define SPI2_PCLK_EN()	  ( RCC->APB1ENR |= (1<<14) )
#define SPI3_PCLK_EN()	  ( RCC->APB1ENR |= (1<<15) )
#define SPI4_PCLK_EN()	  ( RCC->APB2ENR |= (1<<13) )

/*
 * Clock Enable Macro for USARTx / UARTx  Peripherals
 */

#define USART1_PCLK_EN()	  ( RCC->APB2ENR |= (1<<4)  )
#define USART2_PCLK_EN()	  ( RCC->APB1ENR |= (1<<17) )
#define USART3_PCLK_EN()	  ( RCC->APB1ENR |= (1<<18) )
#define UART4_PCLK_EN()	 	  ( RCC->APB1ENR |= (1<<19) )
#define UART5_PCLK_EN()	  	  ( RCC->APB1ENR |= (1<<20) )
#define USART6_PCLK_EN()	  ( RCC->APB2ENR |= (1<<5)  )

//------------------------------------------------------------------------------------------------------------------------

/*
 * Clock Disable Macro for GPIOx Peripherals
 */

#define GPIOA_PCLK_DI()	  ( RCC->AHB1ENR &= ~(1<<0) )
#define GPIOB_PCLK_DI()	  ( RCC->AHB1ENR &= ~(1<<1) )
#define GPIOC_PCLK_DI()	  ( RCC->AHB1ENR &= ~(1<<2) )
#define GPIOD_PCLK_DI()	  ( RCC->AHB1ENR &= ~(1<<3) )
#define GPIOE_PCLK_DI()	  ( RCC->AHB1ENR &= ~(1<<4) )
#define GPIOF_PCLK_DI()	  ( RCC->AHB1ENR &= ~(1<<5) )
#define GPIOG_PCLK_DI()	  ( RCC->AHB1ENR &= ~(1<<6) )
#define GPIOH_PCLK_DI()	  ( RCC->AHB1ENR &= ~(1<<7) )
#define GPIOI_PCLK_DI()	  ( RCC->AHB1ENR &= ~(1<<8) )


/*
 *
 * Clock Disable Macro for SYSCFG Peripherals
 */


#define SYSCFG_PCLK_DI()	  ( RCC->APB2ENR &= ~(1<<14) )



/*
 * Clock Disable Macro for I2Cx Peripherals
 */

#define I2C1_PCLK_DI()	  ( RCC->APB1ENR &= ~(1<<21) )
#define I2C2_PCLK_DI()	  ( RCC->APB1ENR &= ~(1<<22) )
#define I2C3_PCLK_DI()	  ( RCC->APB1ENR &= ~(1<<23) )


/*
 * Clock Disable Macro for SPIx Peripherals
 */

#define SPI1_PCLK_DI()	  ( RCC->APB2ENR &= ~(1<<12) )
#define SPI2_PCLK_DI()	  ( RCC->APB1ENR &= ~(1<<14) )
#define SPI3_PCLK_DI()	  ( RCC->APB1ENR &= ~(1<<15) )
#define SPI4_PCLK_DI()	  ( RCC->APB2ENR &= ~(1<<13) )

/*
 * Clock Disable Macro for USARTx / UARTx  Peripherals
 */

#define USART1_PCLK_DI()	  ( RCC->APB2ENR &= ~(1<<4)  )
#define USART2_PCLK_DI()	  ( RCC->APB1ENR &= ~(1<<17) )
#define USART3_PCLK_DI()	  ( RCC->APB1ENR &= ~(1<<18) )
#define UART4_PCLK_DI()	 	  ( RCC->APB1ENR &= ~(1<<19) )
#define UART5_PCLK_DI()	  	  ( RCC->APB1ENR &= ~(1<<20) )
#define USART6_PCLK_DI()	  ( RCC->APB2ENR &= ~(1<<5)  )


/*
 * 	Macro to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()	  do{( RCC->AHB1RSTR |= (1<<0) );	( RCC->AHB1RSTR &= ~(1<<0) ); } while(0)
#define GPIOB_REG_RESET()	  do{( RCC->AHB1RSTR |= (1<<1) );	( RCC->AHB1RSTR &= ~(1<<1) ); } while(0)
#define GPIOC_REG_RESET()  	  do{( RCC->AHB1RSTR |= (1<<2) );	( RCC->AHB1RSTR &= ~(1<<2) ); } while(0)
#define GPIOD_REG_RESET()  	  do{( RCC->AHB1RSTR |= (1<<3) );	( RCC->AHB1RSTR &= ~(1<<3) ); } while(0)
#define GPIOE_REG_RESET()	  do{( RCC->AHB1RSTR |= (1<<4) );	( RCC->AHB1RSTR &= ~(1<<4) ); } while(0)
#define GPIOF_REG_RESET()	  do{( RCC->AHB1RSTR |= (1<<5) );	( RCC->AHB1RSTR &= ~(1<<5) ); } while(0)
#define GPIOG_REG_RESET()	  do{( RCC->AHB1RSTR |= (1<<6) );	( RCC->AHB1RSTR &= ~(1<<6) ); } while(0)
#define GPIOH_REG_RESET()	  do{( RCC->AHB1RSTR |= (1<<7) );	( RCC->AHB1RSTR &= ~(1<<7) ); } while(0)
#define GPIOI_REG_RESET()	  do{( RCC->AHB1RSTR |= (1<<8) );	( RCC->AHB1RSTR &= ~(1<<8) ); } while(0)


/*
 * 	Macro to reset SPIx peripherals
 */

#define SPI1_REG_RESET()	  do{( RCC->APB2ENR |= (1<<12) );	( RCC->APB2ENR &= ~(1<<12) ); } while(0)
#define SPI2_REG_RESET()	  do{( RCC->APB1ENR |= (1<<14) );	( RCC->APB1ENR &= ~(1<<14) ); } while(0)
#define SPI3_REG_RESET()  	  do{( RCC->APB1ENR |= (1<<15) );	( RCC->APB1ENR &= ~(1<<15) ); } while(0)
#define SPI4_REG_RESET()  	  do{( RCC->APB2ENR |= (1<<13) );	( RCC->APB2ENR &= ~(1<<13) ); } while(0)


/*
 *  Returns port code for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)	(	(x == GPIOA) ? 0: \
										(x == GPIOB) ? 1: \
										(x == GPIOC) ? 2: \
										(x == GPIOD) ? 3: \
										(x == GPIOE) ? 4: \
										(x == GPIOF) ? 5: \
										(x == GPIOG) ? 6: \
										(x == GPIOH) ? 7: 0 )




// Some Generic Macros

#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_SET			SET
#define FLAG_RESET			RESET



/*
 * 	IRQ	(Interrupt Request) Numbers of STM32F407x MCU
 */

#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40

/*
 *  IRQ	(Interrupt Request) All Possible Priority Levels
 */

#define NVIC_IRQ_PRIO0			0
#define NVIC_IRQ_PRIO1			1
#define NVIC_IRQ_PRIO2			2
#define NVIC_IRQ_PRIO3			3
#define NVIC_IRQ_PRIO4			4
#define NVIC_IRQ_PRIO5			5
#define NVIC_IRQ_PRIO6			6
#define NVIC_IRQ_PRIO7			7
#define NVIC_IRQ_PRIO8			8
#define NVIC_IRQ_PRIO9			9
#define NVIC_IRQ_PRIO10			10
#define NVIC_IRQ_PRIO11			11
#define NVIC_IRQ_PRIO12			12
#define NVIC_IRQ_PRIO13			13
#define NVIC_IRQ_PRIO14			14
#define NVIC_IRQ_PRIO15			15


/**************************************************************************************************************************
 	 	 	 	 	 	 	 	 	 	 	 *Bit Position Definition Macro For SPI Peripheral*
 **************************************************************************************************************************/

// SPI_CR1
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15


// SPI_CR2
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7


// SPI_SR
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCEER			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8



// Include the peripherals drivers
#include <stm32f407xx_spi_driver.h>
#include <stm32f407xx_gpio_driver.h>

#endif /* INC_STM32F407XX_H_ */
