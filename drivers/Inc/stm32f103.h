/*
 * stm32f103.h
 *
 *  Created on: Jun 10, 2022
 *      Author: Lan Pham
 */

#ifndef INC_STM32F103_H_
#define INC_STM32F103_H_

#include<stdint.h>
//#include "stm32f103xx_gpio_driver.h"
#define __vo volatile

/*
 * base address of flash and sram
 * */

#define flash_baseAddr			 0x08000000U
#define sram_baseAddr			 0x20000000U
#define ram						 0x1FFFF000U     	/*this is the address of system memory*/


/*
 * ahb and apb bus peripheral base address
*/
#define peripheral_base			0x40000000U
#define apb1Periph_baseAddr		peripheral_base
#define apb2Periph_baseAddr		0x40010000U
#define	ahbPeriph_baseAddr		0x40018000U

/*
 * base address of peripheral which are hanging on ahb bus
 * TODO: complete for all other peripheral
 * */
#define rcc_baseAddr			(ahbPeriph_baseAddr + 0x9000)

/*
 * base address of peripheral which are hanging on apb1 bus
 * TODO: complete for all other peripheral
 * */

#define i2c1_baseAddr		(apb1Periph_baseAddr + 0x5400)
#define i2c3_baseAddr		(apb1Periph_baseAddr + 0x5800)

#define spi2_baseAddr		(apb1Periph_baseAddr + 0x3800)
#define	spi3_baseAddr		(apb1Periph_baseAddr + 0x3C00)

#define usart2_baseAddr		(apb1Periph_baseAddr + 0x4400)
#define usart3_baseAddr		(apb1Periph_baseAddr + 0x4800)

#define uart4_baseAddr		(apb1Periph_baseAddr + 0x4C00)
#define uart5_baseAddr		(apb1Periph_baseAddr + 0x5000)


/*
 * base address of peripheral which are hanging on apb2 bus
 * */

#define gpioa_baseAddr 		(apb2Periph_baseAddr + 0x0800)
#define gpiob_baseAddr		(apb2Periph_baseAddr + 0x0C00)
#define gpioc_baseAddr 		(apb2Periph_baseAddr + 0x1000)
#define gpiod_baseAddr		(apb2Periph_baseAddr + 0x1400)
#define gpioe_baseAddr 		(apb2Periph_baseAddr + 0x1800)
#define gpiof_baseAddr 		(apb2Periph_baseAddr + 0x1C00)
#define gpiog_baseAddr 		(apb2Periph_baseAddr + 0x2000)

#define afio_baseAddr		(apb2Periph_baseAddr + 0x0000)
#define spi1_baseAddr 		(apb2Periph_baseAddr + 0x3000)
#define exti_baseAddr 		(apb2Periph_baseAddr + 0x0400)
#define	usart1_baseAddr		(apb2Periph_baseAddr + 0x3800)


/**************************************peripheral register definition structures*****************************/

/*
 * Note: registers of a peripheral are specific to MCU
 * e.g:  Number of registers of SPI peripheral of STM32F1xx family of MCUs may be different
 */

typedef struct
{
	__vo uint32_t CRL;			/*!<TODO, 																Address offset: 0x00*/
	__vo uint32_t CRH;			/*!<TODO, 																Address offset: 0x04*/
	__vo uint32_t IDR;			/*!<TODO, 																Address offset: 0x08*/
	__vo uint32_t ODR;			/*!<TODO, 																Address offset: 0x0C*/
	__vo uint32_t BSRR;			/*!<TODO, 																Address offset: 0x10*/
	__vo uint32_t BRR;			/*!<TODO, 																Address offset: 0x14*/
	__vo uint32_t LCKR;			/*!<TODO, 																Address offset: 0x18*/

}gpio_regDef_t;
/*
 * peripheral register definition structure for RCC
 * */

typedef struct{
	__vo uint32_t CR;				/*!<TODO, 																Address offset: 0x00*/
	__vo uint32_t CFGR;				/*!<TODO, 																Address offset: 0x04*/
	__vo uint32_t CIR;				/*!<TODO, 																Address offset: 0x08*/
	__vo uint32_t APB2RSTR;			/*!<TODO, 			APB2 peripheral reset register						Address offset: 0x0C*/
	__vo uint32_t APB1RSTR;			/*!<TODO, 			APB1 peripheral reset register						Address offset: 0x10*/
	__vo uint32_t AHBENR;			/*!<TODO, 			AHB Peripheral Clock enable register				Address offset: 0x14*/
	__vo uint32_t APB2ENR;			/*!<TODO, 			APB2 peripheral clock enable register				Address offset: 0x18*/
	__vo uint32_t APB1ENR;			/*!<TODO, 			APB1 peripheral clock enable register				Address offset: 0x1c*/
	__vo uint32_t BDCR;				/*!<TODO, 			Backup domain control register						Address offset: 0x20*/
	__vo uint32_t CSR;				/*!<TODO, 			Control/status register								Address offset: 0x24*/
	__vo uint32_t AHBSTR; 			/*!<TODO, 			Control/status register								Address offset: 0x28*/
	__vo uint32_t CFGR2;			/*!<TODO, 			Control/status register								Address offset: 0xA0*/

}rcc_regDef_t;


/*
 * Note: registers of exti are specific to MCU
 */
typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}exti_regDef_t;

/*
 * Note: registers of afio are specific to MCU
 */
typedef struct{
	__vo uint32_t EVCR;
	__vo uint32_t MAPR[3];
	__vo uint32_t EXTICR[4];
	__vo uint32_t MAPR2[2];
}afio_regDef_t;
/*
 * peripheral definitions ( peripheral base addresses typecasted to xxxx_regDef_t)
 * */

#define gpioa					((gpio_regDef_t*)gpioa_baseAddr)
#define gpiob					((gpio_regDef_t*)gpiob_baseAddr)
#define gpioc					((gpio_regDef_t*)gpioc_baseAddr)
#define gpiod					((gpio_regDef_t*)gpiod_baseAddr)
#define gpioe					((gpio_regDef_t*)gpioe_baseAddr)
#define gpiof					((gpio_regDef_t*)gpiof_baseAddr)
#define gpiog					((gpio_regDef_t*)gpiog_baseAddr)

#define rcc						((rcc_regDef_t*)rcc_baseAddr)

#define exti					((exti_regDef_t*)exti_baseAddr)

#define afio					((afio_regDef_t*)afio_baseAddr)



/*
 * clock enable macros for afio peripherals
 * */
#define afio_pclk_en()			((rcc->APB2ENR) |= (1 << 0))

/**
 * this macro return a codes between 0 to 7 for given gpio base address(x)
 */
#define gpio_baseAddr_to_code(x) ( (x == gpioa) ? 0 : (x == gpiob) ? 1 : (x == gpioc) ? 2 : (x == gpiod) ? 3 : (x == gpioe) ? 4 : (x == gpiof) ? 5 : (x == gpiog) ? 6 : 0)

/*
 * clock enable macros for gpiox peripherals
 * */

#define gpioa_pclk_en()			(rcc->APB2ENR |= (1 << 2))
#define gpiob_pclk_en()			(rcc->APB2ENR |= (1 << 3))
#define gpioc_pclk_en()			(rcc->APB2ENR |= (1 << 4))
#define gpiod_pclk_en()			(rcc->APB2ENR |= (1 << 5))
#define gpioe_pclk_en()			(rcc->APB2ENR |= (1 << 6))
#define gpiof_pclk_en()			(rcc->APB2ENR |= (1 << 7))
#define gpiog_pclk_en()			(rcc->APB2ENR |= (1 << 8))

/*
 * clock enable macros for i2c peripherals
 * */

#define i2c1_pclk_en() 			(rcc->APB1ENR |= ( 1 << 21 ))
#define i2c2_pclk_en() 			(rcc->APB1ENR |= ( 1 << 22 ))

/*
 * clock enable macros for SPIx peripherals
 * */
#define spi1_pclk_en() 			(rcc->APB2ENR |= ( 1 << 12))
#define spi2_pclk_en() 			(rcc->APB1ENR |= ( 1 << 14))
#define spi3_pclk_en() 			(rcc->APB1ENR |= ( 1 << 14))

/*
 * clock enable macros for USARTx peripherals
 * */
#define usart1_pclk_en() 		(rcc->APB2ENR |= (1 << 14))
#define usart2_pclk_en() 		(rcc->APB1ENR |= (1 << 17))
#define usart3_pclk_en() 		(rcc->APB1ENR |= (1 << 18))
#define uart4_pclk_en() 		(rcc->APB1ENR |= (1 << 19))
#define uart5_pclk_en() 		(rcc->APB1ENR |= (1 << 20))

/*
 * clock disable macros for gpiox peripherals
 * */
#define gpioa_pclk_di()			(rcc->APB2ENR &= ~(1 << 2))
#define gpiob_pclk_di()			(rcc->APB2ENR &= ~(1 << 3))
#define gpioc_pclk_di()			(rcc->APB2ENR &= ~(1 << 4))
#define gpiod_pclk_di()			(rcc->APB2ENR &= ~(1 << 5))
#define gpioe_pclk_di()			(rcc->APB2ENR &= ~(1 << 6))
#define gpiof_pclk_di()			(rcc->APB2ENR &= ~(1 << 7))
#define gpiog_pclk_di()			(rcc->APB2ENR &= ~(1 << 8))

/*
 * clock enable macros for i2c peripherals
 * */

#define i2c1_pclk_di() 			(rcc->APB1ENR &= ~( 1 << 21 ))
#define i2c2_pclk_di()			(rcc->APB1ENR &= ~( 1 << 22 ))

/*
 * clock disable macros for SPIx peripherals
 * */
#define spi1_pclk_di() 			(rcc->APB2ENR &= ~( 1 << 12))
#define spi2_pclk_di() 			(rcc->APB1ENR &= ~( 1 << 14))
#define spi3_pclk_di() 			(rcc->APB1ENR &= ~( 1 << 14))

/*
 * clock disable macros for USARTx peripherals
 * */
#define usart1_pclk_di() 		(rcc->APB2ENR &= ~(1 << 14))
#define usart2_pclk_di() 		(rcc->APB1ENR &= ~(1 << 17))
#define usart3_pclk_di() 		(rcc->APB1ENR &= ~(1 << 18))
#define uart4_pclk_di() 		(rcc->APB1ENR &= ~(1 << 19))
#define uart5_pclk_di() 		(rcc->APB1ENR &= ~(1 << 20))

/*
 * Macros to reset gpiox peripherals
 * */
#define gpioa_reg_reset()		do { (rcc->APB2RSTR |= (1 << 2)); (rcc->APB2ENR &= ~(1 << 2));} while(0)
#define gpiob_reg_reset()		do { (rcc->APB2RSTR |= (1 << 3)); (rcc->APB2ENR &= ~(1 << 3));} while(0)
#define gpioc_reg_reset()		do { (rcc->APB2RSTR |= (1 << 4)); (rcc->APB2ENR &= ~(1 << 4));} while(0)
#define gpiod_reg_reset()		do { (rcc->APB2RSTR |= (1 << 5)); (rcc->APB2ENR &= ~(1 << 5));} while(0)
#define gpioe_reg_reset()		do { (rcc->APB2RSTR |= (1 << 6)); (rcc->APB2ENR &= ~(1 << 6));} while(0)



//some generic macros
#define enable 					1
#define disable					0
#define set 					enable
#define reset 					disable
#define gpio_pin_set			set
#define gpio_pin_reset 			reset


#endif /* INC_STM32F103_H_ */
