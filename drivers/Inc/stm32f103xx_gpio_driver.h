/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Jun 20, 2022
 *      Author: Lan Pham
 */

#ifndef INC_STM32F103XX_GPIO_DRIVER_H_
#define INC_STM32F103XX_GPIO_DRIVER_H_

#include "stm32f103.h"




/*this is a configuration structure for a gpio pin*/
typedef struct{
	uint8_t 			gpio_pinNumber;						/*!< possible value from @gpio_pin_numbers>*/
	uint16_t 			gpio_pinMode;						/*!< possible value from @gpio_pin_mode>*/
	uint8_t 			gpio_pinSpeed;						/*!< possible value from @gpio_pin_speed>*/
}gpio_pinconfig_t;



/*
 * @gpio_pin_mode
 * gpio pin possible mode
 * */

#define	GPIO_Mode_AIN 			 0
#define	GPIO_Mode_IN_FLOATING 	 4
#define	GPIO_Mode_IPD 			 8
#define	GPIO_Mode_IPU 			 8
#define	GPIO_Mode_Out_OD 		 4
#define	GPIO_Mode_Out_PP 		 0
#define	GPIO_Mode_AF_OD 		 12
#define	GPIO_Mode_AF_PP 		 8

/**
 * gpio pin interrupt mode ( configure for gpio_pin_mode )
 */
#define gpio_mode_it_ft			20
#define gpio_mode_it_rt			21
#define gpio_mode_it_rft		22

/*
 * @gpio_pin_numbers
 * gpio pin number
 * */
#define gpio_pin_number_0		0
#define gpio_pin_number_1		1
#define gpio_pin_number_2		2
#define gpio_pin_number_3		3
#define gpio_pin_number_4		4
#define gpio_pin_number_5		5
#define gpio_pin_number_6		6
#define gpio_pin_number_7		7
#define gpio_pin_number_8		8
#define gpio_pin_number_9		9
#define gpio_pin_number_10		10
#define gpio_pin_number_11		11
#define gpio_pin_number_12		12
#define gpio_pin_number_13		13
#define gpio_pin_number_14		14
#define gpio_pin_number_15		15


/*
 * @gpio_pin_speed
 * gpio pin possible output speed
 * */
//
#define gpio_input      		0
#define	gpio_speed_10hz 		1
#define	gpio_speed_2hz  		2
#define	gpio_speed_50hz 		3


/* this is a handle structure for a gpio pin */
typedef struct{
	gpio_regDef_t *pGPIOx;  				/*!< this hold the base address of the gpio port to which the pin belong>*/
	gpio_pinconfig_t gpio_pinConfig;		/*!< this hold the gpio pin configuration setting>*/
}gpio_handle_t;




/***********************************************************************************************************************
 * 											APIs supported by this driver											   *
 * 							for more information about the APIs check the function definitions						   *
 * *********************************************************************************************************************/

/*
 * peripheral clock setup
 * */
//void gpio_periClockControl(gpio_regDef_t *pGPIOx, uint8_t enOrDi);
void gpio_periClockControl(gpio_regDef_t *pGPIOx, uint8_t enOrDi);

/* init and de_init */
void gpio_init_handle (gpio_handle_t *pGPIOxHandle);
//void gpio_deInit(gpio_regDef_t *pGPIOx);
void gpio_deInit(gpio_regDef_t *pGPIOx);


/* data read and write */
uint8_t gpio_readFromInputPin(gpio_handle_t *pGPIOx_handle);
uint16_t gpio_readFromInputPort(gpio_handle_t *pGPIOx_handle);
void gpio_writeToOutputPin(gpio_handle_t *pGPIOx_handle, uint8_t value); //value can be set or reset
void gpio_writeToOutputPort(gpio_handle_t *pGPIOx_handle, uint16_t value);
void gpio_toggleOutputPin(gpio_handle_t *pGPIOx_handle);


/* irq configuration and isr handling */
void gpio_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t enOrDi);
void gpio_IRQHandling(uint8_t pinNumber);



//delay function

void delay(void);




































#endif /* INC_STM32F103XX_GPIO_DRIVER_H_ */
