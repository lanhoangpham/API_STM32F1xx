///*
// * stm32f103xx_gpio_driver.c
// *
// *  Created on: Jun 20, 2022
// *      Author: Lan Pham
// */
//

#include "stm32f103xx_gpio_driver.h"
//
//
///*****************************************************************************************************************
// * @fn 				-	gpio_periClockControl
// *
// * @brief			-	this function enable or disable peripheral clock for the given gpio port
// *
// * @param[]			- 	base address of the gpio peripheral
// * @param[]			-   enable or disable macros
// * @param[]
// *
// * @return			-	none
// *
// * @Note    		-	none
// * */
void gpio_periClockControl(gpio_regDef_t *pGPIOx, uint8_t enOrDi){
	if (enOrDi == enable){
		if(pGPIOx == gpioa){
			gpioa_pclk_en();
		}
		else if(pGPIOx == gpiob){
			gpiob_pclk_en();
		}
		else if(pGPIOx == gpiob){
			gpiob_pclk_en();
		}
		else if(pGPIOx == gpioc){
			gpioc_pclk_en();
		}
		else if(pGPIOx == gpiod ){
			gpiod_pclk_en();
		}
		else if(pGPIOx == gpioe){
			gpioe_pclk_en();
		}
		else if(pGPIOx == gpiof){
			gpiof_pclk_en();
		}
		else if(pGPIOx == gpiog ){
			gpiog_pclk_en();
		}
	}
	else{
		if(pGPIOx == gpioa){
			gpioa_pclk_di();
		}
		else if(pGPIOx == gpiob){
			gpiob_pclk_di();
		}
		else if(pGPIOx == gpiob){
			gpiob_pclk_di();
		}
		else if(pGPIOx == gpioc){
			gpioc_pclk_di();
		}
		else if(pGPIOx == gpiod ){
			gpiod_pclk_di();
		}
		else if(pGPIOx == gpioe){
			gpioe_pclk_di();
		}
		else if(pGPIOx == gpiof){
			gpiof_pclk_di();
		}
		else if(pGPIOx == gpiog ){
			gpiog_pclk_di();
		}

	}

}
//
///*****************************************************************************************************************
// * @fn 				-	gpio_init
// *
// * @brief			-	this function initialize value for gpio
// *
// * @param[]			- 	pin number want to configure
// * @param[]			-   speed
// * @param[]			-	mode
// * @param[]			-	gpio port
// *
// * @return			-	none
// *
// * @Note    		-	none   		-
// * */
//
//void gpio_init(uint8_t gpio_pinNumber, uint8_t gpio_Speed, uint16_t gpio_Mode, gpio_regDef_t *pGPIOx){
//	//1.config the mode, speed, put - pull of gpio pin
//	if( gpio_pinNumber >= 8){//configure for crh
//		//reset pinNumber
//		pGPIOx->CRH &= ~( 15 << 4*(gpio_pinNumber - 8));
//		//set mode for gpio
//		pGPIOx->CRH |= (gpio_Mode + gpio_Speed) << (4 * (gpio_pinNumber - 8 ));
//	}
//	else { // configure for crl
//		//reset pinNumber
//		pGPIOx->CRL &= ~( 15 << 4*(gpio_pinNumber));
//		//set mode for gpio
//		pGPIOx->CRL |= (gpio_Mode + gpio_Speed) << (4 * (gpio_pinNumber));
//	}
//
//	//2. config the out out data register of gpio pin
//	//reset the pin
//	pGPIOx->ODR &= ~ (1 << gpio_pinNumber);
//	//set the pin
//	pGPIOx->ODR |= ( 1 << gpio_pinNumber);
//}

void gpio_init_handle (gpio_handle_t *pGPIOxHandle){
	if (pGPIOxHandle->gpio_pinConfig.gpio_pinMode > 20 ){
		//1.config the mode, speed, put - pull of gpio pin
		if (pGPIOxHandle->gpio_pinConfig.gpio_pinNumber >= 8){
			//reset pinNumber
			pGPIOxHandle->pGPIOx->CRH &= ~( 15 << 4*(pGPIOxHandle->gpio_pinConfig.gpio_pinNumber - 8));
			//set mode for gpio
			pGPIOxHandle->pGPIOx->CRH &= (pGPIOxHandle->gpio_pinConfig.gpio_pinMode + pGPIOxHandle->gpio_pinConfig.gpio_pinSpeed) << (4 * (pGPIOxHandle->gpio_pinConfig.gpio_pinNumber - 8 ));
		}
		else {
			pGPIOxHandle->pGPIOx->CRL &= ~( 15 << 4*(pGPIOxHandle->gpio_pinConfig.gpio_pinNumber));
			//set mode for gpio
			pGPIOxHandle->pGPIOx->CRL &= (pGPIOxHandle->gpio_pinConfig.gpio_pinMode + pGPIOxHandle->gpio_pinConfig.gpio_pinSpeed) << (4 * (pGPIOxHandle->gpio_pinConfig.gpio_pinNumber));

		}
		//2. config the out out data register of gpio pin
		//reset the pin
		pGPIOxHandle->pGPIOx->ODR &= ~ (1 << pGPIOxHandle->gpio_pinConfig.gpio_pinNumber);
		//set the pin
		pGPIOxHandle->pGPIOx->ODR |= ( 1 << pGPIOxHandle->gpio_pinConfig.gpio_pinNumber);
	}
	else{
		//A. this part code for interrupts mode
		if (pGPIOxHandle->gpio_pinConfig.gpio_pinMode == gpio_mode_it_ft){
			//1. configure for ftsr
			exti->FTSR |= ( 1 << pGPIOxHandle->gpio_pinConfig.gpio_pinNumber);
			//clear the rtsa bit
			exti->RTSR &= ~( 1 << pGPIOxHandle->gpio_pinConfig.gpio_pinNumber);
		}
		else if (pGPIOxHandle->gpio_pinConfig.gpio_pinMode == gpio_mode_it_rt ){
			//2. configure for rtsa
			exti->RTSR &= ~( 1 << pGPIOxHandle->gpio_pinConfig.gpio_pinNumber);
			//clear the FTSR bit
			exti->FTSR |= ( 1 << pGPIOxHandle->gpio_pinConfig.gpio_pinNumber);
		}
		else if (pGPIOxHandle->gpio_pinConfig.gpio_pinMode == gpio_mode_it_rft){
			//3. configure both ftsr and rts
			exti->RTSR |= ( 1 << pGPIOxHandle->gpio_pinConfig.gpio_pinNumber);
			//clear the FTSR bit
			exti->FTSR |= ( 1 << pGPIOxHandle->gpio_pinConfig.gpio_pinNumber);

		}

		//B. configure the gpio port selection in afio_exticr
		uint8_t temp1 = pGPIOxHandle->gpio_pinConfig.gpio_pinNumber / 4;
		uint8_t temp2 = pGPIOxHandle->gpio_pinConfig.gpio_pinNumber % 4;
		uint8_t portCode = gpio_baseAddr_to_code(pGPIOxHandle->pGPIOx);

		afio_pclk_en();
		afio->EXTICR[temp1] |= portCode << (temp2);


		//C. emable the external interrupt delivery using IMR
		exti->IMR |= ( 1 << pGPIOxHandle->gpio_pinConfig.gpio_pinNumber);

	}
}
//
//
///*****************************************************************************************************************
// * @fn 				-	gpio_deInit
// *
// * @brief			-	this function deinitialize value for gpio
// *
// * @param[]			-	gpio port
// *
// * @return			-	none
// *
// * @Note    		-	none   		-
// * */
void gpio_deInit(gpio_regDef_t *pGPIOx){
	if(pGPIOx == gpioa){
		gpioa_reg_reset();
	}
	else if(pGPIOx == gpiob){
		gpiob_reg_reset();
	}
	else if(pGPIOx == gpiob){
		gpiob_reg_reset();
	}
	else if(pGPIOx == gpioc){
		gpioc_reg_reset();
	}
	else if(pGPIOx == gpiod ){
		gpiod_reg_reset();
	}
	else if(pGPIOx == gpioe){
		gpioe_reg_reset();
	}

}
//
///*****************************************************************************************************************
// * @fn 				- 	gpio_readFromInputPin
// *
// * @brief				-	read value 8 bits from a pin
// *
// * @param[]			- 	the gpio  port which the pin belong
// * @param[]			- 	pin number which need to read
//
// *
// * @return			-	0 or 1
// *
// * @Note    			-	none
// * */
uint8_t gpio_readFromInputPin(gpio_handle_t *pGPIOx_handle){
	uint8_t value;

	value = (uint8_t) ((pGPIOx_handle->pGPIOx->IDR >> (pGPIOx_handle->gpio_pinConfig.gpio_pinNumber)) & 0x00000001);

	return value;
}

//
///*****************************************************************************************************************
// * @fn 				- 	gpio_readFromInputPort
// *
// * @brief				-	read value 16 bits from a port
// *
// * @param[]			-   gpio  port
// * @param[]			-
//
// *
// * @return			-	uint16_t
// *
// * @Note    		-	none
// * */
uint16_t gpio_readFromInputPort(gpio_handle_t *pGPIOx_handle){
	uint16_t value;

	value = pGPIOx_handle->pGPIOx->IDR;

	return value;
}
//
//
///*****************************************************************************************************************
// * @fn 				- 	gpio_writeToOutputPin
// *
// * @brief			-	write 1 or 0 to the output data for a pin
// *
// * @param[]			-   gpio  port
// * @param[]			- 	gpio pin number
// * @param[]			-   value can be set or reset
//
// *
// * @return			-	none
// *
// * @Note    		-	none
// * */
//void gpio_writeToOutputPin(gpio_regDef_t *pGPIOx, uint8_t pinNumber, uint8_t value)
//{
//	value == gpio_pin_reset ?  pGPIOx->ODR &= ~(1 << pinNumber) : pGPIOx->ODR |=  ( 1 << pinNumber);
//}

void gpio_writeToOutputPin(gpio_handle_t* pGPIOx_handle,  uint8_t value){
	if ( value == gpio_pin_reset){
		 pGPIOx_handle->pGPIOx->ODR &= ~(1 << pGPIOx_handle->gpio_pinConfig.gpio_pinNumber);
	}
	else{
		pGPIOx_handle->pGPIOx->ODR |= (1 << pGPIOx_handle->gpio_pinConfig.gpio_pinNumber);
	}

}

//
///*****************************************************************************************************************
// * @fn 				- 	gpio_writeToOutputPort
// *
// * @brief				-	write value 0 or 1 to the port
// *
// * @param[]			-   gpio  port
// * @param[]			-   value can be set or reset
//
// *
// * @return			-	none
// *
// * @Note    			-	none
// * */
//void gpio_writeToOutputPort(gpio_regDef_t *pGPIOx, uint16_t value){
//	pGPIOx->ODR = value;
//}

void gpio_writeToOutputPort(gpio_handle_t* pGPIOx_handle, uint16_t value){
	pGPIOx_handle->pGPIOx->ODR = value;
}
//
///*****************************************************************************************************************
// * @fn 				- 	gpio_writeToOutputPort
// *
// * @brief			-	write value 0 or 1 to the port
// *
// * @param[]			-   gpio  port
// * @param[]			-   value can be set or reset
//
// *
// * @return			-	none
// *
// * @Note    		-	none
// * */
void gpio_toggleOutputPin(gpio_handle_t *pGPIOx_handle){
	pGPIOx_handle->pGPIOx->ODR ^= (1 << pGPIOx_handle->gpio_pinConfig.gpio_pinNumber);
}
//
///*****************************************************************************************************************
// * @fn 				-
// *
// * @brief			-
// *
// * @param[]
// * @param[]
// * @param[]
// *
// * @return			-
// *
// * @Note    		-
// * */
//void gpio_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t enOrDi);
//void gpio_IRQHandling(uint8_t pinNumber);
//
//
//
//
//
///*****************************************************************************************************************
// * @fn 				-
// *
// * @brief			-
// *
// * @param[]
// * @param[]
// * @param[]
// *
// * @return			-
// *
// * @Note    		-
// * */

void delay(void){
	for (uint32_t i = 0; i < 500000/2; i ++);
}
