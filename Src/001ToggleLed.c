/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "stm32f103.h"
#include "stm32f103xx_gpio_driver.h"

#include <stdint.h>

#define high		 1
#define buttonPress  high


int main(void)
{

	gpio_handle_t gpiox, gpioButton;

	//configure the led
	gpiox.pGPIOx = gpioa;
	gpiox.gpio_pinConfig->gpio_pinNumber = gpio_pin_number_8;
	gpiox.gpio_pinConfig->gpio_pinMode = GPIO_Mode_Out_OD;
	gpiox.gpio_pinConfig->gpio_pinSpeed = gpio_speed_50hz;

	gpio_init_handle(&gpiox);



	while(1){

			delay();
			gpio_toggleOutputPin(&gpioButton);

	}

}
