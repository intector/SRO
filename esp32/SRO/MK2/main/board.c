/* Soldering Reflow Oven
 * Copyright (c) 2019-2020 Intector
 *
 * SRO board definitions
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "board.h"

void System_IO_Init()
{
/*******************************************************************
 *  led IO init
*******************************************************************/
	gpio_pad_select_gpio(GPIO_LED_01);
	gpio_pad_select_gpio(GPIO_LED_02);
	
	// Set the GPIO as a push/pull output
	gpio_set_direction(GPIO_LED_01, GPIO_MODE_INPUT_OUTPUT);
	gpio_set_direction(GPIO_LED_02, GPIO_MODE_INPUT_OUTPUT);
	
/*******************************************************************
 *  temperature IO init
 *******************************************************************/
 	gpio_pad_select_gpio(GPIO_6675_SCK);
	gpio_pad_select_gpio(GPIO_6675_CS);
	gpio_pad_select_gpio(GPIO_6675_SO);

	gpio_set_direction(GPIO_6675_SCK, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_6675_CS, GPIO_MODE_OUTPUT);
	gpio_set_direction(GPIO_6675_SO, GPIO_MODE_INPUT);
	
	gpio_set_level(GPIO_6675_CS, 1);
	gpio_set_level(GPIO_6675_SCK, 1);

/*******************************************************************
 *  solid state relay IO init
 *******************************************************************/
	gpio_pad_select_gpio(GPIO_SSR_HEATER);
	gpio_pad_select_gpio(GPIO_SSR_FAN);
	
	// Set the GPIO as a push/pull output
	gpio_set_direction(GPIO_SSR_HEATER, GPIO_MODE_INPUT_OUTPUT);
	gpio_set_direction(GPIO_SSR_FAN, GPIO_MODE_INPUT_OUTPUT);
	
/*******************************************************************
 *  PWM IO init
 *******************************************************************/  
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_SERVO_PWM);       //Set GPIO 22 as PWM0A, to which servo is connected

}

