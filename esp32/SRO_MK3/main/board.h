/* Soldering Reflow Oven
 * Copyright (c) 2019-2020 Intector
 *
 * board definitions
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

#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "driver/gpio.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"


// board LED's
#define GPIO_LED_01 02		// board LED red
#define GPIO_LED_02 32		// board LED blue

// APA102 LED stripe
#define PIN_NUM_MOSI 23		// APA102C data in 
#define PIN_NUM_CLK  18		// APA102C clock in
#define SPI_NOT_USED -1		// value for unused SPI pins

// MAX6675 temperature sensor
#define GPIO_6675_SCK 17	// MAX6675 SCK
#define GPIO_6675_CS 16		// MAX6675 CS
#define GPIO_6675_SO 04		// MAX6675 SO

// DC-Motor H-Bridge
#define GPIO_DCM_CW 32		// DC-Motor (+)
#define GPIO_DCM_CCW 33		// DC-Motor (-)

// solenoid relays
#define GPIO_SSR_HEATER 25	// heater relay
#define GPIO_SSR_FAN 26		// fan relay

// Servo Motor
#define GPIO_SERVO_PWM 22	// Servo-Motor PWM signal

#define SERVO_MIN_PULSEWIDTH 500	//Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2500	//Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 180		//Maximum angle in degree upto which servo can rotate

void System_IO_Init();
