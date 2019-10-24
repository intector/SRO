/* Soldering Reflow Oven
 * Copyright (c) 2019-2020 Intector
 *
 * MAX6675 lib
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

#define NO_SENSOR -1764
#define TEMP_READ_INTERVAL (1000 / portTICK_PERIOD_MS)
#define TEMP_SEND_INTERVAL (1000 / portTICK_PERIOD_MS)

typedef struct __temperature {
	bool Sensor_OK;
	uint16_t int_C_Value;
	uint16_t int_F_Value;
	char str_C_Value[5];
	char str_F_Value[5];
} SRO_Temp;

SRO_Temp ActTemp;

extern xQueueHandle Temperature_QueueHandleId;

#define TEMPERATURE_INIT() {\
	.Sensor_OK = false, \
	.int_C_Value = 0, \
	.int_F_Value = 0, \
	.str_C_Value = { 0, 0, 0, 0, 0 }, \
	.str_F_Value = { 0, 0, 0, 0, 0 }, \
};

extern void MAX6675_Init();
extern void MAX6675_ReadTemperature(SRO_Temp *data);


