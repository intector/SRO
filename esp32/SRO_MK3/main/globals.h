/* Soldering Reflow Oven
 * Copyright (c) 2019-2020 Intector
 *
 * a2s wifi lib
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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

// task notification definitions for LED
#define TN_NOTHING				0x00000000

#define TN_WIFI_CONNECTED		0x00000001
#define TN_WIFI_DISCONNECTED	0x00000002
#define TN_WIFI_ERROR			0x00000008

#define TN_MQTT_CONNECTED		0x00000010
#define TN_MQTT_DISCONNECTED	0x00000020
#define TN_MQTT_OFF				0x00000040
#define TN_MQTT_ERROR			0x00000080

#define TN_TEMP_COLD			0x00000100
#define TN_TEMP_HOT				0x00000200
#define TN_TEMP_ERROR			0x00000800

#define TN_HEATER_ON			0x00010000
#define TN_HEATER_OFF			0x00020000
#define TN_HEATER_ERROR			0x00080000

#define TN_FAN_ON				0x00100000
#define TN_FAN_OFF				0x00200000
#define TN_FAN_ERROR			0x00800000

#define TN_LED_READY			0x01000000

extern bool LED_InitReadyFlag;
extern BaseType_t xReturned;
extern TaskHandle_t hLED_CTRL_Task;     				// LED control task handle
extern TaskHandle_t hServo_CTRL_Task;    				// Servo control task handle
extern TimerHandle_t LED_Blink_Timer;    				// timer to blink LED
extern TimerHandle_t TemperatureRead_Timer;          	// timer to read temperature


