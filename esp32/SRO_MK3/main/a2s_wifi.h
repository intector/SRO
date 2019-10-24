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

# pragma once

#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
//#include "freertos/timers.h"
//#include "freertos/semphr.h"
//#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_event_loop.h"

#include "APA102C.h"


#define SRO_ESP_WIFI_SSID		your WIFI SSID goes here
#define SRO_ESP_WIFI_PASS		your WIFI password goes here

//extern char SRO_WIFI_SSID[] = SRO_ESP_WIFI_SSID;
//extern char SRO_WIFI_PASS[] = SRO_ESP_WIFI_PASS;


extern void wifi_init();
extern void wifi_connect();

