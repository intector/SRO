/* Soldering Reflow Oven
 * Copyright (c) 2019-2020 Intector
 *
 * a2s nvs lib
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

#include "nvs_flash.h"
#include "cJSON.h"

#include "SRO_PID.h"


#define STORAGE_NAMESPACE "sro"
#define PS_01	"ps-01"		// storage name parameter set 01
#define PS_02	"ps-02"		// storage name parameter set 02
#define PS_03	"ps-03"		// storage name parameter set 03
#define PS_04	"ps-04"		// storage name parameter set 04
#define PS_05	"ps-05"		// storage name parameter set 05
#define PS_06	"ps-06"		// storage name parameter set 06
#define PS_07	"ps-07"		// storage name parameter set 07
#define PS_08	"ps-08"		// storage name parameter set 08
#define PS_09	"ps-09"		// storage name parameter set 09
#define PS_10	"ps-10"		// storage name parameter set 10

esp_err_t init_nvs_data(void);
esp_err_t PID_DataSave(cJSON *_PID_Data);
esp_err_t PID_DataLoad();
esp_err_t ParameterSetSave(cJSON *_ParameterSet);
esp_err_t ParameterSetLoad(uint8_t ParameterSetIndex);

