/* Soldering Reflow Oven
 * Copyright (c) 2019-2020 Intector
 *
 * a2s mqtt lib
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

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "mqtt_client.h"
#include "cJSON.h"
#include "board.h"
#include "MAX6675.h"
#include "a2s_nvs.h"

#include "SRO_PID.h"
#include "a2s_servo.h"


#define MQTT_PUB_TEMPERATURE	"/a2s/temperature"
#define MQTT_PUB_LIFEBIT		"/a2s/lifebit_from_esp32"
//#define MQTT_PUB_PAR_SAVE		"/a2s/parameter_set_save_response"
#define MQTT_PUB_PAR_LOAD		"/a2s/parameter_set_load_response"
//#define MQTT_PUB_PID_SAVE		"/a2s/pid_parameter_save_response"
#define MQTT_PUB_PID_LOAD		"/a2s/pid_parameter_load_response"
#define MQTT_PUB_CMD_RUN		"/a2s/run_from_esp32"
#define MQTT_PUB_CMD_HEATER		"/a2s/heater_from_esp32"
#define MQTT_PUB_CMD_FAN		"/a2s/fan_from_esp32"
#define MQTT_PUB_CMD_RAMP		"/a2s/measure_ramp_from_esp32"
#define MQTT_PUB_CMD_PID		"/a2s/pid_try_autotune_from_esp32"
#define MQTT_PUB_CMD_DOOR		"/a2s/door_from_esp32"

#define MQTT_SUB_LIFEBIT		"/a2s/lifebit_to_esp32"
#define MQTT_SUB_PAR_SAVE		"/a2s/parameter_set_save"
#define MQTT_SUB_PAR_LOAD		"/a2s/parameter_set_load"
#define MQTT_SUB_PID_SAVE		"/a2s/pid_parameter_save"
#define MQTT_SUB_PID_LOAD		"/a2s/pid_parameter_load"
#define MQTT_SUB_CMD_RUN		"/a2s/run_to_esp32"
#define MQTT_SUB_CMD_HEATER		"/a2s/heater_to_esp32"
#define MQTT_SUB_CMD_FAN		"/a2s/fan_to_esp32"
#define MQTT_SUB_CMD_RAMP		"/a2s/measure_ramp_to_esp32"
#define MQTT_SUB_CMD_PID		"/a2s/pid_try_autotune_to_esp32"
#define MQTT_SUB_CMD_DOOR		"/a2s/door_to_esp32"
#define MQTT_SUB_CMD_UPDATE		"/a2s/data_update_to_esp32"

#define MQTT_SERVER_URI		address of your MQTT-Broker
#define MQTT_CLIENT_ID		your MQTT client ID
#define MQTT_USERNAME		your MQTT user name
#define MQTT_PASSWORD		your MQTT password

//char SRO_MQTT_URI[] = MQTT_SERVER_URI;

typedef struct __CTRL_CMD_VALUE {
	uint8_t SEQ_RUN;
	uint8_t HEATER;
	uint8_t FAN;
	uint8_t RAMP;
	uint8_t PID;
	uint8_t ACT_DOOR_POS;
	uint8_t SET_DOOR_POS;
} CTRL_CMD_VALUE;



CTRL_CMD_VALUE CTRL_CMD;

SRO_ParSet SolParSet;

#define MQTT_INIT_CONFIG_DEFAULT() {			\
		.uri = MQTT_SERVER_URI,					\
		.client_id = MQTT_CLIENT_ID,			\
		.username = MQTT_USERNAME,				\
		.password = MQTT_PASSWORD,				\
		.event_handle = mqtt_event_handler		\
};

extern TaskHandle_t th_mqtt_periodic_task;

cJSON *PID_Data_to_JSON(SRO_PID_Data _PID_Data);
cJSON *JSON_to_PID_Data(char *_PID_DataString);
cJSON *ParameterSet_to_JSON(SRO_ParSet _ParameterSet);
cJSON *JSON_to_ParameterSet(char *_ParameterString);
void mqtt_app_start(void);

