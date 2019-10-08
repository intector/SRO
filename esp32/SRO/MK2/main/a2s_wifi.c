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

//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
//#include "freertos/timers.h"
//#include "freertos/semphr.h"
//#include "freertos/queue.h"
//#include "freertos/event_groups.h"


#include "esp_wifi.h"
//#include "esp_system.h"
//#include "nvs_flash.h"
#include "esp_event_loop.h"
#include "globals.h"

#include "a2s_wifi.h"

static EventGroupHandle_t wifi_event_group;
const static int WIFI_CONNECTED_BIT = BIT0;

//char SRO_WIFI_SSID[] = SRO_ESP_WIFI_SSID;
//char SRO_WIFI_PASS[] = SRO_ESP_WIFI_PASS;


void wifi_connect()
{
	//	wifi_config_t wifi_config;
	//	strcpy((char*)wifi_config.sta.ssid, SRO_WIFI_SSID);
	//	strcpy((char*)wifi_config.sta.password, SRO_WIFI_PASS);

		wifi_config_t wifi_config = {
		.sta = {
		.ssid = SRO_ESP_WIFI_SSID,
		.password = SRO_ESP_WIFI_PASS,
	},
	};

	ESP_ERROR_CHECK(esp_wifi_disconnect());
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_connect());
}


static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	switch (event->event_id) {
	case SYSTEM_EVENT_STA_START:
		xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(hLED_CTRL_Task, TN_WIFI_DISCONNECTED, eSetBits, &xHigherPriorityTaskWoken);
		ESP_ERROR_CHECK(tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, "ESP32-SRO-01"));
		wifi_connect();
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(hLED_CTRL_Task, TN_WIFI_CONNECTED | TN_MQTT_DISCONNECTED, eSetBits, &xHigherPriorityTaskWoken);
		xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(hLED_CTRL_Task, TN_WIFI_DISCONNECTED | TN_MQTT_OFF, eSetBits, &xHigherPriorityTaskWoken);
		wifi_connect();
		xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
		break;
	default:
		break;
	}
	
	if (xHigherPriorityTaskWoken)
		taskYIELD(); 

	return ESP_OK;
}


void wifi_init()
{
	//	wifi_config_t wifi_config;
	
	tcpip_adapter_init();
	wifi_event_group = xEventGroupCreate();
	ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

	//	strcpy((char*)wifi_config.sta.ssid, SRO_WIFI_SSID);
	//	strcpy((char*)wifi_config.sta.password, SRO_WIFI_PASS);

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	//	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, false, portMAX_DELAY);
}

