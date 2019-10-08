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

#include "a2s_mqtt.h"
#include "globals.h"
#include "APA102C.h"


static EventGroupHandle_t mqtt_event_group;
const static int MQTT_CONNECTED_BIT = BIT0;
esp_mqtt_client_handle_t _global_MQTT_Client;
TaskHandle_t th_mqtt_periodic_task = NULL; 			// MQTT publish task handle
TimerHandle_t Lifebit_Timer;						// timer generate lifebit


//msg_id = esp_mqtt_client_publish(_client, MQTT_PUB_LIFEBIT, "Lifebit from EAP32", 0, 0, 0);

void Lifebit_Timer_Callback(TimerHandle_t pxTimer)
{
	int msg_id;
//	esp_mqtt_client_handle_t _client = (esp_mqtt_client_handle_t)pxTimer;

	msg_id = esp_mqtt_client_publish(_global_MQTT_Client, MQTT_PUB_LIFEBIT, "Lifebit from ESP32", 0, 0, 0);
}

static void mqtt_periodic_task(void *pvParameters)
{
	SRO_Temp _ActTemp;
	int msg_id;
	esp_mqtt_client_handle_t _client = (esp_mqtt_client_handle_t)pvParameters;

	while(1)
	{
		// waiting for message
		xQueueReceive(Temperature_QueueHandleId, &(_ActTemp), portMAX_DELAY); 
		
		cJSON *root = cJSON_CreateObject();
		cJSON_AddStringToObject(root, "sTemp_C", _ActTemp.str_C_Value);
		cJSON_AddStringToObject(root, "sTemp_F", _ActTemp.str_F_Value);
		cJSON_AddNumberToObject(root, "iTemp_C", _ActTemp.int_C_Value);
		cJSON_AddNumberToObject(root, "iTemp_F", _ActTemp.int_F_Value);
		cJSON_AddBoolToObject(root, "bSensor_OK", _ActTemp.Sensor_OK);
		
		char* _SendText = cJSON_Print(root);		

		msg_id = esp_mqtt_client_publish(_client, MQTT_PUB_TEMPERATURE, _SendText, 0, 0, 0);

		free(_SendText);
		cJSON_Delete(root);
		
	}
	vTaskDelete(NULL);
}

cJSON *PID_Data_to_JSON(SRO_PID_Data _PID_Data)
{
	cJSON *root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "PID_KP", _PID_Data.Kp);
	cJSON_AddNumberToObject(root, "PID_KI", _PID_Data.Ki);
	cJSON_AddNumberToObject(root, "PID_KD", _PID_Data.Kd);
	
	return root;
}

cJSON *JSON_to_PID_Data(char *_PID_DataString)
{
	cJSON *root = cJSON_Parse(_PID_DataString);
	PID_Data.Kp = cJSON_GetObjectItem(root, "PID_KP")->valuedouble;
	PID_Data.Ki = cJSON_GetObjectItem(root, "PID_KI")->valuedouble;
	PID_Data.Kd = cJSON_GetObjectItem(root, "PID_KD")->valuedouble;
	
	return root;
}

cJSON *ParameterSet_to_JSON(SRO_ParSet _ParameterSet)
{
	cJSON *root = cJSON_CreateObject();
	cJSON_AddNumberToObject(root, "idSRO_ParSetSel", _ParameterSet.ActParSet);
	cJSON_AddNumberToObject(root, "InputTime01", SolParSet.ParSet[_ParameterSet.ActParSet].TimeValues[0]);
	cJSON_AddNumberToObject(root, "InputTime02", SolParSet.ParSet[_ParameterSet.ActParSet].TimeValues[1]);
	cJSON_AddNumberToObject(root, "InputTime03", SolParSet.ParSet[_ParameterSet.ActParSet].TimeValues[2]);
	cJSON_AddNumberToObject(root, "InputTime04", SolParSet.ParSet[_ParameterSet.ActParSet].TimeValues[3]);
	cJSON_AddNumberToObject(root, "InputTime05", SolParSet.ParSet[_ParameterSet.ActParSet].TimeValues[4]);
		
	cJSON_AddNumberToObject(root, "InputTemp01", SolParSet.ParSet[_ParameterSet.ActParSet].TempValues[0]);
	cJSON_AddNumberToObject(root, "InputTemp02", SolParSet.ParSet[_ParameterSet.ActParSet].TempValues[1]);
	cJSON_AddNumberToObject(root, "InputTemp03", SolParSet.ParSet[_ParameterSet.ActParSet].TempValues[2]);
	cJSON_AddNumberToObject(root, "InputTemp04", SolParSet.ParSet[_ParameterSet.ActParSet].TempValues[3]);
	cJSON_AddNumberToObject(root, "InputTemp05", SolParSet.ParSet[_ParameterSet.ActParSet].TempValues[4]);
	
	return root;
}

cJSON *JSON_to_ParameterSet(char *_ParameterString)
{
	cJSON *root = cJSON_Parse(_ParameterString);
	SolParSet.ActParSet = cJSON_GetObjectItem(root, "idSRO_ParSetSel")->valueint;

	SolParSet.ParSet[SolParSet.ActParSet].TimeValues[0] = cJSON_GetObjectItem(root, "InputTime01")->valueint;
	SolParSet.ParSet[SolParSet.ActParSet].TimeValues[1] = cJSON_GetObjectItem(root, "InputTime02")->valueint;
	SolParSet.ParSet[SolParSet.ActParSet].TimeValues[2] = cJSON_GetObjectItem(root, "InputTime03")->valueint;
	SolParSet.ParSet[SolParSet.ActParSet].TimeValues[3] = cJSON_GetObjectItem(root, "InputTime04")->valueint;
	SolParSet.ParSet[SolParSet.ActParSet].TimeValues[4] = cJSON_GetObjectItem(root, "InputTime05")->valueint;

	SolParSet.ParSet[SolParSet.ActParSet].TempValues[0] = cJSON_GetObjectItem(root, "InputTemp01")->valueint;
	SolParSet.ParSet[SolParSet.ActParSet].TempValues[1] = cJSON_GetObjectItem(root, "InputTemp02")->valueint;
	SolParSet.ParSet[SolParSet.ActParSet].TempValues[2] = cJSON_GetObjectItem(root, "InputTemp03")->valueint;
	SolParSet.ParSet[SolParSet.ActParSet].TempValues[3] = cJSON_GetObjectItem(root, "InputTemp04")->valueint;
	SolParSet.ParSet[SolParSet.ActParSet].TempValues[4] = cJSON_GetObjectItem(root, "InputTemp05")->valueint;
	
	return root;
}


static void handle_mqtt_data(esp_mqtt_event_handle_t event)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	BaseType_t xReturned;

	char _tmpTopic[50] = { 0x00 };

	// extracting mqtt tag
	strncpy(_tmpTopic, event->topic, event->topic_len);

	if (strcmp(_tmpTopic, MQTT_SUB_PID_SAVE) == 0)
	{
		cJSON *root = JSON_to_PID_Data(event->data);
		PID_DataSave(root);
		cJSON_Delete(root);
	
	} 
	
	if (strcmp(_tmpTopic, MQTT_SUB_PID_LOAD) == 0)
	{
		PID_DataLoad();
		
		char *_tmpJSON_String = cJSON_Print(PID_Data_to_JSON(PID_Data));
		esp_mqtt_client_publish(event->client, MQTT_PUB_PID_LOAD, _tmpJSON_String, 0, 0, 0);
	}
	
	if (strcmp(_tmpTopic, MQTT_SUB_PAR_SAVE) == 0)
	{
		cJSON *root = cJSON_Parse(event->data);
		SolParSet.ActParSet = cJSON_GetObjectItem(root, "idSRO_ParSetSel")->valueint;

		SolParSet.ParSet[SolParSet.ActParSet].TimeValues[0] = cJSON_GetObjectItem(root, "InputTime01")->valueint;
		SolParSet.ParSet[SolParSet.ActParSet].TimeValues[1] = cJSON_GetObjectItem(root, "InputTime02")->valueint;
		SolParSet.ParSet[SolParSet.ActParSet].TimeValues[2] = cJSON_GetObjectItem(root, "InputTime03")->valueint;
		SolParSet.ParSet[SolParSet.ActParSet].TimeValues[3] = cJSON_GetObjectItem(root, "InputTime04")->valueint;
		SolParSet.ParSet[SolParSet.ActParSet].TimeValues[4] = cJSON_GetObjectItem(root, "InputTime05")->valueint;

		SolParSet.ParSet[SolParSet.ActParSet].TempValues[0] = cJSON_GetObjectItem(root, "InputTemp01")->valueint;
		SolParSet.ParSet[SolParSet.ActParSet].TempValues[1] = cJSON_GetObjectItem(root, "InputTemp02")->valueint;
		SolParSet.ParSet[SolParSet.ActParSet].TempValues[2] = cJSON_GetObjectItem(root, "InputTemp03")->valueint;
		SolParSet.ParSet[SolParSet.ActParSet].TempValues[3] = cJSON_GetObjectItem(root, "InputTemp04")->valueint;
		SolParSet.ParSet[SolParSet.ActParSet].TempValues[4] = cJSON_GetObjectItem(root, "InputTemp05")->valueint;
		
		ParameterSetSave(root);
		cJSON_Delete(root);
	
	} 
	
	if (strcmp(_tmpTopic, MQTT_SUB_PAR_LOAD) == 0)
	{
		cJSON *root = cJSON_Parse(event->data);
		SolParSet.ActParSet = cJSON_GetObjectItem(root, "idSRO_ParSetSel")->valueint;
		cJSON_Delete(root);

		ParameterSetLoad(SolParSet.ActParSet);
		
		char *_tmpJSON_String = cJSON_Print(ParameterSet_to_JSON(SolParSet));
			
		esp_mqtt_client_publish(event->client, MQTT_PUB_PAR_LOAD, _tmpJSON_String, 0, 0, 0);
	}
	
	if (strcmp(_tmpTopic, MQTT_SUB_CMD_RUN) == 0)
	{
		// soldering sequence run command from web interface
		cJSON *root = cJSON_Parse(event->data);
		CTRL_CMD.SEQ_RUN = cJSON_GetObjectItem(root, "SEQ_RUN")->valueint;
		
		char *_tmpJSON_String = cJSON_Print(root);
		cJSON_Delete(root);

		// need to add action for sequence start / stop		
		
		esp_mqtt_client_publish(event->client, MQTT_PUB_CMD_RUN, _tmpJSON_String, 0, 0, 0);

	}

	if (strcmp(_tmpTopic, MQTT_SUB_CMD_HEATER) == 0)
	{
		// heater ON/OFF run command from web interface
		cJSON *root = cJSON_Parse(event->data);
		CTRL_CMD.HEATER = cJSON_GetObjectItem(root, "HEATER")->valueint;
		
		char *_tmpJSON_String = cJSON_Print(root);
		cJSON_Delete(root);

		gpio_set_level(GPIO_SSR_HEATER, CTRL_CMD.HEATER);
		
		esp_mqtt_client_publish(event->client, MQTT_PUB_CMD_HEATER, _tmpJSON_String, 0, 0, 0);

	}
	
	if (strcmp(_tmpTopic, MQTT_SUB_CMD_FAN) == 0)
	{
		// heater ON/OFF run command from web interface
		cJSON *root = cJSON_Parse(event->data);
		CTRL_CMD.FAN = cJSON_GetObjectItem(root, "FAN")->valueint;
		
		char *_tmpJSON_String = cJSON_Print(root);
		cJSON_Delete(root);

		gpio_set_level(GPIO_SSR_FAN, CTRL_CMD.FAN);
		
		esp_mqtt_client_publish(event->client, MQTT_PUB_CMD_FAN, _tmpJSON_String, 0, 0, 0);

	}
	
	if (strcmp(_tmpTopic, MQTT_SUB_CMD_DOOR) == 0)
	{
		// heater ON/OFF run command from web interface
		cJSON *root = cJSON_Parse(event->data);
		CTRL_CMD.DOOR = ((atoi(cJSON_GetObjectItem(root, "DOOR")->valuestring)) * 180) / 100;
		
		//char *_tmpJSON_String = cJSON_Print(root);
		cJSON_Delete(root);

		// set the door value to PWD
		xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(hServo_CTRL_Task, CTRL_CMD.DOOR, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
		
		//esp_mqtt_client_publish(event->client, MQTT_PUB_CMD_DOOR, _tmpJSON_String, 0, 0, 0);

	}

	if (strcmp(_tmpTopic, MQTT_SUB_CMD_UPDATE) == 0)
	{
		// all parameter update command from web interface
		char *_tmpJSON_String = NULL;
		cJSON *root = NULL;

		_tmpJSON_String = cJSON_Print(ParameterSet_to_JSON(SolParSet));
		esp_mqtt_client_publish(event->client, MQTT_PUB_PAR_LOAD, _tmpJSON_String, 0, 0, 0);

		_tmpJSON_String = cJSON_Print(PID_Data_to_JSON(PID_Data));
		esp_mqtt_client_publish(event->client, MQTT_PUB_PID_LOAD, _tmpJSON_String, 0, 0, 0);
		
		root = cJSON_CreateObject();
		cJSON_AddNumberToObject(root, "SEQ_RUN", CTRL_CMD.SEQ_RUN);
		cJSON_AddNumberToObject(root, "HEATER", CTRL_CMD.HEATER);
		cJSON_AddNumberToObject(root, "FAN", CTRL_CMD.FAN);
		cJSON_AddNumberToObject(root, "DOOR", CTRL_CMD.DOOR);
		
		_tmpJSON_String = cJSON_Print(root);
		
		esp_mqtt_client_publish(event->client, MQTT_PUB_CMD_RUN, _tmpJSON_String, 0, 0, 0);
		esp_mqtt_client_publish(event->client, MQTT_PUB_CMD_HEATER, _tmpJSON_String, 0, 0, 0);
		esp_mqtt_client_publish(event->client, MQTT_PUB_CMD_FAN, _tmpJSON_String, 0, 0, 0);
		esp_mqtt_client_publish(event->client, MQTT_PUB_CMD_DOOR, _tmpJSON_String, 0, 0, 0);

		cJSON_Delete(root);
	}
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
	BaseType_t xReturned;
	esp_mqtt_client_handle_t client = event->client;
	int msg_id;

	switch(event->event_id)
	{
	case MQTT_EVENT_CONNECTED:
		xTimerReset(Lifebit_Timer, 0);		
		msg_id = esp_mqtt_client_subscribe(client, MQTT_SUB_PAR_SAVE, 0);
		msg_id = esp_mqtt_client_subscribe(client, MQTT_SUB_PAR_LOAD, 0);

		msg_id = esp_mqtt_client_subscribe(client, MQTT_SUB_PID_SAVE, 0);
		msg_id = esp_mqtt_client_subscribe(client, MQTT_SUB_PID_LOAD, 0);

		msg_id = esp_mqtt_client_subscribe(client, MQTT_SUB_CMD_RUN, 0);
		msg_id = esp_mqtt_client_subscribe(client, MQTT_SUB_CMD_HEATER, 0);
		msg_id = esp_mqtt_client_subscribe(client, MQTT_SUB_CMD_FAN, 0);
		msg_id = esp_mqtt_client_subscribe(client, MQTT_SUB_CMD_DOOR, 0);

		msg_id = esp_mqtt_client_subscribe(client, MQTT_SUB_CMD_UPDATE, 0);
		
		xEventGroupSetBits(mqtt_event_group, MQTT_CONNECTED_BIT);

		xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(hLED_CTRL_Task, TN_MQTT_CONNECTED, eSetBits, &xHigherPriorityTaskWoken);
		break;
	case MQTT_EVENT_DISCONNECTED:
		xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(hLED_CTRL_Task, TN_MQTT_DISCONNECTED, eSetBits, &xHigherPriorityTaskWoken);
		xEventGroupClearBits(mqtt_event_group, MQTT_CONNECTED_BIT);
//		if (th_mqtt_periodic_task != NULL)
//			vTaskDelete(th_mqtt_periodic_task);
//		xTimerStop(Lifebit_Timer, 0);
		break;
	case MQTT_EVENT_SUBSCRIBED:
		//		msg_id = esp_mqtt_client_publish(client, "/a2s/qos0", "data", 0, 0, 0);
				break;
	case MQTT_EVENT_UNSUBSCRIBED:
		break;
	case MQTT_EVENT_PUBLISHED:
		break;
	case MQTT_EVENT_DATA:
		handle_mqtt_data(event);
		//		printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
		//		printf("DATA=%.*s\r\n", event->data_len, event->data);
				break;
	case MQTT_EVENT_ERROR:
		xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(hLED_CTRL_Task, TN_MQTT_ERROR, eSetBits, &xHigherPriorityTaskWoken);
		break;
	default:
		break;
	}

	if (xHigherPriorityTaskWoken)
		taskYIELD(); 
	
	return ESP_OK;
}

void mqtt_app_start(void)
{
	xTaskNotify(hLED_CTRL_Task, TN_MQTT_DISCONNECTED, eSetBits);
	
	mqtt_event_group = xEventGroupCreate();
	esp_mqtt_client_config_t mqtt_cfg = MQTT_INIT_CONFIG_DEFAULT();
	_global_MQTT_Client = esp_mqtt_client_init(&mqtt_cfg);
	esp_mqtt_client_start(_global_MQTT_Client);
	
	xReturned = xTaskCreate(mqtt_periodic_task, "mqtt_periodic_task", 2048, (void *)_global_MQTT_Client, 5, &th_mqtt_periodic_task);

	Lifebit_Timer = xTimerCreate("Timer", 1000 / portTICK_PERIOD_MS, pdTRUE, NULL, Lifebit_Timer_Callback);
	if (Lifebit_Timer == NULL) {
		// The timer was not created.
	}
	else {
		if (xTimerStop(Lifebit_Timer, 0) != pdPASS) {
			// The timer could not be set into the Active state.
		}
	}
}