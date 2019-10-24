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

#include "nvs_flash.h"

#include "SRO_PID.h"
#include "cJSON.h"
#include "a2s_mqtt.h"
#include "a2s_wifi.h"
#include "a2s_nvs.h"


esp_err_t PID_DataSave(cJSON *_PID_Data)
{
	nvs_handle nvs_handle;
	esp_err_t err;
	char *_tmpJSON_String = NULL;

	// open storage system
	err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
	if (err != ESP_OK) {
		// send error to HMI and end function
		return err;
	}
	
	_tmpJSON_String = cJSON_Print(PID_Data_to_JSON(PID_Data));
	err = nvs_set_str(nvs_handle, "PID_Data", _tmpJSON_String);

	if (err != ESP_OK) {
		// send error to HMI and end function
		return err;
	}

	// house keeping
	free(_tmpJSON_String);
	//	free(_tmpPS_Name);
	
	// close storage system
	nvs_commit(nvs_handle);
	nvs_close(nvs_handle);
	// send save OK msg to HMI
	return ESP_OK;
}

esp_err_t PID_DataLoad()
{
	nvs_handle nvs_handle;
	esp_err_t err;
	size_t required_size = 0;
	char *_tmpJSON_String = NULL;

	// open storage system
	err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
	if (err != ESP_OK) {
		// send error to HMI and end function
		return err;
	}

	required_size = 0;
	err = nvs_get_str(nvs_handle, "PID_Data", NULL, &required_size);

	if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
		// send error to HMI and end function
		return err;
	}
	
	if (err != ESP_OK) {
		// send error to HMI and end function
		return err;
	}

	_tmpJSON_String = malloc(required_size);
	err = nvs_get_str(nvs_handle, "PID_Data", _tmpJSON_String, &required_size);

	if (err != ESP_OK) {
		// send error to HMI and end function
		return err;
	}

	JSON_to_PID_Data(_tmpJSON_String);

	// house keeping
	free(_tmpJSON_String);
	
	// close storage system
	err = nvs_commit(nvs_handle);
	ESP_ERROR_CHECK(err);

	nvs_close(nvs_handle);
	// send load OK msg to HMI
	return ESP_OK;
}

esp_err_t ParameterSetSave(cJSON *_ParameterSet)
{
	nvs_handle nvs_handle;
	esp_err_t err;
	char _tmpPS_Name[10];
	char *_tmpJSON_String = NULL;

	// open storage system
	err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
	if (err != ESP_OK) {
		// send error to HMI and end function
		return err;
	}
	
	sprintf(_tmpPS_Name, "ps-%.2i", SolParSet.ActParSet);

	_tmpJSON_String = cJSON_Print(ParameterSet_to_JSON(SolParSet));
	err = nvs_set_str(nvs_handle, _tmpPS_Name, _tmpJSON_String);

	if (err != ESP_OK) {
		// send error to HMI and end function
		return err;
	}

	// house keeping
	free(_tmpJSON_String);
//	free(_tmpPS_Name);
	
	// close storage system
	nvs_commit(nvs_handle);
	nvs_close(nvs_handle);
	// send save OK msg to HMI
	return ESP_OK;

}

esp_err_t ParameterSetLoad(uint8_t ParameterSetIndex)
{
	nvs_handle nvs_handle;
	esp_err_t err;
	size_t required_size = 0;
	char _tmpPS_Name[10];
	char *_tmpJSON_String = NULL;
//	int _tmpRecIdx;

	// open storage system
	err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
	if (err != ESP_OK) {
		// send error to HMI and end function
		return err;
	}

	required_size = 0;
	//	_tmpRecIdx = cJSON_GetObjectItem(ParameterSetIndex, "idSRO_ParSetSel")->valueint;
	sprintf(_tmpPS_Name, "ps-%.2i", ParameterSetIndex);
	err = nvs_get_str(nvs_handle, _tmpPS_Name, NULL, &required_size);

	if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
		// send error to HMI and end function
		return err;
	}
	
	if (err != ESP_OK) {
		// send error to HMI and end function
		return err;
	}

	_tmpJSON_String = malloc(required_size);
	err = nvs_get_str(nvs_handle, _tmpPS_Name, _tmpJSON_String, &required_size);

	if (err != ESP_OK) {
		// send error to HMI and end function
		return err;
	}

	JSON_to_ParameterSet(_tmpJSON_String);

	// house keeping
	free(_tmpJSON_String);
	
	// close storage system
	err = nvs_commit(nvs_handle);
	ESP_ERROR_CHECK(err);

	nvs_close(nvs_handle);
	// send load OK msg to HMI
	return ESP_OK;
	
}

esp_err_t init_nvs_data(void) {

	nvs_handle nvs_handle;
	esp_err_t err;
	size_t required_size = 0;

	char* mqtt_uri;
	char* wifi_ssid;
	char* wifi_pass;
	char _tmpPS_Name[10];
	char *_tmpJSON_String = NULL;
	cJSON *_tmp_JSON;

	// open storage system
	err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &nvs_handle);
	if (err != ESP_OK) return err;


	// check parameter set records
	for(uint8_t Idx = 1 ; Idx <= 10 ; Idx++)
	{
		sprintf(_tmpPS_Name, "ps-%.2i", Idx);
		
		SolParSet.ActParSet = Idx;
		
		required_size = 0;
		err = nvs_get_str(nvs_handle, _tmpPS_Name, NULL, &required_size);
		switch (err)
		{
			case ESP_ERR_NVS_NOT_FOUND:
				_tmpJSON_String = cJSON_Print(ParameterSet_to_JSON(SolParSet));
				err = nvs_set_str(nvs_handle, _tmpPS_Name, _tmpJSON_String);
				ESP_ERROR_CHECK(err);
				free(_tmpJSON_String);
				break;
			case ESP_OK:
				_tmpJSON_String = malloc(required_size);
				err = nvs_get_str(nvs_handle, _tmpPS_Name, _tmpJSON_String, &required_size);
				if (err == ESP_OK)
				{
					JSON_to_ParameterSet(_tmpJSON_String);
					free(_tmpJSON_String);
				}
				break;
			default:
				return err;
		}
	}
	
	// check PID Data records
	required_size = 0;
	err = nvs_get_str(nvs_handle, "PID_Data", NULL, &required_size);
	switch (err)
	{
	case ESP_ERR_NVS_NOT_FOUND:
		_tmpJSON_String = cJSON_Print(PID_Data_to_JSON(PID_Data));
		err = nvs_set_str(nvs_handle, "PID_Data", _tmpJSON_String);
		ESP_ERROR_CHECK(err);
		free(_tmpJSON_String);
		break;
	case ESP_OK:
		_tmpJSON_String = malloc(required_size);
		err = nvs_get_str(nvs_handle, "PID_Data", _tmpJSON_String, &required_size);
		if (err == ESP_OK)
		{
			JSON_to_PID_Data(_tmpJSON_String);
			free(_tmpJSON_String);
		}
		break;
	default:
		return err;
	}

	
	// check MQTT URI
	required_size = 0;
	err = nvs_get_str(nvs_handle, "mqtt_uri", NULL, &required_size);
	if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

	if (required_size == 0) {
		err = nvs_set_str(nvs_handle, "mqtt_uri", MQTT_SERVER_URI);
		ESP_ERROR_CHECK(err);
	}
	else {
		mqtt_uri = malloc(required_size);
		err = nvs_get_str(nvs_handle, "mqtt_uri", mqtt_uri, &required_size);
		if (err == ESP_OK)
//			strcpy(SRO_MQTT_URI, mqtt_uri);
		free(mqtt_uri);
	}

	// get SRO_WIFI_SSID
	required_size = 0;
	err = nvs_get_str(nvs_handle, "wifi_ssid", NULL, &required_size);
	if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

	if (required_size == 0) {
		err = nvs_set_str(nvs_handle, "wifi_ssid", SRO_ESP_WIFI_SSID);
		ESP_ERROR_CHECK(err);
	}
	else {
		wifi_ssid = malloc(required_size);
		err = nvs_get_str(nvs_handle, "wifi_ssid", wifi_ssid, &required_size);
		if (err == ESP_OK)
//			strcpy(SRO_WIFI_SSID, wifi_ssid);
		free(wifi_ssid);
	}

	// get SRO_WIFI_PASS
	required_size = 0;
	err = nvs_get_str(nvs_handle, "wifi_pass", NULL, &required_size);
	if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

	if (required_size == 0) {
		err = nvs_set_str(nvs_handle, "wifi_pass", SRO_ESP_WIFI_PASS);
		ESP_ERROR_CHECK(err);
	}
	else {
		wifi_pass = malloc(required_size);
		err = nvs_get_str(nvs_handle, "wifi_pass", wifi_pass, &required_size);
		if (err == ESP_OK)
//			strcpy(SRO_WIFI_PASS, wifi_pass);
		free(wifi_pass);
	}

	// close storage system
	nvs_commit(nvs_handle);
	nvs_close(nvs_handle);
	return ESP_OK;
}

