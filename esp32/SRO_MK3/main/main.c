/* Soldering Reflow Oven
 * Copyright (c) 2019-2020 Intector
 *
 * main
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

#include <stdio.h>
#include <stdbool.h>
//#include <iostream> 
//#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "sdkconfig.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"

#include "mqtt_client.h"
#include "cJSON.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "globals.h"

#include "board.h"
#include "MAX6675.h"
#include "SRO_PID.h"
#include "a2s_servo.h"
#include "a2s_mqtt.h"
#include "a2s_wifi.h"
#include "a2s_nvs.h"
#include "APA102C.h"

bool LED_InitReadyFlag = false;

//SRO_PID_Parameter PID_Par;

BaseType_t xReturned;
TaskHandle_t hLED_CTRL_Task = NULL;      				// LED control task handle
TaskHandle_t hServo_CTRL_Task = NULL;     				// Servo control task handle
TaskHandle_t hPID_Task = NULL;      					// PID task handle
TimerHandle_t LED_Blink_Timer = NULL;      				// timer to blink LED
TimerHandle_t TemperatureRead_Timer = NULL;    			// timer to read temperature

xQueueHandle Temperature_QueueHandleId;

void servo_control_task(void *arg)
{
	uint32_t ulNotifiedValue;
	BaseType_t xReturned; 

	uint32_t SetAngle = 0;
	int32_t DeltaAngle = 0;
	uint32_t ActAngle = SetAngle;

	// initial mcpwm configuration
	mcpwm_config_t pwm_config;
	pwm_config.frequency = 50;								//frequency = 50Hz, i.e. for every servo motor time period should be 20ms
	pwm_config.cmpr_a = 0;									//duty cycle of PWMxA = 0
	pwm_config.cmpr_b = 0;									//duty cycle of PWMxb = 0
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);	//Configure PWM0A & PWM0B with above settings

	SetAngle = SERVO_ANGLE_TO_PWM(0);
	mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, SetAngle);
	SetAngle = 0;
	vTaskDelay(1000 / portTICK_PERIOD_MS);

	while(1) {
		
		xReturned = xTaskNotifyWait(0x00, 0x00, &ulNotifiedValue, portMAX_DELAY);

		if (ulNotifiedValue >= 0 && ulNotifiedValue <= 180)
		{
			DeltaAngle = ulNotifiedValue - ActAngle;

			if (DeltaAngle < 0)
			{
				for (int Idx = 0; Idx > DeltaAngle; Idx--)
				{
					SetAngle = ActAngle + Idx;
					SetAngle = SERVO_ANGLE_TO_PWM(SetAngle);
					mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, SetAngle);
					vTaskDelay(SERVO_SPEED / portTICK_PERIOD_MS);
				}
			}
			
			if (DeltaAngle >= 0)
			{
				for (int Idx = 0; Idx < DeltaAngle; Idx++)
				{
					SetAngle = ActAngle + Idx;
					SetAngle = SERVO_ANGLE_TO_PWM(SetAngle);
					mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, SetAngle);
					vTaskDelay(SERVO_SPEED / portTICK_PERIOD_MS);
				}
			}


			ActAngle = ulNotifiedValue;
		
		}
		
	}
}

void app_main()
{
	esp_err_t err;

	// mod MAC address
	uint8_t mac[6] = { 0, 0, 0, 0, 0, 0 };
	esp_efuse_mac_get_default(mac);

	uint8_t mac_mod[6] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01 };
	esp_base_mac_addr_set(mac_mod);
	
	// check nvs
//	ESP_ERROR_CHECK(nvs_flash_erase());   		// NVS partition will be erased
	err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());  		// NVS partition was truncated and needs to be erased
		err = nvs_flash_init();  					// Retry nvs_flash_init
	}
	ESP_ERROR_CHECK(err);
	
	// get nvs going
	init_nvs_data();

	// prepare IO
	System_IO_Init();
	
	// get LED's going
	APA102C_Init(5);

	// waiting for LED's to get ready
	while (!LED_InitReadyFlag)
	{
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
	
	MAX6675_Init();
	wifi_init();
	mqtt_app_start();
	
	xReturned = xTaskCreate(servo_control_task, "ServoTask", 2048, NULL, 5, &hServo_CTRL_Task);
	if (xReturned != pdPASS) {
		// task could not be ceated
	}

	xReturned = xTaskCreate(PID_task, "PID_Task", 2048, NULL, 5, &hPID_Task);
	if (xReturned != pdPASS) {
		// task could not be ceated
	}

	while(1)
	{
		// should better never get here.
		
	}
	
}
